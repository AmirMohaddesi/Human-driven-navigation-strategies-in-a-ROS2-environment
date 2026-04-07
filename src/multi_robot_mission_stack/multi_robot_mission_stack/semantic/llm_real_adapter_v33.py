"""
V3.3.a — minimal real-model adapter: allowlisted context → raw model text (or adapter failure).

Provider code stays here; boundary/assembler/eval gates are unchanged. No ROS/store/transport.
"""

from __future__ import annotations

import json
import os
import socket
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Union

# Adapter-local reason tokens (not candidate/boundary tokens)
ADAPTER_TIMEOUT = "adapter_timeout"
ADAPTER_EMPTY_OUTPUT = "adapter_empty_output"
ADAPTER_PROVIDER_ERROR = "adapter_provider_error"


@dataclass(frozen=True)
class LlmAdapterSuccess:
    """Raw model output exactly as returned (trimmed once at orchestration if needed)."""

    raw_text: str


@dataclass(frozen=True)
class LlmAdapterFailure:
    reason: str
    detail: str = ""


LlmAdapterResult = Union[LlmAdapterSuccess, LlmAdapterFailure]

_DEFAULT_SYSTEM_PROMPT = (
    "You output exactly one JSON object and nothing else: no markdown fences, no prose, "
    "no trailing text. The object must conform to the llm_candidate contract: keys may "
    'include schema_version (exactly "v3.3.llm_candidate.1"), assert_blocked (boolean), '
    "and when assert_blocked is true: location_ref, confidence, ttl_sec, sensor_class; "
    "optional rationale_short. If you cannot assert a blocked passage, set assert_blocked "
    "to false with only schema_version and assert_blocked."
)


def _json_user_payload(llm_context: Mapping[str, Any]) -> str:
    """Deterministic serialization for the user message body."""
    return json.dumps(llm_context, sort_keys=True, separators=(",", ":"))


class OpenAiChatCompletionsAdapterV33:
    """
    Minimal OpenAI-compatible ``/v1/chat/completions`` client (stdlib HTTP only).

    Reads API key from ``api_key`` or environment ``OPENAI_API_KEY``. Optional
    ``OPENAI_BASE_URL`` overrides default ``https://api.openai.com/v1``.
    """

    def __init__(
        self,
        *,
        model: str = "gpt-4o-mini",
        api_key: Optional[str] = None,
        base_url: Optional[str] = None,
        timeout_sec: float = 60.0,
        system_prompt: Optional[str] = None,
    ) -> None:
        self._model = model
        self._api_key = (api_key if api_key is not None else os.environ.get("OPENAI_API_KEY", "")).strip()
        env_base = os.environ.get("OPENAI_BASE_URL", "").strip()
        self._base_url = (base_url or env_base or "https://api.openai.com/v1").rstrip("/")
        self._timeout = float(timeout_sec)
        self._system_prompt = system_prompt or _DEFAULT_SYSTEM_PROMPT

    def __call__(self, llm_context: Mapping[str, Any]) -> LlmAdapterResult:
        if not self._api_key:
            return LlmAdapterFailure(
                ADAPTER_PROVIDER_ERROR,
                "missing API key (pass api_key= or set OPENAI_API_KEY)",
            )

        url = f"{self._base_url}/chat/completions"
        payload = {
            "model": self._model,
            "temperature": 0.0,
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": _json_user_payload(llm_context)},
            ],
        }
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=body,
            method="POST",
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self._api_key}",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                raw = resp.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            err_body = exc.read().decode("utf-8", errors="replace")[:4000]
            return LlmAdapterFailure(
                ADAPTER_PROVIDER_ERROR,
                f"HTTP {exc.code}: {err_body}",
            )
        except urllib.error.URLError as exc:
            msg = str(exc)
            reason = getattr(exc, "reason", None)
            if (
                "timed out" in msg.lower()
                or isinstance(reason, TimeoutError)
                or isinstance(reason, socket.timeout)
            ):
                return LlmAdapterFailure(ADAPTER_TIMEOUT, msg)
            return LlmAdapterFailure(ADAPTER_PROVIDER_ERROR, msg)
        except TimeoutError as exc:
            return LlmAdapterFailure(ADAPTER_TIMEOUT, str(exc))

        try:
            data = json.loads(raw)
            content = data["choices"][0]["message"].get("content")
        except (KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
            return LlmAdapterFailure(
                ADAPTER_PROVIDER_ERROR,
                f"unexpected response shape: {exc}; body[:500]={raw[:500]!r}",
            )

        if content is None:
            return LlmAdapterFailure(ADAPTER_EMPTY_OUTPUT, "message.content is null")
        if not isinstance(content, str):
            return LlmAdapterFailure(
                ADAPTER_PROVIDER_ERROR,
                f"message.content is not a string: {type(content).__name__}",
            )
        return LlmAdapterSuccess(raw_text=content)
