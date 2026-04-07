#!/usr/bin/env python3
"""
V3.3.b bounded harness runner (measurement only, not a general CLI).

  # Offline smoke (deterministic fake adapter, no network):
  python3 scripts/run_llm_eval_harness_v33b.py --trials 2

  # Live provider (requires OPENAI_API_KEY; optional HDNS_V33A_LIVE_ADAPTER=1 pattern):
  python3 scripts/run_llm_eval_harness_v33b.py --trials 3 --live

  python3 scripts/run_llm_eval_harness_v33b.py --trials 1 --live
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Mapping

_PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "multi_robot_mission_stack"))
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from multi_robot_mission_stack.semantic.llm_eval_harness_v33b import (  # noqa: E402
    DEFAULT_FIXTURES_V33B,
    run_llm_eval_harness_v33b,
)
from multi_robot_mission_stack.semantic.llm_real_adapter_v33 import (  # noqa: E402
    LlmAdapterSuccess,
    OpenAiChatCompletionsAdapterV33,
)


def _fake_adapter_for_smoke():
    """Valid llm_candidate JSON for any fixture location_ref (not model-realistic)."""
    import json as _json

    def _call(ctx: Mapping[str, Any]) -> LlmAdapterSuccess:
        loc = str(ctx.get("location_ref", "base")).strip()
        payload = {
            "schema_version": "v3.3.llm_candidate.1",
            "assert_blocked": True,
            "location_ref": loc,
            "confidence": 0.5,
            "ttl_sec": 120.0,
            "sensor_class": "lidar_occlusion",
        }
        return LlmAdapterSuccess(raw_text=_json.dumps(payload))

    return _call


def main() -> int:
    p = argparse.ArgumentParser(description="V3.3.b bounded LLM eval harness (local measurement only).")
    p.add_argument("--trials", type=int, default=1, help="Trials per fixture (>=1).")
    p.add_argument(
        "--live",
        action="store_true",
        help="Use OpenAI-compatible chat completions adapter (network + API key).",
    )
    args = p.parse_args()

    if args.trials < 1:
        print("error: --trials must be >= 1", file=sys.stderr)
        return 2

    if args.live:
        adapter = OpenAiChatCompletionsAdapterV33(timeout_sec=120.0)
        if not os.environ.get("OPENAI_API_KEY", "").strip():
            print("error: --live requires OPENAI_API_KEY", file=sys.stderr)
            return 2
    else:
        print(
            "note: using built-in fake adapter (no network). "
            "Use --live for repeated real-model trials.",
            file=sys.stderr,
        )
        adapter = _fake_adapter_for_smoke()

    summary = run_llm_eval_harness_v33b(
        DEFAULT_FIXTURES_V33B,
        adapter_call=adapter,
        trials_per_fixture=args.trials,
    )
    print(json.dumps(summary.to_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
