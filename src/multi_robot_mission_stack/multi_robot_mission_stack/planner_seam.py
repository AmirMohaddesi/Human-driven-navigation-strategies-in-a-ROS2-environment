"""Thin planner-facing seam above Layer B: intent → v1 spec → inspect_checked → execute_checked."""

from __future__ import annotations

from typing import Any, Callable, Dict

from .coordinator import (
    execute_team_named_mission_spec_checked,
    inspect_team_named_mission_spec_checked,
    validate_team_named_mission_spec_contract,
)

InspectCheckedFn = Callable[[Any], Dict[str, Any]]
ExecuteCheckedFn = Callable[[Any], Dict[str, Any]]


def build_team_named_mission_spec_v1_from_intent(intent: Any) -> Dict[str, Any]:
    """
    Map minimal planner intent to a mission contract v1 spec and validate contract shape.

    Intent (dict):
        ``mode`` — optional, default ``"sequence"``; forwarded to spec.
        ``steps`` — list of ``{"robot_id", "location_name"}`` step dicts (default ``[]``).
        ``options`` — optional dict (same keys as Layer B runtime options).

    Returns a compact dict: ``ok``, ``overall_outcome``, ``spec``, ``message``, ``errors``,
    and ``contract`` (full return of ``validate_team_named_mission_spec_contract``).
    """
    bad = "Intent could not be converted to a valid v1 mission spec."
    if not isinstance(intent, dict):
        return {
            "ok": False,
            "overall_outcome": "error",
            "spec": {},
            "message": bad,
            "errors": ["intent must be a dict"],
            "contract": {
                "ok": False,
                "overall_outcome": "error",
                "message": bad,
                "errors": ["intent must be a dict"],
            },
        }

    spec: Dict[str, Any] = {
        "version": "v1",
        "mode": intent.get("mode", "sequence"),
        "steps": intent.get("steps", []),
    }
    if "options" in intent:
        spec["options"] = intent["options"]

    cv = validate_team_named_mission_spec_contract(spec)
    ok = bool(cv.get("ok"))
    errs = list(cv.get("errors", []))
    return {
        "ok": ok,
        "overall_outcome": "success" if ok else "error",
        "spec": spec,
        "message": str(cv.get("message", "")),
        "errors": errs,
        "contract": cv,
    }


def run_planner_team_named_mission_checked(
    intent: Any,
    *,
    inspect_checked: InspectCheckedFn = inspect_team_named_mission_spec_checked,
    execute_checked: ExecuteCheckedFn = execute_team_named_mission_spec_checked,
) -> Dict[str, Any]:
    """
    Build a v1 spec from intent, then ``inspect_team_named_mission_spec_checked`` and,
    if that succeeds, ``execute_team_named_mission_spec_checked``.

    Injectable ``inspect_checked`` / ``execute_checked`` for tests or wrappers.
    """
    b = build_team_named_mission_spec_v1_from_intent(intent)
    if not b.get("ok"):
        err = b["errors"][0] if b.get("errors") else b.get("message", "")
        return {
            "version": "v1",
            "ok": False,
            "overall_outcome": "error",
            "spec_build": b,
            "inspection": None,
            "execution": None,
            "summary": {"message": "Mission spec build or contract validation failed.", "error": err or None},
        }

    spec = b["spec"]
    ins = inspect_checked(spec)
    if not bool(ins.get("ok")):
        isum = ins.get("summary") if isinstance(ins.get("summary"), dict) else {}
        return {
            "version": "v1",
            "ok": False,
            "overall_outcome": "error",
            "spec_build": b,
            "inspection": ins,
            "execution": None,
            "summary": {
                "message": "Mission spec inspection (checked) failed.",
                "error": isum.get("error"),
            },
        }

    exe = execute_checked(spec)
    esum = exe.get("summary") if isinstance(exe.get("summary"), dict) else {}
    return {
        "version": "v1",
        "ok": bool(exe.get("ok")),
        "overall_outcome": str(exe.get("overall_outcome", "error")),
        "spec_build": b,
        "inspection": ins,
        "execution": exe,
        "summary": {
            "message": str(esum.get("message", "")),
            "error": esum.get("error"),
        },
    }
