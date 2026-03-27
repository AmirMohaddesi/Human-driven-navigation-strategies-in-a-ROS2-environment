"""Minimal deterministic LangGraph workflow over MissionTools (no ROS, no LLM)."""

from __future__ import annotations

from typing import Any, Dict, Literal, Optional, cast

from langgraph.graph import END, START, StateGraph

from .graph_state import MissionGraphState
from .mission_policy import MissionPolicy
from .mission_tools import MissionTools
from .policy_config import build_default_policy_config


def _failure(message: str) -> Dict[str, Any]:
    return {
        "status": "failed",
        "message": message,
        "nav_status": "unknown",
    }


class MissionGraph:
    """
    Small StateGraph: parse request → policy gate → route by ``action`` → execute.
    """

    def __init__(
        self,
        tools: MissionTools,
        policy: Optional[MissionPolicy] = None,
    ) -> None:
        self._tools = tools
        self._policy = policy if policy is not None else MissionPolicy(
            build_default_policy_config()
        )
        self._graph = self._build_graph()

    def invoke(self, request: dict) -> dict:
        final: MissionGraphState = cast(
            MissionGraphState,
            self._graph.invoke({"request": request}),
        )
        out = final.get("result")
        if not isinstance(out, dict):
            return _failure("graph produced no result")
        return out

    def _build_graph(self) -> Any:
        builder = StateGraph(MissionGraphState)

        builder.add_node("parse_request", self._parse_request_node)
        builder.add_node("policy_check", self._policy_check_node)
        builder.add_node("finalize_request_error", self._finalize_request_error)
        builder.add_node("finalize_policy_denial", self._finalize_policy_denial)
        builder.add_node(
            "execute_navigate_to_named_location",
            self._execute_navigate_to_named_location,
        )
        builder.add_node(
            "execute_get_navigation_state",
            self._execute_get_navigation_state,
        )
        builder.add_node("unsupported_action", self._unsupported_action)

        builder.add_edge(START, "parse_request")
        builder.add_conditional_edges(
            "parse_request",
            self._route_after_parse,
            {
                "finalize_request_error": "finalize_request_error",
                "policy_check": "policy_check",
            },
        )
        builder.add_conditional_edges(
            "policy_check",
            self._route_after_policy,
            {
                "finalize_policy_denial": "finalize_policy_denial",
                "execute_navigate_to_named_location": "execute_navigate_to_named_location",
                "execute_get_navigation_state": "execute_get_navigation_state",
                "unsupported_action": "unsupported_action",
            },
        )
        for node in (
            "finalize_request_error",
            "finalize_policy_denial",
            "execute_navigate_to_named_location",
            "execute_get_navigation_state",
            "unsupported_action",
        ):
            builder.add_edge(node, END)

        return builder.compile()

    def _parse_request_node(self, state: MissionGraphState) -> Dict[str, Any]:
        req = state.get("request")
        if not isinstance(req, dict):
            return {
                "request": {},
                "action": "",
                "error": "request must be a dict",
            }
        raw_action = req.get("action")
        if not isinstance(raw_action, str) or not raw_action.strip():
            return {
                "request": dict(req),
                "action": "",
                "error": "missing or invalid non-empty string field 'action'",
            }
        action = raw_action.strip()
        return {
            "request": dict(req),
            "action": action,
            "error": "",
        }

    def _route_after_parse(
        self, state: MissionGraphState
    ) -> Literal["finalize_request_error", "policy_check"]:
        if state.get("error"):
            return "finalize_request_error"
        return "policy_check"

    def _policy_check_node(self, state: MissionGraphState) -> Dict[str, Any]:
        req = state.get("request") or {}
        verdict = self._policy.evaluate(req if isinstance(req, dict) else {})
        allowed = bool(verdict.get("allowed"))
        message = str(verdict.get("message", ""))
        return {
            "policy_allowed": allowed,
            "policy_message": message,
        }

    def _route_after_policy(
        self, state: MissionGraphState
    ) -> Literal[
        "finalize_policy_denial",
        "execute_navigate_to_named_location",
        "execute_get_navigation_state",
        "unsupported_action",
    ]:
        if not state.get("policy_allowed", False):
            return "finalize_policy_denial"
        action = state.get("action", "")
        if action == "navigate_to_named_location":
            return "execute_navigate_to_named_location"
        if action == "get_navigation_state":
            return "execute_get_navigation_state"
        return "unsupported_action"

    def _finalize_request_error(self, state: MissionGraphState) -> Dict[str, Any]:
        msg = state.get("error") or "invalid request"
        return {"result": _failure(str(msg))}

    def _finalize_policy_denial(self, state: MissionGraphState) -> Dict[str, Any]:
        msg = state.get("policy_message") or "policy denied"
        return {"result": _failure(str(msg))}

    def _execute_navigate_to_named_location(
        self, state: MissionGraphState
    ) -> Dict[str, Any]:
        req = state.get("request") or {}
        robot_id = req.get("robot_id")
        location_name = req.get("location_name")
        if robot_id is None:
            return {
                "result": _failure("missing required field 'robot_id'"),
            }
        if location_name is None:
            return {
                "result": _failure("missing required field 'location_name'"),
            }
        result = self._tools.navigate_to_named_location(
            str(robot_id),
            str(location_name),
        )
        return {"result": result}

    def _execute_get_navigation_state(
        self, state: MissionGraphState
    ) -> Dict[str, Any]:
        req = state.get("request") or {}
        robot_id = req.get("robot_id")
        goal_id = req.get("goal_id")
        if robot_id is None:
            return {
                "result": _failure("missing required field 'robot_id'"),
            }
        if goal_id is None:
            return {
                "result": _failure("missing required field 'goal_id'"),
            }
        result = self._tools.get_navigation_state(str(robot_id), str(goal_id))
        return {"result": result}

    def _unsupported_action(self, state: MissionGraphState) -> Dict[str, Any]:
        action = state.get("action", "")
        return {
            "result": _failure(f"unsupported action: {action!r}"),
        }
