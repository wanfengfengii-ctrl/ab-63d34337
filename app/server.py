"""HTTP API for joint upper/lower horizon tracing."""

from __future__ import annotations

import os

from flask import Flask, jsonify, request

from .model import ValidationError, normalise_request
from .solver import solve


def _point(point) -> dict:
    return {
        "id": point.id,
        "depth": point.depth,
        "confidence": point.confidence,
        "order": point.order,
    }


def _serialise_result(result) -> dict:
    if not result.feasible:
        # Explicit infeasibility: never return a fabricated partial trace.
        return {
            "feasible": False,
            "columns": result.columns,
            "message": "no feasible joint upper/lower horizon combination",
        }

    upper = [_point(p) for p in result.upper.points]
    lower = [_point(p) for p in result.lower.points]
    return {
        "feasible": True,
        "columns": result.columns,
        "upper_horizon": upper,
        "lower_horizon": lower,
        "thicknesses": list(result.thicknesses),
        "slopes": {
            "upper": list(result.upper_slopes),
            "lower": list(result.lower_slopes),
        },
        "second_differences": {
            "upper": list(result.upper_second_diffs),
            "lower": list(result.lower_second_diffs),
        },
        "adjudication": {
            "total_confidence": result.total_confidence,
            "max_second_difference": result.max_second_difference,
            "total_travel": result.total_travel,
        },
    }


def create_app() -> Flask:
    app = Flask(__name__)

    @app.get("/health")
    def health():
        return jsonify(status="ok")

    @app.post("/api/horizons/trace")
    def trace():
        payload = request.get_json(silent=True)
        if payload is None:
            return (
                jsonify(error="request body must be valid JSON"),
                400,
            )
        try:
            trace_request = normalise_request(payload)
        except ValidationError as exc:
            return jsonify(error=str(exc)), 400

        result = solve(trace_request)
        # Infeasibility is a normal 200 result: the request was valid, the
        # joint pick simply has no admissible combination.
        return jsonify(_serialise_result(result)), 200

    return app


app = create_app()


if __name__ == "__main__":
    port = int(os.environ.get("API_PORT", "8000"))
    app.run(host="0.0.0.0", port=port)
