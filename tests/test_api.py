"""Tests for request validation and the Flask API layer."""

from __future__ import annotations

import json

import pytest

from app.model import ValidationError, normalise_request
from app.server import create_app


def _candidate(col, order, depth=None, confidence=5, cid=None):
    return {
        "id": cid or f"{col}-{order}",
        "depth": depth if depth is not None else 10 + order,
        "confidence": confidence,
    }


def _payload(n_cols=8, n_cands=3, **limit_overrides):
    limits = {
        "min_thickness": 1,
        "max_thickness": 40,
        "max_slope": 20,
        "max_thickness_change": 20,
        "max_second_difference": 20,
    }
    limits.update(limit_overrides)
    columns = []
    for c in range(n_cols):
        col = []
        for k in range(n_cands):
            col.append(_candidate(c, k, depth=10 + c + k * 10))
        columns.append(col)
    return {"columns": columns, "limits": limits}


# ---------------------------------------------------------------- validation

@pytest.mark.parametrize(
    "mutate",
    [
        lambda p: p.pop("columns"),
        lambda p: p.pop("limits"),
        lambda p: p["limits"].pop("max_slope"),
        lambda p: p.update(columns=[p["columns"][0]] * 7),
        lambda p: p.update(columns=[p["columns"][0]] * 25),
        lambda p: p["columns"][0].pop(),
        lambda p: p["columns"][0].extend(
            _candidate(0, 10 + k, cid=f"0-extra-{k}") for k in range(6)
        ),
        lambda p: p.update(columns="nope"),
        lambda p: p["limits"].update(max_slope=-1),
        lambda p: p["limits"].update(min_thickness=9, max_thickness=2),
        lambda p: p["columns"][0][0].update(confidence=0),
        lambda p: p["columns"][0][0].update(depth=1.5),
        lambda p: p["columns"][0][0].update(id=""),
        lambda p: p["columns"][0][1].update(id=p["columns"][0][0]["id"]),
    ],
)
def test_invalid_payloads_are_rejected(mutate):
    payload = _payload()
    mutate(payload)
    with pytest.raises(ValidationError):
        normalise_request(payload)


def test_bool_depth_is_rejected():
    payload = _payload()
    payload["columns"][0][0]["depth"] = True
    with pytest.raises(ValidationError):
        normalise_request(payload)


def test_valid_payload_normalises():
    request = normalise_request(_payload())
    assert len(request.columns) == 8
    assert all(len(c) == 3 for c in request.columns)
    assert request.limits.max_slope == 20


# --------------------------------------------------------------------- API

@pytest.fixture()
def client():
    app = create_app()
    app.testing = True
    return app.test_client()


def test_health(client):
    resp = client.get("/health")
    assert resp.status_code == 200
    assert resp.get_json() == {"status": "ok"}


def test_trace_success_response_shape(client):
    resp = client.post("/api/horizons/trace", json=_payload())
    assert resp.status_code == 200
    data = resp.get_json()
    assert data["feasible"] is True
    assert len(data["upper_horizon"]) == 8
    assert len(data["lower_horizon"]) == 8
    assert len(data["thicknesses"]) == 8
    assert len(data["slopes"]["upper"]) == 7
    assert len(data["slopes"]["lower"]) == 7
    assert len(data["second_differences"]["upper"]) == 6
    assert len(data["second_differences"]["lower"]) == 6
    adjudication = data["adjudication"]
    assert adjudication["total_confidence"] > 0
    assert "max_second_difference" in adjudication
    assert "total_travel" in adjudication
    # Points stay paired column by column and strictly separated.
    for i in range(8):
        u = data["upper_horizon"][i]
        l = data["lower_horizon"][i]
        assert u["id"] != l["id"]
        assert l["depth"] - u["depth"] == data["thicknesses"][i]
        assert data["thicknesses"][i] >= 1


def test_trace_infeasible_is_explicit(client):
    resp = client.post(
        "/api/horizons/trace",
        json=_payload(min_thickness=100, max_thickness=200),
    )
    assert resp.status_code == 200
    data = resp.get_json()
    assert data["feasible"] is False
    assert "upper_horizon" not in data
    assert "message" in data


def test_trace_validation_error_returns_400(client):
    resp = client.post("/api/horizons/trace", json={"columns": []})
    assert resp.status_code == 400
    assert "error" in resp.get_json()


def test_trace_non_json_body_returns_400(client):
    resp = client.post(
        "/api/horizons/trace",
        data="not json",
        content_type="text/plain",
    )
    assert resp.status_code == 400


def test_response_is_json_serialisable(client):
    resp = client.post("/api/horizons/trace", json=_payload())
    # Round-trip must not raise.
    json.loads(resp.data)
