"""Joint horizon picking API smoke tests.

Runs entirely on the standard library so the slim verification image needs no
extra HTTP client.  Exits 0 only when every smoke assertion passes.

Usage: python3 smoke.py [base_url]
"""

from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request


DEFAULT_LIMITS = {
    "min_thickness": 1,
    "max_thickness": 40,
    "max_slope": 20,
    "max_thickness_change": 20,
    "max_second_difference": 20,
}


class SmokeFailure(AssertionError):
    pass


def _request(base_url, path, payload=None):
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(base_url + path, data=data, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def _assert(condition, message):
    if not condition:
        raise SmokeFailure(message)


def _smooth_payload(n_cols=8):
    upper = [10 + i for i in range(n_cols)]
    lower = [22 + i for i in range(n_cols)]
    columns = []
    for i in range(n_cols):
        columns.append(
            [
                {"id": f"u{i}", "depth": upper[i], "confidence": 6},
                {"id": f"n{i}", "depth": upper[i] - 6, "confidence": 1},
                {"id": f"l{i}", "depth": lower[i], "confidence": 7},
            ]
        )
    return {"columns": columns, "limits": DEFAULT_LIMITS}, upper, lower


def _assert_constraints(data, payload):
    n = len(payload["columns"])
    lim = payload["limits"]
    _assert(data["feasible"] is True, "expected feasible response")
    up, lo = data["upper_horizon"], data["lower_horizon"]
    _assert(len(up) == n and len(lo) == n, "horizon length mismatch")

    thicknesses = data["thicknesses"]
    _assert(len(thicknesses) == n, "thickness length mismatch")
    ids_ok = all(
        up[i]["id"] != lo[i]["id"]
        and lo[i]["depth"] > up[i]["depth"]
        and thicknesses[i] == lo[i]["depth"] - up[i]["depth"]
        and lim["min_thickness"] <= thicknesses[i] <= lim["max_thickness"]
        for i in range(n)
    )
    _assert(ids_ok, "per-column strict separation / thickness violated")

    for which in ("upper", "lower"):
        slopes = data["slopes"][which]
        seconds = data["second_differences"][which]
        _assert(len(slopes) == n - 1, f"{which} slopes length")
        _assert(len(seconds) == n - 2, f"{which} second differences length")
        _assert(all(abs(s) <= lim["max_slope"] for s in slopes),
                f"{which} slope limit violated")
        _assert(all(abs(d) <= lim["max_second_difference"] for d in seconds),
                f"{which} second difference limit violated")

    _assert(
        all(
            abs(thicknesses[i] - thicknesses[i - 1])
            <= lim["max_thickness_change"]
            for i in range(1, n)
        ),
        "thickness change limit violated",
    )

    adj = data["adjudication"]
    _assert(
        adj["total_confidence"]
        == sum(p["confidence"] for p in up + lo),
        "adjudication confidence mismatch",
    )
    _assert(
        adj["total_travel"]
        == sum(abs(s) for s in data["slopes"]["upper"]
               + data["slopes"]["lower"]),
        "adjudication travel mismatch",
    )
    expected_max2 = max(
        [0]
        + [abs(x) for x in data["second_differences"]["upper"]]
        + [abs(x) for x in data["second_differences"]["lower"]]
    )
    _assert(
        adj["max_second_difference"] == expected_max2,
        "adjudication max second difference mismatch",
    )


def run(base_url):
    checks = []

    # 1. Health ----------------------------------------------------------------
    status, body = _request(base_url, "/health")
    _assert(status == 200 and body == {"status": "ok"},
            f"health failed: {status} {body}")
    checks.append("health endpoint")

    # 2. Feasible joint pick ---------------------------------------------------
    payload, upper, lower = _smooth_payload(8)
    status, data = _request(base_url, "/api/horizons/trace", payload)
    _assert(status == 200, f"trace status {status}")
    _assert_constraints(data, payload)
    _assert(
        [p["depth"] for p in data["upper_horizon"]] == upper
        and [p["depth"] for p in data["lower_horizon"]] == lower,
        "joint pick did not follow the two smooth high-confidence layers",
    )
    checks.append("feasible joint pick (8 columns)")

    # 3. Strongest single echo must not be reused for both horizons -----------
    columns = [
        [
            {"id": f"a{i}", "depth": 10 + i, "confidence": 3},
            {"id": f"b{i}", "depth": 20 + i, "confidence": 9},
            {"id": f"c{i}", "depth": 30 + i, "confidence": 3},
        ]
        for i in range(8)
    ]
    status, data = _request(
        base_url,
        "/api/horizons/trace",
        {"columns": columns, "limits": DEFAULT_LIMITS},
    )
    _assert(status == 200, f"trace status {status}")
    _assert_constraints(data, {"columns": columns, "limits": DEFAULT_LIMITS})
    _assert(
        all(
            data["upper_horizon"][i]["id"]
            != data["lower_horizon"][i]["id"]
            for i in range(8)
        ),
        "same strongest candidate picked for both horizons",
    )
    checks.append("no shared candidate / no crossing")

    # 4. Explicit infeasibility, no fabricated partial trajectory -------------
    bad, _, _ = _smooth_payload(8)
    bad["limits"] = {**DEFAULT_LIMITS, "min_thickness": 90, "max_thickness": 200}
    status, data = _request(base_url, "/api/horizons/trace", bad)
    _assert(status == 200, f"infeasible status {status}")
    _assert(data["feasible"] is False, "expected feasible=false")
    _assert("upper_horizon" not in data and "lower_horizon" not in data,
            "infeasible response must not contain a fabricated trajectory")
    checks.append("infeasible instance reported without partial trace")

    # 5. Maximum-size request (24 columns x 8 candidates) ----------------------
    big_columns = []
    for i in range(24):
        big_columns.append(
            [
                {"id": f"c{i}_{k}", "depth": 10 + i + k * 9, "confidence": 1 + (k % 7)}
                for k in range(8)
            ]
        )
    big = {"columns": big_columns, "limits": DEFAULT_LIMITS}
    status, data = _request(base_url, "/api/horizons/trace", big)
    _assert(status == 200, f"large trace status {status}")
    _assert_constraints(data, big)
    checks.append("max-size request (24 columns x 8 candidates)")

    # 6. Invalid payload -> 400 ------------------------------------------------
    status, data = _request(
        base_url, "/api/horizons/trace", {"columns": []}
    )
    _assert(status == 400 and "error" in data,
            f"expected 400 with error, got {status} {data}")
    checks.append("validation rejection (400)")

    # 7. Deterministic adjudication -------------------------------------------
    payload, _, _ = _smooth_payload(8)
    _, first = _request(base_url, "/api/horizons/trace", payload)
    _, second = _request(base_url, "/api/horizons/trace", payload)
    _assert(first == second, "repeated requests returned different results")
    checks.append("deterministic stable adjudication")

    return checks


def main():
    base_url = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8000"
    print(f"smoke tests against {base_url}")
    try:
        checks = run(base_url)
    except Exception as exc:  # noqa: BLE001 - report any failure uniformly
        print(f"SMOKE FAILED: {exc}")
        return 1
    for name in checks:
        print(f"  PASS  {name}")
    print(f"SMOKE PASSED ({len(checks)} checks)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
