"""Unit tests for the joint horizon DP solver.

The strongest guarantee here comes from a brute-force oracle: for small
instances every joint upper/lower combination is enumerated and adjudicated
with the exact same scoring rules, then compared against the DP result.
"""

from __future__ import annotations

import random

from app.model import Candidate, Limits, TraceRequest
from app.solver import solve


def _make_request(columns, limits):
    cols = tuple(
        tuple(
            Candidate(
                id=f"c{ci}_{oi}",
                depth=d,
                confidence=conf,
                order=oi,
            )
            for oi, (d, conf) in enumerate(col)
        )
        for ci, col in enumerate(columns)
    )
    return TraceRequest(columns=cols, limits=Limits(**limits))


def _brute_force(request):
    """DFS with full constraint pruning; return the winning score or None.

    Enumerates the same joint combinations as a Cartesian product but prunes
    partial prefixes as soon as a slope, thickness or curvature constraint is
    violated, which keeps the randomised suite fast.
    """
    columns = request.columns
    lim = request.limits
    n = len(columns)

    pairs = []
    for col in columns:
        choices = []
        for u in range(len(col)):
            for l in range(len(col)):
                if u == l:
                    continue
                t = col[l].depth - col[u].depth
                if lim.min_thickness <= t <= lim.max_thickness:
                    choices.append((u, l, t))
        pairs.append(choices)
        if not choices:
            return None

    best = None
    best_pick = None
    pick: list[tuple[int, int, int]] = []

    def consider():
        nonlocal best, best_pick
        travel = 0
        max2 = 0
        conf = 0
        for i in range(n):
            u, l, t = pick[i]
            conf += columns[i][u].confidence + columns[i][l].confidence
            if i >= 1:
                travel += abs(
                    columns[i][u].depth - columns[i - 1][pick[i - 1][0]].depth
                ) + abs(
                    columns[i][l].depth - columns[i - 1][pick[i - 1][1]].depth
                )
            if i >= 2:
                max2 = max(
                    max2,
                    abs(
                        columns[i][u].depth
                        - 2 * columns[i - 1][pick[i - 1][0]].depth
                        + columns[i - 2][pick[i - 2][0]].depth
                    ),
                    abs(
                        columns[i][l].depth
                        - 2 * columns[i - 1][pick[i - 1][1]].depth
                        + columns[i - 2][pick[i - 2][1]].depth
                    ),
                )
        tie = tuple(
            (columns[i][pick[i][0]].order, columns[i][pick[i][1]].order)
            for i in range(n)
        )
        key = (-conf, max2, travel, tie)
        if best is None or key < best:
            best = key
            best_pick = tuple(pick)

    def dfs(i):
        if i == n:
            consider()
            return
        prev = pick[-1] if pick else None
        prev2 = pick[-2] if len(pick) >= 2 else None
        for u, l, t in pairs[i]:
            if prev is not None:
                pu, pl, pt = prev
                if (
                    abs(columns[i][u].depth - columns[i - 1][pu].depth)
                    > lim.max_slope
                    or abs(columns[i][l].depth - columns[i - 1][pl].depth)
                    > lim.max_slope
                    or abs(t - pt) > lim.max_thickness_change
                ):
                    continue
            if prev2 is not None:
                pu, pl, _ = prev
                ppu, ppl, _ = prev2
                if max(
                    abs(
                        columns[i][u].depth
                        - 2 * columns[i - 1][pu].depth
                        + columns[i - 2][ppu].depth
                    ),
                    abs(
                        columns[i][l].depth
                        - 2 * columns[i - 1][pl].depth
                        + columns[i - 2][ppl].depth
                    ),
                ) > lim.max_second_difference:
                    continue
            pick.append((u, l, t))
            dfs(i + 1)
            pick.pop()

    dfs(0)
    if best is None:
        return None
    return best, best_pick


def _check_constraints(result, request):
    """Re-verify every constraint on a returned solution."""
    lim = request.limits
    n = request.columns
    assert result.feasible
    assert len(result.upper.points) == len(n)
    assert len(result.lower.points) == len(n)
    for i in range(len(n)):
        u = result.upper.points[i]
        l = result.lower.points[i]
        assert u.id != l.id
        t = l.depth - u.depth
        assert t == result.thicknesses[i]
        assert lim.min_thickness <= t <= lim.max_thickness
    for i in range(1, len(n)):
        assert abs(result.upper_slopes[i - 1]) <= lim.max_slope
        assert abs(result.lower_slopes[i - 1]) <= lim.max_slope
        assert (
            abs(result.thicknesses[i] - result.thicknesses[i - 1])
            <= lim.max_thickness_change
        )
    for i in range(2, len(n)):
        assert abs(result.upper_second_diffs[i - 2]) <= lim.max_second_difference
        assert abs(result.lower_second_diffs[i - 2]) <= lim.max_second_difference
    assert result.max_second_difference == max(
        [0]
        + [abs(x) for x in result.upper_second_diffs]
        + [abs(x) for x in result.lower_second_diffs]
    )
    assert result.total_travel == sum(
        abs(s) for s in (*result.upper_slopes, *result.lower_slopes)
    )
    assert result.total_confidence == sum(
        p.confidence for p in (*result.upper.points, *result.lower.points)
    )


LIMITS = dict(
    min_thickness=1,
    max_thickness=20,
    max_slope=10,
    max_thickness_change=10,
    max_second_difference=10,
)


def test_smooth_synthetic_profile_is_feasible():
    upper_depths = [10, 11, 12, 12, 13, 14, 14, 15]
    lower_depths = [20, 21, 22, 23, 23, 24, 25, 26]
    columns = []
    for i in range(8):
        candidates = [
            (upper_depths[i] - 7, 1),
            (upper_depths[i], 5),
            (lower_depths[i], 6),
        ]
        columns.append(candidates)
    request = _make_request(columns, LIMITS)
    result = solve(request)
    _check_constraints(result, request)
    # The smooth, high-confidence layers must win.
    assert [p.depth for p in result.upper.points] == upper_depths
    assert [p.depth for p in result.lower.points] == lower_depths


def test_strongest_echo_greedy_would_cross_but_joint_pick_does_not():
    # In every column the middle candidate has the largest confidence; a
    # per-column strongest-echo selection would pick the same depth for both
    # horizons (crossing / zero thickness).  The joint pick must separate them.
    columns = []
    for i in range(8):
        columns.append([(10 + i, 3), (20 + i, 9), (30 + i, 3)])
    request = _make_request(columns, LIMITS)
    result = solve(request)
    _check_constraints(result, request)
    assert result.upper.points[0].depth == 10
    assert result.lower.points[0].depth == 20


def test_infeasible_thickness_returns_no_solution():
    columns = [[(0, 5), (1, 5), (2, 5)] for _ in range(8)]
    request = _make_request(columns, {**LIMITS, "min_thickness": 5})
    result = solve(request)
    assert not result.feasible
    assert result.upper is None and result.lower is None


def test_infeasible_slope_returns_no_solution():
    columns = []
    for i in range(8):
        base = 10 if i % 2 == 0 else 40
        columns.append([(base, 5), (base + 10, 5), (base + 20, 5)])
    request = _make_request(columns, {**LIMITS, "max_slope": 5})
    result = solve(request)
    assert not result.feasible


def test_infeasible_second_difference_returns_no_solution():
    # Zig-zag profile: slope magnitude 10 (admissible) but second difference
    # magnitude 20, so it is the curvature constraint alone that rules it out.
    depths = [10, 20, 10, 20, 10, 20, 10, 20]
    columns = [[(d, 5), (d + 5, 5), (d + 10, 5)] for d in depths]
    # With these offsets the smoothest admissible zig-zag still has second
    # difference magnitude 10, so 9 rules every combination out while 20
    # admits them.
    request = _make_request(columns, {**LIMITS, "max_second_difference": 9})
    result = solve(request)
    assert not result.feasible
    # Relaxing the limit makes it feasible again.
    relaxed = _make_request(columns, {**LIMITS, "max_second_difference": 20})
    assert solve(relaxed).feasible


def test_confidence_outranks_smoothness():
    # A flat weak layer vs a still-admissible but bumpy strong layer.
    columns = []
    for i in range(8):
        bump = 4 if i % 2 == 0 else -4
        columns.append(
            [(10, 2), (10 + bump, 9), (40 + bump, 9), (40, 2)]
        )
    limits = {
        "min_thickness": 1,
        "max_thickness": 40,
        "max_slope": 10,
        "max_thickness_change": 40,
        "max_second_difference": 20,
    }
    request = _make_request(columns, limits)
    result = solve(request)
    _check_constraints(result, request)
    # High confidence bumpy layers beat the smooth but faint ones.
    assert result.total_confidence == 8 * 18
    assert result.max_second_difference > 0


def test_stable_tie_break_prefers_earlier_candidate_order():
    # Two identical-depth layers at different positions/ids: same scores,
    # so the lexicographically earliest candidate orders must win.
    columns = []
    for i in range(8):
        # depths: 10 appears at order 0 and 2; 30 at order 1 and 3.
        columns.append([(10, 5), (30, 5), (10, 5), (30, 5)])
    request = _make_request(columns, LIMITS)
    result = solve(request)
    _check_constraints(result, request)
    assert [p.order for p in result.upper.points] == [0] * 8
    assert [p.order for p in result.lower.points] == [1] * 8


def test_smoother_path_outranks_travel_when_confidence_tied():
    # Two admissible upper profiles with identical confidence; one zig-zags
    # (second difference 12) while the other is monotone.  Both are below the
    # curvature cap, but the monotone one must win on the second criterion.
    zig = [10, 16, 10, 16, 10, 16, 10, 16]
    flat = [10, 11, 12, 13, 14, 15, 16, 17]
    columns = [
        [(zig[i], 5), (flat[i], 5), (40, 5)] for i in range(8)
    ]
    request = _make_request(
        columns,
        dict(
            min_thickness=1,
            max_thickness=40,
            max_slope=10,
            max_thickness_change=10,
            max_second_difference=15,
        ),
    )
    result = solve(request)
    _check_constraints(result, request)
    assert [p.depth for p in result.upper.points] == flat
    assert result.max_second_difference == 0


def test_brute_force_oracle_matches_dp_on_handcrafted_case():
    columns = [
        [(0, 1), (5, 7), (10, 2)],
        [(1, 3), (6, 1), (11, 8)],
        [(0, 6), (5, 2), (12, 3)],
        [(2, 2), (7, 9), (13, 1)],
        [(1, 4), (6, 2), (12, 6)],
        [(0, 8), (7, 1), (11, 2)],
        [(2, 1), (8, 7), (13, 4)],
        [(1, 5), (6, 3), (12, 2)],
    ]
    limits = dict(
        min_thickness=3,
        max_thickness=12,
        max_slope=4,
        max_thickness_change=6,
        max_second_difference=8,
    )
    request = _make_request(columns, limits)
    result = solve(request)
    oracle = _brute_force(request)
    if oracle is None:
        assert not result.feasible
    else:
        _check_constraints(result, request)
        (neg_conf, max2, travel, _tie), pick = oracle
        assert result.total_confidence == -neg_conf
        assert result.max_second_difference == max2
        assert result.total_travel == travel
        for i, (u, l, _) in enumerate(pick):
            assert result.upper.points[i].order == u
            assert result.lower.points[i].order == l


def test_brute_force_oracle_matches_dp_on_random_instances():
    rng = random.Random(20261001)
    checked = 0
    for _ in range(80):
        n = 8
        size = rng.randint(3, 4)
        # Smooth random walk keeps plenty of jointly feasible combinations.
        columns = []
        level = 12
        for _ in range(n):
            level += rng.choice((-2, -1, 0, 1, 2))
            depths = sorted(
                level + offset for offset in (0, 8, 16, 22)[:size]
            )
            columns.append(
                [(d, rng.randint(1, 9)) for d in depths]
            )
        limits = dict(
            min_thickness=rng.randint(1, 4),
            max_thickness=rng.randint(20, 30),
            max_slope=rng.randint(6, 14),
            max_thickness_change=rng.randint(6, 14),
            max_second_difference=rng.randint(6, 16),
        )
        request = _make_request(columns, limits)
        result = solve(request)
        oracle = _brute_force(request)
        if oracle is None:
            assert not result.feasible
            continue
        checked += 1
        _check_constraints(result, request)
        (neg_conf, max2, travel, tie), pick = oracle
        assert result.total_confidence == -neg_conf
        assert result.max_second_difference == max2
        assert result.total_travel == travel
        dp_tie = tuple(
            (result.upper.points[i].order, result.lower.points[i].order)
            for i in range(n)
        )
        assert dp_tie == tie
        for i, (u, l, _) in enumerate(pick):
            assert result.upper.points[i].order == u
            assert result.lower.points[i].order == l
    # Sanity: the random suite actually exercised feasible instances.
    assert checked >= 10
