"""Joint upper/lower horizon picking via paired dynamic programming.

The two reflectors are traced *together*: a DP state at column ``i`` is the
ordered quadruple ``(upper_i, lower_i, upper_{i-1}, lower_{i-1})`` where each
element is a candidate index.  Carrying the previous column in the state lets
every transition evaluate the second order difference over three consecutive
columns for *both* horizons at once.  There is deliberately no stage that
picks one horizon first and fills in the other afterwards.

Adjudication order (applied globally):

    1. maximise total confidence
    2. minimise the largest second order difference on either horizon
    3. minimise total travel (sum of absolute slopes of both horizons)
    4. stable per-column candidate-order tie-break

Pruning
-------
Whether a prefix can be extended from a state depends only on the last two
columns encoded in that state; every constraint from column ``i+1`` onwards
uses depths at ``i-1, i, i+1`` at the earliest.  The set of feasible
continuations, and every achievable suffix score, is therefore identical for
all prefixes ending at the same state.

* A lower-confidence prefix can never beat a higher-confidence prefix at the
  same state (identical suffixes, confidence is additive and is the first
  criterion), so each state keeps only prefixes at its *single best
  confidence value*.
* Within that confidence, prefixes are pruned only when the winner is
  guaranteed for every suffix.  Travel is additive, so a strictly smaller
  travel with no larger curvature always wins.  Curvature is a ``max``: a
  smoother prefix can be caught up by a curved suffix, so when travels tie we
  may discard a prefix only when both its curvature is no smaller and its
  stable tie-break tuple is no earlier.  Everything else is retained, which
  keeps the frontier small without ever changing the adjudication.
"""

from __future__ import annotations

from dataclasses import dataclass

from .model import Candidate, Limits, TraceRequest


@dataclass(frozen=True)
class PickedPoint:
    id: str
    depth: int
    confidence: int
    order: int


@dataclass(frozen=True)
class Horizon:
    points: tuple[PickedPoint, ...]


@dataclass(frozen=True)
class TraceResult:
    feasible: bool
    upper: Horizon | None = None
    lower: Horizon | None = None
    thicknesses: tuple[int, ...] = ()
    upper_slopes: tuple[int, ...] = ()
    lower_slopes: tuple[int, ...] = ()
    upper_second_diffs: tuple[int, ...] = ()
    lower_second_diffs: tuple[int, ...] = ()
    total_confidence: int = 0
    max_second_difference: int = 0
    total_travel: int = 0
    columns: int = 0


class _Entry:
    """One Pareto-optimal partial path ending at a given state."""

    __slots__ = ("max2", "travel", "tie", "prev", "u", "l")

    def __init__(self, max2, travel, tie, prev, u, l):
        self.max2 = max2
        self.travel = travel
        self.tie = tie
        self.prev = prev
        self.u = u
        self.l = l


def _valid_pairs(column: tuple[Candidate, ...], limits: Limits):
    """All ``(upper_idx, lower_idx, thickness)`` choices for one column.

    Upper and lower must be distinct candidates with strictly separated
    depths and a thickness inside the allowed window.
    """
    pairs = []
    n = len(column)
    for u in range(n):
        du = column[u].depth
        for l in range(n):
            if u == l:
                continue
            t = column[l].depth - du
            if limits.min_thickness <= t <= limits.max_thickness:
                pairs.append((u, l, t))
    return pairs


def _safe_dominates(a: _Entry, b: _Entry) -> bool:
    """Whether prefix ``a`` can safely replace prefix ``b`` for every suffix.

    Both entries share a state and a confidence value.

    * A strictly smaller travel is strictly additive: combined with ``max2``
      no larger, ``a`` beats ``b`` on criterion 2 or 3 for *every* possible
      suffix, regardless of tie order.
    * When travel is equal, a smaller ``max2`` can be washed out by a curved
      suffix (``max`` is not additive); ``a`` may then only win on the stable
      tie-break, which is preserved under appending the same suffix to
      equal-length prefixes.  Hence the tie comparison is required.
    """
    if a.travel < b.travel and a.max2 <= b.max2:
        return True
    if a.travel == b.travel and a.max2 <= b.max2 and a.tie <= b.tie:
        return True
    return False


def _add(frontier: list[_Entry], entry: _Entry) -> None:
    """Insert one entry, pruning only suffix-safe dominated prefixes."""
    for kept in frontier:
        if _safe_dominates(kept, entry):
            return

    survivors = [other for other in frontier if not _safe_dominates(entry, other)]

    # Insert sorted by max2 (then travel) for a compact ordered frontier.
    lo, hi = 0, len(survivors)
    while lo < hi:
        mid = (lo + hi) // 2
        if (survivors[mid].max2, survivors[mid].travel) < (
            entry.max2,
            entry.travel,
        ):
            lo = mid + 1
        else:
            hi = mid
    survivors.insert(lo, entry)
    frontier[:] = survivors


def solve(request: TraceRequest) -> TraceResult:
    columns = request.columns
    limits = request.limits
    n_cols = len(columns)

    pairs_per_col = [_valid_pairs(col, limits) for col in columns]
    if any(not p for p in pairs_per_col):
        # A column without even one legal (upper, lower) pair means no global
        # joint solution exists; do not fabricate a partial trajectory.
        return TraceResult(feasible=False, columns=n_cols)

    # Each layer maps state -> (best_confidence, frontier).  The frontier only
    # contains paths at that single best confidence value.
    layer: dict[tuple[int, int, int, int], tuple[int, list[_Entry]]] = {}

    # --- Column 0 ------------------------------------------------------------
    layer0: dict[tuple[int, int], tuple[int, list[_Entry]]] = {}
    col0 = columns[0]
    for u0, l0, _t0 in pairs_per_col[0]:
        conf = col0[u0].confidence + col0[l0].confidence
        entry = _Entry(
            max2=0,
            travel=0,
            tie=((col0[u0].order, col0[l0].order),),
            prev=None,
            u=u0,
            l=l0,
        )
        existing = layer0.get((u0, l0))
        if existing is None:
            layer0[(u0, l0)] = (conf, [entry])
        elif conf > existing[0]:
            layer0[(u0, l0)] = (conf, [entry])
        elif conf == existing[0]:
            _add(existing[1], entry)

    # --- Column 1: first edge (slopes + thickness change, no curvature) ------
    col1 = columns[1]
    for u1, l1, t1 in pairs_per_col[1]:
        d1u = col1[u1].depth
        d1l = col1[l1].depth
        edge_conf = col1[u1].confidence + col1[l1].confidence
        edge_tail = ((col1[u1].order, col1[l1].order),)
        for u0, l0, t0 in pairs_per_col[0]:
            su = d1u - col0[u0].depth
            sl = d1l - col0[l0].depth
            if abs(su) > limits.max_slope or abs(sl) > limits.max_slope:
                continue
            if abs(t1 - t0) > limits.max_thickness_change:
                continue
            conf0, frontier0 = layer0[(u0, l0)]
            conf = conf0 + edge_conf
            state = (u1, l1, u0, l0)
            target = layer.get(state)
            for prev in frontier0:
                entry = _Entry(
                    max2=0,
                    travel=abs(su) + abs(sl),
                    tie=prev.tie + edge_tail,
                    prev=prev,
                    u=u1,
                    l=l1,
                )
                if target is None:
                    layer[state] = (conf, [entry])
                    target = layer[state]
                elif conf > target[0]:
                    target = (conf, [entry])
                    layer[state] = target
                elif conf == target[0]:
                    _add(target[1], entry)

    # --- Columns 2..n-1: full triple transition ------------------------------
    for i in range(2, n_cols):
        col_i = columns[i]
        col_p = columns[i - 1]
        col_pp = columns[i - 2]
        next_layer: dict[tuple[int, int, int, int], tuple[int, list[_Entry]]] = {}

        for u, l, t in pairs_per_col[i]:
            du = col_i[u].depth
            dl = col_i[l].depth
            edge_conf = col_i[u].confidence + col_i[l].confidence
            tie_tail = ((col_i[u].order, col_i[l].order),)

            for pu, pl, pt in pairs_per_col[i - 1]:
                su = du - col_p[pu].depth
                sl = dl - col_p[pl].depth
                if abs(su) > limits.max_slope or abs(sl) > limits.max_slope:
                    continue
                if abs(t - pt) > limits.max_thickness_change:
                    continue
                edge_travel = abs(su) + abs(sl)

                target_state = (u, l, pu, pl)
                target = next_layer.get(target_state)

                for ppu, ppl, _pt2 in pairs_per_col[i - 2]:
                    pred = layer.get((pu, pl, ppu, ppl))
                    if pred is None:
                        continue
                    d2u = du - 2 * col_p[pu].depth + col_pp[ppu].depth
                    d2l = dl - 2 * col_p[pl].depth + col_pp[ppl].depth
                    peak2 = abs(d2u) if abs(d2u) > abs(d2l) else abs(d2l)
                    if peak2 > limits.max_second_difference:
                        continue

                    conf0, pred_frontier = pred
                    conf = conf0 + edge_conf
                    for prev in pred_frontier:
                        m2 = prev.max2
                        if peak2 > m2:
                            m2 = peak2
                        entry = _Entry(
                            max2=m2,
                            travel=prev.travel + edge_travel,
                            tie=prev.tie + tie_tail,
                            prev=prev,
                            u=u,
                            l=l,
                        )
                        if target is None:
                            target = (conf, [entry])
                            next_layer[target_state] = target
                        elif conf > target[0]:
                            target = (conf, [entry])
                            next_layer[target_state] = target
                        elif conf == target[0]:
                            _add(target[1], entry)

        layer = next_layer
        if not layer:
            return TraceResult(feasible=False, columns=n_cols)

    # --- Global adjudication over every feasible complete path ---------------
    best_entry = None
    best_key = None
    for conf, frontier in layer.values():
        for entry in frontier:
            key = (-conf, entry.max2, entry.travel, entry.tie)
            if best_key is None or key < best_key:
                best_key = key
                best_entry = entry

    if best_entry is None:
        return TraceResult(feasible=False, columns=n_cols)
    winner = best_entry

    # --- Reconstruct the two horizons column by column ----------------------
    upper_idx = [0] * n_cols
    lower_idx = [0] * n_cols
    cur = winner
    for i in range(n_cols - 1, -1, -1):
        upper_idx[i] = cur.u
        lower_idx[i] = cur.l
        cur = cur.prev

    def make_horizon(indices) -> Horizon:
        return Horizon(
            points=tuple(
                PickedPoint(
                    id=columns[i][idx].id,
                    depth=columns[i][idx].depth,
                    confidence=columns[i][idx].confidence,
                    order=columns[i][idx].order,
                )
                for i, idx in enumerate(indices)
            )
        )

    upper = make_horizon(upper_idx)
    lower = make_horizon(lower_idx)

    thicknesses = tuple(
        lower.points[i].depth - upper.points[i].depth for i in range(n_cols)
    )
    upper_slopes = tuple(
        upper.points[i].depth - upper.points[i - 1].depth
        for i in range(1, n_cols)
    )
    lower_slopes = tuple(
        lower.points[i].depth - lower.points[i - 1].depth
        for i in range(1, n_cols)
    )
    upper_second = tuple(
        upper.points[i].depth
        - 2 * upper.points[i - 1].depth
        + upper.points[i - 2].depth
        for i in range(2, n_cols)
    )
    lower_second = tuple(
        lower.points[i].depth
        - 2 * lower.points[i - 1].depth
        + lower.points[i - 2].depth
        for i in range(2, n_cols)
    )

    return TraceResult(
        feasible=True,
        upper=upper,
        lower=lower,
        thicknesses=thicknesses,
        upper_slopes=upper_slopes,
        lower_slopes=lower_slopes,
        upper_second_diffs=upper_second,
        lower_second_diffs=lower_second,
        total_confidence=-best_key[0],
        max_second_difference=winner.max2,
        total_travel=winner.travel,
        columns=n_cols,
    )
