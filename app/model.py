"""Request validation and normalisation for the horizon tracing API.

Input contract
--------------
POST /api/horizons/trace
{
  "columns": [
    [{"id": "a", "depth": 10, "confidence": 5}, ...],   # 3..8 candidates
    ...                                                   # 8..24 columns
  ],
  "limits": {
    "min_thickness": int,            # upper depth strictly above lower depth,
    "max_thickness": int,            # thickness = lower.depth - upper.depth
    "max_slope": int,                # |depth difference| between adjacent cols
    "max_thickness_change": int,     # |thickness difference| between adjacent cols
    "max_second_difference": int    # |second order difference| over any 3 cols
  }
}

Every candidate carries a unique id (unique within its column), an integer
depth and a positive integer confidence.
"""

from __future__ import annotations

from dataclasses import dataclass

# Hard bounds straight from the specification.  They are validated here so that
# every other layer can rely on them.
MIN_COLUMNS = 8
MAX_COLUMNS = 24
MIN_CANDIDATES = 3
MAX_CANDIDATES = 8


class ValidationError(ValueError):
    """Raised when a request payload does not satisfy the input contract."""


@dataclass(frozen=True)
class Candidate:
    """A single echo candidate in one column."""

    id: str
    depth: int
    confidence: int
    # Original position of the candidate inside its column, used only as the
    # final (stable) tie-break key.
    order: int


@dataclass(frozen=True)
class Limits:
    min_thickness: int
    max_thickness: int
    max_slope: int
    max_thickness_change: int
    max_second_difference: int


@dataclass(frozen=True)
class TraceRequest:
    columns: tuple[tuple[Candidate, ...], ...]
    limits: Limits


def _require_object(value, path: str) -> dict:
    if not isinstance(value, dict):
        raise ValidationError(f"{path} must be an object")
    return value


def _require_int(value, path: str, *, minimum: int | None = None) -> int:
    # bool is a subclass of int in Python; reject it explicitly.
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValidationError(f"{path} must be an integer")
    if minimum is not None and value < minimum:
        raise ValidationError(f"{path} must be >= {minimum}")
    return value


def _normalise_limits(raw: object) -> Limits:
    if raw is None:
        raise ValidationError("limits is required")
    obj = _require_object(raw, "limits")
    known = {
        "min_thickness",
        "max_thickness",
        "max_slope",
        "max_thickness_change",
        "max_second_difference",
    }
    missing = known - obj.keys()
    if missing:
        raise ValidationError(f"limits is missing fields: {sorted(missing)}")

    min_thickness = _require_int(
        obj["min_thickness"], "limits.min_thickness", minimum=1
    )
    max_thickness = _require_int(
        obj["max_thickness"], "limits.max_thickness", minimum=1
    )
    if min_thickness > max_thickness:
        raise ValidationError(
            "limits.min_thickness must be <= limits.max_thickness"
        )
    return Limits(
        min_thickness=min_thickness,
        max_thickness=max_thickness,
        max_slope=_require_int(obj["max_slope"], "limits.max_slope", minimum=0),
        max_thickness_change=_require_int(
            obj["max_thickness_change"],
            "limits.max_thickness_change",
            minimum=0,
        ),
        max_second_difference=_require_int(
            obj["max_second_difference"],
            "limits.max_second_difference",
            minimum=0,
        ),
    )


def _normalise_columns(raw: object) -> tuple[tuple[Candidate, ...], ...]:
    if not isinstance(raw, list):
        raise ValidationError("columns must be an array")
    if not (MIN_COLUMNS <= len(raw) <= MAX_COLUMNS):
        raise ValidationError(
            f"columns length must be between {MIN_COLUMNS} and {MAX_COLUMNS}"
        )

    columns: list[tuple[Candidate, ...]] = []
    for ci, raw_col in enumerate(raw):
        path = f"columns[{ci}]"
        if not isinstance(raw_col, list):
            raise ValidationError(f"{path} must be an array")
        if not (MIN_CANDIDATES <= len(raw_col) <= MAX_CANDIDATES):
            raise ValidationError(
                f"{path} length must be between {MIN_CANDIDATES} and "
                f"{MAX_CANDIDATES}"
            )

        seen: set[str] = set()
        col: list[Candidate] = []
        for order, raw_cand in enumerate(raw_col):
            cpath = f"{path}[{order}]"
            obj = _require_object(raw_cand, cpath)
            for field in ("id", "depth", "confidence"):
                if field not in obj:
                    raise ValidationError(f"{cpath} is missing '{field}'")

            cand_id = obj["id"]
            if not isinstance(cand_id, str) or not cand_id:
                raise ValidationError(f"{cpath}.id must be a non-empty string")
            if cand_id in seen:
                raise ValidationError(
                    f"{cpath}.id {cand_id!r} is duplicated within {path}"
                )
            seen.add(cand_id)

            depth = _require_int(obj["depth"], f"{cpath}.depth")
            confidence = _require_int(
                obj["confidence"], f"{cpath}.confidence", minimum=1
            )
            col.append(
                Candidate(id=cand_id, depth=depth, confidence=confidence, order=order)
            )
        columns.append(tuple(col))
    return tuple(columns)


def normalise_request(payload: object) -> TraceRequest:
    """Validate the raw JSON payload and build a :class:`TraceRequest`."""
    obj = _require_object(payload, "request body")
    return TraceRequest(
        columns=_normalise_columns(obj.get("columns")),
        limits=_normalise_limits(obj.get("limits")),
    )
