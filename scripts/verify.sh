#!/bin/sh
# One-shot verification entrypoint for the `verify` compose service.
#
# Runs, in order:
#   1. code tests (pytest)
#   2. image build check (docker build of the application image)
#   3. joint horizon picking API smoke tests against the running api service
#
# Exits non-zero if any stage fails; compose reports that as the service exit
# code.

set -eu

API_HOST="${API_HOST:-api}"
API_PORT="${API_PORT:-8000}"
PROJECT_ROOT="${PROJECT_ROOT:-/workspace}"
BASE_URL="http://${API_HOST}:${API_PORT}"

status=0

run_stage() {
    name="$1"
    shift
    echo "== verify: ${name} =="
    if "$@"; then
        echo "== verify: ${name} PASSED =="
    else
        rc=$?
        echo "== verify: ${name} FAILED (exit ${rc}) =="
        status=1
    fi
}

# 1. Code tests --------------------------------------------------------------
run_stage "code tests" python3 -m pytest -q tests

# 2. Image build check -------------------------------------------------------
# The host docker socket is mounted, so this exercises a real build of the
# application image using the repository as build context.
run_stage "image build check" docker build \
    --target app \
    --tag horizon-api:verify-check \
    "${PROJECT_ROOT}"

# 3. API smoke tests ---------------------------------------------------------
# depends_on guarantees health, but poll once anyway so the check is
# self-contained when the script is run outside compose.
echo "== verify: waiting for api at ${BASE_URL}/health =="
i=0
while [ "$i" -lt 30 ]; do
    if python3 - "$BASE_URL" <<'PY'
import json
import sys
import urllib.request

with urllib.request.urlopen(sys.argv[1] + "/health", timeout=2) as resp:
    assert json.load(resp)["status"] == "ok"
PY
    then
        echo "== verify: api healthy =="
        break
    fi
    i=$((i + 1))
    sleep 2
done
if [ "$i" -ge 30 ]; then
    echo "== verify: api did not become healthy FAILED =="
    status=1
else
    run_stage "joint picking API smoke" python3 scripts/smoke.py "$BASE_URL"
fi

if [ "$status" -eq 0 ]; then
    echo "== verify: ALL STAGES PASSED =="
else
    echo "== verify: ONE OR MORE STAGES FAILED =="
fi
exit "$status"
