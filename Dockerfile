# syntax=docker/dockerfile:1

# ---------------------------------------------------------------------------
# Application image
# ---------------------------------------------------------------------------
FROM python:3.11-slim AS app

# Keep Python output unbuffered so container logs appear immediately.
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    API_PORT=8000

WORKDIR /app

# Install dependencies first for better layer caching.
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy application, tests and verification scripts.
COPY app ./app
COPY tests ./tests
COPY scripts ./scripts

# Run as an unprivileged user.
RUN useradd --create-home --uid 10001 appuser \
    && chown -R appuser:appuser /app
USER appuser

EXPOSE 8000

# The health probe hits the same endpoint the smoke tests use.  It reads the
# configurable API port from the container environment.
HEALTHCHECK --interval=5s --timeout=3s --start-period=5s --retries=10 \
    CMD python -c "import json,os,urllib.request; port=os.environ.get('API_PORT','8000'); r=urllib.request.urlopen('http://127.0.0.1:'+port+'/health',timeout=2); assert json.load(r)['status']=='ok'"

# gunicorn binds to the configurable API port.
CMD ["sh", "-c", "gunicorn --bind 0.0.0.0:${API_PORT} --workers 2 --threads 2 --access-logfile - --error-logfile - app.server:app"]

# ---------------------------------------------------------------------------
# One-shot verification image
#
# Combines the docker CLI (to perform the image build check against the mounted
# daemon socket) with Python + pytest (code tests and API smoke tests).
# ---------------------------------------------------------------------------
FROM docker:27.2-cli AS verify

RUN apk add --no-cache python3 py3-pip

WORKDIR /app

COPY requirements.txt ./
RUN pip3 install --break-system-packages --no-cache-dir -r requirements.txt

COPY app ./app
COPY tests ./tests
COPY scripts ./scripts

# API host/port are provided through compose; defaults match the compose setup.
ENV API_HOST=api \
    API_PORT=8000

ENTRYPOINT ["sh", "scripts/verify.sh"]
