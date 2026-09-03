# ADR-0003 — `imresamu/postgis` on host port 5433, with a native fallback

**Status:** Accepted (Phase 0)

## Context
The official `postgis/postgis` image publishes **no arm64 manifest** (verified across 30 tags via
the Docker Hub API; the upstream README confirms "Supported architecture: amd64"). On this M5 it
would run emulated. Homebrew PostgreSQL 17 is already running and owns host port 5432. The Docker
daemon is currently stopped.

## Decision
`imresamu/postgis:17-3.5-alpine` (genuine arm64, maintained by the #2 committer on the official
`postgis/docker-postgis`), mapped **5433:5432**, with the digest pinned. A `brew install postgis`
path is documented and supported as a fallback so `make dev` works with the daemon down.

## Consequences
- A one-line comment in `docker-compose.yml` explains *why* the official image is not used — that
  comment is itself the signal that the choice was verified rather than copy-pasted.
- After `brew install postgis`, restart `postgresql@17`: the running server is 17.10 while
  `pg_config` is 17.11, and PostGIS compiles against the latter.
- CI on x86 runners may use the official image via a compose override.
