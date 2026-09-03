.DEFAULT_GOAL := help
SHELL := /bin/bash

# This machine has no IPv6 route and Node stalls on hosts that publish AAAA
# records unless address auto-selection is disabled. pnpm ignores NODE_OPTIONS
# (compiled launcher), so npm is the only reliable package manager here.
export NODE_OPTIONS := --no-network-family-autoselection --dns-result-order=ipv4first

PY      := .venv/bin/python
UV      := uv
WEB     := apps/web
API     := apps/api
COMPOSE := docker compose

-include .env
export

.PHONY: help
help: ## Show this help
	@grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
	  | awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}'

.PHONY: bootstrap
bootstrap: ## Install all dependencies (python workspace + web)
	$(UV) sync --python 3.12 --all-packages
	cd $(WEB) && npm install --no-audit --no-fund
	@[ -f .env ] || { cp .env.example .env; echo "created .env from .env.example"; }
	@echo ""
	@echo "Bootstrapped. Next: 'make up' (Docker) or 'make up-native' (no Docker), then 'make dev'."

# ---------------------------------------------------------------- services --
.PHONY: up
up: ## Start Postgres/PostGIS + MQTT via Docker Compose
	@docker info >/dev/null 2>&1 || { \
	  echo "Docker daemon is not running. Start Docker Desktop, or use 'make up-native'."; exit 1; }
	$(COMPOSE) up -d
	@echo "waiting for database..."
	@for i in $$(seq 1 40); do \
	  [ "$$($(COMPOSE) ps --format '{{.Health}}' db 2>/dev/null)" = "healthy" ] && break; sleep 2; done
	$(COMPOSE) ps

.PHONY: up-native
up-native: ## Start PostGIS + MQTT natively via Homebrew (no Docker needed)
	@command -v mosquitto >/dev/null || { echo "run: brew install mosquitto"; exit 1; }
	@brew services start postgresql@17 >/dev/null 2>&1 || true
	@brew services start mosquitto     >/dev/null 2>&1 || true
	@echo "native services started. Ensure DATABASE_URL in .env points at port 5432."

.PHONY: down
down: ## Stop the Docker Compose stack
	$(COMPOSE) down

.PHONY: db-create
db-create: ## Create the database and enable PostGIS (idempotent)
	$(PY) -m parkscan_api.cli db-create

.PHONY: migrate
migrate: ## Apply Alembic migrations
	cd $(API) && ../../$(PY) -m alembic upgrade head

# --------------------------------------------------------------------- run --
.PHONY: dev
dev: ## Run the API and the dashboard (two processes; Ctrl-C stops both)
	@$(MAKE) --no-print-directory api & \
	 $(MAKE) --no-print-directory web & \
	 wait

.PHONY: api
api: ## Run the FastAPI backend only
	$(PY) -m uvicorn parkscan_api.main:app --host $${API_HOST:-127.0.0.1} --port $${API_PORT:-8000} --reload

.PHONY: web
web: ## Run the dashboard dev server only
	cd $(WEB) && npm run dev

# ------------------------------------------------------------------- check --
.PHONY: test
test: ## Run all tests (python + web)
	$(PY) -m pytest
	cd $(WEB) && npm run test -- --run

.PHONY: test-py
test-py: ## Run python tests only
	$(PY) -m pytest

.PHONY: lint
lint: ## Lint and type-check everything
	$(PY) -m ruff check .
	$(PY) -m ruff format --check .
	$(PY) -m mypy packages apps/api
	cd $(WEB) && npm run lint && npm run typecheck

.PHONY: fmt
fmt: ## Auto-format
	$(PY) -m ruff format .
	$(PY) -m ruff check --fix .

.PHONY: licence-guard
licence-guard: ## Fail if an AGPL/PML-licensed model dependency creeps in (see ADR-0006)
	@if $(UV) pip list 2>/dev/null | grep -Eiq '^(ultralytics|rfdetr[-_]plus)\b'; then \
	  echo "FAIL: AGPL-3.0 / PML-1.0 dependency present. See docs/adr/0006-detector-licensing.md"; \
	  exit 1; \
	else echo "licence guard: clean"; fi

.PHONY: smoke
smoke: ## End-to-end check: DB reachable, migrations applied, MQTT round-trips
	$(PY) -m parkscan_api.cli smoke

# -------------------------------------------------------- later phases ------
.PHONY: demo bench eval
demo:  ## (Phase 2) Run the single-robot demo
	@echo "not implemented until Phase 2 — see TASKS.md"; exit 1
bench: ## (Phase 4) 30 seeds x 3 strategies -> results.csv
	@echo "not implemented until Phase 4 — see TASKS.md"; exit 1
eval:  ## (Phase 3) Geolocation + detector-contract evaluation
	@echo "not implemented until Phase 3 — see TASKS.md"; exit 1

.PHONY: lot
lot: ## Regenerate sim/lot/lot.geojson from the metric lot spec
	$(PY) -m parkscan_core.lot build sim/lot/lot.yaml sim/lot/lot.geojson
