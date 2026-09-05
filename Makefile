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

.PHONY: up-ros2
up-ros2: ## Start Postgres/MQTT plus the ros2-gz backend (ADR-0008; additive, opt-in)
	@docker info >/dev/null 2>&1 || { \
	  echo "Docker daemon is not running. Start Docker Desktop."; exit 1; }
	$(COMPOSE) --profile ros2 up -d --build
	@echo "waiting for database..."
	@for i in $$(seq 1 40); do \
	  [ "$$($(COMPOSE) ps --format '{{.Health}}' db 2>/dev/null)" = "healthy" ] && break; sleep 2; done
	$(COMPOSE) --profile ros2 ps

.PHONY: down
down: ## Stop the Docker Compose stack
	$(COMPOSE) --profile ros2 down

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
.PHONY: demo demo-dock demo-watch bench eval
demo: ## Single robot sweeps a zone, as fast as the CPU allows
	$(PY) -m parkscan_runner.run --seed 42 --robots 1 --zones zone_01

demo-watch: ## Same run throttled to real time, so the dashboard is watchable
	@echo "Open http://127.0.0.1:5173 (make web) and http://127.0.0.1:8000/docs (make api) first."
	$(PY) -m parkscan_runner.run --seed 42 --robots 1 --zones zone_03 --realtime

demo-dock: ## Start near-flat: the robot must decide to return and dock unaided
	$(PY) -m parkscan_runner.run --seed 42 --robots 1 --zones zone_09 --start-soc 0.08

bench: ## 30 seeds x 3 strategies -> apps/runner/results/{results.csv,results.json}
	$(PY) -m parkscan_runner.bench --seeds 30 --out apps/runner/results
	@echo ""
	@echo "Every metric in the README must be traceable to a row of results.csv, which is"
	@echo "committed. Re-run this and diff it to check any claim."
eval:  ## Geolocation accuracy vs known truth, binned by range (writes apps/eval/results/)
	$(PY) -m parkscan_eval.run_eval --seed 42 --out apps/eval/results
	@echo ""
	@echo "Every number above is TIER 1 (pipeline). See apps/eval/EVAL.md for what that"
	@echo "does and does not mean — real-model accuracy is NOT measured here."

.PHONY: lot
lot: ## Regenerate sim/lot/lot.geojson from the metric lot spec
	$(PY) -m parkscan_core.lot build sim/lot/lot.yaml sim/lot/lot.geojson
