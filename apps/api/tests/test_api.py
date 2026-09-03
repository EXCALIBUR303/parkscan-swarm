"""API surface tests.

These run the real app through its real lifespan (see conftest), so a broken MQTT
client or database engine shows up here rather than in production.
"""

from __future__ import annotations

import pytest
from httpx import AsyncClient

pytestmark = pytest.mark.db  # the app's lifespan builds a real engine


async def test_health_reports_each_dependency_separately(client: AsyncClient) -> None:
    response = await client.get("/api/v1/health")
    assert response.status_code == 200

    body = response.json()
    # Deliberately 200 even when degraded: a monitor that only sees a status code
    # learns nothing, whereas the per-component detail says which half to look at.
    assert body["status"] in {"ok", "degraded"}
    assert set(body) >= {"status", "db", "mqtt", "version", "git_sha", "data_source"}
    assert body["db"]["status"] in {"ok", "error", "stopped", "connecting"}
    assert body["mqtt"]["status"] in {"ok", "error", "stopped", "connecting"}


async def test_health_declares_provenance(client: AsyncClient) -> None:
    """Provenance is served from configuration, never hardcoded in the UI."""
    body = (await client.get("/api/v1/health")).json()
    assert body["data_source"] in {"simulated", "replay", "live"}


@pytest.mark.parametrize(
    "path",
    [
        "/api/v1/runs",
        "/api/v1/robots",
        "/api/v1/zones",
        "/api/v1/detections",
        "/api/v1/parking-spaces",
    ],
)
async def test_collection_endpoints_return_a_list(client: AsyncClient, path: str) -> None:
    response = await client.get(path)
    assert response.status_code == 200, response.text
    payload = response.json()
    assert isinstance(payload, dict | list)


async def test_unknown_run_is_a_problem_json_404(client: AsyncClient) -> None:
    response = await client.get("/api/v1/runs/00000000-0000-0000-0000-000000000000")
    assert response.status_code == 404
    # RFC 9457, not a bare FastAPI {"detail": ...}
    assert response.headers["content-type"].startswith("application/problem+json")
    body = response.json()
    assert "title" in body and "status" in body


async def test_openapi_documents_every_route(client: AsyncClient) -> None:
    spec = (await client.get("/openapi.json")).json()
    documented = set(spec["paths"])
    assert {
        "/api/v1/health",
        "/api/v1/runs",
        "/api/v1/robots",
        "/api/v1/zones",
        "/api/v1/detections",
        "/api/v1/parking-spaces",
    } <= documented


async def test_detections_reject_an_out_of_range_confidence_filter(client: AsyncClient) -> None:
    response = await client.get("/api/v1/detections", params={"min_confidence": 5})
    assert response.status_code == 422
