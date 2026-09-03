"""RFC 9457 ``application/problem+json`` error responses.

FastAPI's default error body is ``{"detail": ...}``. The API contract says
problem+json, so every error path — raised problems, ``HTTPException``, request
validation, and anything unhandled — is funnelled through one renderer.
"""

from __future__ import annotations

from typing import Any

import structlog
from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

from parkscan_api.schemas import ProblemDetail

PROBLEM_MEDIA_TYPE = "application/problem+json"

log: structlog.stdlib.BoundLogger = structlog.get_logger("parkscan.api")


class ProblemException(Exception):
    """An error the API deliberately reports as a problem document."""

    def __init__(
        self,
        status: int,
        title: str,
        detail: str | None = None,
        *,
        type_: str = "about:blank",
    ) -> None:
        super().__init__(detail or title)
        self.status = status
        self.title = title
        self.detail = detail
        self.type_ = type_


def problem_response(
    request: Request,
    status: int,
    title: str,
    detail: str | None = None,
    *,
    type_: str = "about:blank",
    errors: list[dict[str, Any]] | None = None,
) -> JSONResponse:
    """Render one problem document."""
    body = ProblemDetail(
        type=type_,
        title=title,
        status=status,
        detail=detail,
        instance=str(request.url.path),
        errors=errors,
    )
    return JSONResponse(
        status_code=status,
        content=body.model_dump(mode="json", exclude_none=True),
        media_type=PROBLEM_MEDIA_TYPE,
    )


async def _handle_problem(request: Request, exc: Exception) -> JSONResponse:
    assert isinstance(exc, ProblemException)
    return problem_response(request, exc.status, exc.title, exc.detail, type_=exc.type_)


async def _handle_http(request: Request, exc: Exception) -> JSONResponse:
    assert isinstance(exc, StarletteHTTPException)
    detail = exc.detail if isinstance(exc.detail, str) else None
    title = {400: "Bad Request", 404: "Not Found", 405: "Method Not Allowed"}.get(
        exc.status_code, "Error"
    )
    return problem_response(request, exc.status_code, title, detail)


async def _handle_validation(request: Request, exc: Exception) -> JSONResponse:
    assert isinstance(exc, RequestValidationError)
    errors: list[dict[str, Any]] = [
        {"loc": [str(part) for part in err.get("loc", ())], "msg": str(err.get("msg", ""))}
        for err in exc.errors()
    ]
    return problem_response(
        request,
        422,
        "Unprocessable Content",
        "The request did not validate against the API contract.",
        errors=errors,
    )


async def _handle_unexpected(request: Request, exc: Exception) -> JSONResponse:
    log.error("unhandled_error", path=request.url.path, error=type(exc).__name__, detail=str(exc))
    return problem_response(
        request,
        500,
        "Internal Server Error",
        "The server failed to handle the request. See the server log.",
    )


def register_error_handlers(app: FastAPI) -> None:
    """Install the problem+json handlers on ``app``."""
    app.add_exception_handler(ProblemException, _handle_problem)
    app.add_exception_handler(StarletteHTTPException, _handle_http)
    app.add_exception_handler(RequestValidationError, _handle_validation)
    app.add_exception_handler(Exception, _handle_unexpected)
