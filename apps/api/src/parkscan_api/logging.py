"""structlog configuration.

Console renderer on a TTY (readable during ``make dev``), JSON otherwise (so the
output is greppable when redirected). Configured once, from the lifespan.
"""

from __future__ import annotations

import logging
import sys

import structlog
from structlog.typing import Processor


def configure_logging(level: str = "INFO") -> None:
    """Configure stdlib logging and structlog. Idempotent."""
    numeric = logging.getLevelNamesMapping().get(level.upper(), logging.INFO)
    logging.basicConfig(format="%(message)s", stream=sys.stdout, level=numeric, force=True)
    # uvicorn's own access log is noisy at 10 Hz telemetry; keep it at WARNING.
    logging.getLogger("uvicorn.access").setLevel(max(numeric, logging.WARNING))

    renderer: Processor = (
        structlog.dev.ConsoleRenderer()
        if sys.stdout.isatty()
        else structlog.processors.JSONRenderer()
    )
    structlog.configure(
        processors=[
            structlog.contextvars.merge_contextvars,
            structlog.stdlib.add_log_level,
            structlog.stdlib.add_logger_name,
            structlog.processors.TimeStamper(fmt="iso", utc=True),
            structlog.processors.StackInfoRenderer(),
            structlog.processors.format_exc_info,
            renderer,
        ],
        wrapper_class=structlog.make_filtering_bound_logger(numeric),
        logger_factory=structlog.stdlib.LoggerFactory(),
        cache_logger_on_first_use=True,
    )
