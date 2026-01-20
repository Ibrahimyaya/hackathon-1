"""Structured logging setup with JSON and human-readable formats."""

import logging
import sys
from typing import Optional

import structlog

from utils.config import Config


def setup_logging(config: Config) -> logging.Logger:
    """Set up structured logging with JSON or human-readable format.

    Args:
        config: Configuration object with log_level and log_format.

    Returns:
        logging.Logger: Configured logger instance.
    """
    log_level = getattr(logging, config.log_level.upper())

    if config.log_format == "json":
        _setup_json_logging(log_level)
    else:
        _setup_human_logging(log_level)

    return logging.getLogger(__name__)


def _setup_json_logging(level: int) -> None:
    """Set up structured JSON logging using structlog."""
    structlog.configure(
        processors=[
            structlog.stdlib.add_logger_name,
            structlog.stdlib.add_log_level,
            structlog.stdlib.PositionalArgumentsFormatter(),
            structlog.processors.TimeStamper(fmt="iso"),
            structlog.processors.StackInfoRenderer(),
            structlog.processors.format_exc_info,
            structlog.processors.UnicodeDecoder(),
            structlog.processors.JSONRenderer(),
        ],
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        cache_logger_on_first_use=True,
    )

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)
    root_logger = logging.getLogger()
    root_logger.addHandler(handler)
    root_logger.setLevel(level)


def _setup_human_logging(level: int) -> None:
    """Set up human-readable logging to console."""
    handler = logging.StreamHandler(sys.stdout)
    formatter = logging.Formatter(
        fmt="[%(asctime)s] %(levelname)s - %(name)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    handler.setFormatter(formatter)
    handler.setLevel(level)

    root_logger = logging.getLogger()
    root_logger.addHandler(handler)
    root_logger.setLevel(level)


def get_logger(name: str) -> logging.Logger:
    """Get a logger instance.

    Args:
        name: Logger name (typically __name__).

    Returns:
        logging.Logger: Logger instance.
    """
    return logging.getLogger(name)
