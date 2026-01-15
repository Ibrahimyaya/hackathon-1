"""
Structured logging setup for RAG ingestion pipeline.

Supports both JSON (structured) and human-readable logging formats.
Can be toggled via LOG_FORMAT environment variable.
"""

import logging
import logging.config
import json
import sys
from typing import Any, Dict, Optional
from datetime import datetime


class JSONFormatter(logging.Formatter):
    """Custom formatter that outputs logs as structured JSON."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON."""
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }

        # Add exception info if present
        if record.exc_info:
            log_data["exception"] = self.formatException(record.exc_info)

        # Add extra context if provided
        if hasattr(record, "extra_context"):
            log_data.update(record.extra_context)

        return json.dumps(log_data)


class HumanFormatter(logging.Formatter):
    """Custom formatter that outputs logs in human-readable format."""

    COLORS = {
        "DEBUG": "\033[36m",      # Cyan
        "INFO": "\033[32m",       # Green
        "WARNING": "\033[33m",    # Yellow
        "ERROR": "\033[31m",      # Red
        "CRITICAL": "\033[35m",   # Magenta
    }
    RESET = "\033[0m"

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as human-readable text with colors."""
        level_color = self.COLORS.get(record.levelname, "")

        # Build base message
        timestamp = datetime.fromtimestamp(record.created).strftime("%Y-%m-%d %H:%M:%S")
        base = f"{level_color}[{timestamp}] {record.levelname:8s}{self.RESET} {record.name}"

        # Add function/line info for debug level
        if record.levelno <= logging.DEBUG:
            base += f" ({record.funcName}:{record.lineno})"

        base += f": {record.getMessage()}"

        # Add exception info if present
        if record.exc_info:
            base += "\n" + self.formatException(record.exc_info)

        return base


def setup_logging(log_level: str = "INFO", log_format: str = "json", debug: bool = False) -> None:
    """
    Configure logging for the application.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_format: Format type ("json" for structured, "human" for readable)
        debug: Enable debug mode with verbose output

    Raises:
        ValueError: If log_level or log_format is invalid
    """
    # Validate inputs
    valid_levels = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}
    if log_level.upper() not in valid_levels:
        raise ValueError(f"log_level must be one of {valid_levels}, got {log_level}")

    valid_formats = {"json", "human"}
    if log_format.lower() not in valid_formats:
        raise ValueError(f"log_format must be one of {valid_formats}, got {log_format}")

    # Set log level
    level = getattr(logging, log_level.upper())

    # Override to DEBUG if debug flag is set
    if debug:
        level = logging.DEBUG
        log_level = "DEBUG"

    # Get root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)

    # Remove existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)

    # Set formatter based on format choice
    if log_format.lower() == "json":
        formatter = JSONFormatter()
    else:
        formatter = HumanFormatter()

    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # Configure specific loggers
    # Suppress verbose third-party loggers in production
    if log_level != "DEBUG":
        logging.getLogger("urllib3").setLevel(logging.WARNING)
        logging.getLogger("requests").setLevel(logging.WARNING)
        logging.getLogger("cohere").setLevel(logging.INFO)
        logging.getLogger("qdrant_client").setLevel(logging.INFO)


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger instance with the given name.

    Args:
        name: Logger name (typically __name__)

    Returns:
        logging.Logger: Configured logger instance
    """
    return logging.getLogger(name)


def log_with_context(logger: logging.Logger, level: str, message: str, **context: Any) -> None:
    """
    Log a message with additional context information.

    Args:
        logger: Logger instance
        level: Log level ("debug", "info", "warning", "error", "critical")
        message: Log message
        **context: Additional context key-value pairs to include in log
    """
    # Create log record with extra context
    log_level = getattr(logging, level.upper())

    record = logger.makeRecord(
        name=logger.name,
        level=log_level,
        fn="",
        lno=0,
        msg=message,
        args=(),
        exc_info=None,
    )

    # Attach context if we're using JSON formatter
    if context:
        record.extra_context = context

    logger.handle(record)


if __name__ == "__main__":
    # Test logging setup
    setup_logging(log_level="DEBUG", log_format="human", debug=False)
    logger = get_logger(__name__)

    logger.debug("This is a debug message")
    logger.info("This is an info message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")

    # Test with context
    log_with_context(
        logger,
        "info",
        "Processing document",
        url="https://example.com",
        page=1,
        total_pages=10
    )
