"""Helpers for optional runtime dependencies."""

from __future__ import annotations

import importlib


def import_dependency(module_name: str, install_hint: str | None = None):
    """Import an optional dependency with a readable error message."""
    try:
        return importlib.import_module(module_name)
    except ImportError as exc:
        hint = install_hint or module_name
        raise RuntimeError(
            f"Missing optional dependency '{module_name}'. Install '{hint}' before running this node."
        ) from exc
