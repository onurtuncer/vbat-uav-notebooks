"""Shared fixtures for the conceptual_design test suite."""

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def config_dir() -> Path:
    """Path to the repository's config/ directory (real design inputs)."""
    return REPO_ROOT / "config"
