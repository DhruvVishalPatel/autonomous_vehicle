"""Load config.toml: cfg = load(); cfg["motors"]["ena"]"""
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def load(path=ROOT / "config.toml"):
    with open(path, "rb") as f:
        return tomllib.load(f)
