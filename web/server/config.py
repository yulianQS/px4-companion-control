"""Canonical project paths and runtime settings."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = json.loads((ROOT / 'config/runtime.json').read_text(encoding='utf-8'))
