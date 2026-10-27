"""The UI and C++ scheduler share config/mission_blocks.json."""
import json
from .config import ROOT

_catalog = json.loads((ROOT / 'config/mission_blocks.json').read_text(encoding='utf-8'))
CATALOG, CATEGORIES = _catalog['blocks'], _catalog['categories']
BY_TYPE = {block['type']: block for block in CATALOG}
