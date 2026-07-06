"""Pytest setup: put scripts/ on sys.path so config/core/auth/web import the same
way they do at runtime (run_web.py does the identical insert).

Run:  python -m pytest tests/          (pytest is a dev-only dependency:
                                        python -m pip install pytest)
"""

import pathlib
import sys

SCRIPTS = pathlib.Path(__file__).resolve().parent.parent / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
