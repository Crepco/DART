"""Launch the DART web UI from anywhere.

Puts scripts/ on sys.path so the `web` package and its siblings (config, core, main,
flowstate) all import cleanly, then starts the Flask app. Run:

    python scripts/run_web.py            # default http://127.0.0.1:5000
    python scripts/run_web.py 50001      # custom port
    python scripts/run_web.py 0.0.0.0 50001   # custom host + port
"""

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent   # scripts/
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from web.app import main   # noqa: E402  (path must be set first)


def _parse_args(argv):
    host, port = "127.0.0.1", 5000
    if len(argv) == 1:
        port = int(argv[0])
    elif len(argv) >= 2:
        host, port = argv[0], int(argv[1])
    return host, port


if __name__ == "__main__":
    host, port = _parse_args(sys.argv[1:])
    main(host=host, port=port)
