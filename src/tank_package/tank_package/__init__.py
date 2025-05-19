# tank_package/__init__.py

import os, pathlib, yaml
from typing import Final

DEFAULT_SERVER_IP:   Final[str] = os.getenv("TANK_SERVER_IP", "192.168.0.2")
DEFAULT_SERVER_PORT: Final[int] = int(os.getenv("TANK_SERVER_PORT", "5050"))

def url(path: str) -> str:
    return f"http://{DEFAULT_SERVER_IP}:{DEFAULT_SERVER_PORT}{path}"

PARAM_DIR = pathlib.Path(__file__).parent / "params"
def param_file(name: str) -> str:
    return str(PARAM_DIR / name)

