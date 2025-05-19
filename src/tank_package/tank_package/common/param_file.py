from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def param_file(name: str) -> str:
    pkg = get_package_share_directory('tank_package')
    return str(Path(pkg) / 'params' / name)