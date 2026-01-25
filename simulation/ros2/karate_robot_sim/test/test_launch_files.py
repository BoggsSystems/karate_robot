from pathlib import Path
import py_compile


def test_launch_files_compile():
    launch_dir = Path(__file__).resolve().parents[1] / "launch"
    for launch_file in launch_dir.glob("*.launch.py"):
        py_compile.compile(str(launch_file), doraise=True)
