from pathlib import Path

from karate_robot_sim.model_checks import run_checks


def test_model_consistency():
    package_root = Path(__file__).resolve().parents[1]
    urdf_path = package_root / "urdf" / "sensei_full.urdf.xacro"
    controllers_path = package_root / "config" / "ros2_controllers.yaml"
    report = run_checks(urdf_path, controllers_path)
    assert report.errors == []
