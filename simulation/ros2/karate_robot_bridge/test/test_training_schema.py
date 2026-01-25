from pathlib import Path

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")

from karate_robot_bridge.training_env_node import TrainingEnvNode  # noqa: E402


def test_training_schema_matches_node():
    schema_path = Path(__file__).resolve().parents[1] / "config" / "training_schema.yaml"
    yaml_schema = yaml.safe_load(schema_path.read_text())

    rclpy.init()
    node = TrainingEnvNode()
    try:
        node_schema = node.build_training_schema()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    assert yaml_schema["schema_version"] == node_schema["schema_version"]
    assert yaml_schema["observations"]["layout"] == node_schema["observations"]["layout"]
    assert yaml_schema["actions"]["layout"] == node_schema["actions"]["layout"]
    assert yaml_schema["reward"] == node_schema["reward"]
    assert yaml_schema["done"]["conditions"] == node_schema["done"]["conditions"]
    assert yaml_schema["joint_names"] == node_schema["joint_names"]
