#!/usr/bin/env python3
"""
Static model sanity checks for Sensei URDF/Xacro and controllers.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass
class CheckReport:
    errors: list[str]
    warnings: list[str]

    def is_ok(self) -> bool:
        return not self.errors


def load_robot_description(urdf_path: Path) -> str:
    return xacro.process_file(str(urdf_path)).toxml()


def parse_urdf(xml_text: str) -> tuple[set[str], list[dict[str, str]]]:
    root = ET.fromstring(xml_text)
    links = {link.get("name") for link in root.findall("link") if link.get("name")}
    joints: list[dict[str, str]] = []
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        joints.append(
            {
                "name": joint.get("name", ""),
                "type": joint.get("type", ""),
                "parent": parent.get("link", "") if parent is not None else "",
                "child": child.get("link", "") if child is not None else "",
                "has_axis": "axis" if joint.find("axis") is not None else "",
                "has_limit": "limit" if joint.find("limit") is not None else "",
            }
        )
    return links, joints


def extract_controller_joints(controllers_path: Path) -> list[str]:
    config = yaml.safe_load(controllers_path.read_text())
    params = (config or {}).get("controller_manager", {}).get("ros__parameters", {})
    joints: list[str] = []
    for controller_cfg in params.values():
        if isinstance(controller_cfg, dict) and "joints" in controller_cfg:
            joints.extend(controller_cfg.get("joints", []))
    return joints


def run_checks(urdf_path: Path, controllers_path: Path) -> CheckReport:
    errors: list[str] = []
    warnings: list[str] = []

    xml_text = load_robot_description(urdf_path)
    links, joints = parse_urdf(xml_text)

    if "base_link" not in links:
        errors.append("Missing required link: base_link.")

    joint_names = [joint["name"] for joint in joints if joint["name"]]
    duplicates = {name for name in joint_names if joint_names.count(name) > 1}
    if duplicates:
        errors.append(f"Duplicate joint names found: {sorted(duplicates)}.")

    child_links = {joint["child"] for joint in joints if joint["child"]}
    root_links = sorted(link for link in links if link not in child_links)
    if len(root_links) != 1:
        errors.append(f"Expected 1 root link, found {len(root_links)}: {root_links}.")
    elif "base_link" not in root_links:
        errors.append(f"Root link is not base_link (root: {root_links[0]}).")

    for joint in joints:
        if joint["parent"] and joint["parent"] not in links:
            errors.append(
                f"Joint {joint['name']} parent link missing: {joint['parent']}."
            )
        if joint["child"] and joint["child"] not in links:
            errors.append(
                f"Joint {joint['name']} child link missing: {joint['child']}."
            )
        if joint["type"] in {"revolute", "continuous"} and not joint["has_axis"]:
            warnings.append(f"Joint {joint['name']} missing axis.")
        if joint["type"] == "revolute" and not joint["has_limit"]:
            warnings.append(f"Joint {joint['name']} missing limits.")

    controller_joints = extract_controller_joints(controllers_path)
    missing = sorted(j for j in controller_joints if j not in joint_names)
    if missing:
        errors.append(f"Controller joints missing from URDF: {missing}.")

    return CheckReport(errors=errors, warnings=warnings)


def main() -> None:
    package_share = Path(get_package_share_directory("karate_robot_sim"))
    urdf_path = package_share / "urdf" / "sensei_full.urdf.xacro"
    controllers_path = package_share / "config" / "ros2_controllers.yaml"

    report = run_checks(urdf_path, controllers_path)
    for warning in report.warnings:
        print(f"WARNING: {warning}")
    for error in report.errors:
        print(f"ERROR: {error}")

    if report.is_ok():
        print("Model checks passed.")
        return
    sys.exit(1)


if __name__ == "__main__":
    main()
