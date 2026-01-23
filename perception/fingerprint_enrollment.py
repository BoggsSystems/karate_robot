#!/usr/bin/env python3
"""
Sensei fingerprint enrollment using MediaPipe Pose.
Bushido note: recognition is respectful; identity is acknowledged before mimicry.
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import mediapipe as mp


def _distance(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2) ** 0.5


def _extract_ratios(landmarks):
    shoulder = landmarks[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
    elbow = landmarks[mp.solutions.pose.PoseLandmark.LEFT_ELBOW]
    wrist = landmarks[mp.solutions.pose.PoseLandmark.LEFT_WRIST]
    hip = landmarks[mp.solutions.pose.PoseLandmark.LEFT_HIP]

    upper_arm = _distance(shoulder, elbow)
    forearm = _distance(elbow, wrist)
    torso = _distance(shoulder, hip)

    if upper_arm == 0 or torso == 0:
        return None

    return {
        "forearm_to_upper_arm": forearm / upper_arm,
        "arm_to_torso": (forearm + upper_arm) / torso,
    }


def enroll_sensei_profile(image_path=None, output_path="sensei_profile.json"):
    mp_pose = mp.solutions.pose
    profile = None

    with mp_pose.Pose(static_image_mode=bool(image_path), model_complexity=1) as pose:
        if image_path:
            image = cv2.imread(str(image_path))
            if image is None:
                raise FileNotFoundError(f"Could not read image: {image_path}")
            results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            if results.pose_landmarks:
                profile = _extract_ratios(results.pose_landmarks.landmark)
        else:
            cap = cv2.VideoCapture(0)
            try:
                for _ in range(120):
                    ok, frame = cap.read()
                    if not ok:
                        continue
                    results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                    if results.pose_landmarks:
                        profile = _extract_ratios(results.pose_landmarks.landmark)
                        if profile:
                            break
            finally:
                cap.release()

    if not profile:
        raise RuntimeError("No valid pose detected; cannot enroll Sensei profile.")

    profile["timestamp_unix"] = int(time.time())
    profile["source"] = "mediapipe_pose"

    output_file = Path(output_path)
    output_file.write_text(json.dumps(profile, indent=2))

    return profile


def main():
    parser = argparse.ArgumentParser(description="Enroll Sensei fingerprint ratios.")
    parser.add_argument("--image", type=str, default=None, help="Path to input image.")
    parser.add_argument(
        "--output", type=str, default="sensei_profile.json", help="Output JSON path."
    )
    args = parser.parse_args()

    image_path = Path(args.image) if args.image else None
    profile = enroll_sensei_profile(image_path=image_path, output_path=args.output)
    print(f"Enrolled Sensei profile: {args.output}")
    print(json.dumps(profile, indent=2))


if __name__ == "__main__":
    main()
