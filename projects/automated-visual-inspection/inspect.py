from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import cv2
import numpy as np


@dataclass(frozen=True)
class Limits:
    min_brightness: float = 40.0
    max_brightness: float = 220.0
    min_edge_density: float = 0.01
    max_edge_density: float = 0.25
    min_components: int = 1


def evaluate(image: np.ndarray, limits: Limits = Limits()) -> tuple[bool, dict[str, float | int | str]]:
    if image is None or image.size == 0:
        return False, {"reason": "empty_image"}
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    brightness = float(gray.mean())
    edges = cv2.Canny(gray, 60, 140)
    edge_density = float(np.mean(edges > 0))
    _, mask = cv2.threshold(gray, int(brightness), 255, cv2.THRESH_BINARY)
    n_labels, _ = cv2.connectedComponents(mask)
    components = max(0, n_labels - 1)

    reasons = []
    if not limits.min_brightness <= brightness <= limits.max_brightness:
        reasons.append("brightness")
    if not limits.min_edge_density <= edge_density <= limits.max_edge_density:
        reasons.append("edge_density")
    if components < limits.min_components:
        reasons.append("feature_count")
    passed = not reasons
    return passed, {
        "reason": "pass" if passed else ";".join(reasons),
        "brightness": brightness,
        "edge_density": edge_density,
        "components": components,
    }


def log_result(path: Path, passed: bool, metrics: dict) -> None:
    new = not path.exists()
    with path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["timestamp_utc", "pass", "reason", "brightness", "edge_density", "components"])
        if new:
            w.writeheader()
        w.writerow({"timestamp_utc": datetime.now(timezone.utc).isoformat(), "pass": int(passed), **metrics})


def demo() -> None:
    img = np.full((240, 320), 110, dtype=np.uint8)
    cv2.rectangle(img, (80, 60), (240, 180), 210, 4)
    passed, metrics = evaluate(img)
    print("PASS" if passed else "FAIL", metrics)
    print("Synthetic image only; no production inspection performance is claimed.")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()
    if args.demo:
        demo()
