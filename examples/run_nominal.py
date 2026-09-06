from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import build_nominal
from gnc.simulation import run_closed_loop


def main() -> None:
    result = run_closed_loop(*build_nominal())
    settling = result.settling_time()
    metrics = {
        "final_attitude_error_deg": result.final_error_deg,
        "settling_time_s": settling,
        "peak_applied_torque_nm": result.peak_torque_nm,
        "peak_wheel_momentum_nms": result.peak_wheel_momentum_nms,
        "rms_estimation_error_deg": result.rms_estimation_error_deg,
    }
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
