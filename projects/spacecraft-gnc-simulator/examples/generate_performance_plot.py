from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import build_nominal
from gnc.simulation import run_closed_loop


def main() -> None:
    result = run_closed_loop(*build_nominal())
    out = ROOT / "artifacts" / "nominal_response.svg"
    out.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(8, 4.8))
    ax = fig.add_subplot(111)
    ax.plot(result.time, result.attitude_error_deg, label="truth attitude error")
    ax.plot(result.time, result.estimation_error_deg, label="estimation error")
    ax.axhline(1.0, linestyle="--", linewidth=1.0, label="1 deg requirement")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle error [deg]")
    ax.set_title("Nominal closed-loop attitude response")
    ax.set_yscale("log")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, format="svg")
    print(out)


if __name__ == "__main__":
    main()
