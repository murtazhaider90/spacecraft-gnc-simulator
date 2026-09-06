from __future__ import annotations

from dataclasses import dataclass
import math

RHO = 1.225
G = 9.80665


@dataclass(frozen=True)
class Requirements:
    mass_kg: float
    payload_kg: float
    cruise_speed_mps: float
    endurance_min: float
    cl_max: float = 1.2
    stall_speed_mps: float = 9.0
    propulsive_efficiency: float = 0.65
    lift_to_drag: float = 10.0


def wing_area(req: Requirements) -> float:
    weight = req.mass_kg * G
    return 2.0 * weight / (RHO * req.stall_speed_mps**2 * req.cl_max)


def cruise_power_w(req: Requirements) -> float:
    drag_n = req.mass_kg * G / req.lift_to_drag
    shaft_power = drag_n * req.cruise_speed_mps / req.propulsive_efficiency
    return shaft_power


def battery_energy_wh(req: Requirements, reserve: float = 0.25) -> float:
    hours = req.endurance_min / 60.0
    return cruise_power_w(req) * hours * (1.0 + reserve)


def print_report(req: Requirements) -> None:
    area = wing_area(req)
    p = cruise_power_w(req)
    e = battery_energy_wh(req)
    print(f"wing area: {area:.3f} m^2")
    print(f"wing loading: {req.mass_kg*G/area:.1f} N/m^2")
    print(f"estimated cruise shaft power: {p:.1f} W")
    print(f"battery energy incl. 25% reserve: {e:.1f} Wh")
    print("Values are first-order sizing estimates, not validated flight performance.")


if __name__ == "__main__":
    print_report(Requirements(mass_kg=3.5, payload_kg=1.0, cruise_speed_mps=16.0, endurance_min=25.0))
