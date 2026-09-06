# Verification Requirements

The requirements below are engineering targets for this educational simulator, not flight-qualification requirements.

| ID | Requirement | Verification |
|---|---|---|
| GNC-ATT-001 | Nominal 20 deg initial attitude error shall enter and remain within 1 deg for at least 2 s within 30 s. | Closed-loop regression test + nominal benchmark |
| GNC-ATT-002 | Nominal final attitude error at 40 s shall be below 0.2 deg. | Closed-loop regression test |
| GNC-NAV-001 | Nominal RMS attitude-estimation error shall remain below 0.2 deg. | Closed-loop regression test |
| GNC-ACT-001 | Applied reaction-wheel body torque shall not exceed 1.0 N m on any axis. | Unit test + Monte Carlo benchmark |
| GNC-ACT-002 | Reaction-wheel stored momentum shall not exceed 8.0 N m s on any axis. | Unit test + Monte Carlo benchmark |
| GNC-NUM-001 | Propagated quaternions shall remain unit-normalized to numerical tolerance. | Dynamics unit tests |
| GNC-DYN-001 | Constant-force translational propagation shall match the analytic constant-acceleration solution for a 1 s case. | Dynamics unit test |
| GNC-ROB-001 | Monte Carlo campaign shall report success rate and p95 metrics across initial-condition, inertia, sensor and disturbance uncertainty. | Reproducible benchmark script |

## Monte Carlo uncertainty model

The baseline robustness campaign samples:

- initial attitude error: uniform 10–35 deg, random axis;
- inertia: independent normally distributed scale factors, 5% standard deviation, clipped to 0.85–1.15;
- initial body rate: zero-mean Gaussian, 0.20 deg/s standard deviation per axis;
- gyro bias: zero-mean Gaussian, 0.025 deg/s standard deviation per axis;
- gyro white noise: 0.01 deg/s standard deviation;
- star-tracker attitude noise: 0.05 deg standard deviation;
- estimator initialization error: uniform 0–1 deg, random axis;
- constant disturbance torque: zero-mean Gaussian, 0.002 N m standard deviation per axis.

These distributions are illustrative engineering test conditions rather than hardware-qualified sensor specifications.
