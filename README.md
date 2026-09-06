# Engineering Portfolio Workspace

Private staging workspace for engineering and software projects referenced across my CV variants.

The portfolio is organized around **reproducible implementations**, **verification assets**, and **sanitized engineering case studies**. Personal projects contain runnable source. Employer/research work is documented only at a non-confidential level; proprietary source, unpublished data, customer material and employer IP are intentionally excluded.

## Implemented portfolio projects

| Project | Implementation | Verification / evidence |
|---|---|---|
| Spacecraft 6-DOF GNC simulator | `projects/spacecraft-gnc-simulator` | RK4 dynamics, quaternion PD control, gyro model, Monte Carlo runner, regression tests |
| Vehicle-dynamics system identification | `projects/vehicle-dynamics-model-correlation` | Synthetic step-steer generator + parameter-fit workflow; real measurements intentionally not embedded |
| WTI sentiment / return research | `projects/wti-sentiment-return-pipeline` | Leakage-aware expanding-window demo on synthetic text/returns |
| Fixed-wing UAV design | `projects/fixed-wing-uav-design` | First-order wing/power/energy sizing with assumptions stated |
| FPGA market-data processing | `projects/fpga-market-data-simulator` | Synthesizable top-of-book Verilog + Python reference model |
| Minimal C++ kernel | `projects/custom-cpp-operating-system` | Fixed-block allocator + cooperative scheduler core; bare-metal integration listed as extension |
| Parametric Formula-style wing study | `projects/parametric-formula-wing` | Design-sweep/result-ranking tool; demo coefficients explicitly synthetic |
| Airflow test & instrumentation rig | `projects/airflow-test-instrumentation-rig` | Pressure-drop/repeatability/CFD-correlation analysis interface |
| Automated visual inspection | `projects/automated-visual-inspection` | OpenCV feature/quality checks and structured result logic |

## Existing public repositories

- [`pinn-airfoil-flow`](https://github.com/murtazhaider90/pinn-airfoil-flow) — physics-informed neural network for incompressible Navier–Stokes flow around a NACA 0012 airfoil.
- [`driving-scene-detection-quality`](https://github.com/murtazhaider90/driving-scene-detection-quality) — autonomous-driving detection data-quality and conditional failure-analysis platform.

Both public repositories have non-destructive `portfolio-cleanup` pull requests open for repository hygiene/reproducibility improvements.

## Sanitized case studies

- `case-studies/thermal-airflow-engineering.md`
- `case-studies/scientific-software-research.md`
- `case-studies/formula-student-aerodynamics.md`
- `case-studies/uas-challenge-systems-integration.md`

These describe engineering process and transferable methods without publishing confidential or third-party material.

## Engineering standard

Each project should be judged against the following standard:

1. State the engineering objective and modelling assumptions.
2. Provide a reproducible setup or execution path.
3. Include verification/tests appropriate to the fidelity of the model.
4. Separate measured, simulated and demonstration data unambiguously.
5. Do not hard-code invented benchmark results or experimental claims.
6. Document limitations and credible next steps.
7. Keep generated artefacts, environments, secrets and large raw data out of source control.

## Publication plan

This repository is a **private staging monorepo**. Strong projects should be promoted to standalone public repositories only after their code, tests, README and result artefacts are complete. Commit dates and authorship history should remain genuine; the portfolio should look professional because it is well engineered, not because its history is disguised.
