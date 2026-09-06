# Parametric Formula-Style Wing Study

A compact aerodynamic design-space exploration for a Formula-style wing. The portfolio version focuses on parameter management, sweep reproducibility, result comparison and mesh/convergence bookkeeping; it does not pretend to reproduce proprietary CFD results.

Parameters include angle of attack, ground clearance and flap deflection. The analysis layer accepts exported CFD/XFOIL results and ranks designs on a robustness-aware score rather than maximum downforce alone.

```bash
pip install numpy pandas
python analyze_sweep.py --demo
```
