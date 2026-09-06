# Fixed-Wing UAV Design & Systems Toolkit

A lightweight sizing and systems-design toolkit reflecting the clean-sheet fixed-wing UAV workflow used in the project: requirements, mass budget, wing loading, power loading, endurance checks, propulsion sizing and integration trade-offs.

The scripts are intentionally transparent rather than high-fidelity. They are meant to make assumptions auditable and to support early design iteration before CAD, manufacture and flight testing.

```bash
pip install numpy
python sizing.py
```

The demonstration inputs are generic. Published project metrics such as flight time, range and payload should be backed by test records separately; the code does not manufacture those results.
