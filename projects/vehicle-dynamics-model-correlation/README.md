# Vehicle Dynamics System Identification & Model Correlation

A reproducible toolkit for estimating simple lateral-dynamics parameters from steering/yaw/lateral-acceleration telemetry and comparing measured response with a linear bicycle model.

The repository deliberately separates **measured data** from **demonstration data**. No vehicle-specific results are hard-coded. The included example generator creates synthetic traces so the processing and estimation pipeline can be tested without presenting simulated values as experimental measurements.

## Workflow

1. Load synchronized time, steering angle, yaw rate, lateral acceleration and speed.
2. Validate sampling, missing values and units.
3. Estimate first-order yaw-rate response parameters from a step-steer segment.
4. Simulate a linear bicycle-model yaw response with configurable mass, wheelbase and cornering stiffnesses.
5. Report RMSE, normalized RMSE and time-domain overlays.

```bash
pip install numpy pandas scipy
python system_id.py --demo
```

The modelling assumptions are intentionally modest: small slip angles, approximately constant longitudinal speed, linear tyre region and planar motion. The code is a portfolio implementation of the analysis workflow and is not a replacement for full vehicle-dynamics identification software.
