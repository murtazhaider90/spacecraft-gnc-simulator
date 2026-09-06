# Airflow Test & Instrumentation Rig

Analysis tooling for a modular duct/airflow test rig with pressure, temperature and airflow measurements. The portfolio implementation focuses on calibration, pressure-drop calculation, repeatability statistics and CFD-to-test comparison while excluding employer-specific geometry and proprietary data.

Expected CSV columns:

`timestamp_s, inlet_pa, outlet_pa, temperature_c, flow_m3_s`

The demo mode generates synthetic sensor traces only to exercise the analysis pipeline.
