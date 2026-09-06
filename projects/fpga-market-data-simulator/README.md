# FPGA-Accelerated Market/Data Processing Simulator

A hardware/software co-design project for replaying simplified market-data events and computing best bid/ask, spread and mid-price with a small synthesizable Verilog core plus Python reference validation.

The goal is to demonstrate deterministic state updates, hardware/software agreement and latency/throughput measurement methodology rather than claim production exchange connectivity.

## Components

- `order_book_top.v` — simplified best-bid/best-ask state machine
- `reference.py` — Python reference model and randomized equivalence test

The hardware model intentionally tracks only top-of-book values. It is not a full matching engine or exchange simulator.
