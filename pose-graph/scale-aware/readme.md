# Info

Requirements are in [homework3](./assets/HW3.pdf) and my report is located at [report](./assets/hw3-report.pdf).

```bash
.
├── CMakeLists.txt
├── cmake_modules
├── data
├── data.md
├── Lie.h
├── poses
├── rank.py
├── scale_drift.cc
├── scale_jump.cc
└── vis.py
```

1. `poses` is a directory contains all modified trajectories in TUM format for visualization.
2. `data` contains raw data.
3. `Lie.h` deals with lie algebra.
4. `rank.py` finds the rank of the matrix for scale jump.
5. `scale_drift.cc` and `scale_jump.cc` are codes for drift and jump cases respectively.
6. `vis.py` process raw nodes and edges for visualizations.