# Info

Results can be found at [here](./assets/hw4_report.pdf)

```bash
.
├── assets
├── CMakeLists.txt
├── cmake_modules
│   └── FindG2O.cmake
├── dense.cc
├── poses
│   ├── dense.txt
│   ├── gt.txt
│   ├── sparse_patt_1.txt
│   └── sparse_patt_8.txt
├── readme.md
├── sparse.cc
└── time_align.py
```

1. `dense.cc` implements the dense VO.
2. `poses` directory contains gt poses as well as optimized poses.
3. `sparse.cc` implements the sparse VO
4. `time_apign.py` aligns the timestamps between `rgb/` and `depth/`.