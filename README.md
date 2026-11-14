# robotcompR

<!-- badges: start -->
[![R-CMD-check](https://github.com/adamrsophie/robotcompR/actions/workflows/R-CMD-check.yaml/badge.svg)](https://github.com/adamrsophie/robotcompR/actions/workflows/R-CMD-check.yaml)
<!-- badges: end -->

robotcompR is an R-native framework for robotics computation: spatial
mathematics on SO(2), SO(3), SE(2) and SE(3), rigid-body tree robot models,
Denavit-Hartenberg models, forward and inverse kinematics, and Jacobians.
It needs no MATLAB, Python, Julia or ROS 2.

Development is in progress; see `NEWS.md` for the current feature set.

## Installation

```r
# install.packages("remotes")
remotes::install_github("adamrsophie/robotcompR")
```

## License

GPL-3. robotcompR is research and teaching software. It is not certified
for controlling industrial, medical, automotive or other safety-critical
hardware.
