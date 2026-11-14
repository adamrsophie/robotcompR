# EKF-SLAM with range-bearing landmarks

Simultaneous localization and mapping with an extended Kalman filter
over the joint state \\(x, y, \theta, m\_{1x}, m\_{1y}, \dots)\\, with
known data association: each measurement carries a landmark id.
`robot_ekf_slam()` starts with the pose only. `ekf_slam_predict()` moves
the pose with the exact unicycle model and its analytic Jacobians,
updating the pose block and the pose-landmark cross covariances.
`ekf_slam_update()` initializes a landmark the first time its id is
seen, from the inverse measurement model with the covariance \\G_x
P\_{xx} G_x^\top + G_z R G_z^\top\\ and cross terms \\G_x P\_{x\cdot}\\,
and fuses later sightings with a Joseph-form EKF update (Thrun, Burgard
and Fox, 2005, chapter 10). `ekf_slam_landmarks()` returns the current
map.

## Usage

``` r
robot_ekf_slam(mean, cov, Q, R)

ekf_slam_predict(filter, control, dt)

ekf_slam_update(filter, measurements)

ekf_slam_landmarks(filter)
```

## Arguments

- mean:

  Initial pose estimate.

- cov:

  Initial 3 by 3 pose covariance.

- Q:

  Control noise covariance (2 by 2) on `c(v, omega)`.

- R:

  Measurement noise covariance (2 by 2, range then bearing).

- filter:

  A `robot_ekf_slam`.

- control:

  `c(v, omega)`.

- dt:

  Duration (s).

- measurements:

  A data frame with columns `id`, `range`, `bearing`, as returned by
  [`landmark_measurements()`](https://adamrsophie.github.io/robotcompR/reference/landmark_measurements.md).

## Value

A `robot_ekf_slam` with `mean`, `cov`, `ids` (landmark ids in state
order) and `nis`; `ekf_slam_landmarks()` returns a matrix of landmark
estimates with ids as row names.

## Examples

``` r
L <- rbind(a = c(2, 0), b = c(0, 2))
f <- robot_ekf_slam(c(0, 0, 0), diag(1e-6, 3), Q = diag(c(0.01, 0.01)),
                    R = diag(c(0.01, 0.001)))
f <- ekf_slam_update(f, landmark_measurements(c(0, 0, 0), L))
f <- ekf_slam_predict(f, c(1, 0), 0.5)
f <- ekf_slam_update(f, landmark_measurements(c(0.5, 0, 0), L))
ekf_slam_landmarks(f)
#>              x y
#> a 2.000000e+00 0
#> b 1.224647e-16 2
```
