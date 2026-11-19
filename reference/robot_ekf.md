# Extended Kalman filter localization

`robot_ekf()` creates a filter for the planar pose with mean `mean` and
covariance `cov`. `ekf_predict()` propagates through the exact unicycle
motion model with control noise covariance `Q` on `c(v, omega)`, using
the analytic Jacobians of
[`unicycle_step()`](https://adamrsophie.github.io/robotcompR/reference/unicycle_step.md).
`ekf_update()` fuses range-bearing measurements of known landmarks with
covariance `R`, wrapping the bearing innovation and using the Joseph
form so the covariance stays symmetric positive semidefinite. The
heading estimate is wrapped after each step. Filters are immutable; each
step returns a new filter.

## Usage

``` r
robot_ekf(mean, cov, landmarks, Q, R)

ekf_predict(filter, control, dt)

ekf_update(filter, measurements)
```

## Arguments

- mean:

  Initial pose estimate.

- cov:

  Initial 3 by 3 covariance.

- landmarks:

  Matrix of landmark positions with row names as ids.

- Q:

  Control noise covariance (2 by 2).

- R:

  Measurement noise covariance (2 by 2, positive definite).

- filter:

  A `robot_ekf`.

- control:

  `c(v, omega)`.

- dt:

  Duration (s).

- measurements:

  A data frame as returned by
  [`landmark_measurements()`](https://adamrsophie.github.io/robotcompR/reference/landmark_measurements.md).

## Value

A `robot_ekf`. `ekf_update()` also records the normalized innovation
squared in `nis`.

## Examples

``` r
L <- rbind(a = c(2, 0), b = c(0, 2))
f <- robot_ekf(c(0, 0, 0), diag(0.1, 3), L, Q = diag(c(0.01, 0.01)),
               R = diag(c(0.01, 0.001)))
f <- ekf_predict(f, c(1, 0), 0.1)
f <- ekf_update(f, landmark_measurements(c(0.1, 0, 0), L))
f$mean
#>     x     y theta 
#>   0.1   0.0   0.0 
```
