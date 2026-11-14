# Unscented Kalman filter localization

A drop-in alternative to
[`robot_ekf()`](https://adamrsophie.github.io/robotcompR/reference/robot_ekf.md)
that propagates the planar pose with the unscented transform instead of
Jacobians. `ukf_predict()` augments the state with the control noise,
draws \\2(3 + 2) + 1\\ sigma points, moves each through
[`unicycle_step()`](https://adamrsophie.github.io/robotcompR/reference/unicycle_step.md)
with its noisy control, and recombines them with the heading averaged on
the circle. `ukf_update()` fuses range-bearing measurements of known
landmarks one at a time with sigma points of the current estimate, a
circular mean of the predicted bearings, and wrapped bearing
innovations. The default scaling `alpha = 1, beta = 2, kappa = 0` makes
every weight nonnegative, so the predicted covariances stay positive
semidefinite.

## Usage

``` r
robot_ukf(mean, cov, landmarks, Q, R, alpha = 1, beta = 2, kappa = 0)

ukf_predict(filter, control, dt)

ukf_update(filter, measurements)
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

  Measurement noise covariance (2 by 2).

- alpha, beta, kappa:

  Unscented transform parameters.

- filter:

  A `robot_ukf`.

- control:

  `c(v, omega)`.

- dt:

  Duration (s).

- measurements:

  A data frame as returned by
  [`landmark_measurements()`](https://adamrsophie.github.io/robotcompR/reference/landmark_measurements.md).

## Value

A `robot_ukf`. `ukf_update()` also records the normalized innovation
squared in `nis`.

## Examples

``` r
L <- rbind(a = c(2, 0), b = c(0, 2))
f <- robot_ukf(c(0, 0, 0), diag(0.1, 3), L, Q = diag(c(0.01, 0.01)),
               R = diag(c(0.01, 0.001)))
f <- ukf_predict(f, c(1, 0), 0.1)
f <- ukf_update(f, landmark_measurements(c(0.1, 0, 0), L))
f$mean
#>            x            y        theta 
#>  0.112416303 -0.006250566  0.005088623 
```
