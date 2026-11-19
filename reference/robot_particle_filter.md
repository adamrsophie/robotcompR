# Particle filter localization

A sequential importance resampling filter for the planar pose.
`robot_particle_filter()` draws `n` particles from a Gaussian around
`mean`. `pf_predict()` moves each particle with the exact unicycle model
under control noise `Q`. `pf_update()` weights particles by the Gaussian
likelihood of range-bearing measurements (with wrapped bearing
residuals) and resamples with low-variance resampling when the effective
sample size falls below `resample_threshold * n`. The estimate uses the
circular mean for the heading. Seeds make each step reproducible without
changing the global random state.

## Usage

``` r
robot_particle_filter(n, mean, cov, landmarks, Q, R, seed = NULL)

pf_predict(filter, control, dt, seed = NULL)

pf_update(filter, measurements, resample_threshold = 0.5, seed = NULL)

pf_estimate(filter)
```

## Arguments

- n:

  Number of particles.

- mean, cov:

  Initial Gaussian.

- landmarks:

  Landmark matrix with row names as ids.

- Q, R:

  Control and measurement noise covariances (`R` positive definite).

- seed:

  Optional seed.

- filter:

  A `robot_particle_filter`.

- control:

  `c(v, omega)`.

- dt:

  Duration.

- measurements:

  Data frame from
  [`landmark_measurements()`](https://adamrsophie.github.io/robotcompR/reference/landmark_measurements.md).

- resample_threshold:

  Fraction of `n` below which to resample.

## Value

A `robot_particle_filter`; `pf_estimate()` returns the weighted mean
pose and covariance.

## Examples

``` r
L <- rbind(a = c(2, 0), b = c(0, 2))
pf <- robot_particle_filter(200, c(0, 0, 0), diag(0.05, 3), L,
                            Q = diag(0.01, 2), R = diag(c(0.01, 0.001)),
                            seed = 1)
pf <- pf_update(pf, landmark_measurements(c(0, 0, 0), L), seed = 2)
pf_estimate(pf)$mean
#>           x           y       theta 
#> -0.02886694  0.01564359 -0.01223024 
```
