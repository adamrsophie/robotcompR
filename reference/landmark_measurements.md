# Range-bearing landmark measurements

Returns the range and bearing (relative to the robot heading, in
\\\[-\pi, \pi)\\) from `pose` to each landmark within `max_range`,
optionally corrupted by Gaussian noise with covariance `R` (2 by 2,
range then bearing).

## Usage

``` r
landmark_measurements(pose, landmarks, R = NULL, max_range = Inf, seed = NULL)
```

## Arguments

- pose:

  Robot pose `c(x, y, theta)`.

- landmarks:

  Matrix of landmark positions (one per row).

- R:

  Optional measurement noise covariance.

- max_range:

  Sensor range.

- seed:

  Optional seed (the global random state is restored).

## Value

A data frame with columns `id`, `range`, `bearing`.

## Examples

``` r
landmark_measurements(c(0, 0, 0), rbind(c(1, 0), c(0, 2)))
#>   id range  bearing
#> 1  1     1 0.000000
#> 2  2     2 1.570796
```
