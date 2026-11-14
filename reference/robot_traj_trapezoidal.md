# Trapezoidal velocity profile trajectories

Moves from `q0` to `q1` along the straight line in joint space with a
common time scaling \\q(t) = q_0 + s(t)(q_1 - q_0)\\, where \\s(t)\\ has
a trapezoidal velocity profile (or a triangular one when the peak
velocity is not reached). The profile is the fastest one with this shape
that respects every joint's velocity and acceleration limits, so all
joints start and stop together. The motion is rest to rest.

## Usage

``` r
robot_traj_trapezoidal(q0, q1, vmax, amax, times = NULL, n = 101)
```

## Arguments

- q0, q1:

  Start and end configurations.

- vmax, amax:

  Positive velocity and acceleration limits (scalars or one per joint).

- times:

  Optional sample times.

- n:

  Number of samples.

## Value

A `robot_trajectory` with field `duration`.

## Examples

``` r
tr <- robot_traj_trapezoidal(c(0, 0), c(1, 2), vmax = 1, amax = 2)
tr$duration
#> [1] 2.5
```
