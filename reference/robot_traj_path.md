# Time-parameterize a geometric path

Converts a path (one configuration per row, for example from
[`robot_plan()`](https://adamrsophie.github.io/robotcompR/reference/robot_plan.md))
into a rest-to-rest trajectory through every waypoint: each segment uses
[`robot_traj_trapezoidal()`](https://adamrsophie.github.io/robotcompR/reference/robot_traj_trapezoidal.md)
with the given limits and the robot stops at each waypoint. The result
is feasible for the stated velocity and acceleration limits, but not
time optimal; a collision-free path is not automatically dynamically
feasible for torque limits.

## Usage

``` r
robot_traj_path(path, vmax, amax, dt = 0.01)
```

## Arguments

- path:

  Matrix of waypoints or a `robot_plan`.

- vmax, amax:

  Velocity and acceleration limits.

- dt:

  Sample period in seconds.

## Value

A `robot_trajectory` with field `duration`.

## Examples

``` r
path <- rbind(c(0, 0), c(1, 0), c(1, 1))
tr <- robot_traj_path(path, vmax = 1, amax = 2, dt = 0.05)
tr$duration
#> [1] 3
```
