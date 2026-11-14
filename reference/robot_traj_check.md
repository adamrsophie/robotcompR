# Check a trajectory against robot limits

Reports samples that violate joint position limits and, where the joints
declare them, velocity and acceleration limits.

## Usage

``` r
robot_traj_check(robot, trajectory, tol = 1e-09)
```

## Arguments

- robot:

  A `robot_model`.

- trajectory:

  A `robot_trajectory`.

- tol:

  Tolerance.

## Value

A list with logical `ok` and data frames of violations.

## Examples

``` r
robot <- robot_example("planar2")
tr <- robot_traj_polynomial(c(0, 0), c(1, 1), duration = 1)
robot_traj_check(robot, tr)$ok
#> [1] TRUE
```
