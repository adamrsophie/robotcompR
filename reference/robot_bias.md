# Gravity, Coriolis and centrifugal terms

`robot_gravity_torque()` returns \\g(q)\\, the efforts that hold the
robot still against gravity (static gravity compensation).
`robot_coriolis()` returns the velocity product terms \\C(q, \dot q)\dot
q\\ without gravity. `robot_bias_forces()` returns their sum, \\C(q,
\dot q)\dot q + g(q)\\.

## Usage

``` r
robot_gravity_torque(robot, q, gravity = c(0, 0, -9.81))

robot_coriolis(robot, q, qd)

robot_bias_forces(robot, q, qd, gravity = c(0, 0, -9.81))
```

## Arguments

- robot:

  A `robot_model` with inertial parameters.

- q, qd:

  Joint positions and velocities.

- gravity:

  Gravity vector in world coordinates.

## Value

Named numeric vector.

## Examples

``` r
robot <- robot_example("planar2")
robot_gravity_torque(robot, c(0, 0), gravity = c(0, -9.81, 0))
#> joint1 joint2 
#> 19.620  4.905 
robot_coriolis(robot, c(0, 0.5), c(1, 0))
#>       joint1       joint2 
#> 8.326673e-17 2.397128e-01 
```
