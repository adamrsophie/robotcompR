# Inverse dynamics with the recursive Newton-Euler algorithm

Computes the joint forces and torques \\\tau = M(q)\ddot q + C(q, \dot
q)\dot q + g(q) - J^\top f\_{ext}\\ for a fixed-base rigid-body tree in
\\O(n)\\ operations. Gravity is a world-frame acceleration vector
(default \\(0, 0, -9.81)\\ m/s^2). External wrenches are given per link
as `c(force, moment)` in world axes with the moment taken about the link
frame origin. Joint friction, damping, motor inertia and contact are not
modeled.

## Usage

``` r
robot_inverse_dynamics(
  robot,
  q,
  qd = NULL,
  qdd = NULL,
  gravity = c(0, 0, -9.81),
  f_ext = NULL
)
```

## Arguments

- robot:

  A `robot_model` with inertial parameters.

- q, qd, qdd:

  Joint positions, velocities and accelerations.

- gravity:

  Gravity vector in world coordinates.

- f_ext:

  Optional named list of external wrenches.

## Value

Named numeric vector of joint efforts (N m or N).

## Examples

``` r
robot <- robot_example("planar1")
# Hold a horizontal 1 kg, 1 m rod against gravity along -y.
robot_inverse_dynamics(robot, 0, gravity = c(0, -9.81, 0))
#> joint1 
#>  4.905 
```
