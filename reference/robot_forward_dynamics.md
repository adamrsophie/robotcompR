# Forward dynamics with the articulated body algorithm

Solves \\M(q)\ddot q = \tau - C(q, \dot q)\dot q - g(q) + J^\top
f\_{ext}\\ for the joint accelerations in \\O(n)\\ operations. Every
degree of freedom must move some mass or rotational inertia; otherwise
the articulated inertia is singular and the function stops.

## Usage

``` r
robot_forward_dynamics(
  robot,
  q,
  qd = NULL,
  tau = NULL,
  gravity = c(0, 0, -9.81),
  f_ext = NULL
)
```

## Arguments

- robot:

  A `robot_model` with inertial parameters.

- q, qd:

  Joint positions and velocities.

- tau:

  Joint efforts (defaults to zero).

- gravity:

  Gravity vector in world coordinates.

- f_ext:

  Optional named list of external wrenches.

## Value

Named numeric vector of joint accelerations.

## Examples

``` r
robot <- robot_example("planar2")
robot_forward_dynamics(robot, c(0, 0), gravity = c(0, -9.81, 0))
#>    joint1    joint2 
#> -12.61286  16.81714 
```
