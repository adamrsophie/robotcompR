# Simulate torque-driven rigid-body dynamics

Integrates \\\ddot q = ABA(q, \dot q, \tau(t, q, \dot q))\\ with
[`deSolve::ode()`](https://rdrr.io/pkg/deSolve/man/ode.html). This is a
dynamic simulation of the rigid-body model only: joint limits, friction,
contact and impacts are not modeled, and the result is not a
high-fidelity physics simulation. Energy drift in an unforced
conservative simulation measures integrator error, not model error;
tighten `rtol` and `atol` to reduce it.

## Usage

``` r
robot_simulate(
  robot,
  q0,
  qd0 = NULL,
  times,
  torque = NULL,
  gravity = c(0, 0, -9.81),
  f_ext = NULL,
  method = "lsoda",
  rtol = 1e-08,
  atol = 1e-10,
  ...
)
```

## Arguments

- robot:

  A `robot_model` with inertial parameters.

- q0, qd0:

  Initial positions and velocities.

- times:

  Output times (seconds), increasing.

- torque:

  A function `function(t, q, qd)` returning joint efforts, a constant
  numeric vector of efforts (one per joint), or `NULL` for zero torque.

- gravity:

  Gravity vector in world coordinates.

- f_ext:

  Optional named list of external wrenches.

- method:

  Integration method passed to
  [`deSolve::ode()`](https://rdrr.io/pkg/deSolve/man/ode.html).

- rtol, atol:

  Integration tolerances.

- ...:

  Further arguments to
  [`deSolve::ode()`](https://rdrr.io/pkg/deSolve/man/ode.html), such as
  `maxsteps`. An integration that stops before the last time is an
  error.

## Value

A `robot_trajectory` with `time`, `q`, `qd` and `qdd`.

## Examples

``` r
robot <- robot_example("planar1")
sim <- robot_simulate(robot, q0 = 0.5, qd0 = 0, times = seq(0, 1, 0.1),
                      gravity = c(0, -9.81, 0))
tail(sim$q, 1)
#>          joint1
#> [11,] -3.555636
```
