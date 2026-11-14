# Polynomial joint trajectories

Cubic polynomials match endpoint positions and velocities; quintic
polynomials also match endpoint accelerations. Each joint gets its own
polynomial on \\\[0, T\]\\; outside this interval the trajectory is held
at rest at its endpoint positions. The boundary conditions are satisfied
exactly.

## Usage

``` r
robot_traj_polynomial(
  q0,
  q1,
  duration,
  times = NULL,
  n = 101,
  qd0 = NULL,
  qd1 = NULL,
  qdd0 = NULL,
  qdd1 = NULL,
  order = c("quintic", "cubic")
)
```

## Arguments

- q0, q1:

  Start and end configurations.

- duration:

  Duration \\T\\ in seconds.

- times:

  Optional sample times; otherwise `n` equally spaced samples.

- n:

  Number of samples.

- qd0, qd1, qdd0, qdd1:

  Endpoint velocities and accelerations (zero by default). Accelerations
  are ignored by cubic trajectories.

- order:

  `"quintic"` or `"cubic"`.

## Value

A `robot_trajectory` whose `evaluate(t)` function returns positions,
velocities and accelerations at arbitrary times.

## Examples

``` r
tr <- robot_traj_polynomial(c(0, 0), c(1, -1), duration = 2, n = 5)
tr$q
#>           [,1]       [,2]
#> [1,] 0.0000000  0.0000000
#> [2,] 0.1035156 -0.1035156
#> [3,] 0.5000000 -0.5000000
#> [4,] 0.8964844 -0.8964844
#> [5,] 1.0000000 -1.0000000
```
