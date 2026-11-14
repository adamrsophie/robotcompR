# Random and home configurations

`robot_random_configuration()` samples uniformly within the position
limits; joints unbounded on both sides are sampled in \\\[-\pi, \pi)\\
(revolute and continuous) or \\\[-1, 1\]\\ meters (prismatic), and a
joint bounded on one side only is sampled over a span of \\2\pi\\ rad or
2 m from its finite limit. The caller's random number state is left
unchanged when `seed` is supplied. `robot_home()` returns zeros
projected onto the limits.

## Usage

``` r
robot_random_configuration(robot, n = 1, seed = NULL)

robot_home(robot)
```

## Arguments

- robot:

  A `robot_model`.

- n:

  Number of configurations.

- seed:

  Optional integer seed.

## Value

A vector (`n = 1`) or a matrix with one row per configuration.

## Examples

``` r
robot <- robot_example("planar2")
robot_random_configuration(robot, 3, seed = 1)
#>          joint1     joint2
#> [1,] -1.4733525 -0.8034692
#> [2,]  0.4577512  2.5648452
#> [3,] -1.8743877  2.5031562
```
