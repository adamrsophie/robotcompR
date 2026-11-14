# Center of mass and mechanical energy

`robot_com()` returns the total mass and the world position of the
center of mass. `robot_energy()` returns the kinetic energy \\\tfrac12
\dot q^\top M(q) \dot q\\, the gravitational potential energy \\-\sum_i
m_i g^\top c_i\\ (zero at the world origin), and their sum.

## Usage

``` r
robot_com(robot, q)

robot_energy(robot, q, qd, gravity = c(0, 0, -9.81))
```

## Arguments

- robot:

  A `robot_model` with inertial parameters.

- q, qd:

  Joint positions and velocities.

- gravity:

  Gravity vector in world coordinates.

## Value

A list (`robot_com`) or named numeric vector (`robot_energy`).

## Examples

``` r
robot <- robot_example("planar2")
robot_com(robot, c(0, 0))
#> $mass
#> [1] 2
#> 
#> $position
#> [1] 1 0 0
#> 
robot_energy(robot, c(0, 0), c(1, 0), gravity = c(0, -9.81, 0))
#>   kinetic potential     total 
#>  1.333333  0.000000  1.333333 
```
