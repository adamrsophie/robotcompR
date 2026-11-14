# Joint-space mass matrix with the composite rigid body algorithm

Returns the symmetric positive semidefinite generalized inertia matrix
\\M(q)\\ (positive definite when every degree of freedom moves some
mass).

## Usage

``` r
robot_mass_matrix(robot, q)
```

## Arguments

- robot:

  A `robot_model` with inertial parameters.

- q:

  Joint positions.

## Value

An n by n matrix.

## Examples

``` r
robot_mass_matrix(robot_example("planar2"), c(0, 0.5))
#>           joint1    joint2
#> joint1 2.5442492 0.7721246
#> joint2 0.7721246 0.3333333
```
