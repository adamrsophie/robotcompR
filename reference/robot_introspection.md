# Model introspection

`robot_dof()` returns the number of independent degrees of freedom (the
number of movable joints). `robot_joints()` returns a data frame of all
joints in traversal order; `q_index` gives the position of each movable
joint in configuration vectors. `robot_joint_names()` returns the names
of movable joints in configuration order. `robot_links()` returns a data
frame of links with inertial summaries. `robot_frames()` returns all
frame names (one per link). `robot_limits()` returns a matrix of
position limits with one row per degree of freedom.

## Usage

``` r
robot_dof(robot)

robot_joints(robot)

robot_joint_names(robot)

robot_links(robot)

robot_frames(robot)

robot_limits(robot)
```

## Arguments

- robot:

  A `robot_model`.

## Value

An integer, data frame, character vector or matrix.

## Examples

``` r
robot <- robot_example("planar2")
robot_dof(robot)
#> [1] 2
robot_joints(robot)
#>          name     type parent child q_index     lower    upper
#> joint1 joint1 revolute   base link1       1 -3.141593 3.141593
#> joint2 joint2 revolute  link1 link2       2 -3.141593 3.141593
robot_limits(robot)
#>            lower    upper
#> joint1 -3.141593 3.141593
#> joint2 -3.141593 3.141593
```
