# Robot models as rigid-body trees

Builds a validated kinematic tree from links and joints. Every link
except one root link has exactly one parent joint; the structure must be
connected and acyclic. Closed kinematic chains are not representable and
are rejected. The configuration vector `q` has one entry per movable
(non-fixed) joint, ordered by a breadth-first traversal from the root
that preserves the order joints were supplied in; see
[`robot_joint_names()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md).
The number of joints therefore can exceed the number of degrees of
freedom when fixed joints are present.

## Usage

``` r
robot_model(
  links = list(),
  joints,
  base = robot_pose(),
  end_effector = NULL,
  name = "robot"
)
```

## Arguments

- links:

  List of
  [`robot_link()`](https://adamrsophie.github.io/robotcompR/reference/robot_link.md)
  objects. Links named by joints but missing here are created as
  massless links.

- joints:

  List of
  [`robot_joint()`](https://adamrsophie.github.io/robotcompR/reference/robot_joint.md)
  objects.

- base:

  Pose of the root link in the world frame.

- end_effector:

  Name of the default end-effector frame. Defaults to the last leaf link
  in traversal order.

- name:

  Model name.

## Value

A `robot_model` object.

## Examples

``` r
links <- list(robot_link("base"), robot_link("link1"), robot_link("link2"))
joints <- list(
  robot_joint("j1", "revolute", "base", "link1"),
  robot_joint("j2", "revolute", "link1", "link2", origin = transl(1, 0, 0))
)
robot <- robot_model(links, joints, name = "planar")
robot
#> <robot_model> planar: 3 links, 2 joints, 2 degrees of freedom
#> end effector: link2
#>  name     type parent child q_index lower upper
#>    j1 revolute   base link1       1  -Inf   Inf
#>    j2 revolute  link1 link2       2  -Inf   Inf
```
