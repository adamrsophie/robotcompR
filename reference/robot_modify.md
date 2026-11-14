# Modify a robot model

Models are immutable; these functions return modified copies.
`robot_add_frame()` attaches a new massless frame (for example a tool
center point) to an existing link through a fixed joint.
`robot_set_base()` changes the pose of the root link in the world.
`robot_set_end_effector()` changes the default end-effector frame.
`robot_set_inertia()` replaces the inertial parameters of a link.
`robot_set_limits()` replaces position limits of movable joints.

## Usage

``` r
robot_add_frame(
  robot,
  name,
  parent,
  origin = robot_pose(),
  set_end_effector = TRUE
)

robot_set_base(robot, base)

robot_set_end_effector(robot, frame)

robot_set_inertia(
  robot,
  link,
  mass,
  com = c(0, 0, 0),
  inertia = matrix(0, 3, 3)
)

robot_set_limits(robot, limits)
```

## Arguments

- robot:

  A `robot_model`.

- name:

  New frame name.

- parent:

  Existing link to attach to.

- origin:

  Pose of the new frame in the parent link frame.

- set_end_effector:

  Make the new frame the default end effector.

- base:

  New base pose.

- frame:

  Frame name.

- link:

  Link name.

- mass, com, inertia:

  Inertial parameters as in
  [`robot_link()`](https://adamrsophie.github.io/robotcompR/reference/robot_link.md).

- limits:

  Matrix or list of `c(lower, upper)` with one entry per degree of
  freedom, or a named list keyed by joint name.

## Value

A modified `robot_model`.

## Examples

``` r
robot <- robot_example("planar2")
robot <- robot_add_frame(robot, "tcp", "link2", transl(0.1, 0, 0))
pose_translation(robot_fk(robot, c(0, 0)))
#> [1] 2.1 0.0 0.0
```
