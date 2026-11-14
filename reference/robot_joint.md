# Robot joints

A joint connects a `parent` link to a `child` link. The pose of the
child link frame relative to the parent link frame is
\$\${}^{parent}T\_{child}(q) = T\_{origin}\\ M(q)\\ T\_{post},\$\$ where
\\T\_{origin}\\ is the fixed joint frame (the URDF `<origin>`), \\M(q)\\
is a rotation by \\q + offset\\ about `axis` (revolute and continuous
joints) or a translation by \\(q + offset)\\ along `axis` (prismatic
joints), and \\T\_{post}\\ is an optional fixed transform applied after
the motion. URDF joints use \\T\_{post} = I\\; Denavit-Hartenberg models
use it to place the link frame at the DH frame. The axis is expressed in
the joint frame and normalized.

## Usage

``` r
robot_joint(
  name,
  type = c("revolute", "continuous", "prismatic", "fixed"),
  parent,
  child,
  origin = robot_pose(),
  axis = c(0, 0, 1),
  limits = NULL,
  velocity = NULL,
  acceleration = NULL,
  effort = NULL,
  offset = 0,
  post = NULL
)
```

## Arguments

- name:

  Joint name.

- type:

  One of `"revolute"`, `"continuous"`, `"prismatic"`, `"fixed"`.

- parent, child:

  Names of the parent and child links.

- origin:

  Fixed `robot_pose` (SE(3)) of the joint frame in the parent link
  frame.

- axis:

  Joint axis in the joint frame.

- limits:

  Length-2 vector `c(lower, upper)` of position limits.

- velocity, acceleration, effort:

  Optional nonnegative limits.

- offset:

  Constant added to the joint variable before the motion.

- post:

  Optional fixed `robot_pose` applied after the motion.

## Value

A `robot_joint` object.

## Details

Units are radians and meters. Revolute joints default to unbounded
limits; continuous joints are always unbounded and reject finite limits.
Velocity, acceleration and effort limits are optional (`NULL` means
unspecified, not zero).

## Examples

``` r
robot_joint("shoulder", "revolute", "base", "upper_arm",
            origin = transl(0, 0, 0.1), axis = c(0, 0, 1),
            limits = c(-pi, pi))
#> <robot_joint> shoulder (revolute): base -> upper_arm, axis (0, 0, 1) limits [-3.14159, 3.14159]
```
