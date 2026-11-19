# Robot models: joints, links, DH parameters and URDF

``` r

library(robotcompR)
```

A `robot_model` is a kinematic tree: links connected by joints, rooted
at a base link. Each joint carries a fixed `origin` pose (from the
parent link frame to the joint frame), a motion along or about its
`axis`, and optional limits. Serial chains, branched trees and models
read from URDF all share the same representation, so every algorithm in
the package works on all of them.

## Building a model by hand

The child link frame of a joint is

T\_{\text{parent},\text{child}}(q) = T\_{\text{origin}} \\ M(q + q_0) \\
T\_{\text{post}},

where M is a rotation about the axis (revolute and continuous joints) or
a translation along it (prismatic joints), q_0 is an optional offset and
T\_{\text{post}} an optional fixed transform after the motion (used by
the DH builder).

``` r

links <- list(
  robot_link("base"),
  robot_link("upper", mass = 2, com = c(0.25, 0, 0),
             inertia = inertia_rod(2, 0.5, axis = "x")),
  robot_link("lower", mass = 1, com = c(0.2, 0, 0),
             inertia = inertia_rod(1, 0.4, axis = "x"))
)
joints <- list(
  robot_joint("shoulder", "revolute", "base", "upper",
              limits = c(-pi, pi), velocity = 2, effort = 40),
  robot_joint("elbow", "revolute", "upper", "lower",
              origin = transl(0.5, 0, 0), limits = c(-2.5, 2.5))
)
arm <- robot_model(links, joints, name = "hand_built")
arm <- robot_add_frame(arm, "tool", "lower", origin = transl(0.4, 0, 0))
arm
#> <robot_model> hand_built: 4 links, 3 joints, 2 degrees of freedom
#> end effector: tool
#>        name     type parent child q_index     lower    upper
#>    shoulder revolute   base upper       1 -3.141593 3.141593
#>       elbow revolute  upper lower       2 -2.500000 2.500000
#>  tool_joint    fixed  lower  tool      NA  0.000000 0.000000
robot_links(arm)
#>    name parent mass n_collision
#> 1  base   <NA>    0           0
#> 2 upper   base    2           0
#> 3 lower  upper    1           0
#> 4  tool  lower    0           0
robot_limits(arm)
#>              lower    upper
#> shoulder -3.141593 3.141593
#> elbow    -2.500000 2.500000
```

Inertia helpers return tensors about the center of mass:

``` r

inertia_box(1, c(0.1, 0.2, 0.3))
#>            [,1]        [,2]        [,3]
#> [1,] 0.01083333 0.000000000 0.000000000
#> [2,] 0.00000000 0.008333333 0.000000000
#> [3,] 0.00000000 0.000000000 0.004166667
inertia_cylinder(2, radius = 0.05, length = 0.4)
#>            [,1]       [,2]   [,3]
#> [1,] 0.02791667 0.00000000 0.0000
#> [2,] 0.00000000 0.02791667 0.0000
#> [3,] 0.00000000 0.00000000 0.0025
inertia_sphere(1, 0.1)
#>       [,1]  [,2]  [,3]
#> [1,] 0.004 0.000 0.000
#> [2,] 0.000 0.004 0.000
#> [3,] 0.000 0.000 0.004
```

[`robot_link()`](https://adamrsophie.github.io/robotcompR/reference/robot_link.md)
rejects inertia tensors that are not symmetric positive semidefinite or
violate the triangle inequality on the principal moments.

## Denavit-Hartenberg parameters

[`robot_dh()`](https://adamrsophie.github.io/robotcompR/reference/robot_dh.md)
builds a chain from standard (distal) or modified (proximal, Craig) DH
parameters. With the standard convention, each link transform is

T_i = R_z(\theta_i) \\ T_z(d_i) \\ T_x(a_i) \\ R_x(\alpha_i).

With the modified convention it is T_i = R_x(\alpha\_{i-1}) \\
T_x(a\_{i-1}) \\ R_z(\theta_i) \\ T_z(d_i).

``` r

scara <- robot_dh(a = c(0.4, 0.3, 0), alpha = c(0, pi, 0), d = c(0.2, 0, 0),
                  theta = c(0, 0, 0),
                  joint_type = c("revolute", "revolute", "prismatic"),
                  limits = rbind(c(-2.5, 2.5), c(-2, 2), c(0, 0.15)),
                  name = "scara")
scara
#> <robot_model> scara: 4 links, 3 joints, 3 degrees of freedom
#> end effector: link3
#>    name      type parent child q_index lower upper
#>  joint1  revolute   base link1       1  -2.5  2.50
#>  joint2  revolute  link1 link2       2  -2.0  2.00
#>  joint3 prismatic  link2 link3       3   0.0  0.15
pose_translation(robot_fk(scara, c(0, 0, 0.1)))
#> [1]  7.000000e-01 -1.224647e-17  1.000000e-01
dh_transform(0.4, 0, 0.2, 0)
#> <robot_pose> SE(3)
#>      [,1] [,2] [,3] [,4]
#> [1,]    1    0    0  0.4
#> [2,]    0    1    0  0.0
#> [3,]    0    0    1  0.2
#> [4,]    0    0    0  1.0
```

The PUMA 560 example uses the standard DH table of Corke (1996), in
meters:

``` r

puma <- robot_example("puma560")
puma$dh
#>        a     alpha       d theta     type
#> 1 0.0000  1.570796 0.00000     0 revolute
#> 2 0.4318  0.000000 0.00000     0 revolute
#> 3 0.0203 -1.570796 0.15005     0 revolute
#> 4 0.0000  1.570796 0.43180     0 revolute
#> 5 0.0000 -1.570796 0.00000     0 revolute
#> 6 0.0000  0.000000 0.00000     0 revolute
pose_translation(robot_fk(puma, rep(0, 6)))
#> [1]  0.45210 -0.15005  0.43180
```

## Branched trees

``` r

tree <- robot_example("branched")
robot_joints(tree)[, c("name", "parent", "child")]
#>                                  name parent        child
#> shoulder                     shoulder   base          arm
#> elbow                           elbow    arm        wrist
#> palm_mount                 palm_mount  wrist         palm
#> finger_left_joint   finger_left_joint   palm  finger_left
#> finger_right_joint finger_right_joint   palm finger_right
names(robot_fk(tree, c(0.3, -0.2, 0.4, 0.1), frame = "all"))
#> [1] "base"         "arm"          "wrist"        "palm"         "finger_left" 
#> [6] "finger_right"
```

## URDF import and export

[`robot_read_urdf()`](https://adamrsophie.github.io/robotcompR/reference/robot_read_urdf.md)
accepts a file path or an XML string. It supports revolute, continuous,
prismatic and fixed joints, inertials (rotated into the link frame), and
visual and collision primitives. It refuses things it cannot represent
instead of guessing:

- `xacro` files must be expanded first.
- Floating, planar and mimic joints are errors.
- Documents with entity declarations are rejected, and network access is
  disabled while parsing.
- Mesh geometry is stored by file name and never opened.
- Joint `<dynamics>` (damping and friction) is read, stored and reported
  with a warning, because the dynamics functions do not model it.

``` r

pt <- robot_read_urdf(system.file("extdata", "pan_tilt.urdf", package = "robotcompR"))
pt
#> <robot_model> pan_tilt: 4 links, 3 joints, 3 degrees of freedom
#> end effector: lens
#>        name       type parent child q_index lower upper
#>   pan_joint continuous   base   pan       1  -Inf   Inf
#>  tilt_joint   revolute    pan  tilt       2  -1.2  1.20
#>  zoom_joint  prismatic   tilt  lens       3   0.0  0.05
robot_joints(pt)[, c("name", "type", "lower", "upper")]
#>                  name       type lower upper
#> pan_joint   pan_joint continuous  -Inf   Inf
#> tilt_joint tilt_joint   revolute  -1.2  1.20
#> zoom_joint zoom_joint  prismatic   0.0  0.05
```

Exporting writes a URDF that reproduces the same kinematics and inertia.
Joint offsets are folded into the origin, and DH post transforms become
an extra fixed joint. URDF requires effort and velocity limits on
revolute and prismatic joints, and every URDF reader treats a value of 0
literally as “no motion”. The example models leave these limits
unspecified, so the export asks for the values to write through
`fill_limits`:

``` r

xml <- robot_write_urdf(robot_example("planar3"),
                        fill_limits = c(effort = 50, velocity = 5))
cat(substr(xml, 1, 600))
#> <?xml version="1.0"?>
#> <robot name="planar3">
#>   <link name="base">
#>   </link>
#>   <link name="link1">
#>     <inertial><origin xyz="            -0.5                0                0" rpy="0 0 0"/><mass value="               1"/><inertia ixx="               0" ixy="               0" ixz="               0" iyy="0.0833333333333333" iyz="               0" izz="0.0833333333333333"/></inertial>
#>   </link>
#>   <link name="link2">
#>     <inertial><origin xyz="            -0.4                0                0" rpy="0 0 0"/><mass value="             0.8"/><inertia ixx="               0" ixy="               0" ixz
back <- robot_read_urdf(xml)
q <- c(0.1, 0.2, 0.3)
max(abs(robot_fk(back, q)$matrix - robot_fk(robot_example("planar3"), q)$matrix))
#> [1] 0
```

## Modifying models

Models are immutable values; modifiers return a new model.

``` r

mounted <- robot_set_base(arm, robot_pose(rotz(pi / 2), c(0, 0, 1)))
pose_translation(robot_fk(mounted, c(0, 0)))
#> [1] 5.510911e-17 9.000000e-01 1.000000e+00
heavier <- robot_set_inertia(arm, "lower", mass = 3, com = c(0.2, 0, 0))
robot_links(heavier)$mass
#> [1] 0 2 3 0
tight <- robot_set_limits(arm, rbind(c(-1, 1), c(-1, 1)))
robot_within_limits(tight, c(0.5, 1.5))
#> [1] FALSE
robot_clamp(tight, c(0.5, 1.5))
#> [1] 0.5 1.0
```
