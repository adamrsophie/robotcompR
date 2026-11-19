# Read a URDF robot description

Parses a URDF file or string with 'xml2' and builds a validated
[`robot_model()`](https://adamrsophie.github.io/robotcompR/reference/robot_model.md).
Supported: links with inertial parameters, collision and visual geometry
(box, sphere, cylinder, capsule, and mesh references kept as metadata),
and revolute, continuous, prismatic and fixed joints with origins, axes
(default `1 0 0` as in the URDF specification) and limits. The inertia
tensor is rotated from the `<inertial><origin>` frame into the link
frame.

## Usage

``` r
robot_read_urdf(file, end_effector = NULL)
```

## Arguments

- file:

  Path to a URDF file, or a character string with URDF XML.

- end_effector:

  Optional end-effector link name.

## Value

A `robot_model`.

## Details

Features that change kinematics or dynamics but are not modeled are
rejected with an error: `<mimic>`, and `floating` and `planar` joints.
Joint `<dynamics>` damping and friction values are stored on the joint
but not used by the dynamics functions; a warning says so.

Input is treated as untrusted: xacro files are refused (expand them with
ROS tools first), documents declaring entities are refused, network
access is disabled in the XML parser, the file content is parsed as XML
text (never followed as a further path or URL), and mesh files are never
opened. Every link named by a joint must be declared. Limit values are
kept literally, as in the reference `urdfdom` parser: a `velocity` or
`effort` of 0 is a zero limit, and an absent attribute is stored as
unspecified (`NULL`).

URDF versions 1.0, 1.1 and 1.2 are accepted (any other `version` is an
error). Origins may give their orientation as `quat_xyzw` (1.1) instead
of `rpy`. The 1.2 `acceleration`, `deceleration` and `jerk` limits are
not modeled; they are ignored with a warning.

## Examples

``` r
robot <- robot_read_urdf(system.file("extdata", "planar2.urdf",
                                     package = "robotcompR"))
robot_fk(robot, c(pi / 2, pi / 2))
#> <robot_pose> SE(3) world <- tool
#>      [,1] [,2] [,3] [,4]
#> [1,]   -1    0    0   -1
#> [2,]    0   -1    0    1
#> [3,]    0    0    1    0
#> [4,]    0    0    0    1
```
