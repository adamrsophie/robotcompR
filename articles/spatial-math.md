# Rotations, poses and Lie groups

``` r

library(robotcompR)
```

Every robotics computation rests on a handful of conventions. Mixing
them silently is the most common source of wrong results, so robotcompR
fixes one set and checks it wherever it can.

## Rotation matrices

A rotation matrix R \in SO(3) satisfies R^\top R = I and \det R = 1.
Rotations are *active*: R p rotates the vector p. The elementary
rotations are

R_z(\theta) = \begin{pmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta
& \cos\theta & 0 \\ 0 & 0 & 1 \end{pmatrix}.

``` r

rotz(pi / 2) %*% c(1, 0, 0)
#>              [,1]
#> [1,] 6.123234e-17
#> [2,] 1.000000e+00
#> [3,] 0.000000e+00
is_rotation_matrix(rotz(0.3) %*% rotx(0.2))
#> [1] TRUE
```

Angles are radians everywhere;
[`deg2rad()`](https://adamrsophie.github.io/robotcompR/reference/angle_units.md)
and
[`rad2deg()`](https://adamrsophie.github.io/robotcompR/reference/angle_units.md)
convert at the boundary with human input:

``` r

rotz(deg2rad(90)) %*% c(1, 0, 0)
#>              [,1]
#> [1,] 6.123234e-17
#> [2,] 1.000000e+00
#> [3,] 0.000000e+00
rad2deg(pi / 6)
#> [1] 30
```

Numerical drift accumulates when many rotations are multiplied.
[`orthonormalize()`](https://adamrsophie.github.io/robotcompR/reference/orthonormalize.md)
projects back onto SO(3) with the polar decomposition:

``` r

R <- diag(3)
for (i in 1:1000) R <- R %*% rotx(0.001) %*% roty(0.002)
max(abs(crossprod(R) - diag(3)))
#> [1] 5.451195e-14
R <- orthonormalize(R)
max(abs(crossprod(R) - diag(3)))
#> [1] 2.220446e-16
```

## Euler angles

Sequences are *intrinsic*: `"ZYX"` means rotate about z, then about the
new y, then about the newest x, so R = R_z(\alpha) R_y(\beta)
R_x(\gamma). All twelve proper Euler and Tait-Bryan sequences are
supported.

``` r

r <- robot_rotation(euler = c(0.3, -0.2, 0.1), sequence = "ZYX")
as_euler(r, "ZYX")
#> [1]  0.3 -0.2  0.1
as_euler(r, "XYZ")
#> [1]  0.1564195 -0.1600272  0.3226097
as_euler(r, "ZXZ")
#> [1] -0.8031300  0.2233075  1.1131718
```

At gimbal lock (pitch \pm\pi/2 for `"ZYX"`), the decomposition is not
unique. The returned angles still reproduce the matrix:

``` r

r <- robot_rotation(euler = c(0.4, pi / 2, 0.1), sequence = "ZYX")
a <- as_euler(r, "ZYX")
a
#> [1] 0.300000 1.570796 0.000000
max(abs(robot_rotation(euler = a, sequence = "ZYX")$matrix - r$matrix))
#> [1] 1.110223e-16
```

URDF `rpy` attributes are fixed-axis roll, pitch, yaw, which equal the
intrinsic `"ZYX"` angles in reverse order.

## Axis-angle, rotation vectors and quaternions

``` r

r <- robot_rotation(axis = c(1, 1, 0), angle = 2)
as_axis_angle(r)
#> $axis
#> [1] 0.7071068 0.7071068 0.0000000
#> 
#> $angle
#> [1] 2
as_rotvec(r)
#> [1] 1.414214 1.414214 0.000000
q <- robot_quaternion(r)
q
#> <robot_quaternion> w = 0.540302, (x, y, z) = (0.59501, 0.59501, 0)
```

Quaternions are scalar first and use the Hamilton product. Since q and
-q represent the same rotation, the canonical form has w \ge 0.

``` r

qa <- robot_quaternion(robot_rotation(axis = c(0, 0, 1), angle = 0.5))
qb <- robot_quaternion(robot_rotation(axis = c(0, 1, 0), angle = 0.8))
qa * qb
#> <robot_quaternion> w = 0.892427, (x, y, z) = (-0.0963436, 0.377312, 0.227874)
quaternion_rotate(qa, c(1, 0, 0))
#> [1] 0.8775826 0.4794255 0.0000000
quaternion_distance(qa, qb)
#> [1] 0.9361988
quaternion_slerp(qa, qb, c(0, 0.5, 1))
#> [[1]]
#> <robot_quaternion> w = 0.968912, (x, y, z) = (0, 0, 0.247404)
#> 
#> [[2]]
#> <robot_quaternion> w = 0.971474, (x, y, z) = (0, 0.200167, 0.127169)
#> 
#> [[3]]
#> <robot_quaternion> w = 0.921061, (x, y, z) = (0, 0.389418, 0)
```

The conjugate of a unit quaternion is its inverse rotation:

``` r

qa * quaternion_conjugate(qa)
#> <robot_quaternion> w = 1, (x, y, z) = (0, 0, 0)
```

[`quaternion_slerp()`](https://adamrsophie.github.io/robotcompR/reference/quaternion_slerp.md)
follows the shortest arc and returns unit quaternions.

## Rigid transformations

A pose T\_{ab} \in SE(3) maps coordinates in frame b to frame a:

T\_{ab} = \begin{pmatrix} R\_{ab} & p\_{ab} \\ 0 & 1 \end{pmatrix},
\qquad p_a = R\_{ab} p_b + p\_{ab}.

Frames can be named so that mismatched compositions fail loudly:

``` r

T_world_base <- robot_pose(rotz(pi / 2), c(1, 0, 0), parent = "world", child = "base")
T_base_tool <- robot_pose(translation = c(0, 0, 0.5), parent = "base", child = "tool")
T_world_tool <- T_world_base * T_base_tool
T_world_tool
#> <robot_pose> SE(3) world <- tool
#>      [,1] [,2] [,3] [,4]
#> [1,]    0   -1    0  1.0
#> [2,]    1    0    0  0.0
#> [3,]    0    0    1  0.5
#> [4,]    0    0    0  1.0
robot_transform_point(T_world_tool, rbind(c(0, 0, 0), c(0.1, 0, 0)))
#>      [,1] [,2] [,3]
#> [1,]    1  0.0  0.5
#> [2,]    1  0.1  0.5
try(T_base_tool * T_world_base)
#> Error : Frame mismatch: left pose ends in 'tool' but right pose starts in 'world'.
```

Directions (free vectors) rotate but do not translate, and
[`robot_compose()`](https://adamrsophie.github.io/robotcompR/reference/robot_compose.md)
multiplies any number of poses left to right:

``` r

robot_transform_direction(T_world_tool, c(1, 0, 0))
#> [1] 6.123234e-17 1.000000e+00 0.000000e+00
pose_rotation(T_world_tool)
#> <robot_rotation> SO(3)
#> angle 1.5708 rad about axis (0, 0, 1)
#>      [,1] [,2] [,3]
#> [1,]    0   -1    0
#> [2,]    1    0    0
#> [3,]    0    0    1
pose_translation(robot_compose(T_world_base, T_base_tool))
#> [1] 1.0 0.0 0.5
```

The homogeneous elementary rotations
[`trotx()`](https://adamrsophie.github.io/robotcompR/reference/elementary_transforms.md),
[`troty()`](https://adamrsophie.github.io/robotcompR/reference/elementary_transforms.md)
and
[`trotz()`](https://adamrsophie.github.io/robotcompR/reference/elementary_transforms.md),
and
[`transl()`](https://adamrsophie.github.io/robotcompR/reference/elementary_transforms.md)
for pure translations, build poses quickly:

``` r

pose_translation(transl(1, 0, 0) * trotz(pi / 2) * transl(1, 0, 0))
#> [1] 1 1 0
```

Inverse and relative poses:

``` r

robot_inverse(T_world_base)
#> <robot_pose> SE(3) base <- world
#>      [,1] [,2] [,3] [,4]
#> [1,]    0    1    0    0
#> [2,]   -1    0    0    1
#> [3,]    0    0    1    0
#> [4,]    0    0    0    1
robot_relative_pose(T_world_base, T_world_tool)
#> <robot_pose> SE(3) base <- tool
#>      [,1] [,2] [,3] [,4]
#> [1,]    1    0    0  0.0
#> [2,]    0    1    0  0.0
#> [3,]    0    0    1  0.5
#> [4,]    0    0    0  1.0
robot_pose_distance(T_world_base, T_world_tool)
#> translation    rotation 
#>         0.5         0.0
```

## Lie group tools

The exponential and logarithm maps connect rotations and poses to their
tangent spaces. robotcompR uses series expansions near zero and
dedicated branches near \pi, so round trips stay accurate in both
regimes.

``` r

w <- c(0.1, -0.4, 0.3)
max(abs(so3_log(so3_exp(w)) - w))
#> [1] 0
w_pi <- c(0, 0, pi - 1e-9)
so3_log(so3_exp(w_pi))
#> [1] 0.000000 0.000000 3.141593
xi <- c(0.5, 0.2, -0.1, 0.3, 0.2, -0.4)  # linear part first
max(abs(se3_log(se3_exp(xi)) - xi))
#> [1] 1.110223e-16
```

The hat and vee operators map tangent vectors to their matrix form and
back. `*_vee()` checks that its input really is a Lie algebra element:

``` r

so3_hat(c(1, 2, 3))
#>      [,1] [,2] [,3]
#> [1,]    0   -3    2
#> [2,]    3    0   -1
#> [3,]   -2    1    0
so3_vee(so3_hat(c(1, 2, 3)))
#> [1] 1 2 3
se3_vee(se3_hat(xi))
#> [1]  0.5  0.2 -0.1  0.3  0.2 -0.4
so2_vee(so2_hat(0.4))
#> [1] 0.4
se2_vee(se2_hat(c(1, 2, 0.3)))
#> [1] 1.0 2.0 0.3
try(so3_vee(diag(3)))
#> Error : `x` is not skew-symmetric.
```

The left Jacobian of SO(3) relates small tangent perturbations to
perturbations of the rotation: \exp(\phi + \delta) \approx
\exp(J_l(\phi)\delta)\exp(\phi). The right Jacobian does the same on the
other side, \exp(\phi + \delta) \approx \exp(\phi)\exp(J_r(\phi)\delta),
and both have closed-form inverses.

``` r

phi <- c(0.4, -0.2, 0.7)
d <- c(1e-6, 0, 0)
lhs <- so3_exp(phi + d) %*% t(so3_exp(phi))
max(abs(so3_log(lhs) - so3_jacobian_left(phi) %*% d))
#> [1] 5.223193e-14
rhs <- t(so3_exp(phi)) %*% so3_exp(phi + d)
max(abs(so3_log(rhs) - so3_jacobian_right(phi) %*% d))
#> [1] 5.861263e-14
max(abs(so3_jacobian_left_inverse(phi) %*% so3_jacobian_left(phi) - diag(3)))
#> [1] 4.82014e-17
max(abs(so3_jacobian_right_inverse(phi) %*% so3_jacobian_right(phi) - diag(3)))
#> [1] 4.405759e-17
```

Pose graph optimization and inverse kinematics use exactly these maps.

The adjoint maps twists between frames. With linear-first twists \xi =
(v, \omega):

\mathrm{Ad}\_T = \begin{pmatrix} R & \[p\]\_\times R \\ 0 & R
\end{pmatrix}.

``` r

T <- robot_pose(rotz(0.3), c(1, 2, 0))
round(se3_adjoint(T), 4)
#>        [,1]    [,2] [,3]    [,4]    [,5] [,6]
#> [1,] 0.9553 -0.2955    0  0.0000  0.0000    2
#> [2,] 0.2955  0.9553    0  0.0000  0.0000   -1
#> [3,] 0.0000  0.0000    1 -1.6152  1.5464    0
#> [4,] 0.0000  0.0000    0  0.9553 -0.2955    0
#> [5,] 0.0000  0.0000    0  0.2955  0.9553    0
#> [6,] 0.0000  0.0000    0  0.0000  0.0000    1
```

The planar adjoint acts on planar twists (v_x, v_y, \omega):

``` r

round(se2_adjoint(se2_exp(c(1, 0, pi / 4))), 4)
#>        [,1]    [,2]    [,3]
#> [1,] 0.7071 -0.7071  0.3729
#> [2,] 0.7071  0.7071 -0.9003
#> [3,] 0.0000  0.0000  1.0000
```

## Interpolating poses

[`robot_pose_interp()`](https://adamrsophie.github.io/robotcompR/reference/robot_pose_interp.md)
either interpolates translation linearly and rotation by slerp
(`"decoupled"`), or moves along the constant twist joining the poses
(`"screw"`). Both agree at the endpoints.

``` r

a <- robot_pose()
b <- robot_pose(rotz(pi / 2), c(1, 1, 0))
mid <- robot_pose_interp(a, b, 0.5, method = "decoupled")
pose_translation(mid)
#> [1] 0.5 0.5 0.0
mid_screw <- robot_pose_interp(a, b, 0.5, method = "screw")
pose_translation(mid_screw)
#> [1] 0.7071068 0.2928932 0.0000000
```

## Planar rotations and poses

[`rot2()`](https://adamrsophie.github.io/robotcompR/reference/elementary_rotations.md)
is the planar rotation matrix (the same as
[`so2_exp()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log_2d.md)),
and the planar logarithms return the angle in (-\pi, \pi\] and the
planar twist:

``` r

rot2(pi / 3)
#>           [,1]       [,2]
#> [1,] 0.5000000 -0.8660254
#> [2,] 0.8660254  0.5000000
so2_log(rot2(pi / 3))
#> [1] 1.047198
se2_exp(c(1, 0, pi / 2))
#>              [,1]          [,2]      [,3]
#> [1,] 6.123234e-17 -1.000000e+00 0.6366198
#> [2,] 1.000000e+00  6.123234e-17 0.6366198
#> [3,] 0.000000e+00  0.000000e+00 1.0000000
se2_log(se2_exp(c(1, 0, pi / 2)))
#> [1]  1.000000e+00 -3.168556e-17  1.570796e+00
wrap_angle(c(3 * pi, -3 * pi / 2))
#> [1] -3.141593  1.570796
```

Mobile robot velocities are often given in the body frame (forward and
sideways speed).
[`body_to_world_velocity()`](https://adamrsophie.github.io/robotcompR/reference/planar_velocity.md)
and
[`world_to_body_velocity()`](https://adamrsophie.github.io/robotcompR/reference/planar_velocity.md)
rotate them by the heading:

``` r

v_world <- body_to_world_velocity(c(1, 0), theta = pi / 2)
v_world
#> [1] 6.123234e-17 1.000000e+00
world_to_body_velocity(v_world, theta = pi / 2)
#> [1] 1 0
```
