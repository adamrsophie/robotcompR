# robotcompR: Robot Modeling, Kinematics, Dynamics and Motion Planning in R

***robotcompR*** is an R-native framework for robotics computation. It
covers the path from spatial mathematics to robot models, kinematics,
dynamics, collision checking, motion planning, trajectories and planar
mobile robotics, with explicit conventions and reference-checked
algorithms. It needs no MATLAB, Python, Julia or ROS 2; everything runs
on base R, [Matrix](https://cran.r-project.org/package=Matrix),
[deSolve](https://cran.r-project.org/package=deSolve) and
[xml2](https://cran.r-project.org/package=xml2).

Documentation: <https://adamrsophie.github.io/robotcompR/>

## Installation

``` r

# install.packages("remotes")
remotes::install_github("adamrsophie/robotcompR")
```

## Example

``` r

library(robotcompR)

arm <- robot_example("planar2")
q <- c(pi / 4, -pi / 3)
robot_fk(arm, q)                       # end-effector pose
robot_jacobian(arm, q)                 # geometric Jacobian, linear rows first
robot_mass_matrix(arm, q)              # composite rigid body algorithm

# Inverse kinematics reports convergence instead of hiding failure.
sol <- robot_ik(arm, robot_pose(translation = c(1.2, 0.8, 0)),
                weights = c(1, 1, 0, 0, 0, 0))
sol$converged

# Plan around an obstacle and time the result.
robot <- robot_capsule_links(arm, radius = 0.05)
env <- robot_environment(block = geom_box(c(0.4, 0.4, 0.4), origin = transl(1.5, 0, 0)))
plan <- robot_plan(robot_planner(robot, "rrt_connect", env), c(-0.8, 0), c(0.8, 0),
                   seed = 1, smooth = TRUE)
traj <- robot_traj_path(plan, vmax = 1, amax = 2)
```

## Features

| Area | What is included |
|----|----|
| Spatial math | SO(2), SO(3), SE(2), SE(3); rotation matrices, 12 Euler sequences, axis-angle, rotation vectors, scalar-first quaternions with slerp; exp and log maps, adjoints, SO(3) Jacobians; frame-checked pose composition |
| Models | Kinematic trees with revolute, continuous, prismatic and fixed joints; standard and modified DH; inertia helpers; URDF import and export; PUMA 560 and planar examples |
| Kinematics | Forward kinematics for all frames, geometric (world, local, space) and analytic Jacobians, manipulability and singularity tests |
| Inverse kinematics | Levenberg-Marquardt, damped least squares and pseudoinverse solvers with weights, limits, restarts and honest convergence reports; closed form two-link IK |
| Dynamics | Recursive Newton-Euler, composite rigid body and articulated body algorithms; gravity, Coriolis and bias terms; external wrenches; energy and center of mass; ODE simulation with closed-loop torques |
| Trajectories | Cubic and quintic polynomials, synchronized trapezoidal profiles, timed waypoint paths, Cartesian straight lines with spatial velocities and accelerations and their joint-space conversion, limit checks |
| Collision | Spheres, boxes, cylinders, capsules and convex hulls; GJK distances with closed form sphere and capsule pairs; EPA penetration depth; environment and self-collision checks; discrete and continuous (conservative advancement) path checks |
| Planning | RRT, RRT-Connect, RRT\* and PRM in joint space or any custom space, shortcut smoothing, reproducible seeds |
| Mobile robots | Exact unicycle integration, differential drive with odometry, kinematic bicycle, pure pursuit |
| Localization | EKF with wrapped innovations and Joseph form; unscented Kalman filter; particle filter with low-variance resampling |
| Mapping and SLAM | Log-odds occupancy grids, exact ray casting, scan integration, inflation, A\*; EKF-SLAM with landmark initialization; SE(2) and SE(3) pose graph optimization with sparse Gauss-Newton and LM damping |

## Conventions

- SI units throughout.
- Quaternions are `c(w, x, y, z)` with the Hamilton product.
- Euler sequences are intrinsic: `"ZYX"` returns yaw, pitch, roll.
- `T_a_b` maps coordinates in frame b to frame a; compose with `*`.
- Twists, wrenches and Jacobian rows put the linear part first.

## Limitations

robotcompR is written in R for clarity and teaching. Planners and
filters are practical for a few degrees of freedom and modest sample
counts. Floating, planar and mimic joints, mesh collision geometry,
friction and contact dynamics are out of scope for this release. A
planner failure never proves that no path exists.

## The adamrsophie computational series

robotcompR joins symcompR, z3r, jaxR, modelcompR, optcompR, graphlearnR,
pdecompR, controlcompR and simcompR.

## License

GPL-3. robotcompR is research and teaching software. It is not certified
for controlling industrial, medical, automotive or other safety-critical
hardware.
