# Changelog

## robotcompR 0.1.0

- Spatial mathematics: SO(2), SO(3), SE(2) and SE(3) objects,
  scalar-first unit quaternions, Euler angles for all 12 intrinsic
  sequences, Lie exponential and logarithm maps, adjoints and SO(3)
  Jacobians.
- Rigid-body tree models with revolute, continuous, prismatic and fixed
  joints, standard and modified Denavit-Hartenberg models, and URDF
  import and export.
- Forward kinematics, geometric and analytic Jacobians, manipulability.
- Numerical inverse kinematics with Levenberg-Marquardt, damped least
  squares and pseudoinverse methods, joint limits and restarts.
- Rigid-body dynamics: recursive Newton-Euler, composite rigid body and
  articulated body algorithms, gravity and Coriolis terms, energy, and
  ODE simulation.
- Trajectories: cubic and quintic polynomials, trapezoidal profiles,
  timed waypoint paths and Cartesian straight lines.
- Collision checking with convex primitives and GJK, environment and
  self-collision, and discrete path checks.
- Sampling-based planning with RRT, RRT-Connect, RRT\* and PRM, plus
  shortcut smoothing.
- Planar mobile robots: unicycle, differential drive, bicycle, odometry
  and pure pursuit.
- Localization with an extended Kalman filter and a particle filter.
- Occupancy grid mapping, ray casting, inflation and A\* search.
- SE(2) pose graph optimization.
- Nine vignettes and a pkgdown site.
