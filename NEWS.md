# robotcompR 0.1.0

* Spatial mathematics: SO(2), SO(3), SE(2) and SE(3) objects, scalar-first
  unit quaternions, Euler angles for all 12 intrinsic sequences, Lie
  exponential and logarithm maps, adjoints and SO(3) Jacobians.
* Rigid-body tree models with revolute, continuous, prismatic and fixed
  joints, standard and modified Denavit-Hartenberg models, and URDF import
  and export.
* Forward kinematics, geometric and analytic Jacobians, manipulability.
* Numerical inverse kinematics with Levenberg-Marquardt, damped least
  squares and pseudoinverse methods, joint limits and restarts.
* Rigid-body dynamics: recursive Newton-Euler, composite rigid body and
  articulated body algorithms, gravity and Coriolis terms, energy, and ODE
  simulation.
* Trajectories: cubic and quintic polynomials, trapezoidal profiles, timed
  waypoint paths and Cartesian straight lines.
* Collision checking with convex primitives and GJK, environment and
  self-collision, and path checks.
* Sampling-based planning with RRT, RRT-Connect, RRT* and PRM, plus
  shortcut smoothing.
* Planar mobile robots: unicycle, differential drive, bicycle, odometry and
  pure pursuit.
* Localization with extended and unscented Kalman filters and a particle
  filter.
* Occupancy grid mapping, ray casting, inflation and A* search.
* EKF-SLAM with known data association.
* SE(2) and SE(3) pose graph optimization with Jacobians that stay valid
  for residual rotations near pi and a termination `status`.
* The unscented Kalman filter works in a local tangent chart, so heading
  statistics stay consistent across the angle cut; every filter requires
  symmetric positive semidefinite covariances and validated measurements.
* Particle filter prediction, EKF-SLAM updates and rigid-body dynamics use
  faster kernels (vectorized motion, a factored sparse Joseph update that
  keeps the covariance semidefinite, and cached spatial inertias).
* EPA penetration depth for every convex pair, with certified bounds and a
  `depth_known` flag, and continuous path collision checking by
  conservative advancement.
* Cartesian trajectories report spatial velocities and accelerations and can
  be converted to joint trajectories.
* Collision distances never report clearance that thin, nearly parallel or
  degenerate geometry cannot prove, and do not depend on scale, rotation
  or where a hull's local origin lies.
* Self-collision skips joint neighbors even across geometry-free fixed
  frames, and still checks a tool fixed to a link against the link before
  that link's joint.
* Pose graph optimization tests convergence per residual component with
  equilibrated damping, so graphs with mixed scales converge.
* URDF export writes numbers that read back exactly in any locale and
  declares URDF 1.1 for capsules; import reads `quat_xyzw` origins.
* Planners honor their deadline after every validity call, and A* keeps
  its search on the map with a frontier list.
* Nine vignettes and a pkgdown site.
