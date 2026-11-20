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
  self-collision, and path checks.
- Sampling-based planning with RRT, RRT-Connect, RRT\* and PRM, plus
  shortcut smoothing.
- Planar mobile robots: unicycle, differential drive, bicycle, odometry
  and pure pursuit.
- Localization with extended and unscented Kalman filters and a particle
  filter.
- Occupancy grid mapping, ray casting, inflation and A\* search.
- EKF-SLAM with known data association.
- SE(2) and SE(3) pose graph optimization with Jacobians that stay valid
  for residual rotations near pi and a termination `status`.
- The unscented Kalman filter works in a local tangent chart, so heading
  statistics stay consistent across the angle cut; every filter requires
  symmetric positive semidefinite covariances and validated
  measurements.
- Particle filter prediction, EKF-SLAM updates and rigid-body dynamics
  use faster kernels (vectorized motion, a sparse Joseph update
  quadratic in the state size, and cached spatial inertias). A SLAM
  update that removes almost all of some variance is redone in factored
  form so the small posterior is not lost to cancellation.
- EPA penetration depth for every convex pair, with depth bounds and a
  `depth_known` flag, and continuous path collision checking by
  conservative advancement, which steps by the certified lower bound on
  the distance.
- Cartesian trajectories report spatial velocities and accelerations and
  can be converted to joint trajectories.
- Separation is reported only when a positive lower bound proves it.
  Thin or high aspect ratio shapes that GJK cannot resolve are reported
  as colliding with `depth_known = FALSE` and an interval from support
  function bounds. The guarantees hold up to floating point roundoff;
  tolerances are relative to the object size, so results do not depend
  on scale, rotation or where a hull’s local origin lies.
- Self-collision skips joint neighbors even across geometry-free fixed
  frames, and still checks a tool fixed to a link against the link
  before that link’s joint.
- Pose graph optimization tests convergence per coordinate, with an
  arithmetic roundoff bound and a componentwise gradient test
  (`gradient_tol`), so neither a large world offset nor a large
  irreducible cost in one coordinate hides an error in another.
- URDF export writes numbers that read back exactly in any locale and
  declares URDF 1.1 for capsules; import reads `quat_xyzw` origins.
- Planners check their deadline after every validity call (a callback
  that runs long can still overrun once), report the reason work stopped
  apart from `success`, and RRT-Connect always validates the edge that
  joins its trees. A\* keeps its search on the map with a frontier list.
- Joint arrays are matched by name in every function: unnamed values are
  positional, fully named values may come in any order, and partial,
  duplicated or unknown names are an error.
- [`robot_traj_cartesian()`](https://adamrsophie.github.io/robotcompR/reference/robot_traj_cartesian.md)
  returns joint velocities and accelerations that describe the returned
  joint path, including joints held at a limit, and flags samples
  without a coherent derivative in `derivatives_ok`.
- [`robot_simulate()`](https://adamrsophie.github.io/robotcompR/reference/robot_simulate.md)
  treats a run that stops before the last requested time as an error
  that carries the solver’s message.
- Inertia tensors accepted within roundoff are stored projected onto the
  physical set, and covariance symmetry is checked pair by pair.
- Nine vignettes and a pkgdown site.
