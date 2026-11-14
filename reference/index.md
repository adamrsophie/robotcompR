# Package index

## Package overview

- [`robotcompR`](https://adamrsophie.github.io/robotcompR/reference/robotcompR-package.md)
  [`robotcompR-package`](https://adamrsophie.github.io/robotcompR/reference/robotcompR-package.md)
  : robotcompR: robotics, kinematics, dynamics and motion planning in R

## Rotations and quaternions

- [`robot_rotation()`](https://adamrsophie.github.io/robotcompR/reference/robot_rotation.md)
  : Rotations in two and three dimensions
- [`as_euler()`](https://adamrsophie.github.io/robotcompR/reference/rotation_conversions.md)
  [`as_axis_angle()`](https://adamrsophie.github.io/robotcompR/reference/rotation_conversions.md)
  [`as_rotvec()`](https://adamrsophie.github.io/robotcompR/reference/rotation_conversions.md)
  : Convert a rotation to other representations
- [`rotx()`](https://adamrsophie.github.io/robotcompR/reference/elementary_rotations.md)
  [`roty()`](https://adamrsophie.github.io/robotcompR/reference/elementary_rotations.md)
  [`rotz()`](https://adamrsophie.github.io/robotcompR/reference/elementary_rotations.md)
  [`rot2()`](https://adamrsophie.github.io/robotcompR/reference/elementary_rotations.md)
  : Elementary rotation matrices
- [`is_rotation_matrix()`](https://adamrsophie.github.io/robotcompR/reference/is_rotation_matrix.md)
  : Test whether a matrix is a rotation matrix
- [`orthonormalize()`](https://adamrsophie.github.io/robotcompR/reference/orthonormalize.md)
  : Project a matrix onto the nearest rotation
- [`robot_quaternion()`](https://adamrsophie.github.io/robotcompR/reference/robot_quaternion.md)
  : Unit quaternions
- [`quaternion_conjugate()`](https://adamrsophie.github.io/robotcompR/reference/quaternion_ops.md)
  [`quaternion_rotate()`](https://adamrsophie.github.io/robotcompR/reference/quaternion_ops.md)
  [`quaternion_distance()`](https://adamrsophie.github.io/robotcompR/reference/quaternion_ops.md)
  : Quaternion operations
- [`quaternion_slerp()`](https://adamrsophie.github.io/robotcompR/reference/quaternion_slerp.md)
  : Spherical linear interpolation of rotations
- [`deg2rad()`](https://adamrsophie.github.io/robotcompR/reference/angle_units.md)
  [`rad2deg()`](https://adamrsophie.github.io/robotcompR/reference/angle_units.md)
  : Convert between degrees and radians
- [`wrap_angle()`](https://adamrsophie.github.io/robotcompR/reference/wrap_angle.md)
  : Wrap angles to the interval from minus pi to pi

## Rigid transformations

- [`robot_pose()`](https://adamrsophie.github.io/robotcompR/reference/robot_pose.md)
  : Rigid-body poses in SE(2) and SE(3)
- [`pose_rotation()`](https://adamrsophie.github.io/robotcompR/reference/pose_components.md)
  [`pose_translation()`](https://adamrsophie.github.io/robotcompR/reference/pose_components.md)
  : Pose components
- [`transl()`](https://adamrsophie.github.io/robotcompR/reference/elementary_transforms.md)
  [`trotx()`](https://adamrsophie.github.io/robotcompR/reference/elementary_transforms.md)
  [`troty()`](https://adamrsophie.github.io/robotcompR/reference/elementary_transforms.md)
  [`trotz()`](https://adamrsophie.github.io/robotcompR/reference/elementary_transforms.md)
  : Homogeneous transforms from elementary motions
- [`robot_compose()`](https://adamrsophie.github.io/robotcompR/reference/robot_compose.md)
  : Compose poses
- [`robot_inverse()`](https://adamrsophie.github.io/robotcompR/reference/robot_inverse.md)
  : Inverse of a rotation or pose
- [`robot_relative_pose()`](https://adamrsophie.github.io/robotcompR/reference/robot_relative_pose.md)
  : Relative pose between two frames
- [`robot_transform_point()`](https://adamrsophie.github.io/robotcompR/reference/robot_transform_point.md)
  [`robot_transform_direction()`](https://adamrsophie.github.io/robotcompR/reference/robot_transform_point.md)
  : Transform points and directions
- [`robot_pose_distance()`](https://adamrsophie.github.io/robotcompR/reference/robot_pose_distance.md)
  : Distance between two poses
- [`robot_pose_interp()`](https://adamrsophie.github.io/robotcompR/reference/robot_pose_interp.md)
  : Interpolate between two poses

## Lie groups

- [`so3_hat()`](https://adamrsophie.github.io/robotcompR/reference/hat_vee.md)
  [`so3_vee()`](https://adamrsophie.github.io/robotcompR/reference/hat_vee.md)
  [`se3_hat()`](https://adamrsophie.github.io/robotcompR/reference/hat_vee.md)
  [`se3_vee()`](https://adamrsophie.github.io/robotcompR/reference/hat_vee.md)
  [`so2_hat()`](https://adamrsophie.github.io/robotcompR/reference/hat_vee.md)
  [`so2_vee()`](https://adamrsophie.github.io/robotcompR/reference/hat_vee.md)
  [`se2_hat()`](https://adamrsophie.github.io/robotcompR/reference/hat_vee.md)
  [`se2_vee()`](https://adamrsophie.github.io/robotcompR/reference/hat_vee.md)
  : Hat and vee operators
- [`so3_exp()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log.md)
  [`so3_log()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log.md)
  [`se3_exp()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log.md)
  [`se3_log()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log.md)
  : Exponential and logarithm maps of SO(3) and SE(3)
- [`so2_exp()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log_2d.md)
  [`so2_log()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log_2d.md)
  [`se2_exp()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log_2d.md)
  [`se2_log()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log_2d.md)
  : Exponential and logarithm maps of SO(2) and SE(2)
- [`so3_jacobian_left()`](https://adamrsophie.github.io/robotcompR/reference/so3_jacobians.md)
  [`so3_jacobian_right()`](https://adamrsophie.github.io/robotcompR/reference/so3_jacobians.md)
  [`so3_jacobian_left_inverse()`](https://adamrsophie.github.io/robotcompR/reference/so3_jacobians.md)
  [`so3_jacobian_right_inverse()`](https://adamrsophie.github.io/robotcompR/reference/so3_jacobians.md)
  : Jacobians of SO(3)
- [`se3_adjoint()`](https://adamrsophie.github.io/robotcompR/reference/se3_adjoint.md)
  [`se2_adjoint()`](https://adamrsophie.github.io/robotcompR/reference/se3_adjoint.md)
  : Adjoint representation of SE(3) and SE(2)

## Robot models

- [`robot_joint()`](https://adamrsophie.github.io/robotcompR/reference/robot_joint.md)
  : Robot joints
- [`robot_link()`](https://adamrsophie.github.io/robotcompR/reference/robot_link.md)
  : Robot links
- [`inertia_rod()`](https://adamrsophie.github.io/robotcompR/reference/inertia_solids.md)
  [`inertia_box()`](https://adamrsophie.github.io/robotcompR/reference/inertia_solids.md)
  [`inertia_cylinder()`](https://adamrsophie.github.io/robotcompR/reference/inertia_solids.md)
  [`inertia_sphere()`](https://adamrsophie.github.io/robotcompR/reference/inertia_solids.md)
  : Inertia tensors of simple solids
- [`robot_model()`](https://adamrsophie.github.io/robotcompR/reference/robot_model.md)
  : Robot models as rigid-body trees
- [`robot_dh()`](https://adamrsophie.github.io/robotcompR/reference/robot_dh.md)
  : Robot models from Denavit-Hartenberg parameters
- [`dh_transform()`](https://adamrsophie.github.io/robotcompR/reference/dh_transform.md)
  : Denavit-Hartenberg transform of one link
- [`robot_example()`](https://adamrsophie.github.io/robotcompR/reference/robot_example.md)
  : Example robot models
- [`robot_dof()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md)
  [`robot_joints()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md)
  [`robot_joint_names()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md)
  [`robot_links()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md)
  [`robot_frames()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md)
  [`robot_limits()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md)
  : Model introspection
- [`robot_add_frame()`](https://adamrsophie.github.io/robotcompR/reference/robot_modify.md)
  [`robot_set_base()`](https://adamrsophie.github.io/robotcompR/reference/robot_modify.md)
  [`robot_set_end_effector()`](https://adamrsophie.github.io/robotcompR/reference/robot_modify.md)
  [`robot_set_inertia()`](https://adamrsophie.github.io/robotcompR/reference/robot_modify.md)
  [`robot_set_limits()`](https://adamrsophie.github.io/robotcompR/reference/robot_modify.md)
  : Modify a robot model
- [`robot_within_limits()`](https://adamrsophie.github.io/robotcompR/reference/joint_limits.md)
  [`robot_clamp()`](https://adamrsophie.github.io/robotcompR/reference/joint_limits.md)
  : Joint limits checks
- [`robot_random_configuration()`](https://adamrsophie.github.io/robotcompR/reference/robot_configurations.md)
  [`robot_home()`](https://adamrsophie.github.io/robotcompR/reference/robot_configurations.md)
  : Random and home configurations
- [`robot_read_urdf()`](https://adamrsophie.github.io/robotcompR/reference/robot_read_urdf.md)
  : Read a URDF robot description
- [`robot_write_urdf()`](https://adamrsophie.github.io/robotcompR/reference/robot_write_urdf.md)
  : Write a robot model as URDF

## Kinematics

- [`robot_fk()`](https://adamrsophie.github.io/robotcompR/reference/robot_fk.md)
  : Forward kinematics
- [`robot_fk_positions()`](https://adamrsophie.github.io/robotcompR/reference/robot_fk_positions.md)
  : Positions of a frame for many configurations
- [`robot_jacobian()`](https://adamrsophie.github.io/robotcompR/reference/robot_jacobian.md)
  : Geometric Jacobians
- [`robot_jacobian_numeric()`](https://adamrsophie.github.io/robotcompR/reference/robot_jacobian_numeric.md)
  : Finite-difference Jacobian check
- [`robot_jacobian_analytic()`](https://adamrsophie.github.io/robotcompR/reference/robot_jacobian_analytic.md)
  : Analytic Jacobians
- [`robot_manipulability()`](https://adamrsophie.github.io/robotcompR/reference/robot_manipulability.md)
  : Manipulability and singularity analysis
- [`robot_is_singular()`](https://adamrsophie.github.io/robotcompR/reference/robot_is_singular.md)
  : Detect singular configurations
- [`robot_ik()`](https://adamrsophie.github.io/robotcompR/reference/robot_ik.md)
  : Numerical inverse kinematics
- [`ik_planar2()`](https://adamrsophie.github.io/robotcompR/reference/ik_planar2.md)
  : Closed-form inverse kinematics of a planar two-link arm

## Dynamics and simulation

- [`robot_inverse_dynamics()`](https://adamrsophie.github.io/robotcompR/reference/robot_inverse_dynamics.md)
  : Inverse dynamics with the recursive Newton-Euler algorithm
- [`robot_mass_matrix()`](https://adamrsophie.github.io/robotcompR/reference/robot_mass_matrix.md)
  : Joint-space mass matrix with the composite rigid body algorithm
- [`robot_forward_dynamics()`](https://adamrsophie.github.io/robotcompR/reference/robot_forward_dynamics.md)
  : Forward dynamics with the articulated body algorithm
- [`robot_gravity_torque()`](https://adamrsophie.github.io/robotcompR/reference/robot_bias.md)
  [`robot_coriolis()`](https://adamrsophie.github.io/robotcompR/reference/robot_bias.md)
  [`robot_bias_forces()`](https://adamrsophie.github.io/robotcompR/reference/robot_bias.md)
  : Gravity, Coriolis and centrifugal terms
- [`robot_com()`](https://adamrsophie.github.io/robotcompR/reference/robot_energy.md)
  [`robot_energy()`](https://adamrsophie.github.io/robotcompR/reference/robot_energy.md)
  : Center of mass and mechanical energy
- [`robot_simulate()`](https://adamrsophie.github.io/robotcompR/reference/robot_simulate.md)
  : Simulate torque-driven rigid-body dynamics

## Trajectories

- [`robot_traj_polynomial()`](https://adamrsophie.github.io/robotcompR/reference/robot_traj_polynomial.md)
  : Polynomial joint trajectories
- [`robot_traj_trapezoidal()`](https://adamrsophie.github.io/robotcompR/reference/robot_traj_trapezoidal.md)
  : Trapezoidal velocity profile trajectories
- [`robot_traj_path()`](https://adamrsophie.github.io/robotcompR/reference/robot_traj_path.md)
  : Time-parameterize a geometric path
- [`robot_traj_cartesian()`](https://adamrsophie.github.io/robotcompR/reference/robot_traj_cartesian.md)
  : Task-space (Cartesian) trajectories
- [`robot_traj_check()`](https://adamrsophie.github.io/robotcompR/reference/robot_traj_check.md)
  : Check a trajectory against robot limits

## Collision checking

- [`geom_sphere()`](https://adamrsophie.github.io/robotcompR/reference/geometry.md)
  [`geom_box()`](https://adamrsophie.github.io/robotcompR/reference/geometry.md)
  [`geom_cylinder()`](https://adamrsophie.github.io/robotcompR/reference/geometry.md)
  [`geom_capsule()`](https://adamrsophie.github.io/robotcompR/reference/geometry.md)
  [`geom_convex()`](https://adamrsophie.github.io/robotcompR/reference/geometry.md)
  [`geom_mesh()`](https://adamrsophie.github.io/robotcompR/reference/geometry.md)
  : Collision and visual geometry
- [`geom_distance()`](https://adamrsophie.github.io/robotcompR/reference/geom_distance.md)
  : Distance between two geometric primitives
- [`robot_attach_geometry()`](https://adamrsophie.github.io/robotcompR/reference/robot_attach_geometry.md)
  : Attach collision geometry to a link
- [`robot_capsule_links()`](https://adamrsophie.github.io/robotcompR/reference/robot_capsule_links.md)
  : Capsule collision geometry for DH arms
- [`robot_environment()`](https://adamrsophie.github.io/robotcompR/reference/robot_environment.md)
  : Static environments
- [`robot_check_collision()`](https://adamrsophie.github.io/robotcompR/reference/robot_check_collision.md)
  : Collision checking at a configuration
- [`robot_check_path()`](https://adamrsophie.github.io/robotcompR/reference/robot_check_path.md)
  : Collision checking along a path

## Motion planning

- [`robot_planner()`](https://adamrsophie.github.io/robotcompR/reference/robot_planner.md)
  : Motion planners
- [`robot_plan()`](https://adamrsophie.github.io/robotcompR/reference/robot_plan.md)
  : Plan a path between two configurations
- [`robot_shortcut()`](https://adamrsophie.github.io/robotcompR/reference/robot_shortcut.md)
  : Shortcut smoothing of a planned path

## Mobile robots

- [`robot_unicycle()`](https://adamrsophie.github.io/robotcompR/reference/mobile_models.md)
  [`robot_diffdrive()`](https://adamrsophie.github.io/robotcompR/reference/mobile_models.md)
  [`robot_bicycle()`](https://adamrsophie.github.io/robotcompR/reference/mobile_models.md)
  : Mobile robot models
- [`unicycle_step()`](https://adamrsophie.github.io/robotcompR/reference/unicycle_step.md)
  : Closed-form unicycle motion
- [`diffdrive_forward()`](https://adamrsophie.github.io/robotcompR/reference/diffdrive.md)
  [`diffdrive_inverse()`](https://adamrsophie.github.io/robotcompR/reference/diffdrive.md)
  [`diffdrive_odometry()`](https://adamrsophie.github.io/robotcompR/reference/diffdrive.md)
  : Differential-drive kinematics and odometry
- [`mobile_simulate()`](https://adamrsophie.github.io/robotcompR/reference/mobile_simulate.md)
  : Simulate a mobile robot
- [`body_to_world_velocity()`](https://adamrsophie.github.io/robotcompR/reference/planar_velocity.md)
  [`world_to_body_velocity()`](https://adamrsophie.github.io/robotcompR/reference/planar_velocity.md)
  : Convert between body and world velocities
- [`pure_pursuit()`](https://adamrsophie.github.io/robotcompR/reference/pure_pursuit.md)
  : Pure pursuit path following

## Localization

- [`landmark_measurements()`](https://adamrsophie.github.io/robotcompR/reference/landmark_measurements.md)
  : Range-bearing landmark measurements
- [`robot_ekf()`](https://adamrsophie.github.io/robotcompR/reference/robot_ekf.md)
  [`ekf_predict()`](https://adamrsophie.github.io/robotcompR/reference/robot_ekf.md)
  [`ekf_update()`](https://adamrsophie.github.io/robotcompR/reference/robot_ekf.md)
  : Extended Kalman filter localization
- [`robot_ukf()`](https://adamrsophie.github.io/robotcompR/reference/robot_ukf.md)
  [`ukf_predict()`](https://adamrsophie.github.io/robotcompR/reference/robot_ukf.md)
  [`ukf_update()`](https://adamrsophie.github.io/robotcompR/reference/robot_ukf.md)
  : Unscented Kalman filter localization
- [`robot_particle_filter()`](https://adamrsophie.github.io/robotcompR/reference/robot_particle_filter.md)
  [`pf_predict()`](https://adamrsophie.github.io/robotcompR/reference/robot_particle_filter.md)
  [`pf_update()`](https://adamrsophie.github.io/robotcompR/reference/robot_particle_filter.md)
  [`pf_estimate()`](https://adamrsophie.github.io/robotcompR/reference/robot_particle_filter.md)
  : Particle filter localization

## Mapping and SLAM

- [`robot_occupancy_grid()`](https://adamrsophie.github.io/robotcompR/reference/robot_occupancy_grid.md)
  : Occupancy grid maps
- [`grid_world_to_cell()`](https://adamrsophie.github.io/robotcompR/reference/grid_coordinates.md)
  [`grid_cell_to_world()`](https://adamrsophie.github.io/robotcompR/reference/grid_coordinates.md)
  [`grid_probability()`](https://adamrsophie.github.io/robotcompR/reference/grid_coordinates.md)
  [`grid_occupied()`](https://adamrsophie.github.io/robotcompR/reference/grid_coordinates.md)
  : Grid coordinate conversion and probabilities
- [`grid_raycast()`](https://adamrsophie.github.io/robotcompR/reference/grid_scans.md)
  [`grid_update_scan()`](https://adamrsophie.github.io/robotcompR/reference/grid_scans.md)
  : Simulate and integrate range scans
- [`grid_inflate()`](https://adamrsophie.github.io/robotcompR/reference/grid_inflate.md)
  : Inflate obstacles
- [`grid_astar()`](https://adamrsophie.github.io/robotcompR/reference/grid_astar.md)
  : A\* search on an occupancy grid
- [`robot_ekf_slam()`](https://adamrsophie.github.io/robotcompR/reference/robot_ekf_slam.md)
  [`ekf_slam_predict()`](https://adamrsophie.github.io/robotcompR/reference/robot_ekf_slam.md)
  [`ekf_slam_update()`](https://adamrsophie.github.io/robotcompR/reference/robot_ekf_slam.md)
  [`ekf_slam_landmarks()`](https://adamrsophie.github.io/robotcompR/reference/robot_ekf_slam.md)
  : EKF-SLAM with range-bearing landmarks
- [`robot_pose_graph()`](https://adamrsophie.github.io/robotcompR/reference/robot_pose_graph.md)
  [`pose_graph_add_edge()`](https://adamrsophie.github.io/robotcompR/reference/robot_pose_graph.md)
  : Pose graphs
- [`pose_graph_optimize()`](https://adamrsophie.github.io/robotcompR/reference/pose_graph_optimize.md)
  : Optimize a pose graph

## Visualization

- [`robot_plot()`](https://adamrsophie.github.io/robotcompR/reference/robot_plot.md)
  : Plot a robot configuration
- [`robot_plot_workspace()`](https://adamrsophie.github.io/robotcompR/reference/robot_plot_workspace.md)
  : Plot the reachable workspace
