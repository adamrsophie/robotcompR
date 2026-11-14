# Small catalog of validated example robots.

planar_arm <- function(lengths, masses, name) {
  n <- length(lengths)
  com <- cbind(-lengths / 2, 0, 0)
  inertia <- lapply(seq_len(n), function(i) inertia_rod(masses[i], lengths[i]))
  robot_dh(a = lengths, alpha = rep(0, n), d = rep(0, n), theta = rep(0, n),
           joint_type = "revolute", limits = cbind(rep(-pi, n), rep(pi, n)),
           mass = masses, com = com, inertia = inertia, name = name)
}

#' Example robot models
#'
#' Returns validated example models. All are serial manipulators built
#' from standard Denavit-Hartenberg parameters, with link frames at the
#' distal end of each link.
#'
#' * `"planar1"`: one revolute link of length 1 m, a uniform 1 kg rod.
#' * `"planar2"`: two revolute links of length 1 m, uniform 1 kg rods,
#'   limits \eqn{[-\pi, \pi]}. End effector \eqn{(\cos q_1 +
#'   \cos(q_1 + q_2), \sin q_1 + \sin(q_1 + q_2))}.
#' * `"planar3"`: three revolute links of lengths 1, 0.8 and 0.6 m with
#'   masses 1, 0.8 and 0.6 kg.
#' * `"puma560"`: kinematic model of the PUMA 560 arm using the standard
#'   DH parameters and joint limits published with the Robotics Toolbox
#'   (Corke, 2017, *Robotics, Vision and Control*, 2nd ed., Springer).
#'   No inertial parameters are included.
#' * `"branched"`: a five-link tree with two fingers on a revolute wrist,
#'   including a fixed joint, used to demonstrate branched kinematics.
#'
#' @param name Model name.
#' @return A `robot_model`.
#' @examples
#' robot_example("planar2")
#' robot_example("puma560")
#' @export
robot_example <- function(name = c("planar2", "planar1", "planar3", "puma560",
                                   "branched")) {
  name <- match.arg(name)
  switch(name,
    planar1 = planar_arm(1, 1, "planar1"),
    planar2 = planar_arm(c(1, 1), c(1, 1), "planar2"),
    planar3 = planar_arm(c(1, 0.8, 0.6), c(1, 0.8, 0.6), "planar3"),
    puma560 = robot_dh(
      a = c(0, 0.4318, 0.0203, 0, 0, 0),
      alpha = c(pi / 2, 0, -pi / 2, pi / 2, -pi / 2, 0),
      d = c(0, 0, 0.15005, 0.4318, 0, 0),
      theta = rep(0, 6), joint_type = "revolute",
      limits = deg2rad(rbind(c(-160, 160), c(-45, 225), c(-225, 45),
                             c(-110, 170), c(-100, 100), c(-266, 266))),
      name = "puma560"),
    branched = branched_example()
  )
}

branched_example <- function() {
  links <- list(
    robot_link("base"),
    robot_link("arm", 1, c(0.25, 0, 0), inertia_rod(1, 0.5)),
    robot_link("wrist", 0.2, c(0.05, 0, 0), inertia_box(0.2, c(0.1, 0.05, 0.05))),
    robot_link("palm"),
    robot_link("finger_left", 0.05, c(0.05, 0, 0), inertia_rod(0.05, 0.1)),
    robot_link("finger_right", 0.05, c(0.05, 0, 0), inertia_rod(0.05, 0.1))
  )
  joints <- list(
    robot_joint("shoulder", "revolute", "base", "arm", axis = c(0, 0, 1),
                limits = c(-pi, pi)),
    robot_joint("elbow", "revolute", "arm", "wrist", origin = transl(0.5, 0, 0),
                axis = c(0, 0, 1), limits = c(-2.5, 2.5)),
    robot_joint("palm_mount", "fixed", "wrist", "palm",
                origin = transl(0.1, 0, 0)),
    robot_joint("finger_left_joint", "revolute", "palm", "finger_left",
                origin = transl(0, 0.03, 0), axis = c(0, 0, 1),
                limits = c(0, 1)),
    robot_joint("finger_right_joint", "revolute", "palm", "finger_right",
                origin = transl(0, -0.03, 0), axis = c(0, 0, 1),
                limits = c(-1, 0))
  )
  robot_model(links, joints, end_effector = "finger_left", name = "branched")
}
