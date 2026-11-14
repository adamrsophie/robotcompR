# Denavit-Hartenberg models.

#' Denavit-Hartenberg transform of one link
#'
#' Standard convention: \eqn{T_i = R_z(\theta_i) T_z(d_i) T_x(a_i)
#' R_x(\alpha_i)}. Modified (Craig) convention: \eqn{T_i =
#' R_x(\alpha_{i-1}) T_x(a_{i-1}) R_z(\theta_i) T_z(d_i)}, where the
#' arguments `a` and `alpha` hold \eqn{a_{i-1}} and \eqn{\alpha_{i-1}}.
#'
#' @param a,alpha,d,theta DH parameters (meters and radians).
#' @param convention `"standard"` or `"modified"`.
#' @return A `robot_pose`.
#' @examples
#' dh_transform(a = 1, alpha = 0, d = 0, theta = pi / 2)
#' @export
dh_transform <- function(a, alpha, d, theta,
                         convention = c("standard", "modified")) {
  convention <- match.arg(convention)
  if (convention == "standard") {
    trotz(theta) * transl(0, 0, d) * transl(a, 0, 0) * trotx(alpha)
  } else {
    trotx(alpha) * transl(a, 0, 0) * trotz(theta) * transl(0, 0, d)
  }
}

#' Robot models from Denavit-Hartenberg parameters
#'
#' Builds a serial `robot_model` whose link frames coincide with the DH
#' frames. Link `i` is named `"link<i>"` and its frame is DH frame `i`;
#' the root is `"base"` (DH frame 0). For revolute joints `theta` is the
#' joint offset and \eqn{q_i} is added to it; for prismatic joints `d` is
#' the offset and \eqn{q_i} is added to it. Standard and modified
#' conventions are never mixed in one model; see [dh_transform()] for the
#' formulas.
#'
#' Optional inertial parameters are given per link in the DH link frame:
#' `mass` is a vector, `com` a matrix with one row per link, and `inertia`
#' a list of 3 by 3 matrices about the center of mass.
#'
#' @param a,alpha,d,theta Numeric vectors of equal length with one entry
#'   per link.
#' @param joint_type Character vector of `"revolute"` or `"prismatic"`.
#' @param convention `"standard"` or `"modified"`.
#' @param limits Optional matrix with one row `c(lower, upper)` per joint.
#' @param base Optional base pose.
#' @param tool Optional pose of a `"tool"` frame in the last link frame.
#' @param mass,com,inertia Optional inertial parameters.
#' @param name Model name.
#' @return A `robot_model` with extra field `dh` holding the parameters.
#' @examples
#' robot <- robot_dh(a = c(1, 1), alpha = c(0, 0), d = c(0, 0),
#'                   theta = c(0, 0), joint_type = c("revolute", "revolute"))
#' pose_translation(robot_fk(robot, c(pi / 2, pi / 2)))
#' @export
robot_dh <- function(a, alpha, d, theta, joint_type = "revolute",
                     convention = c("standard", "modified"), limits = NULL,
                     base = robot_pose(), tool = NULL, mass = NULL,
                     com = NULL, inertia = NULL, name = "dh_robot") {
  convention <- match.arg(convention)
  n <- length(a)
  a <- check_numeric_vector(a, n, "a")
  alpha <- check_numeric_vector(alpha, n, "alpha")
  d <- check_numeric_vector(d, n, "d")
  theta <- check_numeric_vector(theta, n, "theta")
  if (n == 0) stop("At least one link is required.", call. = FALSE)
  if (length(joint_type) == 1) joint_type <- rep(joint_type, n)
  if (length(joint_type) != n ||
      !all(joint_type %in% c("revolute", "prismatic"))) {
    stop("`joint_type` must be 'revolute' or 'prismatic' for each link.",
         call. = FALSE)
  }
  if (!is.null(limits)) limits <- check_matrix(limits, n, 2, "limits")
  links <- list(robot_link("base"))
  joints <- list()
  for (i in seq_len(n)) {
    if (convention == "standard") {
      if (joint_type[i] == "revolute") {
        origin <- robot_pose()
        post <- transl(0, 0, d[i]) * transl(a[i], 0, 0) * trotx(alpha[i])
        off <- theta[i]
      } else {
        origin <- trotz(theta[i])
        post <- transl(a[i], 0, 0) * trotx(alpha[i])
        off <- d[i]
      }
    } else {
      if (joint_type[i] == "revolute") {
        origin <- trotx(alpha[i]) * transl(a[i], 0, 0)
        post <- transl(0, 0, d[i])
        off <- theta[i]
      } else {
        origin <- trotx(alpha[i]) * transl(a[i], 0, 0) * trotz(theta[i])
        post <- robot_pose()
        off <- d[i]
      }
    }
    lk <- paste0("link", i)
    m <- if (is.null(mass)) 0 else mass[i]
    cm <- if (is.null(com)) c(0, 0, 0) else com[i, ]
    In <- if (is.null(inertia)) matrix(0, 3, 3) else inertia[[i]]
    links[[i + 1]] <- robot_link(lk, m, cm, In)
    joints[[i]] <- robot_joint(
      paste0("joint", i), joint_type[i],
      parent = if (i == 1) "base" else paste0("link", i - 1), child = lk,
      origin = origin, axis = c(0, 0, 1),
      limits = if (is.null(limits)) NULL else limits[i, ], offset = off,
      post = post)
  }
  robot <- robot_model(links, joints, base = base,
                       end_effector = paste0("link", n), name = name)
  if (!is.null(tool)) robot <- robot_add_frame(robot, "tool", paste0("link", n),
                                               tool)
  robot$dh <- data.frame(a = a, alpha = alpha, d = d, theta = theta,
                         type = joint_type, stringsAsFactors = FALSE)
  robot$dh_convention <- convention
  robot
}
