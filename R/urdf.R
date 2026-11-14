# URDF import and export.

parse_xyz <- function(s, default = c(0, 0, 0), what = "xyz") {
  if (is.na(s) || !nzchar(trimws(s))) return(default)
  v <- suppressWarnings(as.numeric(strsplit(trimws(s), "[[:space:]]+")[[1]]))
  if (length(v) != length(default) || anyNA(v) || any(!is.finite(v))) {
    stop(sprintf("Malformed URDF attribute %s = '%s'.", what, s), call. = FALSE)
  }
  v
}

parse_num <- function(s, default = NA_real_, what = "value") {
  if (is.na(s) || !nzchar(trimws(s))) return(default)
  v <- suppressWarnings(as.numeric(trimws(s)))
  if (is.na(v)) stop(sprintf("Malformed URDF number %s = '%s'.", what, s),
                     call. = FALSE)
  v
}

urdf_origin <- function(node) {
  o <- xml2::xml_find_first(node, "./origin")
  if (inherits(o, "xml_missing")) return(robot_pose())
  xyz <- parse_xyz(xml2::xml_attr(o, "xyz"), what = "origin xyz")
  rpy <- parse_xyz(xml2::xml_attr(o, "rpy"), what = "origin rpy")
  robot_pose(euler_to_matrix(rev(rpy), "ZYX"), xyz)
}

urdf_geometry <- function(node, origin) {
  g <- xml2::xml_find_first(node, "./geometry")
  if (inherits(g, "xml_missing")) {
    stop("URDF collision or visual element without <geometry>.", call. = FALSE)
  }
  kids <- xml2::xml_children(g)
  if (length(kids) != 1) {
    stop("URDF <geometry> must contain exactly one shape.", call. = FALSE)
  }
  s <- kids[[1]]
  switch(xml2::xml_name(s),
    box = geom_box(parse_xyz(xml2::xml_attr(s, "size"), what = "box size"),
                   origin),
    sphere = geom_sphere(parse_num(xml2::xml_attr(s, "radius"),
                                   what = "sphere radius"), origin),
    cylinder = geom_cylinder(parse_num(xml2::xml_attr(s, "radius"),
                                       what = "cylinder radius"),
                             parse_num(xml2::xml_attr(s, "length"),
                                       what = "cylinder length"), origin),
    capsule = geom_capsule(parse_num(xml2::xml_attr(s, "radius"),
                                     what = "capsule radius"),
                           parse_num(xml2::xml_attr(s, "length"),
                                     what = "capsule length"), origin),
    mesh = geom_mesh(xml2::xml_attr(s, "filename"),
                     parse_xyz(xml2::xml_attr(s, "scale"), c(1, 1, 1),
                               "mesh scale"), origin),
    stop(sprintf("Unsupported URDF geometry <%s>.", xml2::xml_name(s)),
         call. = FALSE))
}

#' Read a URDF robot description
#'
#' Parses a URDF file or string with 'xml2' and builds a validated
#' [robot_model()]. Supported: links with inertial parameters, collision
#' and visual geometry (box, sphere, cylinder, capsule, and mesh references
#' kept as metadata), and revolute, continuous, prismatic and fixed joints
#' with origins, axes (default `1 0 0` as in the URDF specification) and
#' limits. The inertia tensor is rotated from the `<inertial><origin>`
#' frame into the link frame.
#'
#' Features that change kinematics or dynamics but are not modeled are
#' rejected with an error: `<mimic>`, and `floating` and `planar` joints.
#' Joint `<dynamics>` damping and friction values are stored on the joint
#' but not used by the dynamics functions; a warning says so.
#'
#' Input is treated as untrusted: xacro files are refused (expand them
#' with ROS tools first), documents declaring entities are refused,
#' network access is disabled in the XML parser, the file content is
#' parsed as XML text (never followed as a further path or URL), and mesh
#' files are never opened. Every link named by a joint must be declared.
#' A `velocity` or `effort` limit of 0, the usual placeholder for a
#' required URDF attribute, is read as unspecified.
#'
#' @param file Path to a URDF file, or a character string with URDF XML.
#' @param end_effector Optional end-effector link name.
#' @return A `robot_model`.
#' @examples
#' robot <- robot_read_urdf(system.file("extdata", "planar2.urdf",
#'                                      package = "robotcompR"))
#' robot_fk(robot, c(pi / 2, pi / 2))
#' @export
robot_read_urdf <- function(file, end_effector = NULL) {
  file <- check_string(file, "file")
  is_text <- grepl("<robot", file, fixed = TRUE)
  if (!is_text) {
    if (grepl("\\.xacro$", file, ignore.case = TRUE)) {
      stop("xacro files are not expanded; run xacro first.", call. = FALSE)
    }
    if (!file.exists(file)) stop(sprintf("File '%s' not found.", file),
                                 call. = FALSE)
    txt <- paste(readLines(file, warn = FALSE), collapse = "\n")
  } else {
    txt <- file
  }
  if (grepl("xmlns:xacro|<xacro:", txt)) {
    stop("The description uses xacro macros; expand it with xacro first.",
         call. = FALSE)
  }
  if (grepl("<!ENTITY", txt, fixed = TRUE)) {
    stop("URDF with DTD entity declarations is refused.", call. = FALSE)
  }
  # Parse the text as raw bytes: given a plain string without markup, xml2
  # would treat it as a path or URL and open it.
  doc <- xml2::read_xml(charToRaw(enc2utf8(txt)), options = c("NONET", "NOBLANKS"))
  root <- if (xml2::xml_name(doc) == "robot") doc
          else xml2::xml_find_first(doc, "//robot")
  if (inherits(root, "xml_missing")) {
    stop("No <robot> element found.", call. = FALSE)
  }
  rname <- xml2::xml_attr(root, "name")
  if (is.na(rname)) rname <- "urdf_robot"

  link_nodes <- xml2::xml_find_all(root, "./link")
  links <- lapply(link_nodes, function(ln) {
    nm <- xml2::xml_attr(ln, "name")
    if (is.na(nm)) stop("URDF <link> without a name.", call. = FALSE)
    inl <- xml2::xml_find_first(ln, "./inertial")
    if (inherits(inl, "xml_missing")) {
      lk <- robot_link(nm)
    } else {
      o <- urdf_origin(inl)
      mass <- parse_num(xml2::xml_attr(xml2::xml_find_first(inl, "./mass"),
                                       "value"), 0, "mass")
      inn <- xml2::xml_find_first(inl, "./inertia")
      gv <- function(a) parse_num(xml2::xml_attr(inn, a), 0, a)
      I <- if (inherits(inn, "xml_missing")) matrix(0, 3, 3) else
        matrix(c(gv("ixx"), gv("ixy"), gv("ixz"), gv("ixy"), gv("iyy"),
                 gv("iyz"), gv("ixz"), gv("iyz"), gv("izz")), 3, 3)
      R <- pose_rotation_matrix(o)
      lk <- robot_link(nm, mass, pose_translation_vector(o), R %*% I %*% t(R))
    }
    for (cn in xml2::xml_find_all(ln, "./collision")) {
      lk$collision[[length(lk$collision) + 1]] <-
        urdf_geometry(cn, urdf_origin(cn))
    }
    for (vn in xml2::xml_find_all(ln, "./visual")) {
      lk$visual[[length(lk$visual) + 1]] <- urdf_geometry(vn, urdf_origin(vn))
    }
    lk
  })

  joint_nodes <- xml2::xml_find_all(root, "./joint")
  warned_dyn <- FALSE
  joints <- lapply(joint_nodes, function(jn) {
    nm <- xml2::xml_attr(jn, "name")
    type <- xml2::xml_attr(jn, "type")
    if (is.na(nm) || is.na(type)) {
      stop("URDF <joint> needs name and type attributes.", call. = FALSE)
    }
    if (type %in% c("floating", "planar")) {
      stop(sprintf("Joint '%s' has unsupported type '%s'.", nm, type),
           call. = FALSE)
    }
    if (!type %in% joint_types) {
      stop(sprintf("Joint '%s' has unknown type '%s'.", nm, type),
           call. = FALSE)
    }
    if (!inherits(xml2::xml_find_first(jn, "./mimic"), "xml_missing")) {
      stop(sprintf("Joint '%s' uses <mimic>, which is not supported.", nm),
           call. = FALSE)
    }
    parent <- xml2::xml_attr(xml2::xml_find_first(jn, "./parent"), "link")
    child <- xml2::xml_attr(xml2::xml_find_first(jn, "./child"), "link")
    if (is.na(parent) || is.na(child)) {
      stop(sprintf("Joint '%s' needs <parent link> and <child link>.", nm),
           call. = FALSE)
    }
    ax <- xml2::xml_find_first(jn, "./axis")
    axis <- if (inherits(ax, "xml_missing")) c(1, 0, 0)
            else parse_xyz(xml2::xml_attr(ax, "xyz"), c(1, 0, 0), "axis xyz")
    lim <- xml2::xml_find_first(jn, "./limit")
    limits <- NULL
    vel <- NULL
    eff <- NULL
    if (!inherits(lim, "xml_missing")) {
      vel <- parse_num(xml2::xml_attr(lim, "velocity"), NULL, "velocity")
      eff <- parse_num(xml2::xml_attr(lim, "effort"), NULL, "effort")
      # URDF requires these attributes, so 0 is the common placeholder for
      # "not specified"; a literal zero limit would forbid any motion.
      if (!is.null(vel) && vel == 0) vel <- NULL
      if (!is.null(eff) && eff == 0) eff <- NULL
      if (type %in% c("revolute", "prismatic")) {
        limits <- c(parse_num(xml2::xml_attr(lim, "lower"), 0, "lower"),
                    parse_num(xml2::xml_attr(lim, "upper"), 0, "upper"))
      }
    } else if (type %in% c("revolute", "prismatic")) {
      stop(sprintf("Joint '%s' of type '%s' needs a <limit> element.", nm,
                   type), call. = FALSE)
    }
    j <- robot_joint(nm, type, parent, child, origin = urdf_origin(jn),
                     axis = axis, limits = limits, velocity = vel,
                     effort = eff)
    dyn <- xml2::xml_find_first(jn, "./dynamics")
    if (!inherits(dyn, "xml_missing")) {
      j$damping <- parse_num(xml2::xml_attr(dyn, "damping"), 0, "damping")
      j$friction <- parse_num(xml2::xml_attr(dyn, "friction"), 0, "friction")
      if ((j$damping != 0 || j$friction != 0) && !warned_dyn) {
        warned_dyn <<- TRUE
        warning("Joint damping and friction were read but are not modeled ",
                "by the dynamics functions.", call. = FALSE)
      }
    }
    j
  })
  if (!length(joints)) {
    stop("The URDF has no joints.", call. = FALSE)
  }
  declared <- vapply(links, `[[`, "", "name")
  used <- unique(c(vapply(joints, `[[`, "", "parent"),
                   vapply(joints, `[[`, "", "child")))
  undeclared <- setdiff(used, declared)
  if (length(undeclared)) {
    stop(sprintf("Joints refer to undeclared link(s): %s.",
                 paste(undeclared, collapse = ", ")), call. = FALSE)
  }
  robot_model(links, joints, end_effector = end_effector, name = rname)
}

fmt_num <- function(x) {
  x[abs(x) < 1e-15] <- 0
  paste(formatC(x, digits = 15, format = "g"), collapse = " ")
}

# Escape a value for use inside a double-quoted XML attribute.
xml_attr_escape <- function(x) {
  x <- gsub("&", "&amp;", x, fixed = TRUE)
  x <- gsub("<", "&lt;", x, fixed = TRUE)
  x <- gsub(">", "&gt;", x, fixed = TRUE)
  x <- gsub('"', "&quot;", x, fixed = TRUE)
  gsub("'", "&apos;", x, fixed = TRUE)
}

# A name not in `taken`, built from `base` with a numeric suffix if needed.
unique_name <- function(base, taken) {
  nm <- base
  i <- 1L
  while (nm %in% taken) {
    nm <- paste0(base, "_", i)
    i <- i + 1L
  }
  nm
}

urdf_origin_xml <- function(M) {
  rpy <- rev(matrix_to_euler(M[1:3, 1:3], "ZYX"))
  sprintf('<origin xyz="%s" rpy="%s"/>', fmt_num(M[1:3, 4]), fmt_num(rpy))
}

urdf_geometry_xml <- function(g, tag) {
  shape <- switch(g$type,
    sphere = sprintf('<sphere radius="%s"/>', fmt_num(g$params$radius)),
    box = sprintf('<box size="%s"/>', fmt_num(g$params$size)),
    cylinder = sprintf('<cylinder radius="%s" length="%s"/>',
                       fmt_num(g$params$radius), fmt_num(g$params$length)),
    capsule = sprintf('<capsule radius="%s" length="%s"/>',
                      fmt_num(g$params$radius), fmt_num(g$params$length)),
    mesh = sprintf('<mesh filename="%s" scale="%s"/>',
                   xml_attr_escape(g$params$filename),
                   fmt_num(g$params$scale)),
    convex = NULL)
  if (is.null(shape)) return(character(0))
  sprintf("    <%s>%s<geometry>%s</geometry></%s>", tag,
          urdf_origin_xml(g$origin), shape, tag)
}

#' Write a robot model as URDF
#'
#' Exports links (inertial parameters and geometry), joints and limits.
#' URDF has no joint offset or post-motion transform, so the export keeps
#' the kinematics exact by folding an offset into the joint origin and by
#' splitting a joint with a post transform into the moving joint, a
#' massless intermediate link named `<child>_motion`, and a fixed joint
#' named `<joint>_post` (a numeric suffix is added if a name is taken).
#' The extra joints can change the breadth-first order of the movable
#' joints; when the reimported configuration order differs, a warning
#' gives both orders. Names are XML-escaped. Unspecified velocity and
#' effort limits are written as 0, which [robot_read_urdf()] reads back as
#' unspecified.
#' Forward kinematics of every original frame is preserved; the exported
#' model has extra frames. Convex-hull geometry has no URDF equivalent and
#' is omitted with a warning.
#'
#' @param robot A `robot_model`.
#' @param file Output path, or `NULL` to return the XML string.
#' @return The URDF text, invisibly when written to a file.
#' @examples
#' robot <- robot_example("planar2")
#' txt <- robot_write_urdf(robot)
#' back <- robot_read_urdf(txt)
#' pose_translation(robot_fk(back, c(pi / 2, pi / 2), "link2"))
#' @export
robot_write_urdf <- function(robot, file = NULL) {
  check_robot(robot)
  esc <- xml_attr_escape
  out <- sprintf('<?xml version="1.0"?>\n<robot name="%s">', esc(robot$name))
  extra_links <- character(0)
  taken <- c(robot$link_names, robot$joint_names)
  convex_dropped <- FALSE
  if (max(abs(robot$base - diag(4))) > 0) {
    warning("The base pose is not part of URDF and is not exported.",
            call. = FALSE)
  }
  for (lk in robot$links) {
    body <- character(0)
    if (lk$mass > 0) {
      I <- lk$inertia
      body <- c(body, sprintf(paste0(
        '    <inertial><origin xyz="%s" rpy="0 0 0"/><mass value="%s"/>',
        '<inertia ixx="%s" ixy="%s" ixz="%s" iyy="%s" iyz="%s" izz="%s"/>',
        '</inertial>'), fmt_num(lk$com), fmt_num(lk$mass), fmt_num(I[1, 1]),
        fmt_num(I[1, 2]), fmt_num(I[1, 3]), fmt_num(I[2, 2]), fmt_num(I[2, 3]),
        fmt_num(I[3, 3])))
    }
    for (g in lk$visual) body <- c(body, urdf_geometry_xml(g, "visual"))
    for (g in lk$collision) {
      if (g$type == "convex") convex_dropped <- TRUE
      body <- c(body, urdf_geometry_xml(g, "collision"))
    }
    out <- c(out, sprintf('  <link name="%s">', esc(lk$name)), body, "  </link>")
  }
  for (j in robot$joints) {
    origin <- j$origin
    if (j$type != "fixed" && j$offset != 0) {
      origin <- origin %*% joint_motion(j, 0)
    }
    child <- j$child
    if (!is.null(j$post)) {
      child <- unique_name(paste0(j$child, "_motion"), taken)
      post_name <- unique_name(paste0(j$name, "_post"), c(taken, child))
      taken <- c(taken, child, post_name)
    }
    lim <- ""
    if (j$type %in% c("revolute", "prismatic", "continuous")) {
      lo <- j$limits[1]
      hi <- j$limits[2]
      if (j$type != "continuous" && (!is.finite(lo) || !is.finite(hi))) {
        stop(sprintf(paste0("Joint '%s' has infinite limits; URDF needs ",
                            "finite limits or type continuous."), j$name),
             call. = FALSE)
      }
      # URDF requires effort and velocity; 0 marks them as unspecified and
      # is read back as unspecified by robot_read_urdf().
      lim <- sprintf('<limit%s effort="%s" velocity="%s"/>',
                     if (j$type == "continuous") ""
                     else sprintf(' lower="%s" upper="%s"', fmt_num(lo),
                                  fmt_num(hi)),
                     fmt_num(j$effort %||% 0), fmt_num(j$velocity %||% 0))
    }
    out <- c(out, sprintf(
      '  <joint name="%s" type="%s"><parent link="%s"/><child link="%s"/>%s%s%s</joint>',
      esc(j$name), j$type, esc(j$parent), esc(child), urdf_origin_xml(origin),
      if (j$type == "fixed") "" else sprintf('<axis xyz="%s"/>', fmt_num(j$axis)),
      lim))
    if (!is.null(j$post)) {
      extra_links <- c(extra_links, child)
      out <- c(out, sprintf(
        '  <joint name="%s" type="fixed"><parent link="%s"/><child link="%s"/>%s</joint>',
        esc(post_name), esc(child), esc(j$child), urdf_origin_xml(j$post)))
    }
  }
  for (nm in extra_links) out <- c(out, sprintf('  <link name="%s"/>', esc(nm)))
  out <- c(out, "</robot>")
  if (convex_dropped) {
    warning("Convex-hull geometry has no URDF form and was omitted.",
            call. = FALSE)
  }
  txt <- paste(out, collapse = "\n")
  # Extra fixed joints change the breadth-first order of the tree, which can
  # reorder the configuration vector of the reimported model.
  back <- tryCatch(suppressWarnings(robot_read_urdf(txt)), error = function(e) NULL)
  if (!is.null(back) && !identical(robot_joint_names(back), robot_joint_names(robot))) {
    warning(sprintf(paste0("The exported model orders its configuration as ",
                           "%s instead of %s; reorder q accordingly."),
                    paste(robot_joint_names(back), collapse = ", "),
                    paste(robot_joint_names(robot), collapse = ", ")),
            call. = FALSE)
  }
  if (is.null(file)) return(txt)
  writeLines(txt, file)
  invisible(txt)
}
