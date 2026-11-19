# Write a robot model as URDF

Exports links (inertial parameters and geometry), joints and limits.
URDF has no joint offset or post-motion transform, so the export keeps
the kinematics exact by folding an offset into the joint origin and by
splitting a joint with a post transform into the moving joint, a
massless intermediate link named `<child>_motion`, and a fixed joint
named `<joint>_post` (a numeric suffix is added if a name is taken). The
extra joints can change the breadth-first order of the movable joints;
when the reimported configuration order differs, a warning gives both
orders. Names are XML-escaped.

## Usage

``` r
robot_write_urdf(robot, file = NULL, fill_limits = NULL)
```

## Arguments

- robot:

  A `robot_model`.

- file:

  Output path, or `NULL` to return the XML string.

- fill_limits:

  `NULL` (the default) to require every revolute and prismatic joint,
  and every continuous joint with one of the two, to carry effort and
  velocity limits, or `c(effort = , velocity = )` giving the values
  written for joints that leave them unspecified. These values are read
  back literally.

## Value

The URDF text, invisibly when written to a file.

## Details

Limits keep their literal meaning: URDF requires `effort` and `velocity`
on every revolute and prismatic joint, and a value of 0 forbids motion
for every URDF reader. When a joint leaves them unspecified (`NULL`),
the export stops unless `fill_limits` supplies the values to write.
Joint damping and friction are written as a `<dynamics>` element,
although the dynamics functions do not apply them. Forward kinematics of
every original frame is preserved; the exported model has extra frames.
Convex-hull geometry has no URDF equivalent and is omitted with a
warning. Capsules are a URDF 1.1 primitive: a model with capsules is
written with `version="1.1"`, which parsers that only know URDF 1.0
reject; replace capsules (for example with cylinders) for such readers.
Numbers are written with enough digits to read back exactly, and the
export stops if the written text does not read back. A continuous joint
may omit both effort and velocity, but if it has only one of them the
other must come from `fill_limits`.

## Examples

``` r
robot <- robot_example("planar2")
txt <- robot_write_urdf(robot, fill_limits = c(effort = 50, velocity = 5))
back <- robot_read_urdf(txt)
pose_translation(robot_fk(back, c(pi / 2, pi / 2), "link2"))
#> [1] -1  1  0
```
