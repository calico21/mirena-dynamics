# Static (non moving) entity in 3d space
geometry_msgs/Point position
string type
float64 confidence

# How well the position is known, as a 2x2 row-major covariance in the x-y plane
# of the header frame: [xx, xy, yx, yy].
#
# A cone measured by a spinning LiDAR is not equally uncertain in every
# direction -- it is known well along the beam and poorly across it, and the
# cross-beam error grows with range. Publishing that lets whoever consumes this
# weight a cone at 25 m for what it is worth instead of trusting it like one at
# 3 m. All zeros means the producer has no estimate, and the consumer should
# fall back to its own default rather than believe the cone is perfect.
float64[4] covariance
