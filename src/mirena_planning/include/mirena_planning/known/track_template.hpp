// Surveyed track layouts for the missions whose geometry is known in advance.

#pragma once

#include <string>
#include <vector>

#include "mirena_planning/planner.hpp"

namespace mirena::planning
{

/// A track as it is drawn in `resource/<mission>.json`: cones and the racing
/// line, both in the template's own frame.
struct TrackTemplate
{
  std::string name;
  std::vector<Observation> cones;
  std::vector<Vec2> path;
  bool closed{false};

  /// Nominal template -> map transform, derived from `setup.car_start_pose`.
  ///
  /// SLAM anchors the map frame at the pose the car started from, and the JSON
  /// states where that pose sits in the template, so this transform is known
  /// before a single cone is seen. Registration only has to correct how
  /// accurately the car was actually placed on the line.
  Transform2 map_from_template;
};

/// Loads a template from JSON. Throws std::runtime_error naming the offending
/// file and field on anything malformed.
TrackTemplate loadTrackTemplate(const std::string & path);

}  // namespace mirena::planning
