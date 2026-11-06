#include "mirena_planning/known/track_template.hpp"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace mirena::planning
{

namespace
{

[[noreturn]] void fail(const std::string & path, const std::string & what)
{
  throw std::runtime_error("track template '" + path + "': " + what);
}

}  // namespace

TrackTemplate loadTrackTemplate(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {fail(path, "cannot be opened");}

  nlohmann::json json;
  try {
    file >> json;
  } catch (const nlohmann::json::exception & e) {
    fail(path, std::string("is not valid JSON (") + e.what() + ")");
  }

  TrackTemplate track;
  track.name = json.value("/metadata/track_name"_json_pointer, std::string{});

  for (const auto & cone : json.value("cones", nlohmann::json::array())) {
    if (!cone.contains("x") || !cone.contains("y")) {fail(path, "a cone is missing x/y");}
    track.cones.push_back(
      Observation{
        {cone["x"].get<double>(), cone["y"].get<double>()},
        coneSizeFromString(cone.value("type", std::string{}))});
  }

  const auto & path_json = json["path"];
  if (!path_json.is_object()) {fail(path, "has no 'path' object");}
  for (const auto & point : path_json.value("points", nlohmann::json::array())) {
    if (!point.contains("x") || !point.contains("y")) {fail(path, "a path point is missing x/y");}
    track.path.push_back({point["x"].get<double>(), point["y"].get<double>()});
  }
  track.closed = path_json.value("closed", false);

  if (track.cones.size() < 2) {fail(path, "needs at least two cones to register against");}
  if (track.path.size() < 2) {fail(path, "needs at least two path points");}

  const auto & start = json["setup"]["car_start_pose"];
  if (!start.is_object()) {fail(path, "has no 'setup.car_start_pose'");}
  const Transform2 template_from_map{
    start.value("psi", 0.0),
    {start.value("x", 0.0), start.value("y", 0.0)}};
  track.map_from_template = template_from_map.inverse();

  return track;
}

}  // namespace mirena::planning
