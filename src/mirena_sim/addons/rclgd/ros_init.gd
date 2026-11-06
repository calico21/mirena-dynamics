extends Node

static func _static_init() -> void:
	if Engine.is_editor_hint(): return
	if rclgd:
		var args := OS.get_cmdline_args()
		# Read ROS parameters from project.godot [ros] section and pass as ROS2 args.
		# Command-line args (from launch files) take precedence over project settings.
		var params := {
			"publish_sim_time": ProjectSettings.get_setting("ros/parameters/publish_sim_time", false),
			"use_sim_time": ProjectSettings.get_setting("ros/parameters/use_sim_time", false),
			"use_separate_thread": ProjectSettings.get_setting("ros/parameters/use_separate_thread", true),
		}
		for key in params:
			var already_set := false
			for i in range(args.size()):
				if args[i].begins_with("%s:=" % key):
					already_set = true
					break
			if not already_set:
				args.append("--ros-args")
				args.append("-p")
				args.append("%s:=%s" % [key, str(params[key]).to_lower()])
		rclgd.init(args)
		print("[ROS Init Args]: ", args)

func _exit_tree() -> void:
	if rclgd and rclgd.ok():
		rclgd.shutdown()
		print("[ROS] Shutdown called from exit_tree.")
