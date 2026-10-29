extends RosMsg
class_name RosMirenaCommonMissionStatus

const ROS_TYPE_NAME = "mirena_common/msg/MissionStatus"

func _init():
	init(ROS_TYPE_NAME)

var mission_status : int:
	get: return get_member(&"mission_status")
	set(v): set_member(&"mission_status", v)

