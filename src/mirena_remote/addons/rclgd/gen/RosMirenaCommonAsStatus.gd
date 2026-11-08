extends RosMsg
class_name RosMirenaCommonAsStatus

const ROS_TYPE_NAME = "mirena_common/msg/ASStatus"

func _init():
	init(ROS_TYPE_NAME)

var as_status : int:
	get: return get_member(&"as_status")
	set(v): set_member(&"as_status", v)

var mission_selected : String:
	get: return get_member(&"mission_selected")
	set(v): set_member(&"mission_selected", v)

