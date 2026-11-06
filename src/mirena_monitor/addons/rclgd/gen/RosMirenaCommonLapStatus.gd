extends RosMsg
class_name RosMirenaCommonLapStatus

const ROS_TYPE_NAME = "mirena_common/msg/LapStatus"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header") as RosMsg
	set(v): set_member(&"header", v)

var cones_hit : int:
	get: return get_member(&"cones_hit")
	set(v): set_member(&"cones_hit", v)

var lap_time : float:
	get: return get_member(&"lap_time")
	set(v): set_member(&"lap_time", v)
