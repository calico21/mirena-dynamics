extends RosMsg
class_name RosDiagnosticMsgsDiagnosticArray

const ROS_TYPE_NAME = "diagnostic_msgs/msg/DiagnosticArray"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var status : Array[RosDiagnosticMsgsDiagnosticStatus]:
	get: return get_member(&"status")
	set(v): set_member(&"status", v)

