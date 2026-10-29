extends RosMsg
class_name RosPclMsgsVertices

const ROS_TYPE_NAME = "pcl_msgs/msg/Vertices"

func _init():
	init(ROS_TYPE_NAME)

var vertices : PackedInt64Array:
	get: return get_member(&"vertices")
	set(v): set_member(&"vertices", v)

