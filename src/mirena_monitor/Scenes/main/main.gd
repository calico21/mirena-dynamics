extends PanelContainer

var monitors : Array[NodeMonitor] = []

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	monitors.push_back(NodeMonitor.new($MarginContainer/HFlowContainer/Camera/ColorRect, "mirena_camera"))
	monitors.push_back(NodeMonitor.new($MarginContainer/HFlowContainer/IMU/ColorRect, "mirena_imu"))
	monitors.push_back(NodeMonitor.new($MarginContainer/HFlowContainer/Lidar/ColorRect, "mirena_lidar"))
	monitors.push_back(NodeMonitor.new($MarginContainer/HFlowContainer/CAN/ColorRect, "mirena_can"))

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	for m : NodeMonitor in monitors:
		m.update()
