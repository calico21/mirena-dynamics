extends SceneTree

const FloatingGraphManager = preload("res://Scenes/Sim/UI/FloatingGraphManager.gd")
const FloatingGraphPanel = preload("res://Scenes/Sim/UI/FloatingGraphPanel.gd")

var _sim_scene: Node = null
var _frame_count: int = 0
var _stage: int = 0
var _wait_frames: int = 0

const ARTIFACT_DIR: String = "/home/alexr/.gemini/antigravity/brain/2dfdaec3-03ac-438c-a5b0-40ab20b7e2e3"

func _init() -> void:
	print("[Capture] Starting Sequential Screenshot Capture Script...")

func _process(_delta: float) -> bool:
	_frame_count += 1
	
	if _sim_scene == null:
		var sim_res := load("res://Scenes/Sim/Sim.tscn") as PackedScene
		_sim_scene = sim_res.instantiate()
		root.add_child(_sim_scene)
		print("[Capture] Sim.tscn instantiated.")
		return false
	
	var car: MirenaCar = root.find_child("Car", true, false) as MirenaCar
	var graph_mgr: FloatingGraphManager = root.find_child("FloatingGraphManager", true, false) as FloatingGraphManager
	
	# Drive car during first 100 frames to build real speed, acceleration, and yaw
	if _frame_count < 100 and car != null:
		car.pilot = MirenaCar.PilotMode.MANUAL
		car.gas = 0.70
		if _frame_count > 30:
			car.steering = 0.14
		return false
	
	if _wait_frames > 0:
		_wait_frames -= 1
		return false
	
	match _stage:
		0:
			# Stage 0: Clean Main HUD without floating graphs
			print("[Capture] Setting up Stage 0 (Clean Top HUD)...")
			if graph_mgr != null:
				graph_mgr.set_all_graphs_visible(false)
			_wait_frames = 5
			_stage = 1
		1:
			_save_screenshot(ARTIFACT_DIR + "/simulator_hud_redesigned.png")
			print("[Capture] Stage 0 captured: simulator_hud_redesigned.png")
			
			# Setup Stage 1: Load Vehicle Dynamics Preset + Add Live Speedometer Gauge
			print("[Capture] Setting up Stage 1 (Multiple Floating Graphs)...")
			if graph_mgr != null:
				graph_mgr.load_preset_vehicle_dynamics()
				graph_mgr.create_graph({
					"id": "gauge_speed",
					"title": "Vehicle Speed",
					"type": FloatingGraphPanel.GraphType.GAUGE,
					"y_channels": ["chassis.speed_kmh"],
					"auto_scale_y": false,
					"manual_min_y": 0.0,
					"manual_max_y": 140.0,
					"pos_x": 400.0,
					"pos_y": 320.0,
					"size_w": 280.0,
					"size_h": 220.0
				})
				graph_mgr.set_all_graphs_visible(true)
			_wait_frames = 20 # Let graphs render a few sample frames
			_stage = 2
		2:
			_save_screenshot(ARTIFACT_DIR + "/simulator_floating_graphs.png")
			print("[Capture] Stage 1 captured: simulator_floating_graphs.png")
			
			# Setup Stage 2: Open Configuration Dialog on first panel
			print("[Capture] Setting up Stage 2 (Graph Configuration Dialog)...")
			if graph_mgr != null and not graph_mgr.panels.is_empty():
				graph_mgr._on_panel_configure_requested(graph_mgr.panels[0])
			_wait_frames = 8
			_stage = 3
		3:
			_save_screenshot(ARTIFACT_DIR + "/simulator_graph_config.png")
			print("[Capture] Stage 2 captured: simulator_graph_config.png")
			
			# Setup Stage 3: Close dialog, collapse first panel
			print("[Capture] Setting up Stage 3 (Collapsed Graph Panel)...")
			if graph_mgr != null:
				if graph_mgr.config_dialog != null:
					graph_mgr.config_dialog.visible = false
				if not graph_mgr.panels.is_empty():
					graph_mgr.panels[0]._toggle_collapse()
			_wait_frames = 8
			_stage = 4
		4:
			_save_screenshot(ARTIFACT_DIR + "/simulator_graph_collapsed.png")
			print("[Capture] Stage 3 captured: simulator_graph_collapsed.png")
			print("[Capture] ALL 4 SCREENSHOTS COMPLETED SUCCESSFULLY!")
			quit(0)
			return true
	
	return false

func _save_screenshot(dest_path: String) -> void:
	var img: Image = root.get_viewport().get_texture().get_image()
	if img != null:
		img.save_png(dest_path)
		var report_dir: String = "/home/alexr/eRacing/mirena_ws-SIM/imu_compare/reports/"
		DirAccess.make_dir_absolute(report_dir)
		img.save_png(report_dir + dest_path.get_file())
