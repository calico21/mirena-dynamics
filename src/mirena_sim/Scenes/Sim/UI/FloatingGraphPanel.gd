extends PanelContainer
class_name FloatingGraphPanel

const MirenaUITheme = preload("res://Scenes/Sim/UI/MirenaUITheme.gd")
const TelemetrySignalRegistry = preload("res://Scenes/Sim/UI/TelemetrySignalRegistry.gd")

## Independent translucent floating telemetry graph panel for MirenaSim.
## Styled in exact visual alignment with the Options Menu design system.
## Supports:
## - 4 graph types: TIME_SERIES, XY_PLOT, HISTOGRAM, GAUGE
## - Dragging by title bar
## - Fluid interactive resizing via bottom-right handle
## - Minimize / expand collapse toggle
## - Multi-signal overlay with auto/manual axis scaling
## - Bounded circular ring buffers for high simulation performance

signal closed(panel: FloatingGraphPanel)
signal configure_requested(panel: FloatingGraphPanel)
signal layout_changed(panel: FloatingGraphPanel)

enum GraphType {
	TIME_SERIES = 0,
	XY_PLOT = 1,
	HISTOGRAM = 2,
	GAUGE = 3,
}

const TYPE_NAMES: Dictionary = {
	GraphType.TIME_SERIES: "Time Series",
	GraphType.XY_PLOT: "XY Plot",
	GraphType.HISTOGRAM: "Distribution",
	GraphType.GAUGE: "Live Gauge",
}

# --- Configuration & State ---
var graph_id: String = ""
var graph_title: String = "Telemetry Graph"
var graph_type: int = GraphType.TIME_SERIES
var x_channel: String = "time" # "time" for time-series, or signal ID for XY
var y_channels: Array[String] = ["chassis.speed_kmh"]
var time_window_s: float = 15.0
var auto_scale_y: bool = true
var manual_min_y: float = -10.0
var manual_max_y: float = 10.0
var auto_scale_x: bool = true
var manual_min_x: float = -10.0
var manual_max_x: float = 10.0
var show_grid: bool = true
var show_legend: bool = true
var is_collapsed: bool = false
var is_paused: bool = false

# Internal buffers
const MAX_SAMPLES: int = 600
var _sample_times: PackedFloat64Array = PackedFloat64Array()
var _channel_data: Dictionary = {} # channel_id -> PackedFloat64Array
var _update_timer: float = 0.0
const UPDATE_INTERVAL_S: float = 0.05 # 20 Hz UI refresh

# Drag & Resize State
var _is_dragging: bool = false
var _drag_offset: Vector2 = Vector2.ZERO
var _is_resizing: bool = false
var _resize_start_mouse: Vector2 = Vector2.ZERO
var _resize_start_size: Vector2 = Vector2.ZERO
var _restored_height: float = 220.0

# UI Child Nodes
var _header_panel: PanelContainer
var _title_label: Label
var _stats_label: Label
var _pause_btn: Button
var _cfg_btn: Button
var _collapse_btn: Button
var _close_btn: Button
var _body_container: VBoxContainer
var _canvas: Control
var _resize_handle: Control

# Hover inspection
var _hover_pos: Vector2 = Vector2(-1, -1)

func _init() -> void:
	custom_minimum_size = Vector2(280, 180)
	size = Vector2(340, 220)
	mouse_filter = Control.MOUSE_FILTER_PASS
	clip_contents = false

func _ready() -> void:
	add_theme_stylebox_override("panel", MirenaUITheme.create_panel_stylebox(0.75, 0.65, 6, 6, 4))
	_build_ui()
	_update_title_display()

func _build_ui() -> void:
	# Clear existing children if any
	for c in get_children():
		c.queue_free()

	var root_vbox := VBoxContainer.new()
	root_vbox.mouse_filter = Control.MOUSE_FILTER_PASS
	root_vbox.add_theme_constant_override("separation", 4)
	add_child(root_vbox)

	# --- Title Bar Header ---
	_header_panel = PanelContainer.new()
	_header_panel.mouse_filter = Control.MOUSE_FILTER_STOP
	_header_panel.add_theme_stylebox_override("panel", MirenaUITheme.create_header_stylebox())
	_header_panel.gui_input.connect(_on_header_gui_input)

	var hbox := HBoxContainer.new()
	hbox.mouse_filter = Control.MOUSE_FILTER_PASS
	hbox.add_theme_constant_override("separation", 6)

	_title_label = Label.new()
	_title_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_title_label.text = graph_title
	_title_label.add_theme_color_override("font_color", MirenaUITheme.ACCENT_COLOR)
	_title_label.add_theme_font_size_override("font_size", 12)
	_title_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	hbox.add_child(_title_label)

	_stats_label = Label.new()
	_stats_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_stats_label.text = ""
	_stats_label.add_theme_color_override("font_color", MirenaUITheme.TEXT_MUTED)
	_stats_label.add_theme_font_size_override("font_size", 10)
	hbox.add_child(_stats_label)

	_pause_btn = Button.new()
	_pause_btn.text = "⏸"
	_pause_btn.tooltip_text = "Pause / Resume Graph Trace"
	_pause_btn.custom_minimum_size = Vector2(22, 20)
	_pause_btn.add_theme_font_size_override("font_size", 10)
	MirenaUITheme.style_button(_pause_btn)
	_pause_btn.pressed.connect(func():
		is_paused = !is_paused
		_pause_btn.text = "▶" if is_paused else "⏸"
	)
	hbox.add_child(_pause_btn)

	_cfg_btn = Button.new()
	_cfg_btn.text = "⚙"
	_cfg_btn.tooltip_text = "Configure Graph Channels & Axis Settings"
	_cfg_btn.custom_minimum_size = Vector2(22, 20)
	_cfg_btn.add_theme_font_size_override("font_size", 11)
	MirenaUITheme.style_button(_cfg_btn)
	_cfg_btn.pressed.connect(func(): configure_requested.emit(self))
	hbox.add_child(_cfg_btn)

	_collapse_btn = Button.new()
	_collapse_btn.text = "▲"
	_collapse_btn.tooltip_text = "Collapse / Expand Graph Window"
	_collapse_btn.custom_minimum_size = Vector2(22, 20)
	_collapse_btn.add_theme_font_size_override("font_size", 10)
	MirenaUITheme.style_button(_collapse_btn)
	_collapse_btn.pressed.connect(_toggle_collapse)
	hbox.add_child(_collapse_btn)

	_close_btn = Button.new()
	_close_btn.text = "✕"
	_close_btn.tooltip_text = "Close Graph"
	_close_btn.custom_minimum_size = Vector2(22, 20)
	_close_btn.add_theme_font_size_override("font_size", 10)
	MirenaUITheme.style_button(_close_btn)
	_close_btn.pressed.connect(func(): closed.emit(self))
	hbox.add_child(_close_btn)

	_header_panel.add_child(hbox)
	root_vbox.add_child(_header_panel)

	# --- Body & Canvas ---
	_body_container = VBoxContainer.new()
	_body_container.mouse_filter = Control.MOUSE_FILTER_PASS
	_body_container.size_flags_vertical = Control.SIZE_EXPAND_FILL

	_canvas = Control.new()
	_canvas.mouse_filter = Control.MOUSE_FILTER_PASS
	_canvas.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_canvas.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_canvas.clip_contents = true
	_canvas.draw.connect(_draw_canvas)
	_canvas.gui_input.connect(_on_canvas_gui_input)
	_body_container.add_child(_canvas)

	# Bottom row with resize grip
	var bottom_bar := HBoxContainer.new()
	bottom_bar.mouse_filter = Control.MOUSE_FILTER_PASS
	var spacer := Control.new()
	spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	bottom_bar.add_child(spacer)

	_resize_handle = Control.new()
	_resize_handle.custom_minimum_size = Vector2(16, 14)
	_resize_handle.mouse_filter = Control.MOUSE_FILTER_STOP
	_resize_handle.mouse_default_cursor_shape = Control.CURSOR_FDIAGSIZE
	_resize_handle.draw.connect(_draw_resize_handle)
	_resize_handle.gui_input.connect(_on_resize_gui_input)
	bottom_bar.add_child(_resize_handle)

	_body_container.add_child(bottom_bar)
	root_vbox.add_child(_body_container)

func _update_title_display() -> void:
	if _title_label != null:
		var type_tag: String = "[%s]" % TYPE_NAMES.get(graph_type, "Graph")
		_title_label.text = "%s %s" % [type_tag, graph_title]

func _toggle_collapse() -> void:
	is_collapsed = !is_collapsed
	if is_collapsed:
		_restored_height = size.y
		_body_container.visible = false
		_collapse_btn.text = "▼"
		custom_minimum_size.y = 36
		size.y = 36
	else:
		_body_container.visible = true
		_collapse_btn.text = "▲"
		custom_minimum_size.y = 160
		size.y = maxf(_restored_height, 180.0)
	layout_changed.emit(self)

func _on_header_gui_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			_is_dragging = true
			_drag_offset = event.global_position - global_position
			move_to_front()
		else:
			if _is_dragging:
				_is_dragging = false
				layout_changed.emit(self)
	elif event is InputEventMouseMotion and _is_dragging:
		var new_pos: Vector2 = event.global_position - _drag_offset
		_clamp_position(new_pos)

func _on_resize_gui_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			_is_resizing = true
			_resize_start_mouse = event.global_position
			_resize_start_size = size
			move_to_front()
		else:
			if _is_resizing:
				_is_resizing = false
				layout_changed.emit(self)
	elif event is InputEventMouseMotion and _is_resizing:
		var delta_mouse: Vector2 = event.global_position - _resize_start_mouse
		var new_w: float = maxf(260.0, _resize_start_size.x + delta_mouse.x)
		var new_h: float = maxf(160.0, _resize_start_size.y + delta_mouse.y)
		size = Vector2(new_w, new_h)
		_clamp_position(position)
		if _canvas != null:
			_canvas.queue_redraw()

func _clamp_position(target_pos: Vector2) -> void:
	var vp_size: Vector2 = get_viewport_rect().size
	if vp_size.x > 100.0 and vp_size.y > 100.0:
		var max_x: float = maxf(0.0, vp_size.x - size.x)
		var max_y: float = maxf(0.0, vp_size.y - size.y)
		position = Vector2(clampf(target_pos.x, 0.0, max_x), clampf(target_pos.y, 76.0, max_y))
	else:
		position = target_pos

func _on_canvas_gui_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		_hover_pos = event.position
		if _canvas != null:
			_canvas.queue_redraw()
	elif event is InputEventMouseButton and event.pressed:
		move_to_front()

func _draw_resize_handle() -> void:
	if _resize_handle == null:
		return
	var s: Vector2 = _resize_handle.size
	var col: Color = MirenaUITheme.TEXT_MUTED
	# 3 diagonal dots or lines
	_resize_handle.draw_line(Vector2(s.x - 4, s.y - 1), Vector2(s.x - 1, s.y - 4), col, 1.0)
	_resize_handle.draw_line(Vector2(s.x - 8, s.y - 1), Vector2(s.x - 1, s.y - 8), col, 1.0)
	_resize_handle.draw_line(Vector2(s.x - 12, s.y - 1), Vector2(s.x - 1, s.y - 12), col, 1.0)

## Sample live vehicle snapshot dictionary and append to internal ring buffers
func sample_snapshot(snap: Dictionary, sim_time_s: float) -> void:
	if is_paused or is_collapsed:
		return

	# Append time
	_sample_times.append(sim_time_s)
	if _sample_times.size() > MAX_SAMPLES:
		_sample_times.remove_at(0)

	# Ensure channel arrays exist
	var all_needed_channels: Array[String] = []
	if graph_type == GraphType.XY_PLOT:
		all_needed_channels.append(x_channel)
		if not y_channels.is_empty():
			all_needed_channels.append(y_channels[0])
	else:
		for ch in y_channels:
			all_needed_channels.append(ch)

	for ch_id in all_needed_channels:
		if not _channel_data.has(ch_id):
			_channel_data[ch_id] = PackedFloat64Array()
		var arr: PackedFloat64Array = _channel_data[ch_id]
		var val: float = TelemetrySignalRegistry.sample_value_from_snapshot(snap, ch_id)
		arr.append(val)
		if arr.size() > MAX_SAMPLES:
			arr.remove_at(0)
		_channel_data[ch_id] = arr

	# Prune time series outside time_window_s
	if graph_type == GraphType.TIME_SERIES and _sample_times.size() >= 2:
		var cutoff_t: float = sim_time_s - time_window_s
		while _sample_times.size() > 2 and _sample_times[0] < cutoff_t:
			_sample_times.remove_at(0)
			for ch_id in _channel_data.keys():
				var arr: PackedFloat64Array = _channel_data[ch_id]
				if arr.size() > _sample_times.size():
					arr.remove_at(0)
					_channel_data[ch_id] = arr

	if _canvas != null:
		_canvas.queue_redraw()

func _draw_canvas() -> void:
	if _canvas == null or is_collapsed:
		return
	var rect := _canvas.get_rect()
	var w: float = rect.size.x
	var h: float = rect.size.y
	if w < 40.0 or h < 30.0:
		return

	# Background fill
	_canvas.draw_rect(Rect2(0, 0, w, h), Color(0.08, 0.09, 0.11, 0.50))

	match graph_type:
		GraphType.TIME_SERIES:
			_draw_time_series(w, h)
		GraphType.XY_PLOT:
			_draw_xy_plot(w, h)
		GraphType.HISTOGRAM:
			_draw_histogram(w, h)
		GraphType.GAUGE:
			_draw_gauge(w, h)

# --- A. TIME SERIES DRAWING ---
func _draw_time_series(w: float, h: float) -> void:
	var pad_l: float = 46.0
	var pad_r: float = 12.0
	var pad_t: float = 20.0
	var pad_b: float = 22.0
	var plot_w: float = maxf(10.0, w - pad_l - pad_r)
	var plot_h: float = maxf(10.0, h - pad_t - pad_b)

	# Compute Y range across all active y_channels
	var min_y: float = INF
	var max_y: float = -INF
	if auto_scale_y:
		for ch_id in y_channels:
			var arr: PackedFloat64Array = _channel_data.get(ch_id, PackedFloat64Array())
			for val in arr:
				if val < min_y: min_y = val
				if val > max_y: max_y = val
		if min_y == INF or max_y == -INF or absf(max_y - min_y) < 1e-4:
			var first_sig = TelemetrySignalRegistry.get_signal(y_channels[0] if not y_channels.is_empty() else "")
			if first_sig != null:
				min_y = first_sig.default_min
				max_y = first_sig.default_max
			else:
				min_y = -5.0
				max_y = 5.0
		else:
			# Add 10% headroom
			var margin: float = (max_y - min_y) * 0.10
			min_y -= margin
			max_y += margin
	else:
		min_y = manual_min_y
		max_y = manual_max_y

	# Compute Time range
	var t_end: float = _sample_times[-1] if not _sample_times.is_empty() else 0.0
	var t_start: float = maxf(0.0, t_end - time_window_s)

	# Gridlines & Axis labels
	var grid_col := Color(0.25, 0.28, 0.35, 0.45)
	var text_col := MirenaUITheme.TEXT_MUTED
	var default_font = ThemeDB.fallback_font
	var font_sz: int = 10

	# Y-grid (4 horizontal lines)
	for i in range(5):
		var frac: float = float(i) / 4.0
		var y_pos: float = pad_t + plot_h * (1.0 - frac)
		var val: float = lerpf(min_y, max_y, frac)
		if show_grid:
			_canvas.draw_line(Vector2(pad_l, y_pos), Vector2(pad_l + plot_w, y_pos), grid_col, 1.0)
		_canvas.draw_string(default_font, Vector2(4.0, y_pos + 3.0), "%5.1f" % val, HORIZONTAL_ALIGNMENT_RIGHT, pad_l - 8.0, font_sz, text_col)

	# Zero line if in range
	if min_y <= 0.0 and max_y >= 0.0:
		var zero_frac: float = (0.0 - min_y) / (max_y - min_y)
		var zero_y: float = pad_t + plot_h * (1.0 - zero_frac)
		_canvas.draw_line(Vector2(pad_l, zero_y), Vector2(pad_l + plot_w, zero_y), Color(0.45, 0.50, 0.60, 0.70), 1.0)

	# X-axis time ticks
	for i in range(4):
		var frac: float = float(i) / 3.0
		var x_pos: float = pad_l + plot_w * frac
		var t_val: float = lerpf(t_start, t_end, frac)
		if show_grid and i > 0 and i < 3:
			_canvas.draw_line(Vector2(x_pos, pad_t), Vector2(x_pos, pad_t + plot_h), grid_col, 1.0)
		_canvas.draw_string(default_font, Vector2(x_pos - 15.0, pad_t + plot_h + 14.0), "%.1fs" % t_val, HORIZONTAL_ALIGNMENT_CENTER, 30.0, font_sz, text_col)

	# Draw multi-traces
	var latest_readouts: String = ""
	for ch_idx in range(y_channels.size()):
		var ch_id: String = y_channels[ch_idx]
		var sig: TelemetrySignalRegistry.SignalDef = TelemetrySignalRegistry.get_signal(ch_id)
		var arr: PackedFloat64Array = _channel_data.get(ch_id, PackedFloat64Array())
		if arr.size() < 2 or _sample_times.size() < 2:
			continue

		var trace_col: Color = sig.default_color if sig != null else Color(0.40, 0.90, 0.60)
		var points: PackedVector2Array = PackedVector2Array()
		var n: int = mini(arr.size(), _sample_times.size())

		for i in range(n):
			var t: float = _sample_times[i]
			var val: float = arr[i]
			var x_frac: float = clampf((t - t_start) / maxf(t_end - t_start, 1e-4), 0.0, 1.0)
			var y_frac: float = clampf((val - min_y) / maxf(max_y - min_y, 1e-4), 0.0, 1.0)
			points.append(Vector2(pad_l + plot_w * x_frac, pad_t + plot_h * (1.0 - y_frac)))

		if points.size() >= 2:
			_canvas.draw_polyline(points, trace_col, 1.5, true)
			# Draw latest point circle
			_canvas.draw_circle(points[-1], 3.0, trace_col)

		if not arr.is_empty():
			var unit_str: String = sig.unit if sig != null else ""
			var fmt: String = sig.format_str if sig != null else "%.1f"
			latest_readouts += "%s: %s %s  " % [sig.name if sig != null else ch_id, fmt % arr[-1], unit_str]

	# Legend & current readout
	if show_legend and not y_channels.is_empty():
		var leg_x: float = pad_l
		for ch_id in y_channels:
			var sig: TelemetrySignalRegistry.SignalDef = TelemetrySignalRegistry.get_signal(ch_id)
			var col: Color = sig.default_color if sig != null else Color.WHITE
			_canvas.draw_rect(Rect2(leg_x, 4.0, 8.0, 8.0), col)
			var label_txt: String = sig.name if sig != null else ch_id
			_canvas.draw_string(default_font, Vector2(leg_x + 12.0, 12.0), label_txt, HORIZONTAL_ALIGNMENT_LEFT, -1, 10, MirenaUITheme.TEXT_SECONDARY)
			leg_x += default_font.get_string_size(label_txt, HORIZONTAL_ALIGNMENT_LEFT, -1, 10).x + 22.0

	if _stats_label != null:
		_stats_label.text = latest_readouts.strip_edges()

# --- B. XY PLOT DRAWING (Custom G-G, Fy vs Alpha, etc.) ---
func _draw_xy_plot(w: float, h: float) -> void:
	var pad_l: float = 46.0
	var pad_r: float = 14.0
	var pad_t: float = 18.0
	var pad_b: float = 24.0
	var plot_w: float = maxf(10.0, w - pad_l - pad_r)
	var plot_h: float = maxf(10.0, h - pad_t - pad_b)

	var sig_x: TelemetrySignalRegistry.SignalDef = TelemetrySignalRegistry.get_signal(x_channel)
	var y_ch: String = y_channels[0] if not y_channels.is_empty() else "imu.ay"
	var sig_y: TelemetrySignalRegistry.SignalDef = TelemetrySignalRegistry.get_signal(y_ch)

	var arr_x: PackedFloat64Array = _channel_data.get(x_channel, PackedFloat64Array())
	var arr_y: PackedFloat64Array = _channel_data.get(y_ch, PackedFloat64Array())

	# Scales
	var min_x: float = manual_min_x
	var max_x: float = manual_max_x
	var min_y: float = manual_min_y
	var max_y: float = manual_max_y

	if auto_scale_x or auto_scale_y:
		var is_gg_plot: bool = (x_channel == "imu.ax" or x_channel == "imu.ay") and (y_ch == "imu.ax" or y_ch == "imu.ay")
		if is_gg_plot:
			# Equal aspect square G-G diagram default ±15 m/s² (~1.5g)
			min_x = -15.0; max_x = 15.0
			min_y = -15.0; max_y = 15.0
		else:
			if auto_scale_x and sig_x != null:
				min_x = sig_x.default_min; max_x = sig_x.default_max
			if auto_scale_y and sig_y != null:
				min_y = sig_y.default_min; max_y = sig_y.default_max

	var default_font = ThemeDB.fallback_font
	var font_sz: int = 10
	var grid_col := Color(0.25, 0.28, 0.35, 0.45)
	var text_col := MirenaUITheme.TEXT_MUTED

	# Grid & Zero Crosshairs
	if show_grid:
		for i in range(5):
			var frac: float = float(i) / 4.0
			var y_pos: float = pad_t + plot_h * (1.0 - frac)
			var x_pos: float = pad_l + plot_w * frac
			_canvas.draw_line(Vector2(pad_l, y_pos), Vector2(pad_l + plot_w, y_pos), grid_col, 1.0)
			_canvas.draw_line(Vector2(x_pos, pad_t), Vector2(x_pos, pad_t + plot_h), grid_col, 1.0)
			_canvas.draw_string(default_font, Vector2(4.0, y_pos + 3.0), "%4.1f" % lerpf(min_y, max_y, frac), HORIZONTAL_ALIGNMENT_RIGHT, pad_l - 8.0, font_sz, text_col)
			_canvas.draw_string(default_font, Vector2(x_pos - 15.0, pad_t + plot_h + 14.0), "%4.1f" % lerpf(min_x, max_x, frac), HORIZONTAL_ALIGNMENT_CENTER, 30.0, font_sz, text_col)

	# Zero crosshairs
	if min_x <= 0.0 and max_x >= 0.0:
		var x_zero: float = pad_l + plot_w * ((0.0 - min_x) / (max_x - min_x))
		_canvas.draw_line(Vector2(x_zero, pad_t), Vector2(x_zero, pad_t + plot_h), Color(0.40, 0.45, 0.55, 0.65), 1.0)
	if min_y <= 0.0 and max_y >= 0.0:
		var y_zero: float = pad_t + plot_h * (1.0 - (0.0 - min_y) / (max_y - min_y))
		_canvas.draw_line(Vector2(pad_l, y_zero), Vector2(pad_l + plot_w, y_zero), Color(0.40, 0.45, 0.55, 0.65), 1.0)

	# Circular reference boundary for G-G diagrams (1.0g = 9.81 m/s²)
	var is_accel_plot: bool = (x_channel.begins_with("imu.a") and y_ch.begins_with("imu.a"))
	if is_accel_plot and min_x <= 0.0 and max_x >= 0.0 and min_y <= 0.0 and max_y >= 0.0:
		var center := Vector2(
			pad_l + plot_w * ((0.0 - min_x) / (max_x - min_x)),
			pad_t + plot_h * (1.0 - (0.0 - min_y) / (max_y - min_y))
		)
		var rad_x: float = (9.80665 / (max_x - min_x)) * plot_w
		var rad_y: float = (9.80665 / (max_y - min_y)) * plot_h
		var avg_rad: float = (rad_x + rad_y) * 0.5
		_canvas.draw_arc(center, avg_rad, 0.0, TAU, 36, Color(0.30, 0.75, 0.85, 0.35), 1.0)

	# Plot XY trajectory trail
	var n: int = mini(arr_x.size(), arr_y.size())
	if n >= 2:
		var trail_points := PackedVector2Array()
		for i in range(n):
			var x_val: float = arr_x[i]
			var y_val: float = arr_y[i]
			var x_frac: float = clampf((x_val - min_x) / maxf(max_x - min_x, 1e-4), 0.0, 1.0)
			var y_frac: float = clampf((y_val - min_y) / maxf(max_y - min_y, 1e-4), 0.0, 1.0)
			trail_points.append(Vector2(pad_l + plot_w * x_frac, pad_t + plot_h * (1.0 - y_frac)))

		# Trajectory polyline
		_canvas.draw_polyline(trail_points, Color(0.35, 0.85, 0.95, 0.55), 1.2, true)

		# Current operating point
		var cur_pt: Vector2 = trail_points[-1]
		_canvas.draw_circle(cur_pt, 5.0, Color(0.745, 1.0, 1.0, 0.35)) # glow halo
		_canvas.draw_circle(cur_pt, 3.0, Color(0.40, 0.95, 0.70))

	# Channel labels on axes
	var x_unit: String = sig_x.unit if sig_x != null else ""
	var y_unit: String = sig_y.unit if sig_y != null else ""
	_canvas.draw_string(default_font, Vector2(pad_l, pad_t - 4.0), "%s [%s]" % [sig_y.name if sig_y != null else y_ch, y_unit], HORIZONTAL_ALIGNMENT_LEFT, -1, 10, MirenaUITheme.ACCENT_COLOR)

	if _stats_label != null and n > 0:
		_stats_label.text = "X: %+.2f %s | Y: %+.2f %s" % [arr_x[-1], x_unit, arr_y[-1], y_unit]

# --- C. HISTOGRAM / DISTRIBUTION DRAWING ---
func _draw_histogram(w: float, h: float) -> void:
	var pad_l: float = 46.0
	var pad_r: float = 12.0
	var pad_t: float = 20.0
	var pad_b: float = 22.0
	var plot_w: float = maxf(10.0, w - pad_l - pad_r)
	var plot_h: float = maxf(10.0, h - pad_t - pad_b)

	var target_ch: String = y_channels[0] if not y_channels.is_empty() else "chassis.speed_kmh"
	var sig: TelemetrySignalRegistry.SignalDef = TelemetrySignalRegistry.get_signal(target_ch)
	var arr: PackedFloat64Array = _channel_data.get(target_ch, PackedFloat64Array())

	var default_font = ThemeDB.fallback_font
	var text_col := MirenaUITheme.TEXT_MUTED
	var bar_col := Color(0.35, 0.75, 0.90, 0.75)

	if arr.size() < 10:
		_canvas.draw_string(default_font, Vector2(w * 0.5 - 40, h * 0.5), "Gathering samples...", HORIZONTAL_ALIGNMENT_CENTER, -1, 11, text_col)
		return

	# Min & Max
	var min_v: float = INF
	var max_v: float = -INF
	var sum_v: float = 0.0
	for v in arr:
		if v < min_v: min_v = v
		if v > max_v: max_v = v
		sum_v += v
	var mean_v: float = sum_v / float(arr.size())
	if absf(max_v - min_v) < 1e-4:
		max_v += 1.0; min_v -= 1.0

	# 16 Bins
	const NUM_BINS: int = 16
	var bin_counts: PackedInt32Array = PackedInt32Array()
	bin_counts.resize(NUM_BINS)
	var max_count: int = 1
	var bin_w_val: float = (max_v - min_v) / float(NUM_BINS)

	for v in arr:
		var b_idx: int = clampi(int((v - min_v) / bin_w_val), 0, NUM_BINS - 1)
		bin_counts[b_idx] += 1
		if bin_counts[b_idx] > max_count:
			max_count = bin_counts[b_idx]

	var bar_pixel_w: float = plot_w / float(NUM_BINS)
	for i in range(NUM_BINS):
		var bar_h: float = (float(bin_counts[i]) / float(max_count)) * plot_h
		var bx: float = pad_l + float(i) * bar_pixel_w
		var by: float = pad_t + plot_h - bar_h
		_canvas.draw_rect(Rect2(bx + 1.0, by, bar_pixel_w - 2.0, bar_h), bar_col)

	# Mean line
	var mean_x: float = pad_l + plot_w * ((mean_v - min_v) / (max_v - min_v))
	_canvas.draw_line(Vector2(mean_x, pad_t), Vector2(mean_x, pad_t + plot_h), Color(0.95, 0.85, 0.35), 1.5)

	# Ticks & Labels
	var unit_str: String = sig.unit if sig != null else ""
	_canvas.draw_string(default_font, Vector2(pad_l, pad_t + plot_h + 14.0), "%4.1f %s" % [min_v, unit_str], HORIZONTAL_ALIGNMENT_LEFT, -1, 10, text_col)
	_canvas.draw_string(default_font, Vector2(pad_l + plot_w, pad_t + plot_h + 14.0), "%4.1f %s" % [max_v, unit_str], HORIZONTAL_ALIGNMENT_RIGHT, -1, 10, text_col)

	if _stats_label != null:
		_stats_label.text = "μ: %.2f | N=%d" % [mean_v, arr.size()]

# --- D. LIVE VALUE GAUGE DRAWING ---
func _draw_gauge(w: float, h: float) -> void:
	var target_ch: String = y_channels[0] if not y_channels.is_empty() else "chassis.speed_kmh"
	var sig: TelemetrySignalRegistry.SignalDef = TelemetrySignalRegistry.get_signal(target_ch)
	var arr: PackedFloat64Array = _channel_data.get(target_ch, PackedFloat64Array())
	var cur_val: float = arr[-1] if not arr.is_empty() else 0.0

	var min_v: float = manual_min_y if not auto_scale_y else (sig.default_min if sig != null else 0.0)
	var max_v: float = manual_max_y if not auto_scale_y else (sig.default_max if sig != null else 100.0)

	var default_font = ThemeDB.fallback_font
	var font_sz: int = 24
	var frac: float = clampf((cur_val - min_v) / maxf(max_v - min_v, 1e-4), 0.0, 1.0)

	# Circular dial
	var center := Vector2(w * 0.5, h * 0.55)
	var radius: float = minf(w * 0.35, h * 0.38)
	var start_ang: float = PI * 0.75
	var sweep: float = PI * 1.5

	# Background track arc
	_canvas.draw_arc(center, radius, start_ang, start_ang + sweep, 32, Color(0.20, 0.22, 0.26, 0.70), 8.0)
	# Filled arc
	var fill_col: Color = sig.default_color if sig != null else MirenaUITheme.ACCENT_COLOR
	if frac > 0.01:
		_canvas.draw_arc(center, radius, start_ang, start_ang + sweep * frac, 32, fill_col, 8.0)

	# Central digital readout
	var fmt: String = sig.format_str if sig != null else "%.1f"
	var val_str: String = fmt % cur_val
	_canvas.draw_string(default_font, Vector2(center.x - 60.0, center.y + 4.0), val_str, HORIZONTAL_ALIGNMENT_CENTER, 120.0, font_sz, fill_col)
	var unit_str: String = sig.unit if sig != null else ""
	_canvas.draw_string(default_font, Vector2(center.x - 40.0, center.y + 20.0), unit_str, HORIZONTAL_ALIGNMENT_CENTER, 80.0, 10, MirenaUITheme.TEXT_SECONDARY)

	# Range limits
	_canvas.draw_string(default_font, Vector2(center.x - radius - 15.0, center.y + radius * 0.6), "%.0f" % min_v, HORIZONTAL_ALIGNMENT_LEFT, -1, 9, MirenaUITheme.TEXT_MUTED)
	_canvas.draw_string(default_font, Vector2(center.x + radius - 5.0, center.y + radius * 0.6), "%.0f" % max_v, HORIZONTAL_ALIGNMENT_RIGHT, -1, 9, MirenaUITheme.TEXT_MUTED)

	if _stats_label != null:
		_stats_label.text = "%s: %s %s" % [sig.name if sig != null else target_ch, val_str, unit_str]

## Serialize configuration to dictionary for layout saving
func serialize_config() -> Dictionary:
	return {
		"id": graph_id,
		"title": graph_title,
		"type": graph_type,
		"x_channel": x_channel,
		"y_channels": y_channels,
		"time_window_s": time_window_s,
		"auto_scale_y": auto_scale_y,
		"manual_min_y": manual_min_y,
		"manual_max_y": manual_max_y,
		"auto_scale_x": auto_scale_x,
		"manual_min_x": manual_min_x,
		"manual_max_x": manual_max_x,
		"show_grid": show_grid,
		"show_legend": show_legend,
		"is_collapsed": is_collapsed,
		"pos_x": position.x,
		"pos_y": position.y,
		"size_w": size.x,
		"size_h": size.y,
	}

## Restore configuration from dictionary
func deserialize_config(data: Dictionary) -> void:
	graph_id = str(data.get("id", graph_id))
	graph_title = str(data.get("title", graph_title))
	graph_type = int(data.get("type", graph_type))
	x_channel = str(data.get("x_channel", x_channel))
	var ys: Array = data.get("y_channels", y_channels) as Array
	y_channels.clear()
	for y in ys:
		y_channels.append(str(y))
	time_window_s = float(data.get("time_window_s", time_window_s))
	auto_scale_y = bool(data.get("auto_scale_y", auto_scale_y))
	manual_min_y = float(data.get("manual_min_y", manual_min_y))
	manual_max_y = float(data.get("manual_max_y", manual_max_y))
	auto_scale_x = bool(data.get("auto_scale_x", auto_scale_x))
	manual_min_x = float(data.get("manual_min_x", manual_min_x))
	manual_max_x = float(data.get("manual_max_x", manual_max_x))
	show_grid = bool(data.get("show_grid", show_grid))
	show_legend = bool(data.get("show_legend", show_legend))
	position = Vector2(float(data.get("pos_x", position.x)), float(data.get("pos_y", position.y)))
	size = Vector2(float(data.get("size_w", size.x)), float(data.get("size_h", size.y)))
	is_collapsed = bool(data.get("is_collapsed", false))
	_update_title_display()
	if _canvas != null:
		_canvas.queue_redraw()
