extends Control
class_name SignalPlotCanvas

enum PlotMode {
	TIME_SERIES = 0,
	TIME_SERIES_WITH_RESIDUAL = 1,
	XY_SCATTER = 2,
}

var plot_mode: PlotMode = PlotMode.TIME_SERIES
var plot_title: String = "Signal Trace"
var x_axis_label: String = "Time [s]"
var y_axis_label: String = "Value"
var residual_label: String = "Residual (Sim - Real)"

var traces: Array[Dictionary] = []

var residual_trace: Dictionary = {}

var xy_reference_circle_radius: float = 0.0

var x_zoom_factor: float = 1.0
var x_pan_normalized: float = 0.0
var _hover_pos: Vector2 = Vector2(-1.0, -1.0)
var _is_panning: bool = false
var _last_pan_mouse_x: float = 0.0

func _init() -> void:
	custom_minimum_size = Vector2(420, 230)
	size_flags_horizontal = Control.SIZE_EXPAND_FILL
	size_flags_vertical = Control.SIZE_EXPAND_FILL
	mouse_filter = Control.MOUSE_FILTER_PASS
	clip_contents = true

func clear_traces() -> void:
	traces.clear()
	residual_trace.clear()
	xy_reference_circle_radius = 0.0
	queue_redraw()

func add_trace(
	p_name: String,
	p_x: PackedFloat64Array,
	p_y: PackedFloat64Array,
	p_color: Color,
	p_width: float = 2.0,
	p_dashed: bool = false
) -> void:
	traces.append({
		"name": p_name,
		"x": p_x,
		"y": p_y,
		"color": p_color,
		"width": p_width,
		"dashed": p_dashed,
	})
	queue_redraw()

func set_residual_trace(p_x: PackedFloat64Array, p_y: PackedFloat64Array, p_color: Color = Color(0.95, 0.45, 0.35)) -> void:
	residual_trace = {
		"x": p_x,
		"y": p_y,
		"color": p_color,
	}
	queue_redraw()

func _gui_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		_hover_pos = event.position
		if _is_panning and size.x > 10.0:
			var dx: float = event.position.x - _last_pan_mouse_x
			_last_pan_mouse_x = event.position.x
			x_pan_normalized = clampf(x_pan_normalized - dx / (size.x * x_zoom_factor), -0.5, 0.5)
		queue_redraw()
	elif event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP and event.pressed:
			x_zoom_factor = clampf(x_zoom_factor * 1.25, 1.0, 25.0)
			queue_redraw()
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN and event.pressed:
			x_zoom_factor = clampf(x_zoom_factor / 1.25, 1.0, 25.0)
			if x_zoom_factor <= 1.01:
				x_pan_normalized = 0.0
			queue_redraw()
		elif (event.button_index == MOUSE_BUTTON_RIGHT or event.button_index == MOUSE_BUTTON_MIDDLE):
			_is_panning = event.pressed
			_last_pan_mouse_x = event.position.x
		elif event.button_index == MOUSE_BUTTON_LEFT and event.double_click:
			x_zoom_factor = 1.0
			x_pan_normalized = 0.0
			queue_redraw()

func _notification(what: int) -> void:
	if what == NOTIFICATION_MOUSE_EXIT:
		_hover_pos = Vector2(-1.0, -1.0)
		_is_panning = false
		queue_redraw()

func _draw() -> void:
	var rect := Rect2(Vector2.ZERO, size)
	draw_rect(rect, Color(0.08, 0.09, 0.12, 1.0), true)
	draw_rect(rect, Color(0.24, 0.27, 0.34, 1.0), false, 1.0)

	var font: Font = ThemeDB.fallback_font
	var font_size: int = 12

	draw_string(font, Vector2(12, 18), plot_title, HORIZONTAL_ALIGNMENT_LEFT, -1, 13, Color(0.92, 0.95, 1.0))

	var leg_x: float = size.x - 14.0
	for i in range(traces.size() - 1, -1, -1):
		var tr: Dictionary = traces[i]
		var tname: String = str(tr.get("name", ""))
		var tcol: Color = tr.get("color", Color.WHITE)
		var tw: float = font.get_string_size(tname, HORIZONTAL_ALIGNMENT_LEFT, -1, 11).x
		leg_x -= (tw + 26.0)
		if leg_x > 180.0:
			draw_line(Vector2(leg_x, 13), Vector2(leg_x + 16.0, 13), tcol, 2.5)
			draw_string(font, Vector2(leg_x + 20.0, 17), tname, HORIZONTAL_ALIGNMENT_LEFT, -1, 11, tcol)

	var pad_left: float = 64.0
	var pad_right: float = 16.0
	var pad_top: float = 28.0
	var pad_bottom: float = 26.0

	if traces.is_empty():
		draw_string(
			font,
			Vector2(size.x * 0.5 - 90.0, size.y * 0.5),
			"No signal samples in buffer",
			HORIZONTAL_ALIGNMENT_LEFT, -1, 13, Color(0.55, 0.58, 0.65)
		)
		return

	var has_residual: bool = (
		plot_mode == PlotMode.TIME_SERIES_WITH_RESIDUAL
		and residual_trace.has("x")
		and (residual_trace["x"] as PackedFloat64Array).size() >= 2
	)

	var usable_h: float = maxf(40.0, size.y - pad_top - pad_bottom)
	var main_h: float = usable_h * 0.66 if has_residual else usable_h
	var main_rect := Rect2(pad_left, pad_top, maxf(40.0, size.x - pad_left - pad_right), main_h)

	_draw_subplot(font, font_size, main_rect, traces, y_axis_label, not has_residual, x_axis_label)

	if has_residual:
		var res_top: float = pad_top + main_h + 10.0
		var res_h: float = maxf(30.0, usable_h - main_h - 10.0)
		var res_rect := Rect2(pad_left, res_top, main_rect.size.x, res_h)
		var res_traces: Array[Dictionary] = [{
			"name": "Error (Sim - Real)",
			"x": residual_trace["x"],
			"y": residual_trace["y"],
			"color": residual_trace.get("color", Color(0.95, 0.45, 0.35)),
			"width": 1.6,
			"dashed": false,
		}]
		_draw_subplot(font, font_size, res_rect, res_traces, "Error", true, x_axis_label)

func _draw_subplot(
	font: Font,
	font_size: int,
	box: Rect2,
	sub_traces: Array[Dictionary],
	y_label: String,
	draw_x_ticks: bool,
	x_label: String
) -> void:
	draw_rect(box, Color(0.05, 0.06, 0.08, 1.0), true)
	draw_rect(box, Color(0.20, 0.23, 0.29, 1.0), false, 1.0)

	var x_min: float = INF
	var x_max: float = -INF
	var y_min: float = INF
	var y_max: float = -INF

	for tr in sub_traces:
		var xa: PackedFloat64Array = tr["x"]
		var ya: PackedFloat64Array = tr["y"]
		var n: int = mini(xa.size(), ya.size())
		for i in range(n):
			var xv: float = xa[i]
			var yv: float = ya[i]
			if not is_nan(xv) and not is_inf(xv) and not is_nan(yv) and not is_inf(yv):
				x_min = minf(x_min, xv)
				x_max = maxf(x_max, xv)

	if x_min == INF or x_max == -INF:
		return
	if absf(x_max - x_min) < 1e-6:
		x_min -= 0.5
		x_max += 0.5

	if plot_mode != PlotMode.XY_SCATTER and x_zoom_factor > 1.001:
		var full_span: float = x_max - x_min
		var center: float = 0.5 * (x_min + x_max) + x_pan_normalized * full_span
		var half_vis: float = 0.5 * full_span / x_zoom_factor
		x_min = center - half_vis
		x_max = center + half_vis

	for tr in sub_traces:
		var xa: PackedFloat64Array = tr["x"]
		var ya: PackedFloat64Array = tr["y"]
		var n: int = mini(xa.size(), ya.size())
		for i in range(n):
			var xv: float = xa[i]
			var yv: float = ya[i]
			if xv >= x_min and xv <= x_max and not is_nan(yv) and not is_inf(yv):
				y_min = minf(y_min, yv)
				y_max = maxf(y_max, yv)

	if xy_reference_circle_radius > 0.0 and plot_mode == PlotMode.XY_SCATTER:
		var r: float = xy_reference_circle_radius * 1.1
		x_min = minf(x_min, -r)
		x_max = maxf(x_max, r)
		y_min = minf(y_min, -r)
		y_max = maxf(y_max, r)

	if y_min == INF or y_max == -INF:
		y_min = -1.0
		y_max = 1.0
	var y_span: float = y_max - y_min
	if y_span < 1e-4:
		y_min -= 0.5
		y_max += 0.5
	else:
		y_min -= 0.08 * y_span
		y_max += 0.08 * y_span

	var n_y_grid: int = 4
	for g in range(n_y_grid + 1):
		var frac: float = float(g) / float(n_y_grid)
		var py: float = box.position.y + box.size.y * (1.0 - frac)
		var y_val: float = lerpf(y_min, y_max, frac)
		draw_line(Vector2(box.position.x, py), Vector2(box.end.x, py), Color(0.16, 0.18, 0.23, 1.0), 1.0)
		draw_string(font, Vector2(4.0, py + 4.0), _fmt_num(y_val), HORIZONTAL_ALIGNMENT_RIGHT, box.position.x - 6.0, 10, Color(0.68, 0.72, 0.80))

	if y_min < 0.0 and y_max > 0.0:
		var zero_frac: float = (0.0 - y_min) / (y_max - y_min)
		var zy: float = box.position.y + box.size.y * (1.0 - zero_frac)
		draw_line(Vector2(box.position.x, zy), Vector2(box.end.x, zy), Color(0.35, 0.40, 0.48, 0.9), 1.2)

	var n_x_grid: int = 5
	for g in range(n_x_grid + 1):
		var frac: float = float(g) / float(n_x_grid)
		var px: float = box.position.x + box.size.x * frac
		var x_val: float = lerpf(x_min, x_max, frac)
		draw_line(Vector2(px, box.position.y), Vector2(px, box.end.y), Color(0.16, 0.18, 0.23, 1.0), 1.0)
		if draw_x_ticks:
			draw_string(font, Vector2(px - 18.0, box.end.y + 13.0), _fmt_num(x_val), HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color(0.68, 0.72, 0.80))

	if x_min < 0.0 and x_max > 0.0:
		var x0_frac: float = (0.0 - x_min) / (x_max - x_min)
		var zx: float = box.position.x + box.size.x * x0_frac
		draw_line(Vector2(zx, box.position.y), Vector2(zx, box.end.y), Color(0.45, 0.45, 0.30, 0.85), 1.2)

	if xy_reference_circle_radius > 0.0 and plot_mode == PlotMode.XY_SCATTER:
		var pts := PackedVector2Array()
		for k in range(65):
			var ang: float = float(k) * TAU / 64.0
			var cx: float = xy_reference_circle_radius * cos(ang)
			var cy: float = xy_reference_circle_radius * sin(ang)
			var sx: float = box.position.x + ((cx - x_min) / (x_max - x_min)) * box.size.x
			var sy: float = box.position.y + (1.0 - (cy - y_min) / (y_max - y_min)) * box.size.y
			pts.append(Vector2(sx, sy))
		draw_polyline(pts, Color(0.85, 0.70, 0.25, 0.65), 1.5)

	var max_draw_pts: int = maxi(200, int(box.size.x * 1.5))
	for tr in sub_traces:
		var xa: PackedFloat64Array = tr["x"]
		var ya: PackedFloat64Array = tr["y"]
		var col: Color = tr.get("color", Color.CYAN)
		var width: float = float(tr.get("width", 1.8))
		var n: int = mini(xa.size(), ya.size())
		if n < 2:
			continue
		var stride: int = maxi(1, n / max_draw_pts)
		var poly := PackedVector2Array()
		for i in range(0, n, stride):
			var xv: float = xa[i]
			var yv: float = ya[i]
			if xv < x_min or xv > x_max or is_nan(yv) or is_inf(yv):
				continue
			var sx: float = box.position.x + ((xv - x_min) / (x_max - x_min)) * box.size.x
			var sy: float = box.position.y + (1.0 - clampf((yv - y_min) / (y_max - y_min), 0.0, 1.0)) * box.size.y
			poly.append(Vector2(sx, sy))

		if (n - 1) % stride != 0:
			var xv_last: float = xa[n - 1]
			var yv_last: float = ya[n - 1]
			if xv_last >= x_min and xv_last <= x_max and not is_nan(yv_last) and not is_inf(yv_last):
				var sx_l: float = box.position.x + ((xv_last - x_min) / (x_max - x_min)) * box.size.x
				var sy_l: float = box.position.y + (1.0 - clampf((yv_last - y_min) / (y_max - y_min), 0.0, 1.0)) * box.size.y
				poly.append(Vector2(sx_l, sy_l))
		if poly.size() >= 2:
			draw_polyline(poly, col, width, true)

	draw_string(font, Vector2(box.position.x + 6.0, box.position.y + 13.0), y_label, HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color(0.58, 0.66, 0.78))
	if draw_x_ticks:
		draw_string(font, Vector2(box.end.x - 110.0, box.end.y + 23.0), x_label, HORIZONTAL_ALIGNMENT_RIGHT, 110.0, 10, Color(0.58, 0.66, 0.78))

	if box.has_point(_hover_pos) and plot_mode != PlotMode.XY_SCATTER:
		var hover_frac: float = (_hover_pos.x - box.position.x) / maxf(box.size.x, 1.0)
		var hover_t: float = lerpf(x_min, x_max, hover_frac)
		draw_line(Vector2(_hover_pos.x, box.position.y), Vector2(_hover_pos.x, box.end.y), Color(0.85, 0.90, 1.0, 0.55), 1.0)
		var info_str: String = "t=%.2fs" % hover_t
		for tr in sub_traces:
			var xa: PackedFloat64Array = tr["x"]
			var ya: PackedFloat64Array = tr["y"]
			if xa.size() >= 2:
				var v_at: float = _sample_array_at(xa, ya, hover_t)
				info_str += " | %s: %s" % [str(tr.get("name", "")), _fmt_num(v_at)]
		draw_rect(Rect2(box.position.x + 4.0, box.end.y - 18.0, minf(box.size.x - 8.0, float(info_str.length() * 6 + 12)), 15.0), Color(0.04, 0.05, 0.07, 0.85), true)
		draw_string(font, Vector2(box.position.x + 8.0, box.end.y - 6.0), info_str, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size - 2, Color(0.95, 0.98, 1.0))

static func _sample_array_at(xa: PackedFloat64Array, ya: PackedFloat64Array, xq: float) -> float:
	var n: int = mini(xa.size(), ya.size())
	if n == 0:
		return 0.0
	if xq <= xa[0]:
		return ya[0]
	if xq >= xa[n - 1]:
		return ya[n - 1]
	var lo: int = 0
	var hi: int = n - 1
	while hi - lo > 1:
		var mid: int = (lo + hi) >> 1
		if xa[mid] <= xq:
			lo = mid
		else:
			hi = mid
	var span: float = xa[hi] - xa[lo]
	if span <= 1e-12:
		return ya[lo]
	return lerpf(ya[lo], ya[hi], (xq - xa[lo]) / span)

static func _fmt_num(v: float) -> String:
	var av: float = absf(v)
	if av >= 1000.0:
		return "%.0f" % v
	elif av >= 100.0:
		return "%.1f" % v
	elif av >= 1.0:
		return "%.2f" % v
	elif av >= 1e-4:
		return "%.3f" % v
	return "0.00"
