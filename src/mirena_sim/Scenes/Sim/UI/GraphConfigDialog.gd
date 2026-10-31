extends PanelContainer
class_name GraphConfigDialog

const MirenaUITheme = preload("res://Scenes/Sim/UI/MirenaUITheme.gd")
const TelemetrySignalRegistry = preload("res://Scenes/Sim/UI/TelemetrySignalRegistry.gd")
const FloatingGraphPanel = preload("res://Scenes/Sim/UI/FloatingGraphPanel.gd")

## Configuration modal dialog for editing FloatingGraphPanel parameters.
## Styled with Options Menu aesthetic.

signal applied(config: Dictionary)
signal deleted(graph_id: String)
signal cancelled()

var target_panel: FloatingGraphPanel = null

# Form Controls
var _title_input: LineEdit
var _type_select: OptionButton
var _duration_spin: SpinBox
var _cat_filter: OptionButton
var _search_input: LineEdit
var _signals_container: VBoxContainer
var _xy_x_select: OptionButton
var _xy_y_select: OptionButton
var _xy_container: VBoxContainer
var _ts_container: VBoxContainer
var _autoscale_y_check: CheckBox
var _min_y_spin: SpinBox
var _max_y_spin: SpinBox
var _autoscale_x_check: CheckBox
var _min_x_spin: SpinBox
var _max_x_spin: SpinBox
var _grid_check: CheckBox
var _legend_check: CheckBox

var _selected_y_channels: Array[String] = []

func _init() -> void:
	custom_minimum_size = Vector2(480, 420)
	mouse_filter = Control.MOUSE_FILTER_STOP

func _ready() -> void:
	add_theme_stylebox_override("panel", MirenaUITheme.create_panel_stylebox(0.92, 0.85, 12, 10, 6))
	_build_ui()

func populate_from_panel(panel: FloatingGraphPanel) -> void:
	target_panel = panel
	if target_panel == null:
		return
	_title_input.text = target_panel.graph_title
	_type_select.selected = target_panel.graph_type
	_duration_spin.value = target_panel.time_window_s
	_selected_y_channels = target_panel.y_channels.duplicate()
	_autoscale_y_check.button_pressed = target_panel.auto_scale_y
	_min_y_spin.value = target_panel.manual_min_y
	_max_y_spin.value = target_panel.manual_max_y
	_autoscale_x_check.button_pressed = target_panel.auto_scale_x
	_min_x_spin.value = target_panel.manual_min_x
	_max_x_spin.value = target_panel.manual_max_x
	_grid_check.button_pressed = target_panel.show_grid
	_legend_check.button_pressed = target_panel.show_legend
	_on_type_selected(target_panel.graph_type)
	_refresh_signal_list()

func _build_ui() -> void:
	for c in get_children():
		c.queue_free()

	var root_vbox := VBoxContainer.new()
	root_vbox.add_theme_constant_override("separation", 8)
	add_child(root_vbox)

	# Dialog Title
	var header := HBoxContainer.new()
	var dlg_lbl := Label.new()
	dlg_lbl.text = "GRAPH CONFIGURATION"
	dlg_lbl.add_theme_color_override("font_color", MirenaUITheme.ACCENT_COLOR)
	dlg_lbl.add_theme_font_size_override("font_size", 14)
	header.add_child(dlg_lbl)
	var sp := Control.new()
	sp.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header.add_child(sp)
	var close_btn := Button.new()
	close_btn.text = "✕"
	close_btn.custom_minimum_size = Vector2(24, 20)
	MirenaUITheme.style_button(close_btn)
	close_btn.pressed.connect(func(): cancelled.emit(); hide())
	header.add_child(close_btn)
	root_vbox.add_child(header)

	var sep1 := HSeparator.new()
	root_vbox.add_child(sep1)

	# Scrollable content
	var scroll := ScrollContainer.new()
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	scroll.custom_minimum_size.y = 300

	var content_vbox := VBoxContainer.new()
	content_vbox.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	content_vbox.add_theme_constant_override("separation", 8)

	# Row 1: Title & Type
	var row1 := HBoxContainer.new()
	row1.add_theme_constant_override("separation", 8)
	var t_lbl := Label.new()
	t_lbl.text = "Title:"
	row1.add_child(t_lbl)
	_title_input = LineEdit.new()
	_title_input.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row1.add_child(_title_input)

	var type_lbl := Label.new()
	type_lbl.text = "Type:"
	row1.add_child(type_lbl)
	_type_select = OptionButton.new()
	_type_select.add_item("Time Series", FloatingGraphPanel.GraphType.TIME_SERIES)
	_type_select.add_item("XY Plot", FloatingGraphPanel.GraphType.XY_PLOT)
	_type_select.add_item("Distribution (Histogram)", FloatingGraphPanel.GraphType.HISTOGRAM)
	_type_select.add_item("Live Gauge", FloatingGraphPanel.GraphType.GAUGE)
	_type_select.item_selected.connect(_on_type_selected)
	row1.add_child(_type_select)
	content_vbox.add_child(row1)

	# Row 2: Duration / History Window
	var row2 := HBoxContainer.new()
	row2.add_theme_constant_override("separation", 8)
	var dur_lbl := Label.new()
	dur_lbl.text = "Time Window (s):"
	row2.add_child(dur_lbl)
	_duration_spin = SpinBox.new()
	_duration_spin.min_value = 3.0
	_duration_spin.max_value = 60.0
	_duration_spin.step = 1.0
	_duration_spin.value = 15.0
	row2.add_child(_duration_spin)

	_grid_check = CheckBox.new()
	_grid_check.text = "Show Grid"
	row2.add_child(_grid_check)
	_legend_check = CheckBox.new()
	_legend_check.text = "Show Legend"
	row2.add_child(_legend_check)
	content_vbox.add_child(row2)

	# XY Plot Channel Pickers
	_xy_container = VBoxContainer.new()
	_xy_container.add_theme_constant_override("separation", 6)
	var xy_h := HBoxContainer.new()
	xy_h.add_theme_constant_override("separation", 8)
	var xl := Label.new()
	xl.text = "X-Axis Channel:"
	xy_h.add_child(xl)
	_xy_x_select = OptionButton.new()
	_xy_x_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	xy_h.add_child(_xy_x_select)
	var yl := Label.new()
	yl.text = "Y-Axis Channel:"
	xy_h.add_child(yl)
	_xy_y_select = OptionButton.new()
	_xy_y_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	xy_h.add_child(_xy_y_select)
	_xy_container.add_child(xy_h)
	_populate_xy_dropdowns()
	content_vbox.add_child(_xy_container)

	# Time-Series / Histogram / Gauge Channel Selector
	_ts_container = VBoxContainer.new()
	_ts_container.add_theme_constant_override("separation", 6)
	var sig_h := HBoxContainer.new()
	sig_h.add_theme_constant_override("separation", 8)
	var slbl := Label.new()
	slbl.text = "Search Signals:"
	sig_h.add_child(slbl)
	_search_input = LineEdit.new()
	_search_input.placeholder_text = "Filter channels..."
	_search_input.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_search_input.text_changed.connect(func(_t): _refresh_signal_list())
	sig_h.add_child(_search_input)

	_cat_filter = OptionButton.new()
	_cat_filter.add_item("All Categories", -1)
	for cat in TelemetrySignalRegistry.CATEGORY_NAMES.keys():
		_cat_filter.add_item(TelemetrySignalRegistry.CATEGORY_NAMES[cat], cat)
	_cat_filter.item_selected.connect(func(_i): _refresh_signal_list())
	sig_h.add_child(_cat_filter)
	_ts_container.add_child(sig_h)

	var sig_scroll := ScrollContainer.new()
	sig_scroll.custom_minimum_size.y = 120
	sig_scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL

	_signals_container = VBoxContainer.new()
	_signals_container.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sig_scroll.add_child(_signals_container)
	_ts_container.add_child(sig_scroll)
	content_vbox.add_child(_ts_container)

	# Axis Scaling Settings
	var scale_grp := HBoxContainer.new()
	scale_grp.add_theme_constant_override("separation", 8)
	_autoscale_y_check = CheckBox.new()
	_autoscale_y_check.text = "Auto Y-Scale"
	scale_grp.add_child(_autoscale_y_check)
	var min_l := Label.new(); min_l.text = "Min Y:"
	scale_grp.add_child(min_l)
	_min_y_spin = SpinBox.new()
	_min_y_spin.min_value = -20000.0; _min_y_spin.max_value = 20000.0
	scale_grp.add_child(_min_y_spin)
	var max_l := Label.new(); max_l.text = "Max Y:"
	scale_grp.add_child(max_l)
	_max_y_spin = SpinBox.new()
	_max_y_spin.min_value = -20000.0; _max_y_spin.max_value = 20000.0
	scale_grp.add_child(_max_y_spin)
	content_vbox.add_child(scale_grp)

	var scale_x_grp := HBoxContainer.new()
	scale_x_grp.add_theme_constant_override("separation", 8)
	_autoscale_x_check = CheckBox.new()
	_autoscale_x_check.text = "Auto X-Scale (XY)"
	scale_x_grp.add_child(_autoscale_x_check)
	var min_xl := Label.new(); min_xl.text = "Min X:"
	scale_x_grp.add_child(min_xl)
	_min_x_spin = SpinBox.new()
	_min_x_spin.min_value = -20000.0; _min_x_spin.max_value = 20000.0
	scale_x_grp.add_child(_min_x_spin)
	var max_xl := Label.new(); max_xl.text = "Max X:"
	scale_x_grp.add_child(max_xl)
	_max_x_spin = SpinBox.new()
	_max_x_spin.min_value = -20000.0; _max_x_spin.max_value = 20000.0
	scale_x_grp.add_child(_max_x_spin)
	content_vbox.add_child(scale_x_grp)

	scroll.add_child(content_vbox)
	root_vbox.add_child(scroll)

	# Action Buttons
	var btn_row := HBoxContainer.new()
	btn_row.add_theme_constant_override("separation", 10)

	var del_btn := Button.new()
	del_btn.text = "Delete Graph"
	del_btn.add_theme_color_override("font_color", Color(1.0, 0.45, 0.40))
	MirenaUITheme.style_button(del_btn)
	del_btn.pressed.connect(func():
		if target_panel != null:
			deleted.emit(target_panel.graph_id)
		hide()
	)
	btn_row.add_child(del_btn)

	var b_sp := Control.new()
	b_sp.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	btn_row.add_child(b_sp)

	var cancel_btn := Button.new()
	cancel_btn.text = "Cancel"
	MirenaUITheme.style_button(cancel_btn)
	cancel_btn.pressed.connect(func(): cancelled.emit(); hide())
	btn_row.add_child(cancel_btn)

	var apply_btn := Button.new()
	apply_btn.text = "Apply Changes"
	MirenaUITheme.style_button(apply_btn)
	apply_btn.pressed.connect(_on_apply_pressed)
	btn_row.add_child(apply_btn)

	root_vbox.add_child(btn_row)

func _on_type_selected(type_idx: int) -> void:
	var is_xy: bool = (type_idx == FloatingGraphPanel.GraphType.XY_PLOT)
	_xy_container.visible = is_xy
	_ts_container.visible = not is_xy

func _populate_xy_dropdowns() -> void:
	if _xy_x_select == null or _xy_y_select == null:
		return
	_xy_x_select.clear()
	_xy_y_select.clear()
	var all_sigs = TelemetrySignalRegistry.get_all_signals()
	for i in range(all_sigs.size()):
		var sig: TelemetrySignalRegistry.SignalDef = all_sigs[i]
		var label: String = "%s (%s)" % [sig.name, sig.unit]
		_xy_x_select.add_item(label, i)
		_xy_y_select.add_item(label, i)
		_xy_x_select.set_item_metadata(i, sig.id)
		_xy_y_select.set_item_metadata(i, sig.id)

func _refresh_signal_list() -> void:
	if _signals_container == null:
		return
	for c in _signals_container.get_children():
		c.queue_free()

	var q: String = _search_input.text if _search_input != null else ""
	var sel_cat: int = _cat_filter.get_selected_id() if _cat_filter != null else -1

	var sigs = TelemetrySignalRegistry.search_signals(q)
	for sig in sigs:
		if sel_cat >= 0 and sig.category != sel_cat:
			continue
		var row := HBoxContainer.new()
		var chk := CheckBox.new()
		chk.text = "%s  [%s]  —  %s" % [sig.name, sig.unit, sig.id]
		chk.button_pressed = _selected_y_channels.has(sig.id)
		chk.add_theme_color_override("font_color", sig.default_color)
		var sig_id: String = sig.id
		chk.toggled.connect(func(pressed: bool):
			if pressed and not _selected_y_channels.has(sig_id):
				_selected_y_channels.append(sig_id)
			elif not pressed and _selected_y_channels.has(sig_id):
				_selected_y_channels.erase(sig_id)
		)
		row.add_child(chk)
		_signals_container.add_child(row)

func _on_apply_pressed() -> void:
	if target_panel == null:
		hide()
		return

	target_panel.graph_title = _title_input.text
	target_panel.graph_type = _type_select.selected
	target_panel.time_window_s = _duration_spin.value
	target_panel.auto_scale_y = _autoscale_y_check.button_pressed
	target_panel.manual_min_y = _min_y_spin.value
	target_panel.manual_max_y = _max_y_spin.value
	target_panel.auto_scale_x = _autoscale_x_check.button_pressed
	target_panel.manual_min_x = _min_x_spin.value
	target_panel.manual_max_x = _max_x_spin.value
	target_panel.show_grid = _grid_check.button_pressed
	target_panel.show_legend = _legend_check.button_pressed

	if target_panel.graph_type == FloatingGraphPanel.GraphType.XY_PLOT:
		var x_idx: int = _xy_x_select.selected
		var y_idx: int = _xy_y_select.selected
		if x_idx >= 0 and x_idx < _xy_x_select.item_count:
			target_panel.x_channel = str(_xy_x_select.get_item_metadata(x_idx))
		if y_idx >= 0 and y_idx < _xy_y_select.item_count:
			target_panel.y_channels = [str(_xy_y_select.get_item_metadata(y_idx))]
	else:
		if _selected_y_channels.is_empty():
			_selected_y_channels = ["chassis.speed_kmh"]
		target_panel.y_channels = _selected_y_channels.duplicate()

	target_panel._update_title_display()
	target_panel.layout_changed.emit(target_panel)
	applied.emit(target_panel.serialize_config())
	hide()
