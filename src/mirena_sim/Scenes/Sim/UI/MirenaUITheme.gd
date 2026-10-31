extends RefCounted
class_name MirenaUITheme

## Central visual theme and styling resource extracted from the Options Menu design system.
## Guarantees pixel-perfect aesthetic consistency across:
## - Top HUD Stats bar
## - Integrated Application Launcher
## - Configurable Floating Telemetry Graph panels
## - Graph Configuration Dialogs & Menus

const ACCENT_COLOR: Color = Color(0.745, 0.999, 1.0, 1.0) # Options Menu cyan
const TEXT_PRIMARY: Color = Color(0.90, 0.92, 0.96, 1.0)
const TEXT_SECONDARY: Color = Color(0.72, 0.77, 0.84, 1.0)
const TEXT_MUTED: Color = Color(0.52, 0.57, 0.64, 1.0)
const SEPARATOR_COLOR: Color = Color(0.38, 0.42, 0.48, 0.55)

const COLOR_SPEED: Color = Color(0.745, 1.0, 1.0, 1.0)
const COLOR_AX_ACCEL: Color = Color(0.40, 0.92, 0.65, 1.0)
const COLOR_AX_BRAKE: Color = Color(1.0, 0.65, 0.35, 1.0)
const COLOR_AY: Color = Color(0.40, 0.88, 0.95, 1.0)
const COLOR_AZ: Color = Color(0.78, 0.84, 0.90, 1.0)

static func create_panel_stylebox(
	bg_alpha: float = 0.72,
	border_alpha: float = 0.60,
	margin_h: int = 8,
	margin_v: int = 6,
	corner_rad: int = 4
) -> StyleBoxFlat:
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.12, 0.12, 0.13, bg_alpha)
	sb.border_color = Color(0.32, 0.35, 0.40, border_alpha)
	sb.border_width_left = 1
	sb.border_width_top = 1
	sb.border_width_right = 1
	sb.border_width_bottom = 1
	sb.corner_radius_top_left = corner_rad
	sb.corner_radius_top_right = corner_rad
	sb.corner_radius_bottom_right = corner_rad
	sb.corner_radius_bottom_left = corner_rad
	sb.content_margin_left = float(margin_h)
	sb.content_margin_right = float(margin_h)
	sb.content_margin_top = float(margin_v)
	sb.content_margin_bottom = float(margin_v)
	return sb

static func create_header_stylebox() -> StyleBoxFlat:
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.16, 0.17, 0.20, 0.88)
	sb.border_color = Color(0.30, 0.33, 0.38, 0.65)
	sb.border_width_left = 1
	sb.border_width_top = 1
	sb.border_width_right = 1
	sb.border_width_bottom = 0
	sb.corner_radius_top_left = 4
	sb.corner_radius_top_right = 4
	sb.corner_radius_bottom_right = 0
	sb.corner_radius_bottom_left = 0
	sb.content_margin_left = 8.0
	sb.content_margin_right = 8.0
	sb.content_margin_top = 4.0
	sb.content_margin_bottom = 4.0
	return sb

static func create_button_stylebox_normal() -> StyleBoxFlat:
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.18, 0.19, 0.22, 0.75)
	sb.border_color = Color(0.35, 0.38, 0.44, 0.60)
	sb.border_width_left = 1
	sb.border_width_top = 1
	sb.border_width_right = 1
	sb.border_width_bottom = 1
	sb.corner_radius_top_left = 3
	sb.corner_radius_top_right = 3
	sb.corner_radius_bottom_right = 3
	sb.corner_radius_bottom_left = 3
	sb.content_margin_left = 6.0
	sb.content_margin_right = 6.0
	sb.content_margin_top = 3.0
	sb.content_margin_bottom = 3.0
	return sb

static func create_button_stylebox_hover() -> StyleBoxFlat:
	var sb := create_button_stylebox_normal()
	sb.bg_color = Color(0.26, 0.28, 0.33, 0.90)
	sb.border_color = Color(0.50, 0.55, 0.65, 0.85)
	return sb

static func create_button_stylebox_pressed() -> StyleBoxFlat:
	var sb := create_button_stylebox_normal()
	sb.bg_color = Color(0.10, 0.11, 0.13, 0.95)
	sb.border_color = Color(0.745, 0.999, 1.0, 0.90)
	return sb

static func create_pill_badge_stylebox(is_active: bool, is_warning: bool = false) -> StyleBoxFlat:
	var sb := StyleBoxFlat.new()
	if is_warning:
		sb.bg_color = Color(0.26, 0.20, 0.10, 0.85)
		sb.border_color = Color(0.75, 0.55, 0.20, 0.80)
	elif is_active:
		sb.bg_color = Color(0.10, 0.24, 0.18, 0.85)
		sb.border_color = Color(0.25, 0.70, 0.45, 0.80)
	else:
		sb.bg_color = Color(0.14, 0.18, 0.24, 0.85)
		sb.border_color = Color(0.30, 0.50, 0.75, 0.75)
	sb.border_width_left = 1
	sb.border_width_top = 1
	sb.border_width_right = 1
	sb.border_width_bottom = 1
	sb.corner_radius_top_left = 4
	sb.corner_radius_top_right = 4
	sb.corner_radius_bottom_right = 4
	sb.corner_radius_bottom_left = 4
	sb.content_margin_left = 8.0
	sb.content_margin_right = 8.0
	sb.content_margin_top = 2.0
	sb.content_margin_bottom = 2.0
	return sb

static func style_button(btn: Button) -> void:
	if btn == null:
		return
	btn.add_theme_stylebox_override("normal", create_button_stylebox_normal())
	btn.add_theme_stylebox_override("hover", create_button_stylebox_hover())
	btn.add_theme_stylebox_override("pressed", create_button_stylebox_pressed())
	btn.add_theme_color_override("font_color", TEXT_PRIMARY)
	btn.add_theme_color_override("font_hover_color", Color(1.0, 1.0, 1.0, 1.0))
	btn.add_theme_color_override("font_pressed_color", ACCENT_COLOR)

static func style_popup_menu(menu: PopupMenu) -> void:
	if menu == null:
		return
	menu.add_theme_stylebox_override("panel", create_panel_stylebox(0.92, 0.75, 8, 8, 4))
	menu.add_theme_color_override("font_color", TEXT_PRIMARY)
	menu.add_theme_color_override("font_hover_color", ACCENT_COLOR)
	menu.add_theme_color_override("font_separator_color", TEXT_MUTED)
