shader_type canvas_item;

void fragment() {
	COLOR.rgb = clamp(COLOR.rgb * (1.0 + 0.1*sin(TIME / 0.3)), 0.0, 1.0);
}
