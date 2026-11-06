shader_type canvas_item;

// Set from main.gd based on the ROS image encoding: true for bgr8/bgra8, where
// the bytes were uploaded as RGB(A)8 and the R/B channels need swapping on the GPU.
uniform bool swap_rb = false;

void fragment() {
	vec4 c = texture(TEXTURE, UV);
	if (swap_rb) {
		c = c.bgra;
	}
	COLOR = vec4(c.rgb, 1.0);
}
