#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

float h(vec2 uv) {
  // You may want to use this helper function...
  return 0.0;
}

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  vec4 p = in_position + in_normal * texture(u_texture_3, in_uv).r * u_height_scaling;
  v_position = u_model * p;
  v_normal = normalize(u_model * in_normal);
  v_uv = in_uv;
  v_tangent = normalize(u_model * in_tangent);
  gl_Position = u_view_projection * u_model * p;
}
