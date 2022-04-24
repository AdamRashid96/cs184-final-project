#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 w_0 = normalize(u_cam_pos - v_position.xyz);
  vec3 w_i = reflect(-w_0, normalize(v_normal.xyz));
  out_color.xyz = texture(u_texture_cubemap, w_i).xyz;
  out_color.a = 1;
}
