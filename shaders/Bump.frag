#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return 0.0;
}

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = cross(n, t);
  mat3 tbn = mat3(t, b, n);
  float d_U = (texture(u_texture_3, vec2(v_uv.x + 1/u_texture_3_size.x, v_uv.y)).r - texture(u_texture_3, v_uv).r) * u_height_scaling * u_normal_scaling;
  float d_V = (texture(u_texture_3, vec2(v_uv.x, v_uv.y + 1/u_texture_3_size.y)).r - texture(u_texture_3, v_uv).r) * u_height_scaling * u_normal_scaling;
  vec3 n_0 = normalize(vec3(-d_U, -d_V, 1));
  vec3 n_d = tbn * n_0;


  float dist = length(v_position.xyz - u_light_pos);
  float k_a = 0.1;
  float k_s = 0.5;
  float p = 100;
  vec3 k_d = u_color.xyz;
  vec3 l = normalize(u_light_pos - v_position.xyz);
  vec3 v = normalize(u_cam_pos - v_position.xyz);
  vec3 i_a = vec3(1, 1, 1);
  vec3 h = normalize(l + v);
  vec3 diffuse = k_d * (u_light_intensity / (dist * dist)) * max(0, dot(normalize(n_d), l));
  out_color.xyz = k_a * i_a + diffuse + k_s * (u_light_intensity / (dist * dist)) * pow(max(0, dot(normalize(n_d), h)), p);
  out_color.a = 1;
}

