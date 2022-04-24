#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  float dist = length(v_position.xyz - u_light_pos);
  float k_a = 0.1;
  float k_s = 0.5;
  float p = 100;
  vec3 k_d = u_color.xyz;
  vec3 l = normalize(u_light_pos - v_position.xyz);
  vec3 v = normalize(u_cam_pos - v_position.xyz);
  vec3 i_a = vec3(1, 1, 1);
  vec3 h = normalize(l + v);
  vec3 diffuse = k_d * (u_light_intensity / (dist * dist)) * max(0, dot(normalize(v_normal.xyz), l));
  out_color.xyz = k_a * i_a + diffuse + k_s * (u_light_intensity / (dist * dist)) * pow(max(0, dot(normalize(v_normal.xyz), h)), p);
  out_color.a = 1;
}

