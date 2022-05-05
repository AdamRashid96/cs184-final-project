#version 330
in vec4 in_position;
in vec4 in_world_pos;
in float in_radius;
in vec2 in_uv;
in vec4 in_color;

// GLSL Hacker automatic uniforms:
uniform mat4 u_view;
uniform mat4 u_projection;

out vec2 v_uv;
out vec4 v_color;
void main() {

  // model << particle->radius, 0, 0, particle->position.x, 0, particle->radius, 0, particle->position.y, 0, 0, particle->radius, particle->position.z, 0, 0, 0, 1;
  mat4 model = mat4(
    vec4(in_radius, 0, 0, 0),
    vec4(0, in_radius, 0, 0),
    vec4(0, 0, in_radius, 0),
    vec4(in_world_pos.x, in_world_pos.y, in_world_pos.z, 1)
  );

  // vec4 vert_world = model * in_position;
  // vec4 view_pos = u_view * in_world_pos;
  // float dist = -view_pos.z;
  // gl_Position = u_projection * (view_pos + vec4(dist * vert_world.xy, 0, 1));
  
  mat4 modelView = u_view * model;
  
  //First colunm.
  modelView[0][0] = in_radius; 
  modelView[0][1] = 0.0; 
  modelView[0][2] = 0.0; 

  // Second colunm.
  modelView[1][0] = 0.0; 
  modelView[1][1] = in_radius; 
  modelView[1][2] = 0.0; 

  // Thrid colunm.
  modelView[2][0] = 0.0; 
  modelView[2][1] = 0.0; 
  modelView[2][2] = in_radius; 
  
   vec4 P = modelView * in_position;
   gl_Position = u_projection * P;
  
  v_uv = in_uv;
  v_color = in_color;
}