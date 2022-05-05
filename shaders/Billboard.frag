#version 330

in vec2 v_uv;
in vec4 v_color;

out vec4 out_color;

void main() {
  vec2 uv_translate = vec2(v_uv.x * 2 - 1, v_uv.y * 2 - 1);

  if (dot(uv_translate, uv_translate) > 1) {
      discard;
  }
  out_color = v_color;// + vec4(0.3 * noise3(10 * v_uv), 1.0);
}

