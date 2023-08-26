#version 330 core
layout(location = 0) in vec3 inpos;
layout(location = 1) in vec3 colourz;

out vec3 fragcolour;
uniform mat4 transform;

void main(){
  gl_Position = transform*vec4(inpos,1.0f);
  gl_Position.y = gl_Position.y * 1.0f;
  gl_Position.w = 1.0;

  fragcolour = colourz;
}
