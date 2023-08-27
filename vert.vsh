#version 330 core
layout(location = 0) in vec3 inpos;
layout(location = 1) in vec3 colourz;

out vec3 fragcolour;
uniform mat4 transform;

uniform vec2 gridsz_in;
uniform vec2 gridoff_in;
uniform vec2 screensize_in;

out vec2 gridsz;
out vec2 gridoff;
out vec2 screensize;

void main(){
  gl_Position = transform*vec4(inpos,1.0f);
  gl_Position.w = 1.0;

  fragcolour = colourz;

   gridsz = gridsz_in;
   gridoff = gridoff_in;
   screensize = screensize_in;
}
