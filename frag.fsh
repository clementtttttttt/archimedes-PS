#version 330 core
out vec3 color;
in vec3 fragcolour;
in vec2 gridsz;
in vec2 gridoff;

in vec4 gl_FragCoord;

in vec2 screensize;

void main(){
 
  vec2 newcoord = gl_FragCoord.xy - (screensize * gridoff);

  float ratio = screensize.x / screensize.y;

  if(gridsz.x != 0 
  && (((mod((newcoord.x), gridsz.x * screensize.x)) < 0.98)
  || ((mod((newcoord.y), gridsz.y * screensize.y)) < 0.98))){

  	color = vec3(1.0f);
  }
  else{
	color = fragcolour;
  }
}

