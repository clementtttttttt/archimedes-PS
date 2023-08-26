#include <GL/gl.h>
#include <GLFW/glfw3.h>
void gui_tick_begin(GLFWwindow *win);
void gui_tick_end();
void gui_init(GLFWwindow *win);
void world_init(GLFWwindow *win);
bool gui_mouse_hovering();

enum A_TOOLS{
    T_NULL,T_BOX,T_CIRCLE,T_MOVE,T_ROT,T_DRAG, T_PIN_CENTRE, T_WELD, T_HINGE
};


struct con_data{
    bool is_mouse = false;
    unsigned long type;
};
