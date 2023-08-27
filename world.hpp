#include <GL/gl.h>
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
#include "types.hpp"

void world_start();
void world_draw_tick(GLfloat *vert);
void world_init(void *window);

a_vec2 get_mouse_world_pos(GLFWwindow *win,glm::mat4 *trans);
void world_create_box(a_vec2 mxy,a_vec2 sz);
void pushvert(GLfloat x, GLfloat y, GLfloat z, unsigned int c);
void world_create_box_bb(a_vec2 lb,a_vec2 rt);
cpShape * world_create_circle(cpVect mxy,double rad);
bool world_toggle_pause(bool in);
cpShape *world_shape_point_query(a_vec2 org_mxy);
void world_move_shape(cpShape *mhape, cpVect curr_mxy);
void world_rotate_shape(cpShape *mshape, double rad);
void world_remove_shape(cpShape *shape);
enum A_CON_TS{
    A_CON_NULL,A_CON_HINGE,A_CON_SPRING,A_CON_GEAR

};

void                     world_create_constraint(cpShape *shape, a_vec2 xy, unsigned long type);
void                     world_create_constraint2(cpShape *shape,cpShape *second, a_vec2 xy, unsigned long type, double prop=1);

enum A_BODY_TS{
    A_BODY_NULL,A_BODY_CIRCLE,A_BODY_POLY
};

struct shape_data{
    unsigned int colour_rgba;
    double z;
    unsigned int type;
};
double world_get_tps();
