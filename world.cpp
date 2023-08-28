#include <iostream>
#include <unistd.h>
#include <mutex>
#include <array>
#include <chrono>

#include <GL/glew.h>
#include "world.hpp"



#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "types.hpp"
#include "gui.hpp"
#include <mapbox/earcut.hpp>

#include <thread>

#include <GLFW/glfw3.h>
void world_start();

extern     GLFWwindow* window;
extern GLuint programID;
extern unsigned long long count;

extern GLfloat vert_colours[];

void storevert(GLfloat x, GLfloat y, GLfloat z, GLfloat *verts, unsigned int c){
    verts[count] =x;
    verts[count+1] = y;
    verts[count+2] = z;

    vert_colours[count] = (c >>24) / (double)0xff;
    vert_colours[count+1] = ((c >>16)&0xff) / (double)0xff;
    vert_colours[count+2] = ((c >>8 )&0xff) / (double)0xff;



    count = count + 3;

}

void pushvert(GLfloat x, GLfloat y, GLfloat z, unsigned int c){
    extern GLfloat vertices[];
    storevert(x,y,z,vertices, c);
}

unsigned long fov;

a_vec2 cam_pos={0,0},start_cam_pos;

bool press_ack = false;
double start_mx,start_my;
unsigned long cam_speed = 240;
double cam_scale = 1;

bool do_pan=true;

 std::mutex phys_tick_lock;

cpShape *ground;
void world_onscroll(GLFWwindow *window, double xoff, double yoff){
    cam_scale += yoff/100;
    cam_scale = std::max(cam_scale,std::abs(yoff/100));



}


bool phys_paused = false;
 cpSpace *space;

void world_move_shape(cpShape *mshape, cpVect curr_mxy){
        if(!mshape) return;
        phys_tick_lock.lock();
            cpBodySetPosition(cpShapeGetBody(mshape), {curr_mxy.x, curr_mxy.y});
            cpSpaceReindexShapesForBody(space,cpShapeGetBody(mshape));
        phys_tick_lock.unlock();

}


void world_rotate_shape(cpShape *mshape, double rad){
        if(!mshape) return;
        phys_tick_lock.lock();
            cpBodySetAngle(cpShapeGetBody(mshape), rad);
            cpSpaceReindexShapesForBody(space,cpShapeGetBody(mshape));
        phys_tick_lock.unlock();

}

bool world_toggle_pause(bool in){
    if(in)
        return phys_paused = !phys_paused;
    else return phys_paused;
}

double clock_to_mil(clock_t ticks){
    // units/(units/time) => time (seconds) * 1000 = milliseconds
    return (ticks/(double)CLOCKS_PER_SEC)*1000.0;
}

double target_tps = 1000;
unsigned long ticks = 0;
double tps;
double t_off;
void world_phys_tick(){


            double begin = glfwGetTime();

    while(1){

        cpFloat timeStep = 1.0/target_tps;

        if(phys_paused) {std::this_thread::yield() ;begin = glfwGetTime(); continue;}

        std::this_thread::yield();

        phys_tick_lock.lock();

        cpSpaceStep(space, timeStep);

        phys_tick_lock.unlock();
std::chrono::duration<double, std::ratio<1>> delta;

        {

            auto start = std::chrono::steady_clock::now();

            do{
                auto end = std::chrono::steady_clock::now();
                 delta = end-start;


            }
            while(delta.count() < (timeStep + t_off));
        }

        double end = glfwGetTime();

        ++ticks;

        if((end-begin) >= 1){

            tps = double(ticks);
            ticks = 0;
            begin += 1;


                if((timeStep + (t_off - (1/tps-1/target_tps))) > 0)
                t_off -= (1/tps - 1/target_tps);
                    }

    }


}

double world_get_tps(){

    return tps;
}

cpShape *world_shape_point_query(a_vec2 org_mxy){
    return cpSpacePointQueryNearest(space, org_mxy, 0, CP_SHAPE_FILTER_ALL, NULL);
}



void world_init(void *window){
    glfwSetScrollCallback((GLFWwindow*)window, world_onscroll);


  // Create an empty space.
space = cpSpaceNew();
  cpSpaceSetGravity(space, cpv(0, -9.8));

    ground = cpBoxShapeNew(cpSpaceGetStaticBody(space), cam_scale*2*1280/720, 0.3, 0);
  cpShapeSetFriction(ground, 0.6);
  cpSpaceAddShape(space, ground);
  cpShapeSetFilter(ground,cpShapeFilterNew(CP_NO_GROUP,1,1));

            struct shape_data *dat = new struct shape_data;
            dat->z = 0.5;
            dat->type = A_BODY_POLY;
            dat->colour_rgba = std::rand() | 0xff;
    cpShapeSetUserData(ground, dat);
    std::thread physics_thread(world_phys_tick);

    cpSpaceSetCollisionSlop(space, 0.00000000001);

    physics_thread.detach();
}

a_vec2 get_mouse_world_pos(GLFWwindow *win,glm::mat4 *trafns){
        a_vec2 ret;

                double ratio;
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (double) height;

    glm::mat4 trans = glm::translate( glm::scale(glm::mat4(1.0f), glm::vec3(cam_scale/ratio,cam_scale,1)),glm::vec3(-cam_pos.x,-cam_pos.y,0));


        glm::mat4 invtrans = glm::inverse(trans);

        glfwGetCursorPos(window, &ret.x,&ret.y);


        ret.x = ret.x / (double)width * 2 - 1;
        ret.y = ret.y / (double)height * 2 - 1;

        glm::vec4 transed = invtrans * glm::vec4(ret.x,-ret.y,0,1.0f);


        return {transed.x,transed.y};

}

void world_remove_shape(cpShape *shape){
    phys_tick_lock.lock();
    cpBody *bod = cpShapeGetBody(shape);

    delete (struct shape_data*) cpShapeGetUserData(shape);

    cpBodyEachConstraint(bod, [](cpBody *bod, cpConstraint *con, void *data){
        cpSpaceRemoveConstraint(cpBodyGetSpace(bod), con);
        cpConstraintFree(con);

    },NULL);


    cpSpaceRemoveShape(space,shape);
    cpShapeFree(shape);
    if(bod != cpSpaceGetStaticBody(cpBodyGetSpace(bod))){
                cpSpaceRemoveBody(cpBodyGetSpace(bod), bod);
                cpBodyFree(bod);
    }

    phys_tick_lock.unlock();
}


double world_default_density=2710; //aluminium

void world_create_box(a_vec2 mxy,a_vec2 sz){
            phys_tick_lock.lock();

            double mass = sz.x * sz.y * world_default_density;

            cpBody *boxbody = cpBodyNew(mass,cpMomentForBox(mass,sz.x,sz.y));
            if(cpMomentForBox(mass,sz.x,sz.y) <= 0){ phys_tick_lock.unlock(); return;}
            cpShape *box = cpBoxShapeNew(boxbody,sz.x,sz.y,0);
            cpShapeSetFilter(box, cpShapeFilterNew(CP_NO_GROUP,1,1));
            cpBodySetPosition(boxbody, cpv(mxy.x,mxy.y));
                        cpShapeSetFriction(box, 0.4d);


            struct shape_data *dat = new struct shape_data;
            dat->z = 0.5;
            dat->type = A_BODY_POLY;
            dat->colour_rgba = std::rand() | 0xff;

            cpShapeSetUserData(box, dat);

            cpSpaceAddBody(space,boxbody);
            cpSpaceAddShape(space, box);
            phys_tick_lock.unlock();
}

void world_create_box_bb(a_vec2 lb,a_vec2 rt){


            world_create_box(a_vec2(cpBBCenter(cpBBNew(lb.x,lb.y,rt.x,rt.y))), a_vec2(std::abs(lb.x-rt.x),std::abs(lb.y-rt.y)));
}

cpShape * world_create_circle(cpVect mxy,double rad){

            phys_tick_lock.lock();

            cpBody *cirbody = cpBodyNew(world_default_density * (CP_PI*(rad*rad)),cpMomentForCircle(world_default_density * (CP_PI*(rad*rad)),0,rad,cpvzero));
            if(cpMomentForCircle(1,0,rad,cpvzero) <= 0){ phys_tick_lock.unlock(); return NULL;}
            cpShape *cir = cpCircleShapeNew(cirbody,rad,cpvzero);
            cpShapeSetFilter(cir, cpShapeFilterNew(CP_NO_GROUP,1,1));

            struct shape_data *dat = new struct shape_data;
            dat->z = 0.5;
            dat->type = A_BODY_CIRCLE;
            dat->colour_rgba = std::rand() | 0xff;

            cpShapeSetUserData(cir,(cpDataPointer)dat);
            cpBodySetPosition(cirbody, cpv(mxy.x,mxy.y));
            cpShapeSetFriction(cir,0.4d);

            cpSpaceAddBody(space,cirbody);
            cpSpaceAddShape(space, cir);
            phys_tick_lock.unlock();
            return cir;

}

void world_handle_inputs(glm::mat4 *trans){
    if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS){

        if(do_pan){

            if(press_ack == false){

                press_ack = true;
                glfwGetCursorPos(window, &start_mx,&start_my);
                start_cam_pos = cam_pos;

            }

            double mx,my;

            int width, height;
            glfwGetFramebufferSize(window, &width, &height);

            glfwGetCursorPos(window, &mx,&my);
            cam_pos.x = (start_mx-mx)/(width/2)+start_cam_pos.x;
            cam_pos.y = (my-start_my)/(height/2)+start_cam_pos.y;

        }

    }

    else {
        press_ack=false;
    }



}

struct s_render_info{
    GLfloat *verts;
    glm::mat4 trans;
};


void shape_renderer(cpShape *shape, void *renderinf){

    double z = ((struct shape_data*)cpShapeGetUserData(shape))->z;
    unsigned int c= ((struct shape_data*)cpShapeGetUserData(shape))->colour_rgba;
    if(((struct shape_data*)cpShapeGetUserData(shape))->type != A_BODY_CIRCLE ){

    unsigned int v = cpPolyShapeGetCount(shape);


    // Create array
using Point = std::array<double, 2>;
std::vector<std::vector<Point>> polygon;

// The index type. Defaults to uint32_t, but you can also pass uint16_t if you know that your
// data won't have more than 65536 vertices.
using N = uint32_t;

    std::vector<Point> currp;

    for(unsigned int i = 0; i < v; ++i){
        cpVect vert = cpPolyShapeGetVert (shape,i);

        vert = cpBodyLocalToWorld(cpShapeGetBody(shape), vert);


        currp.push_back({vert.x,vert.y});
    }
    polygon.push_back(currp);

    std::vector<N> indices = mapbox::earcut<N>(polygon);




    for(unsigned int i=0;i<indices.size();++i){

        pushvert(currp[indices[i]][0], currp[indices[i]][1],z, c);
    }

    }
    else{
        cpVect spos = cpBodyGetPosition(cpShapeGetBody(shape));

        cpFloat rad = cpCircleShapeGetRadius(shape);

    // Create array
using Point = std::array<double, 2>;
std::vector<std::vector<Point>> polygon;


using N = unsigned long;
        cpFloat rot = cpBodyGetAngle(cpShapeGetBody(shape));

    std::vector<Point> currp;
        double cx, cy;
        for(int i=0;i<24;++i){

                cx = spos.x + cos(M_PI*2.0d/24.0d*i+rot)*rad;
                cy = spos.y + sin(M_PI*2.0d/24.0d*i+rot)*rad;
                currp.push_back({cx,cy});


        }

            polygon.push_back(currp);

    std::vector<N> indices = mapbox::earcut<N>(polygon);



    for(unsigned int i=0;i<indices.size();++i){

        pushvert(currp[indices[i]][0], currp[indices[i]][1],z,c);


    }

    }


}
struct query_inf world_query_shape(cpShape *mshape, a_vec2 pos){


                            struct query_inf inf = {INFINITY, NULL, mshape};


                            cpSpacePointQuery(space, pos, 0, CP_SHAPE_FILTER_ALL,
                            [](cpShape *shape, cpVect point, cpFloat dist, cpVect grad, void *data){
                                struct query_inf *qinf = (struct query_inf*)data;
                                struct shape_data *dat = (struct shape_data*)cpShapeGetUserData(shape);
                                struct shape_data *q_dat;
                                if(qinf->shape)
                                    q_dat = (struct shape_data*)cpShapeGetUserData(qinf->shape);
                                struct shape_data *m_dat;
                                if(qinf->mshape)
                                    m_dat = (struct shape_data*) cpShapeGetUserData(qinf->mshape);

                                if(shape != qinf->mshape && (qinf->mshape? (dat->z <= m_dat->z): true)){
                                    if((qinf->shape? (dat->z <= q_dat->z ): true)){
                                        if(dat->z == q_dat->z){
                                            if(dist < qinf->dist) goto skip;
                                        }

                                        qinf->shape = shape;
                                        qinf->dist = dist;
                                        skip:;

                                    }

                                }

                            }
                            ,&inf);
        return inf;
}

cpConstraint*   world_create_constraint2(cpShape *shape, cpShape *shape2, a_vec2 xy, unsigned long type, double prop){

    phys_tick_lock.lock();

    cpBody *bod = cpShapeGetBody(shape);
    cpConstraint *con;
    switch(type){
        case A_CON_HINGE:
            con = cpPivotJointNew(bod, cpShapeGetBody(shape2 ),xy);
            break;
        case A_CON_GEAR:{
            con = cpGearJointNew(bod, cpShapeGetBody(shape2), 0, prop);

            double off = cpBodyGetAngle(cpConstraintGetBodyB(con)) - cpBodyGetAngle(cpConstraintGetBodyA(con));
            cpGearJointSetPhase(con, off);
            break;
        }
        case A_CON_SPRING:
        {
            con = cpDampedSpringNew(bod, cpShapeGetBody(shape2), cpvzero, cpvzero, prop, 99, 99);

            break;
        }
    }

    struct con_data *user_dat = new struct con_data;
    user_dat->is_mouse = false;
    user_dat->type = type;

    cpConstraintSetUserData(con, user_dat);

    cpSpaceAddConstraint(space, con);

    if(type != A_CON_SPRING){
        cpConstraintSetCollideBodies(con, cpFalse);
    }
    phys_tick_lock.unlock();

    return con;


}

cpConstraint*                     world_create_constraint(cpShape *shape, a_vec2 xy, unsigned long type){

        return world_create_constraint2(shape,ground,xy,type);


}
void world_draw_tick(GLfloat *verts){

        double ratio;
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (double) height;

        glViewport(0,0,width,height);


    glm::mat4 trans = glm::translate( glm::scale(glm::mat4(1.0f), glm::vec3(cam_scale/ratio,cam_scale,1)),glm::vec3(-cam_pos.x,-cam_pos.y,0));

    if(!gui_mouse_hovering()){ world_handle_inputs(&trans); }



    cpBB screen_bb =  cpBBNewForExtents({cam_pos.x,cam_pos.y}, 1/cam_scale*ratio,999); //FIXME: absolutely no idea on how adding a yoffset fixes stuff. will figure it out later
    struct s_render_info inf = { verts ,glm::inverse(trans)};


    phys_tick_lock.lock();


    cpSpaceBBQuery(space, screen_bb, CP_SHAPE_FILTER_ALL, shape_renderer, &inf);

    phys_tick_lock.unlock();
         pushvert(screen_bb.l-1,screen_bb.t+1,0.99,0x87CEEBff);
        pushvert(screen_bb.l-1,screen_bb.b-1,0.99,0x87CEEBff);
        pushvert(screen_bb.r+1,screen_bb.b-1,0.99,0x87CEEBff);
        pushvert(screen_bb.l-1,screen_bb.t+1,0.99,0x87CEEBff);
        pushvert(screen_bb.r+1,screen_bb.t+1,0.99,0x87CEEBff);
        pushvert(screen_bb.r+1,screen_bb.b-1,0.99,0x87CEEBff);

    unsigned int transformLoc = glGetUniformLocation(programID, "transform");


    glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(trans));


}


