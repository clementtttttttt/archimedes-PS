#include <GL/glew.h>


#include "gui.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "types.hpp"
#include "imgui_impl_opengl3.h"
#include "world.hpp"

#include <mapbox/earcut.hpp>
#include <iostream>
#include <array>
#include <mutex>


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
void gui_init(GLFWwindow *win){

IMGUI_CHECKVERSION();
ImGui::CreateContext();
ImGuiIO& io = ImGui::GetIO();
io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

// Setup Platform/Renderer backends
ImGui_ImplGlfw_InitForOpenGL(win, true);          // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
ImGui_ImplOpenGL3_Init();

}

unsigned int current_tool;

bool got_origin = false;

a_vec2 org_mxy, curr_mxy;
a_vec2 mshape_orgmxy;
cpShape *mshape;
cpShape *mouse_circle;
cpConstraint *mouse_spring;
cpConstraint *mcon;

a_vec2 grid_sz = {0.05,0.05};
a_vec2 grid_off = {0,0};
bool gui_tool_snap_to_grid = false;

double gui_drag_tool_stiff = 9999, gui_drag_tool_damp = 99;
extern cpSpace *space;
void gui_handle_inputs(GLFWwindow *window){
    if(mshape){
        cpShapeFilter filt = cpShapeGetFilter(mshape);

    }
    if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && !gui_mouse_hovering()){

            int w,h;
            glfwGetFramebufferSize(window, &w, &h);

            if(got_origin == false){

                got_origin = true;

                org_mxy = get_mouse_world_pos(window,NULL);
                mshape = world_shape_point_query(org_mxy);
                mcon = 0;

                if(mshape)
                    mshape_orgmxy = cpBodyGetPosition(cpShapeGetBody(mshape));

                if(current_tool == T_DRAG && mshape && cpBodyGetType(cpShapeGetBody(mshape)) != CP_BODY_TYPE_STATIC){
                    mouse_circle = world_create_circle(curr_mxy,0.1);
                    cpBodySetType(cpShapeGetBody(mouse_circle), CP_BODY_TYPE_STATIC);
                    cpShapeSetFilter(mouse_circle, CP_SHAPE_FILTER_NONE);

                    mouse_spring = cpDampedSpringNew(cpShapeGetBody(mouse_circle), cpShapeGetBody(mshape), cpvzero, cpBodyWorldToLocal(cpShapeGetBody(mshape), org_mxy), 0, gui_drag_tool_stiff, gui_drag_tool_damp);
                    cpSpaceAddConstraint(space,mouse_spring );
                    struct con_data *m_data = new struct con_data;
                    m_data->is_mouse = true;
                    m_data -> type = A_CON_SPRING;
                    cpConstraintSetUserData(mouse_spring, m_data);
                }
                if(mshape){

                                            struct query_inf{double dist; cpShape *shape; cpShape *mshape;};

                            struct query_inf inf = {INFINITY, NULL, mshape};
                                                        cpVect pos = cpBodyGetPosition(cpShapeGetBody(mshape));

                switch(current_tool){
                    case T_PIN_CENTRE:


                            cpSpacePointQuery(space, pos, 0, CP_SHAPE_FILTER_ALL,
                            [](cpShape *shape, cpVect point, cpFloat dist, cpVect grad, void *data){
                                struct query_inf *qinf = (struct query_inf*)data;
                                if(shape != qinf->mshape){
                                    if(dist < qinf->dist){
                                        qinf->shape = shape;
                                        qinf->dist = dist;

                                    }

                                }

                            }
                            ,&inf);
                            if(inf.shape){
                                world_create_constraint2(mshape, inf.shape,  pos, A_CON_HINGE);
                            }
                            else{
                                world_create_constraint(mshape,  pos, A_CON_HINGE);

                            }                    break;
                    case T_HINGE:


                            cpSpacePointQuery(space, org_mxy, 0, CP_SHAPE_FILTER_ALL,
                            [](cpShape *shape, cpVect point, cpFloat dist, cpVect grad, void *data){
                                struct query_inf *qinf = (struct query_inf*)data;
                                if(shape != qinf->mshape){
                                    if(dist < qinf->dist){
                                        qinf->shape = shape;
                                        qinf->dist = dist;
                                    }

                                }

                            }
                            ,&inf);
                            if(inf.shape){
                                world_create_constraint2(mshape, inf.shape,  org_mxy, A_CON_HINGE);
                            }
                            else{
                                world_create_constraint(mshape,  org_mxy, A_CON_HINGE);

                            }
                    break;
                    case T_WELD:
                            cpSpacePointQuery(space, org_mxy, 0, CP_SHAPE_FILTER_ALL,
                            [](cpShape *shape, cpVect point, cpFloat dist, cpVect grad, void *data){
                                struct query_inf *qinf = (struct query_inf*)data;
                                if(shape != qinf->mshape){
                                    if(dist < qinf->dist){
                                        qinf->shape = shape;
                                        qinf->dist = dist;
                                    }

                                }

                            }
                            ,&inf);
                            if(inf.shape){
                                world_create_constraint2(mshape, inf.shape,  org_mxy, A_CON_HINGE);
                                world_create_constraint2(mshape, inf.shape, org_mxy, A_CON_GEAR);
                            }
                            else{
                                world_create_constraint(mshape,  org_mxy, A_CON_HINGE);
                                world_create_constraint(mshape, org_mxy, A_CON_GEAR);

                            }
                    break;

                }
                }
            }


            curr_mxy = get_mouse_world_pos(window,NULL);

            if(gui_tool_snap_to_grid){

                    org_mxy.x = round(org_mxy.x/grid_sz.x)*grid_sz.x;
                    org_mxy.y = round(org_mxy.y/grid_sz.y)*grid_sz.y;
                    curr_mxy.x = round(curr_mxy.x/grid_sz.x)*grid_sz.x;
                    curr_mxy.y = round(curr_mxy.y/grid_sz.y)*grid_sz.y;


            }

                unsigned int c=0xddddddff;

            switch(current_tool){
                case T_BOX:
                    pushvert(org_mxy.x,org_mxy.y,-0.999,c);
                    pushvert(curr_mxy.x,org_mxy.y,-0.999,c);
                    pushvert(curr_mxy.x,curr_mxy.y,-0.999,c);
                    pushvert(org_mxy.x,org_mxy.y,-0.999,c);
                    pushvert(org_mxy.x,curr_mxy.y,-0.999,c);
                    pushvert(curr_mxy.x,curr_mxy.y,-0.999,c);

                break;
                case T_MOVE:
                    world_move_shape(mshape,mshape_orgmxy + (curr_mxy - org_mxy));

                break;
                case T_DRAG:
                    if(mouse_circle){
                        world_move_shape(mouse_circle,curr_mxy);
                    }
                break;
                case T_ROT:
                    world_rotate_shape(mshape,curr_mxy.angle(mshape_orgmxy));
                break;

                case T_CIRCLE:

                    using Point = std::array<double, 2>;
                    std::vector<std::vector<Point>> polygon;

                    std::vector<Point> currp;
                    double cx, cy;

                    double rad = curr_mxy.dist(org_mxy);

                    for(int i=0;i<24;++i){

                        cx = org_mxy.x + cos(M_PI*2.0d/24.0d*i)*rad;
                        cy = org_mxy.y + sin(M_PI*2.0d/24.0d*i)*rad;
                        currp.push_back({cx,cy});
                    }

                    polygon.push_back(currp);

                    std::vector<unsigned long> indices = mapbox::earcut<unsigned long>(polygon);


                    for(unsigned int i=0;i<indices.size();++i){

                        pushvert(currp[indices[i]][0], currp[indices[i]][1],-0.999,0xddddddff);
                    }

                break;




            }



    }
    else{

        if(got_origin){
        switch(current_tool){
                case T_BOX:
                        world_create_box_bb(org_mxy,curr_mxy);

                break;

                case T_CIRCLE:
                        world_create_circle(org_mxy, curr_mxy.dist(org_mxy));
                break;


        }

            got_origin = false;
        }

        if(mouse_circle && mouse_spring){
            world_remove_shape(mouse_circle);
            delete (struct con_data*)cpConstraintGetUserData(mouse_spring);
            cpConstraintDestroy(mouse_spring);
            mouse_spring = 0;
            mouse_circle = 0;
        }

    }

}

void gui_tick_begin(GLFWwindow *window){

    gui_handle_inputs(window);

    extern GLuint programID;

    unsigned int gridsz_loc = glGetUniformLocation(programID, "gridsz_in");
    unsigned int gridoff_loc = glGetUniformLocation(programID, "gridoff_in");
    unsigned int screensz_loc = glGetUniformLocation(programID, "screensize_in");

    extern double cam_scale;
    extern a_vec2 cam_pos;
            double ratio;
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glUniform2f(screensz_loc, width,height);
        ratio = width / (double) height ;


    if(gui_tool_snap_to_grid){
        glUniform2f(gridsz_loc, grid_sz.x * cam_scale / ratio/2, grid_sz.y * cam_scale/2);
        glUniform2f(gridoff_loc, -fmod(cam_pos.x*cam_scale/ratio/2,grid_sz.x*cam_scale/ratio/2) + 0.5, -fmod(cam_pos.y*cam_scale/2.0 + (cam_scale-1)*0.5 ,grid_sz.y*cam_scale/2));
    }else{
        glUniform2f(gridsz_loc, 0, 0);
        glUniform2f(gridoff_loc, 0, 0);

    }



}

bool gui_mouse_hovering(){

    return ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)  || ImGui::IsAnyItemHovered();

}



void gui_tick_end(){
ImGui_ImplOpenGL3_NewFrame();
ImGui_ImplGlfw_NewFrame();
ImGui::NewFrame();

    ImGui::Begin("Tools");

    if(ImGui::Button("Box")){current_tool = T_BOX;};

    ImGui::SameLine();

    if(ImGui::Button("Circle")){current_tool = T_CIRCLE;};

    ImGui::Separator();

    if(ImGui::Button("Centre Hinge")){current_tool = T_PIN_CENTRE;}
    ImGui::SameLine();
    if(ImGui::Button("Weld")){current_tool = T_WELD;}
    ImGui::SameLine();
    if(ImGui::Button("Hinge")) {current_tool = T_HINGE;};
    ImGui::SameLine();
    if(ImGui::Button("Spring")) {current_tool = T_SPRING;};


    ImGui::Separator();


    if(ImGui::Button("Select+Move")){current_tool = T_MOVE;};

    ImGui::SameLine();

    if(ImGui::Button("Drag")){current_tool = T_DRAG;};

    ImGui::SameLine();

    if(ImGui::Button("Rotate")){current_tool = T_ROT;};

    if(current_tool == T_DRAG ){
        if(ImGui::InputDouble("Drag Stiffness", &gui_drag_tool_stiff)){}
            if(ImGui::InputDouble("Drag Damping", &gui_drag_tool_damp)){}
    }

    ImGui::Separator();


    ImGui::Checkbox("Snap to grid", &gui_tool_snap_to_grid);
    ImGui::InputDouble("Grid size X", &grid_sz.x);
    ImGui::InputDouble("Grid size Y", &grid_sz.y);


    ImGui::End();

    ImGui::Begin("Simulation Control");

    ImGui::LabelText("##tpscounter", "Ticks Per Second=%lf", world_get_tps());
   extern double target_tps;
    ImGui::InputDouble("Target Ticks Per Second", &target_tps);


    ImGui::Separator();


    std::string playtext = world_toggle_pause(false) ? "Stopped" : "Running";
    if(ImGui::Button(playtext.c_str())){
        world_toggle_pause(true);
    };



    double gravity;

    cpVect g = cpSpaceGetGravity(space);
    gravity = g.y;

    if(ImGui::InputDouble("Gravity", &gravity)){

        cpSpaceSetGravity(space, {0,gravity});
    }



    ImGui::End();

    if(mshape){
        ImGui::Begin("Properties");

        double mass = cpBodyGetMass(cpShapeGetBody(mshape));
        if(ImGui::InputDouble("Mass(KGs)", &mass) && mass > 0){
            if(cpBodyGetType(cpShapeGetBody(mshape)) == CP_BODY_TYPE_DYNAMIC){

                cpBodySetMass(cpShapeGetBody(mshape),mass);
                cpBodySetMoment(cpShapeGetBody(mshape), cpMomentForBox2(mass, cpShapeGetBB(mshape)));
            }
        }

        double fric = cpShapeGetFriction(mshape);
        if(ImGui::InputDouble("Friction(0=none)", &fric)){
            if(fric >=0)
            cpShapeSetFriction(mshape, fric);
        }

        struct shape_data *dat = ((struct shape_data*)cpShapeGetUserData(mshape));

        if(ImGui::InputScalar("Colour(RGBA)", ImGuiDataType_U32, &dat->colour_rgba, NULL, NULL, "%08X", ImGuiInputTextFlags_CharsHexadecimal)){
        }

        ImGui::InputDouble("Z-Value", &dat->z);

        cpBitmask collmask = cpShapeGetFilter(mshape).categories;

        if(ImGui::BeginListBox("Collision Layers")){
            for(unsigned int i=0;i<sizeof(cpBitmask)*8;++i){

                bool sel = collmask & (1 << i);
                if(ImGui::Checkbox(("Layer " + std::to_string(i)).c_str(), &sel)){
                    if(sel){
                        collmask |= (1 << i);
                    }
                    else{
                        collmask &= ~(1<<i);
                    }

                    cpShapeFilter filt = cpShapeGetFilter(mshape);
                    filt.categories = collmask;
                    filt.mask = filt.categories;
                    cpShapeSetFilter(mshape,filt);
                }


            }

            ImGui::EndListBox();

        }

        ImGui::Separator();


        if(ImGui::BeginListBox("Constraints")){
        unsigned long long sel_id = 0;

        cpBodyEachConstraint(cpShapeGetBody(mshape),
        [](cpBody *body, cpConstraint *con, void *in) {
                unsigned long *sel_id =(unsigned long*) in;
                struct con_data *dat = (struct con_data*)cpConstraintGetUserData(con);

                if(!dat) return;
                if(dat->is_mouse) return;

                std::string sel_text;
                switch(dat->type){
                    case A_CON_HINGE: sel_text = "HINGE"; break;
                    case A_CON_SPRING: sel_text = "SPRING"; break;
                    case A_CON_GEAR: sel_text = "GEAR"; break;
                    default: sel_text = "ERROR"; break;

                }

                if(ImGui::Selectable((sel_text + " ##" + std::to_string(++*sel_id)).c_str())){

                    mcon = con;
                }


          } , &sel_id);


        ImGui::EndListBox();
        }
        if(mcon){
            struct con_data *dat = (struct con_data*)cpConstraintGetUserData(mcon);
            if(dat){
                switch(dat->type){
                    case A_CON_HINGE:
                        {

                            cpVect anchora = cpPivotJointGetAnchorA(mcon);
                            cpVect anchorb = cpPivotJointGetAnchorB(mcon);

                            if(ImGui::InputDouble("Body A X Offset", &anchora.x)){
                                cpPivotJointSetAnchorA(mcon,anchora);
                            }
                            if(ImGui::InputDouble("Body A Y Offset", &anchora.y)){
                                cpPivotJointSetAnchorA(mcon,anchora);
                            }
                            if(ImGui::InputDouble("Body B X Offset", &anchorb.x)){
                                cpPivotJointSetAnchorB(mcon,anchorb);
                            }
                             if(ImGui::InputDouble("Body B Y Offset", &anchorb.y)){
                                cpPivotJointSetAnchorB(mcon,anchorb);
                            }
                            break;
                        }
                    case A_CON_SPRING:
                        {

                            cpVect anchora = cpDampedSpringGetAnchorA(mcon);
                            cpVect anchorb = cpDampedSpringGetAnchorB(mcon);

                            if(ImGui::InputDouble("Body A X Offset", &anchora.x)){
                                cpDampedSpringSetAnchorA(mcon,anchora);
                            }
                            if(ImGui::InputDouble("Body A Y Offset", &anchora.y)){
                                cpDampedSpringSetAnchorA(mcon,anchora);
                            }
                            if(ImGui::InputDouble("Body B X Offset", &anchorb.x)){
                                cpDampedSpringSetAnchorB(mcon,anchorb);
                            }
                             if(ImGui::InputDouble("Body B Y Offset", &anchorb.y)){
                                cpDampedSpringSetAnchorB(mcon,anchorb);
                            }

                            double len = cpDampedSpringGetRestLength(mcon);
                            double stiff = cpDampedSpringGetStiffness(mcon);
                            double damp = cpDampedSpringGetStiffness(mcon);

                            break;
                        }


                }

            }

        }

        ImGui::Separator();

        bool mshape_is_static = (cpBodyGetType(cpShapeGetBody(mshape)) == CP_BODY_TYPE_STATIC);

        extern std::mutex phys_tick_lock;
        if(ImGui::Checkbox("Static: ", &mshape_is_static)){
            phys_tick_lock.lock();
        if(mshape_is_static){
            cpBodySetType(cpShapeGetBody(mshape), CP_BODY_TYPE_STATIC);
        }
        else{
            cpBodySetType(cpShapeGetBody(mshape), CP_BODY_TYPE_DYNAMIC);
            cpBodySetMass(cpShapeGetBody(mshape), 1);
            cpBodySetMoment(cpShapeGetBody(mshape), cpMomentForBox(1, cpShapeGetBB(mshape).r - cpShapeGetBB(mshape).l, cpShapeGetBB(mshape).t - cpShapeGetBB(mshape).b));
        }
            phys_tick_lock.unlock();

        }

        ImGui::Separator();

        if(ImGui::Button("Delete")){
            world_remove_shape(mshape);


            mshape = 0;
            mcon = 0;
        }

        ImGui::End();
    }

// (Your code clears your framebuffer, renders your other stuff etc.)
ImGui::Render();
ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


}
