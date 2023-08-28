#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include "world.hpp"
#include <unistd.h>
#include <cstring>

#include "gui.hpp"
GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path);

using namespace std;

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

GLuint programID;

GLfloat vertices[65535*3] ,vert_colours[65535*3];
    GLFWwindow* window;
        unsigned long long count=0;
bool prio_fail = false;

int main(void)
{


    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(EXIT_FAILURE);

    // cout << "default shader lang: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;

    // select opengl version
    // int major, minor, rev;
    // glfwGetVersion(&major, &minor, &rev);
    // cout << "glfw major.minor " << major << "." << minor << "." << rev << endl;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES,4);


    window = glfwCreateWindow(1280, 720, "Archimedes physics sandbox", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);

    cout << "OpenGL shader language version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;

    glewExperimental = true; // Needed for core profile
if (glewInit() != GLEW_OK) {
    fprintf(stderr, "Failed to initialize GLEW\n");
    return -1;
}

    programID = LoadShaders("./vert.vsh", "./frag.fsh");
    glUseProgram(programID);

    unsigned int vertexbuf;
    glGenBuffers(1, &vertexbuf);
    unsigned int colourbuf;
    glGenBuffers(1,&colourbuf);


	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	GLuint ColourID;
	glGenVertexArrays(1, &ColourID);
	glBindVertexArray(ColourID);


    glfwSetKeyCallback(window, key_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);



    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    gui_init(window);
    world_init((void*)window);

    while (!glfwWindowShouldClose(window))
    {

        count=0;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


// 1st attribute buffer : vertices
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, vertexbuf);

        glVertexAttribPointer(
   0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
   3,                  // size
   GL_FLOAT,           // type
   GL_FALSE,           // normalized?
   0,                  // stride
   (void*)0            // array buffer offset
    );



        gui_tick_begin(window);
        world_draw_tick(vertices);

        glBufferData(GL_ARRAY_BUFFER, count*sizeof(GLfloat), vertices, GL_DYNAMIC_DRAW);



        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colourbuf);

    glVertexAttribPointer(
    1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
    3,                                // size
    GL_FLOAT,                         // type
    GL_FALSE,                         // normalized?
    0,                                // stride
    (void*)0                          // array buffer offset
);



                glBufferData(GL_ARRAY_BUFFER, count*sizeof(GLfloat), vert_colours, GL_DYNAMIC_DRAW);



        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, count); // Starting from vertex 0; 3 vertices total -> 1 triangle

                glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        gui_tick_end();



        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
