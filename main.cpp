#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
// OpenGL include
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>
// Framework
#include "stb_image.h"
#include "camera.h"
#include "model.h"
#include "shader.h"
#include "resource_manager.h"

// Deformation Graph
#include "graph/deformGraph.h"
#include "graph/graphVertex.h"
#include "graph/node.h"
#include "graph/boundingObject.h"

// Function prototypes.
void optionInit(GLFWwindow *window);

void shaderModelInit();

// Call-back functions.
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mode);

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

void mouse_callback(GLFWwindow *window, double xpos, double ypos);

void doMovement();

void display();

// Simulate.
void simulate();

// Deform the graph
void deformGraph();

// Deform the model
void doDeformation();

void print(glm::vec3 &v)
{
    std::cout << v[0] << ", " << v[1] << ", " << v[2] << std::endl;
}

bool keys[1024];
GLfloat lastX = 400, lastY = 300;
bool firstMouse = true;

GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

float epsilon = 0.000001f;

glm::vec3 lightPos(5.0f, 0.0f, 10.0f);
glm::vec3 catPos(0.0f, 0.0f, 0.0f);

GLFWwindow *window;
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));

// Window size.
int WIDTH, HEIGHT;
GLuint width = 1280, height = 720;

// Deformation Graph
DeformGraph * dgraph = nullptr;

// Start our application and run our Application loop.
int main()
{
    // Init GLFW.
    glfwInit();
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_AUTO_ICONIFY, GL_FALSE);
#endif
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 4);

#ifdef FULL_SCREEN
    // Create a "Windowed full screen" window in the primary monitor.
    GLFWmonitor *monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(monitor);
    glfwWindowHint(GLFW_RED_BITS, mode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
    glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);
    window = glfwCreateWindow(mode->width, mode->height, "FlightX", monitor, nullptr);
#else
    window = glfwCreateWindow(width, height, "Cloth Simulator", nullptr, nullptr);
#endif
    glfwMakeContextCurrent(window);

    // Set the required callback functions.
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions.
    glewExperimental = GL_TRUE;
    // Initialize GLEW to setup the OpenGL Function pointers.
    glewInit();

    // Define the viewport dimensions.
    glfwGetFramebufferSize(window, &WIDTH, &HEIGHT);
    glViewport(0, 0, WIDTH, HEIGHT);

    width = WIDTH;
    height = HEIGHT;

    std::cout << "GLFW version                : " << glfwGetVersionString() << std::endl;
    std::cout << "GL_VERSION                  : " << glGetString(GL_VERSION) << std::endl;
    std::cout << "GL_VENDOR                   : " << glGetString(GL_VENDOR) << std::endl;
    std::cout << "GL_RENDERER                 : " << glGetString(GL_RENDERER) << std::endl;
    std::cout << "GL_SHADING_LANGUAGE_VERSION : " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
    std::cout << "WINDOW WIDTH                : " << WIDTH << std::endl;
    std::cout << "WINDOW HEIGHT               : " << HEIGHT << std::endl;

    // Setup some OpenGL options.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    // Anti-aliasing.
    glEnable(GL_MULTISAMPLE);
    glfwSwapInterval(1);

    // Input Options.
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Do some initialization for our application (include loading shaders, models, etc.)
    shaderModelInit();

    // Initialize the deform graph
    deformGraph();

    doDeformation();

    // Loop.
    while (!glfwWindowShouldClose(window))
    {
        // Calculate delta time of current frame.
        GLfloat currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Check if any events have been activated (key pressed, mouse moved etc.) and call corresponding response functions.
        glfwPollEvents();
        doMovement();

        // Clear the color buffer.
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw scene, create transformations.
        display();

        // Simulate.
        simulate();

        // Swap the buffers.
        glfwSwapBuffers(window);
    }

    // Terminate GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();

    delete dgraph;

    return 0;
}


void shaderModelInit()
{
    // Load shaders.
    ResourceManager::LoadShader(_SHADER_PREFIX_"/phong/phong.vert",
                                _SHADER_PREFIX_"/phong/phong.frag",
                                "",
                                "phong");
    ResourceManager::LoadShader(_SHADER_PREFIX_"/color/color.vert",
                                _SHADER_PREFIX_"/color/color.frag",
                                "",
                                "color");

    // Load models.
    ResourceManager::LoadModel(_MODEL_PREFIX_"/cat/cat.obj", "cat");
    ResourceManager::LoadModel(_MODEL_PREFIX_"/rabbit/rabbit.obj", "rabbit");
    ResourceManager::LoadModel(_MODEL_PREFIX_"/sphere/sphere.obj", "sphere");
    // Load the model to be deformed
    ResourceManager::LoadModel(_MODEL_PREFIX_"/cat/cat.obj", "deform_cat");


    // Load samples
    ResourceManager::LoadSample(_MODEL_PREFIX_"/cat/cat.224.sample", "cat");

}

/* User input for camera-related and sphere-related transformations.
 * Moves/alters the camera/sphere positions based on user input.
 */
void doMovement()
{
    // Camera controls.
    if (keys[GLFW_KEY_W])
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (keys[GLFW_KEY_S])
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (keys[GLFW_KEY_A])
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (keys[GLFW_KEY_D])
        camera.ProcessKeyboard(RIGHT, deltaTime);

}

// Is called whenever a key is pressed/released via GLFW.
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mode)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);

    if (action == GLFW_PRESS)
        keys[key] = true;
    else if (action == GLFW_RELEASE)
        keys[key] = false;
}

void mouse_callback(GLFWwindow *window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    int left_state, right_state;
    left_state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    right_state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);

    GLfloat xoffset = xpos - lastX;
    GLfloat yoffset = ypos - lastY;
    lastX = xpos;
    lastY = ypos;

    if (left_state == GLFW_PRESS)
        camera.ProcessLeftMouseMovement(xoffset, yoffset);
    if (right_state == GLFW_PRESS)
        camera.ProcessRightMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}

void display()
{
    // Draw scene, create transformations.
    glm::mat4 model;
    glm::mat4 view = camera.GetViewMatrix();
    glm::mat4 projection = glm::perspective(camera.Zoom, (float) width / (float) height,
                                            camera.NearClippingPlaneDistance, camera.FarClippingPlaneDistance);

    // Draw the rigid bodies (cubes).
    Shader phong = ResourceManager::GetShader("phong");
    phong.Use();
    phong.SetMatrix4("view", view);
    phong.SetMatrix4("projection", projection);
    // Set the light and view position.
    phong.SetVector3f("light.position", lightPos);
    phong.SetVector3f("viewPos", camera.GetViewPosition());
    // Set light properties.
    phong.SetVector3f("light.ambient", glm::vec3(0.85f, 0.85f, 0.85f));
    phong.SetVector3f("light.diffuse", glm::vec3(0.85f, 0.85f, 0.85f));
    phong.SetVector3f("light.specular", glm::vec3(0.90f, 0.90f, 0.90f));
    // Set material properties.
    phong.SetVector3f("material.ambient", glm::vec3(1.0f, 1.0f, 1.0f));
    phong.SetVector3f("material.diffuse", glm::vec3(1.0f, 1.0f, 1.0f));
    phong.SetVector3f("material.specular", glm::vec3(0.0f));
    phong.SetFloat("material.shininess", 0.0f);

    // Draw the model Cat
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    model = glm::mat4();
    model = glm::translate(model, catPos);
    phong.SetMatrix4("model", model);
    // Set material properties.
    phong.SetVector3f("material.ambient", glm::vec3(1.0f, 1.0f, 0.25f));
    phong.SetVector3f("material.diffuse", glm::vec3(1.0f, 1.0f, 0.2f));
    phong.SetVector3f("material.specular", glm::vec3(0.9f, 0.8f, 0.1f));
    phong.SetFloat("material.shininess", 0.3f);
    Model *catModel = ResourceManager::GetModel("cat");
    catModel->Draw(phong);

    Shader color = ResourceManager::GetShader("color");
    color.Use();
    Model *sphere = ResourceManager::GetModel("sphere");
    color.SetMatrix4("view", view);
    color.SetMatrix4("projection", projection);
    glm::mat4 scaling(0.3f, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.3f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.3f, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f);
    color.SetMatrix4("scaling", scaling);

    Sample *sample = ResourceManager::GetSample("cat");

    // Draw sample particles.
    bool drawParticle = true;
    if (sample == NULL)
        std::cout << "cannot find the sample" << std::endl;
    else
    {
        if (drawParticle)
        {
            std::cout << "sample size:" << sample->size() << std::endl;
            try
            {
                std::vector<glm::vec3>::iterator pos;
                for (pos = sample->begin(); pos != sample->end(); pos++)
                {
                 glm::vec3 position = *pos;
                 color.SetVector3f("givenColor", glm::vec3(1.0f, 1.0f, 1.0f));
                 // Draw the particle.
                 model = glm::mat4();
                 // Translate to the particles position.
                 model = glm::translate(model, position);
                 color.SetMatrix4("model", model);
                 sphere->Draw(color);
                }
            }
            catch (std::exception e)
            {
             std::cout << "ERROR::LoadSample Failed to read sample files" << std::endl;
            }
        }
    }

    // Draw the deform model
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // model = glm::mat4();
    // model = glm::translate(model, catPos);
    // phong.SetMatrix4("model", model);
    // // Set material properties.
    // phong.SetVector3f("material.ambient", glm::vec3(0.0f, 1.0f, 0.0f));
    // phong.SetVector3f("material.diffuse", glm::vec3(0.0f, 1.0f, 0.0f));
    // phong.SetVector3f("material.specular", glm::vec3(0.0f, 1.0f, 0.0f));
    // phong.SetFloat("material.shininess", 0.3f);
    // Model *deformModel = ResourceManager::GetModel("deform");
    // deformModel->DrawVertices();

    // Draw the deform model Cat that is directly deformed
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    model = glm::mat4();
    model = glm::translate(model, (catPos + glm::vec3(1.0f, 0.0f, 0.0f)));
    phong.SetMatrix4("model", model);
    // Set material properties.
    phong.SetVector3f("material.ambient", glm::vec3(0.25f, 1.0f, 1.0f));
    phong.SetVector3f("material.diffuse", glm::vec3(1.0f, 1.0f, 0.2f));
    phong.SetVector3f("material.specular", glm::vec3(0.9f, 0.8f, 0.1f));
    phong.SetFloat("material.shininess", 0.3f);
    Model *deformCatModel = ResourceManager::GetModel("deform_cat");
    deformCatModel->Draw(phong);

}

void simulate()
{
}

void deformGraph()
{
    Model *catModel = ResourceManager::GetModel("cat");
    vector<Vertex> vertices = catModel->returnMeshVertices(0);
    vector<GraphVertex *> gvertices;

    for(auto v:vertices)
    {
        gvertices.push_back(new GraphVertex(v));
    }

    vector<Node *> gnodes;
    std::vector<glm::vec3> *sample = ResourceManager::GetSample("cat");
    for(auto s:(*sample))
    {
        gnodes.push_back(new Node(s));
    }

    dgraph = new DeformGraph(gvertices, gnodes);
    dgraph->print();
}

void doDeformation()
{
    float sin45, cos45, sin90, cos90;
    sin90 = 1.0f; cos90 = 0.0f;
    sin45 = cos45 = std::sqrt(2.0f) / 2.0f;
    glm::mat3 rotation(cos45,  0.0f,  sin45,
                       0.0f,   1.0f,  0.0f,
                       -sin45, 0.0f,  cos45);
    // glm::mat3 rotation(1.0f, 0.0f, 0.0f,
    //                    0.0f, 1.0f, 0.0f,
    //                    0.0f, 0.0f, 1.0f);
    glm::vec3 translation(0.1f, -1.0f, 0.1f);

    std::cout << "rotation inv:" << std::endl;
    glm::mat3 inverse = glm::inverse(rotation);
    print(inverse[0]);
    print(inverse[1]);
    print(inverse[2]);

    // transform tail
    AABB aabb;
    aabb.setAABB(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);
    // aabb.setAABB(-0.032, 0.032, 0.757, 0.920, -0.360, -0.232);

    dgraph->applyTransformation(rotation, translation, aabb);

    // tranform head
    // aabb.setAABB(-0.1678, 0.171623, 0.022794, 0.700577, 0.689812, 0.973195);
    // translation = glm::vec3(0.0f, 0.0f, 0.0f);

    // dgraph->applyTransformation(rotation, translation, aabb);

    dgraph->optimize();

    dgraph->outputToFile();

    // Try to deform another model directly
    Model *deformCatModel = ResourceManager::GetModel("deform_cat");
    deformCatModel->setMeshVertices(0, dgraph->returnVertices());

    // Load deform model
    ResourceManager::LoadVertices(_MODEL_PREFIX_"/deform/cat.obj", "deform");

}

