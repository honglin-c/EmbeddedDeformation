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

string modelName = "cat";

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
glm::vec3 modelPos(0.0f, 0.0f, 0.0f);
glm::vec3 deformPos(2.0f, 0.0f, 0.0f);

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
    window = glfwCreateWindow(mode->width, mode->height, "Embedded Deformation", monitor, nullptr);
#else
    window = glfwCreateWindow(width, height, "Embedded Deformation", nullptr, nullptr);
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

    std::cout << "#1" << std::endl;

    // Initialize the deform graph
    deformGraph();

    std::cout << "#2" << std::endl;

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
    ResourceManager::LoadModel(_MODEL_PREFIX_"/sphere/sphere.obj", "sphere");
    ResourceManager::LoadModel(_MODEL_PREFIX_"/"+ modelName +"/"+ modelName +".obj", modelName);
    // Load the model to be deformed
    ResourceManager::LoadModel(_MODEL_PREFIX_"/"+ modelName +"/"+ modelName +".obj", "deform_" + modelName);

    // Load samples
    ResourceManager::LoadSample(_MODEL_PREFIX_"/" + modelName + "/" + modelName + ".sample", modelName);

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
    model = glm::translate(model, modelPos);
    phong.SetMatrix4("model", model);
    // Set material properties.
    phong.SetVector3f("material.ambient", glm::vec3(1.0f, 1.0f, 0.25f));
    phong.SetVector3f("material.diffuse", glm::vec3(1.0f, 1.0f, 0.2f));
    phong.SetVector3f("material.specular", glm::vec3(0.9f, 0.8f, 0.1f));
    phong.SetFloat("material.shininess", 0.3f);
    Model *originalModel = ResourceManager::GetModel(modelName);
    originalModel->Draw(phong);

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

    Sample *sample = ResourceManager::GetSample(modelName);

    // Draw original sample nodes.
    bool drawParticle = true;
    if (sample == NULL)
        std::cout << "cannot find the sample" << std::endl;
    else
    {
        if (drawParticle)
        {
            // std::cout << "sample size:" << sample->size() << std::endl;
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

    // Draw the deformed sample nodes
    std::vector<glm::vec3> deform_sample = dgraph->returnNodes();
    if (drawParticle)
    {
        try
        {
            for (auto pos:deform_sample)
            {
                glm::vec3 position = pos + deformPos;
                color.SetVector3f("givenColor", glm::vec3(1.0f, 0.0f, 0.0f));
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
         std::cout << "ERROR::DrawSample Failed to draw deformed sample" << std::endl;
        }
    }


    // Draw the deform model
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // model = glm::mat4();
    // model = glm::translate(model, modelPos);
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
    model = glm::translate(model, deformPos);
    phong.SetMatrix4("model", model);
    // Set material properties.
    phong.SetVector3f("material.ambient", glm::vec3(0.25f, 1.0f, 1.0f));
    phong.SetVector3f("material.diffuse", glm::vec3(1.0f, 1.0f, 0.2f));
    phong.SetVector3f("material.specular", glm::vec3(0.9f, 0.8f, 0.1f));
    phong.SetFloat("material.shininess", 0.3f);
    Model *deformModel = ResourceManager::GetModel("deform_" + modelName);
    deformModel->Draw(phong);

}

void simulate()
{
}

void deformGraph()
{
    Model *deformModel = ResourceManager::GetModel(modelName);
    vector<Vertex> vertices = deformModel->returnMeshVertices(0);
    vector<GraphVertex *> gvertices;
    std::cout << "~" << std::endl;

    for(auto v:vertices)
    {
        gvertices.push_back(new GraphVertex(v));
    }

    vector<Node *> gnodes;
    std::vector<glm::vec3> *sample = ResourceManager::GetSample(modelName);
    for(auto s:(*sample))
    {
        gnodes.push_back(new Node(s));
    }
    std::cout << "~" << std::endl;
    dgraph = new DeformGraph(modelName, gvertices, gnodes);
    // dgraph->print();
}

void doDeformation()
{
    double sin45, cos45, sin90, cos90;
    sin90 = 1.0; cos90 = 0.0;
    sin45 = cos45 = std::sqrt(2.0) / 2.0;
    // glm::mat3 rotation(cos45,  0.0f,  sin45,
    //                    0.0f,   1.0f,  0.0f,
    //                    -sin45, 0.0f,  cos45);
    Matrix3d rotation;
    // rotation << 1.0, 0.0, 0.0,
    //             0.0, 1.0, 0.0,
    //             0.0, 0.0, 1.0;
    rotation << cos45, 0.0f, sin45,
                0.0f,  1.0f, 0.0f,
                -sin45,0.0f, cos45;
    // rotation << cos90, 0.0f, sin90,
    //             0.0f,  1.0f, 0.0f,
    //             -sin90,0.0f, cos90;
    // rotation << 1.0f, 0.0f, 0.0f,
    //             0.0f, 1.0f, 0.0f,
    //             0.0f, 0.0f, 1.0f;

    std::cout << "rotation: " << std::endl;
    std::cout << rotation << std::endl;

    Vector3d translation(0.0, 0.0, 0.0);

    std::cout << "rotation inv:" << std::endl;
    Matrix3d inverse = rotation.inverse();
    std::cout << inverse << std::endl;
    // print(inverse[0]);
    // print(inverse[1]);
    // print(inverse[2]);

    AABB aabb;

    // pin the former part of the tail
    aabb.setAABB(-0.129416, 0.092578, 0.410533, 0.592776, -0.055549, 0.103149);
    dgraph->addFixedConstraint(aabb);

    // pin the body
    aabb.setAABB(-0.144363, 0.144363, 0.202625, 0.602783, -0.125669, 0.758065);
    dgraph->addFixedConstraint(aabb);

    // pin the rear legs
    // aabb.setAABB(-0.16731, 0.16731, -0.002068, 0.429177, -0.064655, 0.270305);
    // dgraph->addFixedConstraint(aabb);

    // pin the right leg
    // aabb.setAABB(-0.16731, 0.126305, -0.001683, 0.304127, 0.10341, 0.691942);
    // dgraph->addFixedConstraint(aabb);

    // pin the head
    aabb.setAABB(-0.1678, 0.171623, 0.022794, 0.700577, 0.689812, 0.973195);
    dgraph->addFixedConstraint(aabb);
    // rotation << cos90, 0.0f, sin90,
    //             0.0f,  1.0f, 0.0f,
    //             -sin90,0.0f, cos90;
    // translation = Vector3d(0.0, 0.0, 0.0);
    // dgraph->applyTransformation(rotation, translation, aabb);


    // transform the middle part of the tail
    // aabb.setAABB(-0.039399, 0.038201, 0.583398, 0.787471, -0.272848, -0.110492);
    // translation = Vector3d(0.2, -0.4, 0.03);
    // dgraph->applyTransformation(rotation, translation, aabb);

    // transform the end of the tail
    // aabb.setAABB(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);
    // aabb.setAABB(-0.032, 0.032, 0.757, 0.920, -0.360, -0.232); // longer end
    aabb.setAABB(-0.016785, 0.016785, 0.900588, 0.919792, -0.358661, -0.336833);
    translation = Vector3d(-0.20, -0.50, -0.30);
    dgraph->applyTransformation(rotation, translation, aabb);

    rotation << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;

    // transform the front paw
    aabb.setAABB(-0.10704, 0.10704, -0.001683, 0.052816, 0.587649, 0.691307);
    translation = Vector3d(0.0, 0.1, 0.25);
    dgraph->applyTransformation(rotation, translation, aabb);

    // transform the rear paw
    aabb.setAABB(-0.16731, 0.16731, -0.002068, 0.093362, 0.015152, 0.165848);
    translation = Vector3d(0.0, 0.05, -0.25);
    dgraph->applyTransformation(rotation, translation, aabb);

    // transform the left leg
    // aabb.setAABB(-0.1678, 0.171623, -0.002068, 0.919792, -0.358661, 0.973195);
    // translation = Vector3d(0.0, 0.3, 0.3);
    // dgraph->applyTransformation(rotation, translation, aabb);

    // aabb.setAABB(-0.16731, 0.126305, -0.001683, 0.304127, 0.10341, 0.691942);
    // dgraph->applyTransformation(rotation, translation, aabb);

    // translation = glm::vec3(0.0f, 0.0f, 0.0f);

    // dgraph->applyTransformation(rotation, translation, aabb);

    dgraph->optimize();

    dgraph->outputToFile();

    // Try to deform another model directly
    Model *deformModel = ResourceManager::GetModel("deform_" + modelName);
    deformModel->setMeshVertices(0, dgraph->returnVertices());

    // Load deform model
    // ResourceManager::LoadVertices(_MODEL_PREFIX_"/deform/" + modelName +".obj", "deform");

}

