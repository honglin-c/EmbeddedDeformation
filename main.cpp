#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
// Measure time
#include <ctime>
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
// JSON Reader
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <cstdio>

// Deformation Graph
#include "graph/deformGraph.h"
#include "graph/graphVertex.h"
#include "graph/node.h"
#include "graph/boundingObject.h"

// Create delay
#include <chrono>
#include <thread>

// #define DEBUG
using namespace std;
using namespace rapidjson;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

bool animation = true;
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
bool simulate(int &step, const int max_iter);

// Deform the graph
void deformGraph();

// Deform the model
void doDeformation();

// Optimize
void optimize();

bool parse(int argc, char * argv[]);

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

glm::vec3 lightPos(-20.0f, 0.0f, 30.0f);
glm::vec3 modelPos(-1.0f,  0.0f, 0.0f);
glm::vec3 deformPos(1.0f, 0.0f, 0.0f);

GLFWwindow *window;
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));

// Window size.
int WIDTH, HEIGHT;
GLuint width = 1280, height = 720;

// Deformation Graph
DeformGraph * dgraph = nullptr;

using namespace std;

// Start our application and run our Application loop.
int main(int argc, char *argv[])
{
    // parse command line option
    if(!parse(argc, argv))
        return -1;

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
    while(!glfwWindowShouldClose(window))
    {
        // Do some initialization for our application (include loading shaders, models, etc.)
        shaderModelInit();

        // Initialize the deform graph
        deformGraph();

        doDeformation();

        if(!animation) optimize();
        
        cout << "Start Optimization" << endl;

        int step = 0, max_iter = 16;
        bool stopped = false;

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
            if(animation) 
                stopped = simulate(step, max_iter);

            // Swap the buffers.
            glfwSwapBuffers(window);

            if(step == max_iter || stopped)
            {
                sleep_for(nanoseconds(1000));
                break;
            }
        }
        delete dgraph;
    }

    // Terminate GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();

    return 0;
}

bool parse(int argc, char * argv[])
{
    // parse command line option
    if(argc == 2)
    {
        if(strcmp(argv[1], "-cat") == 0)
            modelName = "cat";
        else if (strcmp(argv[1], "-giraffe") == 0)
            modelName = "giraffe";
        else if (strcmp(argv[1], "-dcat") == 0)
        {
            modelName = "cat";
            animation = false;
        }
        else if (strcmp(argv[1], "-dgiraffe") == 0)
        {
            modelName = "giraffe";
            animation = false;           
        }
        else if (strcmp(argv[1], "--help") == 0)
        {
            cout << "option: " << endl;
            cout << "-cat: display original cat and cat model animation" << endl;
            cout << "-giraffe: display original giraffe and giraffe model animation" << endl;
            cout << "-dcat: display original cat and deformed cat model" << endl;
            cout << "-dgiraffe: display original giraffe and deformed giraffe model" << endl;
            return false;
        }
        else
        {
            cout << "invalid argument, see --help" << endl;
            return false;
        }
    }
    else if(argc != 1)
        return false;

    return true;
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

    // Draw the model Cat
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    model = glm::mat4();
    model = glm::translate(model, modelPos);
    phong.SetMatrix4("model", model);
    // Set material properties.
    phong.SetVector3f("material.ambient", glm::vec3(0.4f, 0.5f, 0.6f));
    phong.SetVector3f("material.diffuse", glm::vec3(0.5f, 0.7f, 0.9f));
    phong.SetVector3f("material.specular", glm::vec3(0.06f, 0.08f, 0.1f));
    phong.SetFloat("material.shininess", 0.0f);
    // Set the scaling
    glm::mat4 phong_scaling;
    if(modelName == "giraffe")
    {
        phong_scaling = glm::mat4(0.1f, 0.0f,  0.0f,  0.0f,
                                  0.0f,  0.1f, 0.0f,  0.0f,
                                  0.0f,  0.0f,  0.1f, 0.0f,
                                  0.0f,  0.0f,  0.0f,  1.0f);
    }
    else
    {
        phong_scaling = glm::mat4(1.0f,  0.0f,  0.0f,  0.0f,
                                  0.0f,  1.0f,  0.0f,  0.0f,
                                  0.0f,  0.0f,  1.0f,  0.0f,
                                  0.0f,  0.0f,  0.0f,  1.0f);
    }
    phong.SetMatrix4("scaling", phong_scaling);
    Model *originalModel = ResourceManager::GetModel(modelName);
    originalModel->Draw(phong);

#ifdef DEBUG

    Shader color = ResourceManager::GetShader("color");
    color.Use();
    Model *sphere = ResourceManager::GetModel("sphere");
    color.SetMatrix4("view", view);
    color.SetMatrix4("projection", projection);
    glm::mat4 color_scaling(0.3f, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.3f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.3f, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f);
    color.SetMatrix4("scaling", color_scaling);

    Sample *sample = ResourceManager::GetSample(modelName);

    // Draw original sample nodes.
    bool drawParticle = true;
    if (sample == NULL)
        std::cout << "cannot find the sample" << std::endl;
    else
    {
        if (drawParticle)
        {
            try
            {
                std::vector<glm::vec3>::iterator pos;
                for (pos = sample->begin(); pos != sample->end(); pos++)
                {
                    glm::vec3 position = *pos + modelPos;
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
                color.SetVector3f("givenColor", glm::vec3(0.0f, 1.0f, 0.0f));
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
#endif

    // Draw the deform model Cat that is directly deformed
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    phong.Use();
    model = glm::mat4();
    model = glm::translate(model, deformPos);
    phong.SetMatrix4("model", model);
    // Set material properties.
    phong.SetVector3f("material.ambient", glm::vec3(0.4f, 0.5f, 0.6f));
    phong.SetVector3f("material.diffuse", glm::vec3(0.5f, 0.7f, 0.9f));
    phong.SetVector3f("material.specular", glm::vec3(0.06f, 0.08f, 0.1f));
    phong.SetFloat("material.shininess", 0.0f);
    Model *deformModel = ResourceManager::GetModel("deform_" + modelName);
    deformModel->Draw(phong);

}

// Perform a single step in the  gradual deformation process
bool simulate(int &step, const int max_iter)
{
    if(step >= max_iter)
        return false;
    else 
        step++;

    bool stopped = dgraph->optimizeSingleStep();

    // Try to deform another model directly
    Model *deformModel = ResourceManager::GetModel("deform_" + modelName);
    deformModel->setMeshVertices(0, dgraph->returnVertices());

    return stopped;
}

void deformGraph()
{
    Model *deformModel = ResourceManager::GetModel(modelName);
    vector<Vertex> vertices = deformModel->returnMeshVertices(0);
    vector<GraphVertex *> gvertices;

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
    dgraph = new DeformGraph(modelName, gvertices, gnodes);
}

Document readJSON()
{
    string jsonPath = _MODEL_PREFIX_"/json/" + modelName + ".json";
    FILE* fp = fopen(jsonPath.c_str(), "r"); // non-Windows use "r"
    assert(fp != NULL);
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document d;
    d.ParseStream(is);
    fclose(fp);
    return d;
}

void doDeformation(int temp)
{
    AABB aabb;    
    Matrix3d rotation;
    Vector3d translation(0.0, 0.0, 0.0);
    rotation << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;

    // Parse JSON
    Document d = readJSON();
    assert(d.IsObject());
    // Pin
    if(d.HasMember("pin"))
    {
        for(Value::ConstMemberIterator it = d["pin"].MemberBegin(); it != d["pin"].MemberEnd(); it++)
        {
            assert(it->value.HasMember("obj"));
            string objName =  it->value["obj"].GetString();
            string objPath = _MODEL_PREFIX_"/"+ modelName +"/"+ objName;
            std::cout << "Pin: " << objPath << endl;
            aabb = ResourceManager::GetAABB(objPath.c_str());
            dgraph->addFixedConstraint(aabb);
        }
    }
    // Transform
    if(d.HasMember("deform"))
    {
        for(Value::ConstMemberIterator it = d["deform"].MemberBegin(); it != d["deform"].MemberEnd(); it++)

        {
            translation = Vector3d(0.0, 0.0, 0.0);
            rotation << 1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0;
            assert(it->value.HasMember("obj"));
            string objName = it->value["obj"].GetString();
            string objPath = _MODEL_PREFIX_"/"+ modelName +"/"+ objName;
            std::cout << "Deform: " << objPath << endl;
            aabb = ResourceManager::GetAABB(objPath.c_str());
            if(it->value.HasMember("translation"))
            {
                assert(it->value["translation"].IsArray());
                const Value& tl = it->value["translation"];
                assert(tl.IsArray() && tl.Size() == 3);
                translation = Vector3d(tl[0].GetFloat(), tl[1].GetFloat(), tl[2].GetFloat());
            }
            if(it->value.HasMember("rotation"))
            {
                assert(it->value["rotation"].IsArray());
                const Value& rot = it->value["rotation"];
                assert(rot.IsArray() && rot.Size() == 9);
                rotation << rot[0].GetFloat(), rot[1].GetFloat(), rot[2].GetFloat(),
                            rot[3].GetFloat(), rot[4].GetFloat(), rot[5].GetFloat(),
                            rot[6].GetFloat(), rot[7].GetFloat(), rot[8].GetFloat();
            }
            dgraph->applyTransformation(rotation, translation, aabb);
        }
    }
}

// Deprecated
void doDeformation()
{
    AABB aabb;
    Matrix3d rotation;
    Vector3d translation(0.0, 0.0, 0.0);
    rotation << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;

    double sin45, cos45, sin90, cos90, sin30, cos30;
    sin90 = 1.0; cos90 = 0.0;
    sin45 = cos45 = std::sqrt(2.0) / 2.0;
    sin30 = 1.0 / std::sqrt(3.0);
    cos30 = std::sqrt(3.0) / 3.0;

    if(modelName == "giraffe")
    {
        // pin the body
        aabb = ResourceManager::GetAABB(_MODEL_PREFIX_"/"+ modelName +"/"+ "body.obj");
        dgraph->addFixedConstraint(aabb);

        // Transform the front right hoof
        aabb = ResourceManager::GetAABB(_MODEL_PREFIX_"/"+ modelName +"/"+ "front_right_hoof.obj");

        translation = Vector3d(-2.5, 0.0, 0.8);
        dgraph->applyTransformation(rotation, translation, aabb);

        // Transform the front left hoof
        aabb = ResourceManager::GetAABB(_MODEL_PREFIX_"/"+ modelName +"/"+ "front_left_hoof2.obj");
        translation = Vector3d(2.5, 0.0, 0.8);
        dgraph->applyTransformation(rotation, translation, aabb);

        rotation << 1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;

        // // Transform the front right knee
        // aabb = ResourceManager::GetAABB(_MODEL_PREFIX_"/"+ modelName +"/"+ "front_right_knee.obj");
        // translation = Vector3d(-1.5, 0.0, 0.4);
        // dgraph->applyTransformation(rotation, translation, aabb);

        // // Transform the front left knee
        // aabb = ResourceManager::GetAABB(_MODEL_PREFIX_"/"+ modelName +"/"+ "front_left_knee.obj");
        // translation = Vector3d(1.5, 0.0, 0.4);
        // dgraph->applyTransformation(rotation, translation, aabb);

        // Transform the head
        aabb = ResourceManager::GetAABB(_MODEL_PREFIX_"/"+ modelName +"/"+ "head.obj");
        translation = Vector3d(0.0, -8.0, 5.0);
        dgraph->applyTransformation(rotation, translation, aabb);
    }
    else if (modelName == "cat")
    {
        double sin45, cos45, sin90, cos90;
        sin90 = 1.0; cos90 = 0.0;
        sin45 = cos45 = std::sqrt(2.0) / 2.0;
        Matrix3d rotation;
        rotation << cos45, 0.0f, sin45,
                    0.0f,  1.0f, 0.0f,
                    -sin45,0.0f, cos45;

        Vector3d translation(0.0, 0.0, 0.0);

        AABB aabb;

        // pin the body
        aabb.setAABB(-0.144363, 0.144363, 0.202625, 0.602783, -0.125669, 0.758065);
        dgraph->addFixedConstraint(aabb);

        // pin the head or rotate the head
        aabb.setAABB(-0.1678, 0.171623, 0.022794, 0.700577, 0.689812, 0.973195);
        // dgraph->addFixedConstraint(aabb);
        rotation << cos90, 0.0f, -sin90,
                    0.0f,  1.0f, 0.0f,
                    sin90,0.0f, cos90;  // rotate to 90 degree around the y axis
        translation = Vector3d(0.0, 0.0, 0.0);
        // dgraph->applyTransformation(rotation, translation, aabb);

        rotation << 1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;

        // transform the end of the tail
        aabb.setAABB(-0.016785, 0.016785, 0.900588, 0.919792, -0.358661, -0.336833);
        translation = Vector3d(0.0, -0.80, -0.45);
        dgraph->applyTransformation(rotation, translation, aabb);

        // transform the front paws
        aabb.setAABB(-0.10704, 0.10704, -0.001683, 0.052816, 0.587649, 0.691307);
        translation = Vector3d(0.0, 0.1, 0.25);
        dgraph->applyTransformation(rotation, translation, aabb);

        // transform the rear paws
        aabb.setAABB(-0.16731, 0.16731, -0.002068, 0.150633, -0.064655, 0.165848);
        translation = Vector3d(0.0, 0.00, -0.25);
        dgraph->applyTransformation(rotation, translation, aabb);
    }

}

void optimize()
{
    dgraph->optimize();

    // Try to deform another model directly
    Model *deformModel = ResourceManager::GetModel("deform_" + modelName);
    deformModel->setMeshVertices(0, dgraph->returnVertices());

}