/* Adapted from the online tuturials by Joey de Vries found at 'http://learnopengl.com/'
 * Defines performs calculations for a basic virtual camera to be used in OpenGL
 */
#ifndef _CAMERA_H
#define _CAMERA_H

#include <GL/glew.h>
#include <glm/glm.hpp>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Default camera values
const GLfloat YAW = -90.0f;
const GLfloat PITCH = 0.0f;
const GLfloat SPEED = 20.0f;
const GLfloat SENSITIVTY = 0.25f;
const GLfloat ZOOM = 45.0f;
const GLfloat NEAR = 0.1f;
const GLfloat FAR = 3000.0f;


// An abstract camera class that processes input and calculates the corresponding Eular Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:
    // Camera Attributes
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
	glm::vec3 Offset;
    // Eular Angles
    GLfloat Yaw;
    GLfloat Pitch;
    // Camera options
    GLfloat MovementSpeed;
    GLfloat MouseSensitivity;
    GLfloat Zoom;
    GLfloat NearClippingPlaneDistance;
    GLfloat FarClippingPlaneDistance;

    // Constructor with vectors
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
           GLfloat yaw = YAW, GLfloat pitch = PITCH);

    // Constructor with scalar values
    Camera(GLfloat posX, GLfloat posY, GLfloat posZ, GLfloat upX, GLfloat upY, GLfloat upZ, GLfloat yaw, GLfloat pitch);

    // Returns the view matrix calculated using Eular Angles and the LookAt Matrix
    virtual glm::mat4 GetViewMatrix() const;
	virtual glm::mat4 GetProjectionMatrix() const;
	virtual glm::mat4 getVPMatrix() const;
    // Return the position of the camera.
	virtual glm::vec3 GetViewPosition();
	virtual void KeyBoardControl(bool *keys, GLfloat deltaTime);
    // Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
	virtual void ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime);

    // Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    virtual void ProcessLeftMouseMovement(GLfloat xoffset, GLfloat yoffset);

    // Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    virtual void ProcessRightMouseMovement(GLfloat xoffset, GLfloat yoffset, GLboolean constrainPitch = true);

    // Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    virtual void ProcessMouseScroll(GLfloat yoffset);

protected:
    // Calculates the front vector from the Camera's (updated) Eular Angles
    void updateCameraVectors();
};

#endif
