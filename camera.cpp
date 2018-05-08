#include "camera.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

extern int WIDTH, HEIGHT;
extern GLuint width, height;

// Constructor with vectors
Camera::Camera(glm::vec3 position, glm::vec3 up, GLfloat yaw, GLfloat pitch) :
        Front(glm::vec3(1.0f, 0.0f, 0.0f)),
        MovementSpeed(SPEED),
        MouseSensitivity(SENSITIVTY),
        Zoom(ZOOM),
        NearClippingPlaneDistance(NEAR),
        FarClippingPlaneDistance(FAR),
        Offset(0, 0, 0)
{
    try
    {
        this->Position = position;
        this->WorldUp = up;
        this->Yaw = yaw;
        this->Pitch = pitch;
        this->updateCameraVectors();
    }
    catch(...)
    {
        throw;
    }
}

// Constructor with scalar values
Camera::Camera(GLfloat posX, GLfloat posY, GLfloat posZ, GLfloat upX, GLfloat upY, GLfloat upZ, GLfloat yaw,
               GLfloat pitch) :
        Front(glm::vec3(0.0f, 0.0f, -1.0f)),
        MovementSpeed(SPEED),
        MouseSensitivity(SENSITIVTY),
        Zoom(ZOOM),
        NearClippingPlaneDistance(NEAR),
        FarClippingPlaneDistance(FAR),
        Offset(0, 0, 0)
{
    this->Position = glm::vec3(posX, posY, posZ);
    this->WorldUp = glm::vec3(upX, upY, upZ);
    this->Yaw = yaw;
    this->Pitch = pitch;
    this->updateCameraVectors();
}

// Returns the view matrix calculated using Eular Angles and the LookAt Matrix
glm::mat4 Camera::GetViewMatrix() const
{
    glm::vec3 offset;
    offset += Front * Offset.x;
    offset += Up * Offset.y;
    offset += Right * Offset.z;
    return glm::lookAt(this->Position + offset, this->Position + offset + this->Front, this->Up);
}

glm::mat4 Camera::GetProjectionMatrix() const
{
    return glm::perspective(Zoom, (float) width / (float) height, NearClippingPlaneDistance, FarClippingPlaneDistance);
}

glm::mat4 Camera::getVPMatrix() const
{
    return GetProjectionMatrix() * GetViewMatrix();
}

// Return the position of the camera.
glm::vec3 Camera::GetViewPosition()
{
    glm::vec3 offset;
    offset += Front * Offset.x;
    offset += Up * Offset.y;
    offset += Right * Offset.z;
    return Position + offset;
}

void Camera::KeyBoardControl(bool *keys, GLfloat deltaTime)
{}

// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
void Camera::ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime)
{
    GLfloat velocity = this->MovementSpeed * deltaTime;
    if (direction == FORWARD)
        this->Position += this->Front * velocity;
    if (direction == BACKWARD)
        this->Position -= this->Front * velocity;
    if (direction == LEFT)
        this->Position -= this->Right * velocity;
    if (direction == RIGHT)
        this->Position += this->Right * velocity;
}

// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
void Camera::ProcessLeftMouseMovement(GLfloat xoffset, GLfloat yoffset)
{
    this->Position.x += 0.01 * (xoffset * glm::dot(glm::vec3(1.0, 0.0, 0.0), this->Right)
                                + yoffset * glm::dot(glm::vec3(0.0, 1.0, 0.0), this->Right));
    this->Position.y += 0.01 * (xoffset * glm::dot(glm::vec3(1.0, 0.0, 0.0), this->Front)
                                + yoffset * glm::dot(glm::vec3(0.0, 1.0, 0.0), this->Front));
}

// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
void Camera::ProcessRightMouseMovement(GLfloat xoffset, GLfloat yoffset, GLboolean constrainPitch)
{
    xoffset *= this->MouseSensitivity;
    yoffset *= this->MouseSensitivity;

    this->Yaw += 0.3 * xoffset;
    this->Pitch += 0.3 * yoffset;

    // Make sure that when pitch is out of bounds, screen doesn't get flipped
    if (constrainPitch)
    {
        if (this->Pitch > 89.0f)
            this->Pitch = 89.0f;
        if (this->Pitch < -89.0f)
            this->Pitch = -89.0f;
    }

    // Update Front, Right and Up Vectors using the updated Eular angles
    this->updateCameraVectors();
}

// Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
void Camera::ProcessMouseScroll(GLfloat yoffset)
{
    if (this->Zoom >= 1.0f && this->Zoom <= 45.0f)
        this->Zoom -= yoffset;
    if (this->Zoom <= 1.0f)
        this->Zoom = 1.0f;
    if (this->Zoom >= 45.0f)
        this->Zoom = 45.0f;
}

// Calculates the front vector from the Camera's (updated) Eular Angles
void Camera::updateCameraVectors()
{
    // Calculate the new Front vector
    glm::vec3 front;
    front.x = cos(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
    front.y = sin(glm::radians(this->Pitch));
    front.z = sin(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
    this->Front = glm::normalize(front);
    // Also re-calculate the Right and Up vector
    this->Right = glm::normalize(glm::cross(this->Front,
                                            this->WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
    this->Up = glm::normalize(glm::cross(this->Right, this->Front));
}
