#ifndef _NODE_H_
#define _NODE_H_

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>


class Node
{
public:

    // Default constructor.
    Node();

    Node(glm::vec3 _position);

    ~Node();

    void setPosition(glm::vec3 _position);

    glm::vec3 getPosition() const;

    glm::vec3 applyMapping(glm::vec3 p);


private:
	glm::vec3 position;
	glm::mat3 rotation;
	glm::vec3 translation;
};

#endif
