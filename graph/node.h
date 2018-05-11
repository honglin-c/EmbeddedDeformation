#ifndef _NODE_H_
#define _NODE_H_

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <set>


class Node
{
public:

    // Default constructor.
    Node();

    Node(glm::vec3 _position);

    ~Node();

    void setPosition(glm::vec3 _position);

    glm::vec3 getPosition() const;

    glm::vec3 getTranslation() const;

    glm::vec3 applyMapping(glm::vec3 p);

    void setTransformation(glm::mat3 &_rotation, glm::vec3 &_translation);

    glm::vec3 transformPosition(glm::vec3 vpos);

    glm::vec3 transformNormal(glm::vec3 normal);

    void addNeighbor(Node * n);

    // Rotation term
    float getRot();

    // Regularization term
    float getReg();

private:
	bool transformed; // if the node has been transformed
	glm::vec3 position;
	glm::mat3 rotation;
	glm::vec3 translation;
	std::set<Node *> neighbors; // Neighbor nodes

};

#endif
