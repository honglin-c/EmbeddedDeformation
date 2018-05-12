#ifndef _NODE_H_
#define _NODE_H_

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <set>
#include <Eigen/Dense>

using Eigen::Vector3f;
using Eigen::VectorXf;
using Eigen::MatrixXf;
using Eigen::Matrix3f;

class Node
{
public:

    // Default constructor.
    Node();

    Node(glm::vec3 _position);

    ~Node();

    void addDeltaRotation(glm::mat3 delta);

    void addDeltaTranslation(glm::vec3 delta);

    void setPosition(glm::vec3 _position);

    glm::vec3 getPosition() const;

    Matrix3f matRotation() const;

    glm::vec3 getTranslation() const;

    glm::vec3 applyMapping(glm::vec3 p);

    void setTransformation(glm::mat3 &_rotation, glm::vec3 &_translation);

    glm::vec3 transformPosition(glm::vec3 vpos);

    glm::vec3 transformNormal(glm::vec3 normal);

    void addNeighbor(Node * n);

    std::set<Node *> getNeighbors();

    // Rotation term: not currently used
    float getRotValue();

    // Regularization term: not currently used
    float getRegValue();

    // Get [(c1*c2) (c1*c3) (c2*c3) (c1*c1-1) (c2*c2-1) (c3*c3-1)]
    VectorXf getRotTerm();

    // Get all neighbor's [Rj * (gk - gj) + gj + tj - (gk + tk)]
    MatrixXf getRegTerm();

    // Get a certain neighbor's [Rj * (gk - gj) + gj + tj - (gk + tk)]
    Vector3f getRegTerm(Node * neighbor);

private:
	bool transformed; // if the node has been transformed
	glm::vec3 position;
	glm::mat3 rotation;
	glm::vec3 translation;
	std::set<Node *> neighbors; // Neighbor nodes

};

#endif
