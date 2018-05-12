#ifndef _GRAPH_VERTEX_H
#define _GRAPH_VERTEX_H

#include <glm/glm.hpp>
#include <vector>
#include <Eigen/Dense>

using Eigen::Vector3f;
using Eigen::VectorXf;
using Eigen::Matrix3f;

class Node;
struct Vertex;

class GraphVertex {
public:
	GraphVertex();
	GraphVertex(glm::vec3 _position);
	GraphVertex(struct Vertex _vertex);
	~GraphVertex();
	void setPositionAndNormal(glm::vec3 _position, glm::vec3 _normal);
	glm::vec3 getPosition() const;
	glm::vec3 getNormal() const;
	void setNodes(std::vector<Node *> _nodes, std::vector<float> _weights);
	void updatePosition();
	std::vector<Node *> getNodes();
	std::vector<float> getWeights();
	void updateNeighbor();
	void setFixed(bool is_fixed);

	void userSetPosition(glm::vec3 _user_position);

	// Constraint term
	float getConValue();

	// Get vertex's [ture position - user-specific position]
	Vector3f getConTerm();



//private:
	glm::vec3 position_init; // initial position and vertex
	glm::vec3 normal_init;
	glm::vec3 position;
	glm::vec3 normal;
	std::vector<Node *> nodes;
	std::vector<float> weights;
	bool isFixed;

	glm::vec3 user_position;
	bool isHandled; // if it is a constrainted vertex handled by the user
};

#endif