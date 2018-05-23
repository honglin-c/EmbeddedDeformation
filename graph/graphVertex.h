#ifndef _GRAPH_VERTEX_H
#define _GRAPH_VERTEX_H

#include <glm/glm.hpp>
#include <vector>
#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;

class Node;
struct Vertex;

class GraphVertex {
public:
	GraphVertex();
	GraphVertex(Vector3d _position);
	GraphVertex(struct Vertex _vertex);
	~GraphVertex();
	void setPositionAndNormal(Vector3d _position, Vector3d _normal);
	Vector3d getPosition() const;
	Vector3d getNormal() const;
	void setNodes(std::vector<Node *> _nodes, std::vector<double> _weights);
	void updatePosition();
	std::vector<Node *> getNodes();
	std::vector<double> getWeights();
	void updateNeighbor();
	void setFixed(bool is_fixed);
	void setHandled(bool is_Handled);

	void userSetPosition(Vector3d _user_position);

	// Constraint term
	double getConValue();

	// Get vertex's [ture position - user-specific position]
	Vector3d getConTerm();



//private:
	Vector3d position_init; // initial position and vertex
	Vector3d normal_init;
	Vector3d position;
	Vector3d normal;
	std::vector<Node *> nodes;
	std::vector<double> weights;
	bool isFixed;

	Vector3d user_position;
	bool isHandled; // if it is a constrainted vertex handled by the user
};

#endif