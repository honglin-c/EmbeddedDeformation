#ifndef _GRAPH_VERTEX_H
#define _GRAPH_VERTEX_H

#include <glm/glm.hpp>
#include <vector>
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

//private:
	glm::vec3 position;
	glm::vec3 normal;
	std::vector<Node *> nodes;
	std::vector<float> weights;
};

#endif