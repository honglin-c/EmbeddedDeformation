#ifndef _GRAPH_VERTEX_H
#define _GRAPH_VERTEX_H

#include <glm/glm.hpp>
#include <vector>
class Node;

typedef struct _VertexNode {
	Node * node;
	float weight;
}VertexNode;

class GraphVertex {
public:
	GraphVertex();
	GraphVertex(glm::vec3 _position);
	~GraphVertex();
	void setPosition(glm::vec3 _position);
	void setVertexNode(std::vector<VertexNode> _nodes);
	void updatePosition();
private:
	glm::vec3 position;
	std::vector<VertexNode> nodes;
};

#endif