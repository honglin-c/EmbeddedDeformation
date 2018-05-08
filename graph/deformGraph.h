#ifndef _DEFORM_GRAPH_H
#define _DEFORM_GRAPH_H

#include <GL/glew.h>

#include "node.h"
#include "graphvertex.h"

typedef struct _DistNode
{
	Node * node;
	GLfloat dist;
}DistNode;

class DeformGraph
{
public:
	DeformGraph();
	DeformGraph(std::vector<GraphVertex *> &vertices, std::vector<Node *> &nodes);
	~DeformGraph();

	void update();
	void draw();

private:
	std::vector<GraphVertex *> vertices;
	std::vector<Node *> nodes;

	void findKNN();
};

#endif