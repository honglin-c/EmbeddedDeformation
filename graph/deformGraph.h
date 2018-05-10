#ifndef _DEFORM_GRAPH_H
#define _DEFORM_GRAPH_H

#include <GL/glew.h>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include "node.h"
#include "graphvertex.h"
#include "boundingObject.h"

class DeformGraph
{
public:
	DeformGraph();
	DeformGraph(std::vector<GraphVertex *> &vertices, std::vector<Node *> &nodes);
	~DeformGraph();

	// Transform the vertices within a aabb
	void applyTransformation(glm::mat3 &rotation, glm::vec3 &translation, AABB &aabb);
	void update();
	void draw();
	void outputToFile();
	void print(const glm::vec3 &v);

	void print() const;

private:
	const int k = 4;
	std::vector<GraphVertex *> vertices;
	std::vector<Node *> nodes;

	/*  Render data  */
    GLuint VAO, VBO, EBO;

	void findKNN();
};

#endif