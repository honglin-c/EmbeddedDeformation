#ifndef _DEFORM_GRAPH_H
#define _DEFORM_GRAPH_H

#include <vector>
#include <string> //for debug

#include <GL/glew.h>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <Eigen/Eigenvalues>

#include "node.h"
#include "graphVertex.h"
#include "boundingObject.h"
#include "../mesh.h"

typedef Eigen::SparseMatrix<double> SparseMd;
typedef Eigen::Triplet<double> Tf;

using std::vector;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SimplicialLDLT;
using Eigen::SelfAdjointEigenSolver;
using Eigen::SparseVector;

class DeformGraph
{
public:
	DeformGraph(std::string &name, std::vector<GraphVertex *> &vertices, std::vector<Node *> &nodes);
	~DeformGraph();

	// Transform the vertices within a aabb
	void applyTransformation(Matrix3d &rotation, Vector3d &translation, AABB &aabb);
	// Fixed all the vertices within a aabb
	void addFixedConstraint(AABB &aabb);
	// Optimized by Gauss-Newton method
	void optimize();

	vector<Vertex> returnVertices();

	vector<glm::vec3> returnNodes();

private:
	const std::string modelName;
	const int k = 4;
	int constraint_count; // use to measure benchmark

	std::vector<GraphVertex *> vertices;
	std::vector<Node *> nodes;

	void findKNN();
	void updateOrder();
	void update();
};

#endif