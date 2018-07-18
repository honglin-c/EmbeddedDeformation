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
	DeformGraph();
	DeformGraph(std::string &name, std::vector<GraphVertex *> &vertices, std::vector<Node *> &nodes);
	~DeformGraph();

	// Transform the vertices within a aabb
	void applyTransformation(Matrix3d &rotation, Vector3d &translation, AABB &aabb);
	// Fixed all the vertices within a aabb
	void addFixedConstraint(AABB &aabb);
	void draw();
	void print(const Vector3d &v);

	void print() const;

	// Optimized by Gauss-Newton method
	void optimize();
	void optimize(int tmp);

	vector<Vertex> returnVertices();

	vector<glm::vec3> returnNodes();

private:
	const std::string modelName;
	const int k = 4;
	const int max_iter = 40;
	const double sqrt10 = 3.16227766;
	const double epsilon = 1e-6;
	const int x_rt = 12;
	const double w_rot = 2.3;
	const double w_reg = 10.0;
	const double w_con = 100.0;

	int rot_end;
	int reg_end;

	int x_order;
	int fx_order;

	std::vector<GraphVertex *> vertices;
	std::vector<Node *> nodes;

	/* Render data */
    GLuint VAO, VBO, EBO;

	void findKNN();

	void updateOrder();

	// Update all the vertices using the current transformation
	void update();

//============================================================================
// Optimization

	SparseMd getJf();
	VectorXd getfx();
	VectorXd descentDirection(const SparseMd &Jf, const VectorXd &fx, SimplicialLDLT<SparseMd> &chol, bool symbolic);

	// Perform x_k+1 = x_k + delta
	void updateNodesRt(VectorXd delta);


//============================================================================
	void debug(std::string s);
	int constraint_count; // use to measure benchmark
};

#endif