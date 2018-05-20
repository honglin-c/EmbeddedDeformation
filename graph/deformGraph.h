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

typedef Eigen::SparseMatrix<float> SparseMf;
typedef Eigen::Triplet<float> Tf;

using std::vector;

using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::SimplicialCholesky;
using Eigen::SelfAdjointEigenSolver;
using Eigen::SparseVector;

class DeformGraph
{
public:
	DeformGraph();
	DeformGraph(std::string &name, std::vector<GraphVertex *> &vertices, std::vector<Node *> &nodes);
	~DeformGraph();

	// Transform the vertices within a aabb
	void applyTransformation(glm::mat3 &rotation, glm::vec3 &translation, AABB &aabb);
	// Fixed all the vertices within a aabb
	void addFixedConstraint(AABB &aabb);
	void draw();
	void outputToFile();
	void print(const glm::vec3 &v);

	void print() const;

	// Optimized by Gauss-Newton method
	void optimize();

	vector<Vertex> returnVertices();

private:
	const std::string modelName;
	const int k = 4;
	const int max_iter = 6;
	const float sqrt10 = 3.16227766;
	const float epsilon = 1e-6;
	const int x_rt = 12;

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

	SparseMf getJf();
	VectorXf getfx();
	VectorXf descentDirection(const SparseMf &Jf, const VectorXf &fx, SimplicialCholesky<SparseMf> &chol, bool symbolic);
	// Golden Section Search - not currently working
	float exactLineSearch();

	// // Deprecated: Get term[i] / norm2(term)
	// float dRegTerm(Vector3f &regTerm, int i);
	// float dConTerm(Vector3f &conTerm, int i);

	// Perform x_k+1 = x_k + delta
	void updateNodesRt(VectorXf delta);


//============================================================================
	void debug(std::string s);

};

#endif