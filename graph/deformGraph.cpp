#include "deformGraph.h"
#include "distCompare.h"
#include "../optimization/GaussNewtonOptimizer.h"
#include "deformTargetFunction.h"
#include "../animation/animateTargetFunction.h"
#include <limits>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <ctime>

using namespace std;

DeformGraph::DeformGraph(std::string &name,
						 vector<GraphVertex *> 
						 &vertices,
						 vector<Node *> &nodes):
						modelName(name),
						vertices(vertices),
						nodes(nodes),
						constraint_count(0)
{
	findKNN();
}

DeformGraph::~DeformGraph()
{
	for(auto v:vertices)
		delete v;
	for(auto n:nodes)
		delete n;
}

void DeformGraph::findKNN()
{
	for(auto v:vertices)
	{
		Vector3d pos = v->getPosition();
		std::priority_queue<Node *, std::vector<Node *>, distCompare> heap(pos);
		Vector3d curpos;
		double min = 1e10, max = 0.0;

		// Iterate through all nodes to find the nearest k nodes
		for(auto t:nodes)
		{
			heap.push(t);
			if ((t->getPosition() - pos).norm() < min)
				min = (t->getPosition() - pos).norm();
			if ((t->getPosition() - pos).norm() > max)
				max = (t->getPosition() - pos).norm();
		}

		assert(heap.size() >= this->k);
		vector<Node *> vnodes;
		vector<double> dists, weights;
		Node * pnode = nullptr;

		// Get the top k nodes from heap
		for(int i = 0; i < this->k; i++)
		{
			pnode = heap.top();
			vnodes.push_back(pnode);
			dists.push_back((pnode->getPosition() - pos).norm());
			curpos = pnode->getPosition();
			heap.pop();
		}

		// dist_max = dist of k+1 nearest nodes
		double dist_max = (heap.top()->getPosition() - pos).norm();
		double norm_sum = 0.0; // sum used to normalizing
		double weight;

		// Equation 4
		for(auto d:dists)
		{
			weight = (1.0 - d / dist_max) * (1.0 - d / dist_max);
			weights.push_back(weight);
			norm_sum += weight;
		}
		// Normalize
		for(int i = 0; i < weights.size(); i++)
		{
			weights[i] /= norm_sum;
			assert(weights[i] == weights[i]); // avoid nan
		}
		v->setNodes(vnodes, weights);

		// Update the neighbor of each nodes that belongs to the vertex
		v->updateNeighbor();
	}
}

vector<Vertex> DeformGraph::returnVertices()
{
	vector<Vertex> ret;
	for(auto v:vertices)
	{
		glm::vec3 position = glm::vec3(v->position[0], v->position[1], v->position[2]);
		glm::vec3 normal = glm::vec3(v->normal[0], v->normal[1], v->normal[2]);
		Vertex rv = {position, normal};
		ret.push_back(rv);
	}
	return ret;
}

vector<glm::vec3> DeformGraph::returnNodes()
{
	vector<glm::vec3> ret;
	for(auto n:nodes)
	{
		Vector3d position = n->getPosition() + n->getTranslation();
		ret.push_back(glm::vec3(position[0], position[1], position[2]));
	}
	return ret;
}

void DeformGraph::applyTransformation(Matrix3d &rotation, Vector3d &translation, AABB &aabb)
{
	for(auto n:nodes)
	{
		if(aabb.isInside(n->getPosition()))
		{
			n->setTransformation(rotation, translation);
		}
	}

	// Update the positions of all vertices within the aabb
	Vector3d position, normal;
	for(auto v:vertices)
	{
		if(!aabb.isInside(v->getPosition()))
			continue;

		position = normal = Vector3d(0.0, 0.0, 0.0);
		for(int i = 0; i < v->nodes.size(); i++)
		{
			position += v->weights[i] * v->nodes[i]->transformPosition(v->position_init);
			normal += v->weights[i] * v->nodes[i]->transformNormal(v->normal_init);
		}
		v->userSetPosition(position);
		constraint_count++;
	}

	this->update();
}


void DeformGraph::update()
{
	// transform vertex position and normal using the current transformation
	Vector3d position, normal;
	for(auto v:vertices)
	{
		position = normal = Vector3d(0.0, 0.0, 0.0);
		for(int i = 0; i < v->nodes.size(); i++)
		{
			position += v->weights[i] * v->nodes[i]->transformPosition(v->position_init);
			normal += v->weights[i] * v->nodes[i]->transformNormal(v->normal_init);
		}
		v->setPositionAndNormal(position, normal);
	}
}


void DeformGraph::addFixedConstraint(AABB &aabb)
{
	for(auto v:vertices)
	{
		if(aabb.isInside(v->getPosition()))
		{
			v->setFixed(true);
			constraint_count ++;
		}
	}
}

void DeformGraph::optimize()
{
	shared_ptr<DeformParam> xparam(new DeformParam);
	xparam->setParamInfo(modelName, vertices, nodes);
	shared_ptr<DeformTargetFunction> targetFunc(new DeformTargetFunction(xparam));
	GaussNewtonSolver * optimizer = new GaussNewtonOptimizer(xparam);	
	optimizer->solve(targetFunc, xparam);
	delete optimizer;
}

bool DeformGraph::optimizeSingleFrame()
{
	cout << "[Optimize single step]" << endl;
	shared_ptr<DeformParam> xparam(new DeformParam);
	xparam->setParamInfo(modelName, vertices, nodes);
	shared_ptr<AnimateTargetFunction> targetFunc(new AnimateTargetFunction(xparam));
	GaussNewtonOptimizer * optimizer = new GaussNewtonOptimizer(xparam);	
	bool stopped = optimizer->solveSingleFrame(targetFunc, xparam);
	delete optimizer;
	return false;
	// return stopped;
}
