#include "deformGraph.h"
#include <limits>

using namespace std;

DeformGraph::DeformGraph()
{

}

DeformGraph::DeformGraph(vector<GraphVertex *> &vertices, 
						 vector<Node *> &nodes):
						vertices(vertices),
						nodes(nodes)	
{}

DeformGraph::~DeformGraph()
{

}

void DeformGraph::update()
{

}

void DeformGraph::findKNN()
{


	// vector<DistNode> knn;
	// // Initialize knn set
	// for(int i = 0; i < 4; i++)
	// {
	// 	DistNode distnode;
	// 	distnode.node = nullptr;
	// 	distnode.dist =  std::numeric_limits<float>::infinity();
	// 	knn.push_back(distnode);
	// }

	// for(auto vertex : vertices)
	// {
	// 	for(auto node : nodes)
	// 	{
	// 		dist = glm::length(node->position - vertex->position);
	// 		if(knn.size() < 4)
	// 		{
	// 			DistNode distnode;
	// 			distnode.node = node;
	// 			distnode.dist = dist;
	// 			knn.push_back(node);
	// 		}
	// 		else
	// 		{
	// 			dist = glm::length(node->position - vertex->position);

	// 		}
	// 	}
	// }
}
