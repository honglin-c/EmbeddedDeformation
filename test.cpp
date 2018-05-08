#include <iostream>
#include <vector>

#include <glm/vec3.hpp>

#include "graph/deformGraph.h"
#include "graph/kdtree.h"
#include "graph/node.h"
#include "graph/boundingObject.h"
#include "graph/kdnode.h"
#include "resource_manager.h"

using namespace std;

int main()
{
	ResourceManager::LoadSample(_MODEL_PREFIX_"/cat/cat.224.sample", "cat");
	Sample *sample = ResourceManager::GetSample("cat");


	vector<Node *> nodes;
	for(auto t:*sample)
	{
		nodes.push_back(new Node(t));
	}

	AABB aabb;
	aabb.setAABB(-10.0f, 10.0f, -10.0f, 10.0f,- 10.0f, 10.0f);
	KdTree kdtree;
	kdtree.buildKdTree(nodes, aabb, 0);
	kdtree.draw();

}