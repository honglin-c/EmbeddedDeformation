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

int _main()
{
// k-d tree test
//=======================================================================================
	ResourceManager::LoadSample(_MODEL_PREFIX_"/cat/cat.224.sample", "cat");
	Sample *sample = ResourceManager::GetSample("cat");


	vector<Node *> nodes;
	for(auto t:*sample)
	{
		nodes.push_back(new Node(t));
	}

	glm::vec3 pos(-0.066671f, 0.025287f, 0.691158f);

	std::priority_queue<Node *, std::vector<Node *>, distCompare> heap(pos);
	glm::vec3 curpos;
	float min = 1e10, max = 0.0;

	for(auto t:nodes)
	{
		heap.push(t);
		if (glm::length(t->getPosition() - pos) < min)
			min = glm::length(t->getPosition() - pos);
		if (glm::length(t->getPosition() - pos) > max)
			max = glm::length(t->getPosition() - pos);
	}

	cout << endl << "expected min: " << min << " max: " << max << endl;

	for(int i = 0; i < 4 && !heap.empty(); i++)
	{
		curpos = heap.top()->getPosition();
		cout << curpos[0] << "," << curpos[1] << "," << curpos[2] <<  " dist:" <<  glm::length(curpos - pos) << endl;
		heap.pop();
	}

	AABB aabb;
	aabb.setAABB(-10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f);
	KdTree kdtree;
	kdtree.buildKdTree(nodes, aabb, 0);

	cout << endl << "result: " << endl;
	vector<Node *> res = kdtree.kNearestNeighborSearch(pos);
	for(auto t:res)
	{
		cout << t->getPosition()[0] << "," << t->getPosition()[1] << "," << t->getPosition()[2] <<  " dist:" <<  glm::length(t->getPosition() - pos) << endl;
	}

	kdtree.draw();
//=======================================================================================

}