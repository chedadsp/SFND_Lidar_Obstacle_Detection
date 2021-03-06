/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	private:

	void insertRecursive(Node*& node, std::vector<float> point, int id, int depth)
	{
		int index = depth % point.size();
		
		if(node == NULL)
			node = new Node(point, id);
		else 
			if(node->point[index] < point[index])
				insertRecursive(node->right, point, id, depth + 1);
			else
				insertRecursive(node->left, point, id, depth + 1);
	}

	bool isWithinBox(std::vector<float>& point, std::vector<float>& target, float distanceTol)
	{
		bool retVal = true;
		for(int i=0; i < point.size(); i++) // This works for KD
			retVal = retVal && (point[i] - distanceTol) < target[i] && (point [i] + distanceTol) > target[i];

		return retVal;
	}
	
	void searchRecursive(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if(isWithinBox(target, node->point, distanceTol))
				{
					float distance = sqrt(pow(node->point[0] - target[0] ,2) + pow(node->point[1] - target[1] ,2));
					if(distance <= distanceTol)
						ids.push_back(node->id);
				}

			if((target[depth % target.size()] - distanceTol) < node->point[depth % target.size()])
				searchRecursive(target, distanceTol, node->left, depth + 1, ids);
			if((target[depth % target.size()] + distanceTol) >= node->point[depth % target.size()])
				searchRecursive(target, distanceTol, node->right, depth + 1, ids);
		}
	}

	public:

	void insert(std::vector<float> point, int id)
	{
		//  create a new node and place correctly with in the root
		insertRecursive(root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRecursive(target, distanceTol, root, 0, ids);

		return ids;
	}
	

};




