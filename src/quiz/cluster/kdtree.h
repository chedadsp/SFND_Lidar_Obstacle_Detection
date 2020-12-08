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
		return ids;
	}
	

};




