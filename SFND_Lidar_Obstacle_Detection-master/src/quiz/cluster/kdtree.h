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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(Node **node, uint depth, std::vector<float> point, int id)		// changed from "void insert(std::vector<float> point, int id)" to "void insert(Node **node, uint depth, std::vector<float> point, int id)"
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (*node == NULL)		// Tree is empty
		{
			node = new Node(point, id);
		}
		else if (id < (*node)->id)
		{
			insert(&(*node)->left, id);
		}
		else
		{
			insert(&(*node)->right, id);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




