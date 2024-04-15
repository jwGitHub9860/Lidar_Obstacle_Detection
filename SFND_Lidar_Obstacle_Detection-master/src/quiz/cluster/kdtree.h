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

	void insertBinaryTree(Node **node, uint depth, std::vector<float> point, int id)		// Recursive Function definition		--> Purpose: reassign node in tree, when it is hit & it is NULL		** - double pointer --> memory address pointing at node currently on tree --> dereference it to see what value is
	{
		// Tree is empty
		if (*node == NULL)		// if node is NULL		TERMINATE "insertBinaryTree" function
		{
			*node = new Node(point, id);	// dereference node & make root pointer point to brand new node			REASSIGN NODE IN TREE
		}
		else	// if not at nodal point, traverse (walk through elements of data structure) the tree	--> NOT A NULL NODE
		{
			// Calculate current dim
			uint current_depth = depth % 2;		// traversing the tree DEPENDS ON current depth			uint - unsigned int			ALWAYS POSITIVE			BE EITHER 0 or 1			depth % 2 --> working with 2D case
			
			if (point[current_depth] < ((*node)->point[current_depth]))		// if "current_depth" is EVEN			can index directly into point			IF 0, look at x-values
			{
				insertBinaryTree(&((*node)->left), depth+1, point, id);
			}
			else	// if "current_depth" is ODD
			{
				insertBinaryTree(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertBinaryTree(&root, 0, point, id);		// Recursive Function - function that calls itself			NEED TO DEREFENCE &root TO CHANGE FROM (Node*) TO (Node **node)			(point, id) --> information needed to create BRAND NEW NODE
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




