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
	void inserthelper(Node** node, std::vector<float> point, uint depth, int id)
	{
		if (*node == NULL)
		{
			*node = new Node(point,id);
		}
		else
		{
			//calculate current dimension
			uint cd = depth%2;

			if (point[cd] < ((*node)->point[cd]))
			{
				inserthelper(&(*node)->left,point,depth+1,id);
			}
			else
			{
				inserthelper(&(*node)->right,point,depth+1,id);
			}
			
			
		}
		
		
	}
	void insert(std::vector<float> point, int id)
	{
		inserthelper(&root,point,0,id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




