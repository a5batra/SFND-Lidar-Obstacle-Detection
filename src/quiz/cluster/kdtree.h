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

	void insertHelper(Node* &root, std::vector<float> point, int id, int currDepth) {
	    if (root == nullptr) {
            root = new Node(point, id);
            return;
	    }

        int dim = currDepth % 2;
        if (point[dim] < root->point[dim]) {
            insertHelper(root->left, point, id, currDepth + 1);
        }
        else {
            insertHelper(root->right, point, id, currDepth + 1);
        }
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
        insertHelper(root, point, id, 0);
	}

	void searchHelper(Node* &root, std::vector<float>& target, float distanceTol, std::vector<int>& ids, int currDepth) {
	    if (root != nullptr) {
	        if ((root->point[0] >= (target[0] - distanceTol) && root->point[0] <= (target[0] + distanceTol)) && (root->point[1] >= (target[1] - distanceTol) && root->point[1] <= (target[1] + distanceTol))) {
	            float x = target[0], y = target[1];
	            float x1 = root->point[0], y1 = root->point[1];
	            float dist = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	            if (dist <= distanceTol) ids.push_back(root->id);
	        }
	        int dim = currDepth % 2;
	        if (root->point[dim] >= target[dim] - distanceTol) searchHelper(root->left, target, distanceTol, ids, currDepth + 1);
	        if (root->point[dim] <= target[dim] + distanceTol) searchHelper(root->right, target, distanceTol, ids, currDepth + 1);
	    }
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root, target, distanceTol, ids, 0);
		return ids;
	}
	

};




