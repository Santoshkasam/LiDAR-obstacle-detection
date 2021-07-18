

#ifndef KDTREE_H
#define KDTREE_H
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	// Constructor for Node struct 
	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;	// Top most node in the tree

	// Constructor for kdTree struct
	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node **node, int depth, std::vector<float> point, int id)
	{
		// Check if the given node is empty
		if(*node == NULL)	// If yes a new point is inserted into the node
		{
			*node = new Node(point, id);
		}
		else	// If the given node is not empty, the point's dimension corrosponding to
				// depth is compared to given node's dimension and navigated to left or right
				// according to their values
		{
			// Current depth
			uint cd = depth % 3;
			
			// Compares Xs, Ys or Zs depending on cd, and inserts in left or right child accordingly
			if(point[cd] < ((*node)->point[cd])) insertHelper(&((*node)->left), depth+1, point, id);
			else insertHelper(&((*node)->right), depth+1, point, id);
		}

	}
	
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	void searchHelper(Node *node, int depth, std::vector<float> target, float distanceTol, std::vector<int> &ids)
	{
		if(node != NULL)
		{
			// Check if the node is within the target cube within distance tolerance
			if ((node->point[0]>= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) ) 
				&& (node->point[1]>= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol) ) 
				&& (node->point[2]>= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol) ) )
			{
				// If yes, calculate the euclidien distance between the target point and node point
				float dist = sqrt( (node->point[0] - target[0]) * (node->point[0] - target[0]) 
								 + (node->point[1] - target[1]) * (node->point[1] - target[1]) 
								 + (node->point[2] - target[2]) * (node->point[2] - target[2]) );
				// If the distance is less than threshold, add the id of the point to vector of ids
				if (dist <= distanceTol) ids.push_back(node->id);
			}

			// Check branches of node
			// If the node is to the right of target's -ve threshold co-ordinate, check left branch of the node
			if ((target[depth%3]-distanceTol) < node->point[depth%3]) searchHelper(node->left, depth+1, target, distanceTol, ids);
			// If the node is to the left of target's +ve threshold co-ordinate, check right branch of the node
			if ((target[depth%3]+distanceTol) > node->point[depth%3]) searchHelper(node->right, depth+1, target, distanceTol, ids);
		}

	}
	
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};
#endif


