#include "SingleAgentSolver.h"


list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
	list<int> rst = instance.getNeighbors(curr);
	rst.emplace_back(curr);
	return rst;
}


void SingleAgentSolver::compute_heuristics()
{
	struct Node
	{
		int location;
		int value;
		int direction;

		Node() = default;
		Node(int location,int direction, int value) : location(location), direction(direction), value(value) {}
		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};

	my_heuristic.resize(instance.map_size*4, MAX_TIMESTEP);

	// generate a heap that can save nodes (and a open_handle)
	boost::heap::pairing_heap<Node, boost::heap::compare<Node::compare_node> > heap;

	Node root(goal_location,goal_direction, 0);
	my_heuristic[goal_location*4+goal_direction] = 0;
	heap.push(root);  // add root to heap

	//int current_direction = goal_direction;
	int moves_forward_offset[] = {-instance.getCols(),1,instance.getCols(),-1};

	while (!heap.empty())
	{
		Node curr = heap.top();
		heap.pop();
		for (int move = 1; move < 4; move++)
		{
			//current_direction = curr.direction;

			//check directions change
			int previous_direction = curr.direction;
			if(move == 3)
			{
				previous_direction = curr.direction - 1;
				if (previous_direction == -1){
					previous_direction = 3;
				}
			}else if(move == 2){
				previous_direction = (curr.direction+ 1)%4;
			}

			//reverse the move calculateion
			int previous_loc = curr.location;
			if (move == 1)
				previous_loc = curr.location - moves_forward_offset[curr.direction];
			if (instance.validMove(curr.location, previous_loc))
			{
				if (my_heuristic[previous_loc*4 + previous_direction] > curr.value + 1)
				{
					my_heuristic[previous_loc*4 + previous_direction] = curr.value + 1;
					Node next(previous_loc, previous_direction, curr.value + 1);
					heap.push(next);
				}
			}
		}
	}
	// for (int i = 0; i < my_heuristic.size(); i++)
	// {
	// 	std::cout<<my_heuristic[i]<<" ";
	// }
}
