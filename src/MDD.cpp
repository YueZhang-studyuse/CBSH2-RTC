#include "MDD.h"
#include <iostream>
#include "common.h"


void MDD::printNodes() const
{
	// std::cout<<"leveles "<<std::endl;
	for (const auto& level : levels)
	{
		cout << "[";
		for (auto node : level)
		{
			cout << node->location<<"-"<<node->direction << ",";
		}
		cout << "]," << endl;
	}

	// for (const auto& level : prune_levels)
	// {
	// 	cout << "[";
	// 	for (auto node : level)
	// 	{
	// 		cout << node->location<<"-"<<node->direction << ",";
	// 	}
	// 	cout << "]," << endl;
	// }
}


// void MDD::printMergedNodes() const
// {
// 	for (const auto& level : merged_levels)
// 	{
// 		cout << "[";
// 		for (auto node : level)
// 		{
// 			cout << node->location << ",";
// 		}
// 		cout << "]," << endl;
// 	}
// }

/*bool MDD::isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons)  const
{
	if (cons.empty())
		return false;
	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (next_timestep < static_cast<int>(cons.size()))
	{
		for (const auto & it : cons[next_timestep])
		{
			if ((std::get<0>(it) == next_id && std::get<1>(it) < 0)//vertex constraint
				|| (std::get<0>(it) == curr_id && std::get<1>(it) == next_id)) // edge constraint
				return true;
		}
	}
	return false;
}*/

bool MDD::buildMDD(const ConstraintTable& ct, int num_of_levels, const SingleAgentSolver* _solver)
{
  	this->solver = _solver;
  	auto root = new MDDNode(solver->start_location, solver->start_direction, nullptr); // Root
  	root->cost = num_of_levels - 1;
	std::queue<MDDNode*> open;
	list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(num_of_levels);
	prune_levels.resize(num_of_levels);
	while (!open.empty())
	{
		auto curr = open.front();
		open.pop();
		// Here we suppose all edge cost equals 1
		// reach cost
		if (curr->level == num_of_levels - 1)
		{
			levels.back().push_back(curr);
			prune_levels.back().push_back(curr);
			assert(open.empty());
			break;
		}
		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g - 2. -1 because it's the bound of the children.
		int heuristicBound = num_of_levels - curr->level - 2; //because heuristic is based on dijkstra
		//pending modify: direction, location 
						//mdd nodes: add directions
						//my heuristic index
						//moves -> change to turning, wait, and forward
		//list<int> next_locations = solver->getNextLocations(curr->location);
		int moves_forward_offset[] = {-solver->getInstanceCols(),1,solver->getInstanceCols(),-1};

		//list<pair<int,int>> next_states = solver->getNeighbors(curr->location,curr->direction);
		
		//int turning_count = 0;
		//for (pair<int,int> next_state: next_states)
		for (int i = 0; i < 4; i++) // Try every possible move. We only add backward edges in this step.
		{
			// int next_location = next_state.first;
			// int next_direction = next_state.second;
			int next_location = curr->location;
			if (i == 1)
				next_location = curr->location + moves_forward_offset[curr->direction];
			// else if (i != 0)//check useless moves here
			// //if (next_direction != curr->direction)
			// {
				//turning_count++; //turn left = 1; turn right = 2;
				// int turnleft = 0;
				// int turnright = 0;
				// int location = curr->location;
				// int direction1 = curr->direction;
				// bool prune = false;
				//while(temp->parent != nullptr)
				// for(auto it = curr->parents.rbegin(); it != curr->parents.rend();++it)
				// {
				// 	//break;
				// 	if((*it)->location != location)
				// 	{
				// 		break;
				// 	}
				// 	//int direction1 = temp->direction;
				// 	int direction2 = (*it)->direction;
				// 	if (direction1 == direction2)
				// 	{
				// 		location = (*it)->location;
				// 		direction1 = (*it)->direction;
				// 		continue;
				// 	}
				// 	if (direction1 - direction2 == 1 || (direction1 == 0 && direction2 == 3))//due to turn right
				// 	{
				// 		if (i==2)
				// 		{
				// 			prune = true;
				// 			break;
				// 		}
				// 		else //before is turn left
				// 		{
				// 			turnright++;
				// 		}
				// 	}
				// 	if (direction1 - direction2 == -1 || (direction1 == 3 && direction2 == 0))//due to turn left
				// 	{
				// 		if (i==3)
				// 		{
				// 			prune = true;
				// 			break;
				// 		}
				// 		else
				// 		{
				// 			turnleft++;
				// 		}
				// 	}
				// 	if((turnleft >= 2 && i==2) || (turnright>=2 && i==3))
				// 	{
				// 		prune = true;
				// 		break;
				// 	}
				// 	location = (*it)->location;
				// 	direction1 = (*it)->direction;
				// }
				// if(prune)
				// {
				// 	//continue;
				// }
			//}
			//curr->location + instance.calculateMoves(i,curr->cur_direction);
			//loc + moves_offset[i] * forward_move_offset[curr->cur_direction];

			if (!solver->validMove(curr->location,next_location))
			{
				continue;
			}
			//next direction
			int current_direction = curr->direction;
			int next_direction = current_direction;
			
			if(i == 2)
			{
				next_direction = current_direction - 1;
				if (next_direction == -1){
					next_direction = 3;
				}
			}else if(i == 3){
				next_direction = (current_direction + 1)%4;
			}

			if (solver->my_heuristic[next_location*4 + next_direction] <= heuristicBound &&
				!ct.constrained(next_location, curr->level + 1) &&
				!ct.constrained(curr->location, next_location, curr->level + 1))
				//add direct constraint
				//&& !ct.vertex_direct_constrained(next_location,next_direction,curr->level+1)) // valid move
			{
				auto child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == curr->level + 1); ++child)
				{
					if ((*child)->location == next_location && (*child)->direction == next_direction) // If the child node exists
					{
						(*child)->parents.push_back(curr); // then add corresponding parent link and child link
						find = true;
						break;
					}
				}
				if (!find) // Else generate a new mdd node
				{
					auto childNode = new MDDNode(next_location,next_direction, curr);
					childNode->cost = num_of_levels - 1;
					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	assert(levels.back().size() == 1);

	// Backward
	auto goal_node = levels.back().back();
	MDDNode* del = nullptr;
	for (auto parent : goal_node->parents)
	{
		if (parent->location == goal_node->location && parent->direction == goal_node->direction) // the parent of the goal node should not be at the goal location
		{
			del = parent;
			continue;
		}
		levels[num_of_levels - 2].push_back(parent);
		prune_levels[num_of_levels - 2].push_back(parent);
		parent->children.push_back(goal_node); // add forward edge	
	}
	if (del != nullptr)
		goal_node->parents.remove(del);
	for (int t = num_of_levels - 2; t > 0; t--)
	{
		for (auto node : levels[t])
		{
			for (auto parent : node->parents)
			{
				if (parent->children.empty()) // a new node
				{
					levels[t - 1].push_back(parent);
					prune_levels[t - 1].push_back(parent);
				}
				parent->children.push_back(node); // add forward edge	
			}
		}
	}

	// unordered_map<int,list<MDDNode*>> temp_mdds;
	// auto goal_node = levels.back().back();
	// MDDNode* del = nullptr;
	// for (auto parent : goal_node->parents)
	// {
	// 	if (parent->location == goal_node->location && parent->direction == goal_node->direction) // the parent of the goal node should not be at the goal location
	// 	{
	// 		del = parent;
	// 		continue;
	// 	}
	// 	//merge together by locaion
	// 	if (temp_mdds.find(parent->location) != temp_mdds.end())
	// 	{
	// 		temp_mdds[parent->location].push_back(parent);
	// 	}
	// 	else
	// 	{
	// 		list<MDDNode*> node;
	// 		node.push_back(parent);
	// 		temp_mdds.insert({parent->location,node});
	// 	}
	// 	//levels[num_of_levels - 2].push_back(parent);
	// 	//parent->children.push_back(goal_node); // add forward edge	
	// }
	// if (del != nullptr)
	// 	goal_node->parents.remove(del);
	// //insert the num_of_levels-2 here
	// for (auto temp:temp_mdds)
	// {
	// 	if (temp.second.size() == 1)
	// 	{
	// 		temp.second.front()->direction = -1;
	// 		levels[num_of_levels - 2].push_back(temp.second.front());
	// 		temp.second.front()->children.push_back(goal_node);
	// 	}
	// 	else
	// 	{
	// 		MDDNode* n = temp.second.front();
	// 		unordered_set<MDDNode*> temp_parents;
	// 		for (auto p: n->parents)
	// 		{
	// 			if (temp_parents.find(p) == temp_parents.end())
	// 			{
	// 				n->parents.push_back(p);
	// 				temp_parents.insert(p);
	// 			}
	// 			//temp_parents.insert(p);
	// 		}
	// 		list<MDDNode*>::iterator it;
	// 		it = temp.second.begin();
	// 		it++;
	// 		while(it != temp.second.end())
	// 		{
	// 			for (auto p: (*it)->parents)
	// 			{
	// 				if (temp_parents.find(p) == temp_parents.end())
	// 				{
	// 					n->parents.push_back(p);
	// 					temp_parents.insert(p);
	// 				}
	// 			}
	// 			//delete *(it);
	// 			it++;
	// 		}
	// 		n->direction = -1;
	// 		levels[num_of_levels - 2].push_back(n);
	// 		n->children.push_back(goal_node);
	// 	}
	// }
	// goal_node->parents.clear();
	// for (auto p: levels[num_of_levels-2])
	// {
	// 	goal_node->parents.push_back(p);
	// }

	// for (int t = num_of_levels - 2; t > 0; t--)
	// {
	// 	//map to store nodes that from different children, but has the same location
	// 	temp_mdds.clear();
	// 	for (auto node : levels[t]) //different node may have the same parent
	// 	{
	// 		//merge node has the same location
	// 		for (auto parent : node->parents)
	// 		{
	// 			//get node with unique locations here
	// 			if (temp_mdds.find(parent->location) != temp_mdds.end())
	// 			{
	// 				temp_mdds[parent->location].push_back(parent);
	// 			}
	// 			else
	// 			{
	// 				list<MDDNode*> node;
	// 				node.push_back(parent);
	// 				temp_mdds.insert({parent->location,node});
	// 			}

	// 			//need to modify here
	// 			// if (parent->children.empty()) // a new node
	// 			// {
	// 			// 	levels[t - 1].push_back(parent);
	// 			// }
	// 			//mark the children first
	// 			parent->children.push_back(node); // add forward edge	
	// 		}
	// 	}
	// 	//std::cout<<temp_mdds.size()<<std::endl;
	// 	//insert to levels now: merge parent and merge children
	// 	for (auto temp:temp_mdds)
	// 	{
	// 		if (temp.second.size() == 1)
	// 		{
	// 			temp.second.front()->direction = -1;
	// 			//if (temp.second.front()->children.empty()) // a new node
	// 			//{
	// 				levels[t - 1].push_back(temp.second.front());
	// 			//}
	// 			//levels[num_of_levels - 2].push_back(temp.second.front());
	// 			//temp.second.front()->children.push_back(node);
	// 		}
	// 		else
	// 		{
	// 			MDDNode* n = temp.second.front();
	// 			unordered_set<MDDNode*> temp_parents;
	// 			unordered_set<MDDNode*> temp_children;
	// 			for (auto p: n->parents)
	// 			{
	// 				//temp_parents.insert(p);
	// 				if (temp_parents.find(p) == temp_parents.end())
	// 				{
	// 					n->parents.push_back(p);
	// 					temp_parents.insert(p);
	// 				}
	// 			}
	// 			for (auto c: n->children)
	// 			{
	// 				//temp_children.insert(c);
	// 				if (temp_children.find(c) == temp_children.end())
	// 				{
	// 					n->children.push_back(c);
	// 					temp_children.insert(c);
	// 				}
	// 			}
	// 			list<MDDNode*>::iterator it;
	// 			it = temp.second.begin();
	// 			it++;
	// 			while(it != temp.second.end())
	// 			{
	// 				for (auto p: (*it)->parents)
	// 				{
	// 					if (temp_parents.find(p) == temp_parents.end())
	// 					{
	// 						n->parents.push_back(p);
	// 						temp_parents.insert(p);
	// 					}
	// 				}
	// 				for (auto c: (*it)->children)
	// 				{
	// 					if (temp_children.find(c) == temp_children.end())
	// 					{
	// 						n->children.push_back(c);
	// 						temp_children.insert(c);
	// 					}
	// 				}
	// 				it++;
	// 				//delete *(it);
	// 			}
	// 		n->direction = -1;
	// 		// if (n->children.empty())
	// 		// {
	// 		levels[t-1].push_back(n);
	// 		//}
	// 		//n->children.push_back(node);
	// 		}
	// 	}
	// 	//}
	// 	//after get all parents from level t, also need to merge again here
	// 	//check redundant in levels t-1 and merge children and parents together
	// 	//maybe not insert in the previous step
	// }

	// Delete useless nodes (nodes who don't have any children)
	// for (auto it : closed)
	// 	if (it->children.empty() && it->level < num_of_levels - 1)
	// 		delete it;
	// closed.clear();

	//test
	//mergeLevels();
	for (auto it : closed)
	{
		if (it->children.empty() && it->level < num_of_levels - 1)
		{
			for (auto parent : it->parents)
			{
				// if (parent->children.empty()) // a new node
				// {
				// 	prune_levels[it->level - 1].push_back(parent);
				// }
				parent->children.push_back(it); // add forward edge	
			}
			prune_levels[it->level].push_back(it);
		}
	}
	return true;
}

/*bool MDD::buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels,
	int start_location, const int* moves_offset, const std::vector<int>& my_heuristic, int map_size, int num_col)
{
	auto root = new MDDNode(start_location, nullptr); // Root
	std::queue<MDDNode*> open;
	std::list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(numOfLevels);
	while (!open.empty())
	{
		MDDNode* node = open.front();
		open.pop();
		// Here we suppose all edge cost equals 1
		if (node->level == numOfLevels - 1)
		{
			levels[numOfLevels - 1].push_back(node);
			if (!open.empty())
			{
				std::cerr << "Failed to build MDD!" << std::endl;
				exit(1);
			}
			break;
		}
		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the children.
		double heuristicBound = numOfLevels - node->level - 2 + 0.001;
		for (int i = 0; i < 5; i++) // Try every possible move. We only add backward edges in this step.
		{
			int newLoc = node->location + moves_offset[i];
			if (validMove(node->location, newLoc, map_size, num_col) &&
				my_heuristic[newLoc] < heuristicBound &&
				!isConstrained(node->location, newLoc, node->level + 1, constraints)) // valid move
			{
				auto child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
					if ((*child)->location == newLoc) // If the child node exists
					{
						(*child)->parents.push_back(node); // then add corresponding parent link and child link
						find = true;
						break;
					}
				if (!find) // Else generate a new mdd node
				{
					auto childNode = new MDDNode(newLoc, node);
					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	// Backward
	for (int t = numOfLevels - 1; t > 0; t--)
	{
		for (auto it = levels[t].begin(); it != levels[t].end(); ++it)
		{
			for (auto parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
			{
				if ((*parent)->children.empty()) // a new node
				{
					levels[t - 1].push_back(*parent);
				}
				(*parent)->children.push_back(*it); // add forward edge	
			}
		}
	}

	// Delete useless nodes (nodes who don't have any children)
	for (auto & it : closed)
		if (it->children.empty() && it->level < numOfLevels - 1)
			delete it;
	closed.clear();
	return true;
}*/


void MDD::deleteNode(MDDNode* node)
{
	levels[node->level].remove(node);
	for (auto child = node->children.begin(); child != node->children.end(); ++child)
	{
		(*child)->parents.remove(node);
		if ((*child)->parents.empty())
			deleteNode(*child);
	}
	for (auto parent = node->parents.begin(); parent != node->parents.end(); ++parent)
	{
		(*parent)->children.remove(node);
		if ((*parent)->children.empty())
			deleteNode(*parent);
	}
}

void MDD::clear()
{
	// if (levels.empty())
	// 	return;
	// for (auto& level : levels)
	// {
	// 	for (auto& it : level)
	// 		delete it;
	// }
	if (prune_levels.empty())
		return;
	for (auto& level : prune_levels)
	{
		for (auto& it : level)
			delete it;
	}
	prune_levels.clear();
	levels.clear();
}

// void MDD::clearMergedMdd()
// {
// 	if (merged_levels.empty())
// 		return;
// 	for (auto& level : merged_levels)
// 	{
// 		for (auto& it : level)
// 			delete it;
// 	}
// 	merged_levels.clear();
// }

// MDDNode* MDD::find(int location, int level) const
// {
// 	if (level < (int) levels.size())
// 		for (auto it : levels[level])
// 			if (it->location == location)
// 				return it;
// 	return nullptr;
//}
MDDNode* MDD::find(int location, int direction, int level) const
{
	if (level < (int) levels.size())
		for (auto it : levels[level])
			if (it->location == location && it->direction == direction)
				return it;
	return nullptr;
}

MDDNode* MDD::findInPruneLevel(int location, int direction, int level) const
{
	if (level < (int) prune_levels.size())
		for (auto it : prune_levels[level])
			if (it->location == location && it->direction == direction)
				return it;
	return nullptr;
}

MDD::MDD(const MDD& cpy) // deep copy
{
	levels.resize(cpy.levels.size());
	auto root = new MDDNode(cpy.levels[0].front()->location,cpy.levels[0].front()->direction, nullptr);
  	root->cost = cpy.levels.size() - 1;
	levels[0].push_back(root);
	for (size_t t = 0; t < levels.size() - 1; t++)
	{
		for (auto node = levels[t].begin(); node != levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.find((*node)->location, (*node)->direction, (*node)->level);
			for (list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				MDDNode* child = find((*cpyChild)->location,(*cpyChild)->direction, (*cpyChild)->level);
				if (child == nullptr)
				{
					child = new MDDNode((*cpyChild)->location, (*cpyChild)->direction, (*node));
					child->cost = (*cpyChild)->cost;
					levels[child->level].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}

	}

	prune_levels.resize(cpy.prune_levels.size());
	auto prune_root = new MDDNode(cpy.prune_levels[0].front()->location, nullptr);
  	prune_root->cost = cpy.prune_levels.size() - 1;
	prune_levels[0].push_back(prune_root);
	//levels[0].insert(root->location,root)
	for (size_t t = 0; t < prune_levels.size() - 1; t++)
	{
		for (auto node = prune_levels[t].begin(); node != prune_levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.findInPruneLevel((*node)->location, (*node)->direction, (*node)->level);
			for (list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				MDDNode* child = findInPruneLevel((*cpyChild)->location,(*cpyChild)->direction, (*cpyChild)->level);
				if (child == nullptr)
				{
					child = new MDDNode((*cpyChild)->location, (*cpyChild)->direction, (*node));
					child->cost = (*cpyChild)->cost;
					prune_levels[child->level].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}

	}

	solver = cpy.solver;
}

MDD::~MDD()
{
	clear();
}

//not change to rotation now, pending change
//modify now
//increase also need to increse the merged mdd
void MDD::increaseBy(const ConstraintTable&ct, int dLevel, SingleAgentSolver* _solver) //TODO:: seems that we do not need solver
{
	auto oldHeight = levels.size();
	auto numOfLevels = levels.size() + dLevel;
	for (auto &l : levels)
		for (auto node: l)
		  	node->parents.clear();

	levels.resize(numOfLevels);

	int moves_forward_offset[] = {-solver->getInstanceCols(),1,solver->getInstanceCols(),-1};
	for (int l = 0; l < numOfLevels - 1; l++)
	{
		double heuristicBound = numOfLevels - l - 2 + 0.001;

		auto node_map = collectRawMDDlevel(this, l + 1);

		for (auto& it: levels[l])
		{
			MDDNode* node_ptr = it;

			//auto next_locations = solver->getNextLocations(it->location);
			//for (int newLoc: next_locations)
			for (int i = 0; i < 4; i++) // Try every possible move. We only add backward edges in this step.
			{
				int next_location = it->location;
				if (i == 1)
					next_location = it->location + moves_forward_offset[it->direction];
				
				if (!solver->validMove(it->location,next_location))
				{
					continue;
				}
				//next direction
				int current_direction = it->direction;
				int next_direction = current_direction;
				
				if(i == 2)
				{
					next_direction = current_direction - 1;
					if (next_direction == -1){
						next_direction = 3;
					}
				}else if(i == 3){
					next_direction = (current_direction + 1)%4;
				}

				if (solver->my_heuristic[next_location*4+next_direction] <= heuristicBound &&
					!ct.constrained(next_location, it->level + 1) &&
					!ct.constrained(it->location, next_location, it->level + 1))
					//add direct constraint
					//&& !ct.vertex_direct_constrained(next_location,next_direction,it->level+1)) // valid move
				{
					if (node_map.find(next_location*4 + next_direction) == node_map.end())
					{
						auto newNode = new MDDNode(next_location,next_direction, node_ptr);
						levels[l + 1].push_back(newNode);
						node_map[next_location*4 + next_direction] = newNode;
					}
					else
					{
						node_map[next_location*4 + next_direction]->parents.push_back(node_ptr);
					}
				}
			}
		}
	}
	// Backward
	for (int l = oldHeight; l < numOfLevels; l++)
	{
		MDDNode* goal_node = nullptr;
		for (auto it:levels[l])
		{
			if (it->location == solver->goal_location && it->direction == solver->goal_direction)
			{
				goal_node = it;
				break;
			}
		}

		std::queue<MDDNode*> bfs_q({ goal_node });
		boost::unordered_set<MDDNode*> closed;


		while (!bfs_q.empty())
		{
			auto ptr = bfs_q.front();
			ptr->cost = l;
			bfs_q.pop();
			for (auto parent_ptr:ptr->parents)
			{
				parent_ptr->children.push_back(ptr); // add forward edge

				if (closed.find(parent_ptr) == closed.end() && parent_ptr->cost == 0)
				{
					bfs_q.push(parent_ptr);
					closed.insert(parent_ptr);
				}
			}
		}
	}

	// Delete useless nodes (nodes who don't have any children)
	for (int l = 0; l < numOfLevels - 1; l++)
	{
		auto it = levels[l].begin();
		while (it != levels[l].end())
		{
			if ((*it)->children.empty())
			{
				it = levels[l].erase(it);
			}
			else
			{
				it++;
			}
		}
	}
}

MDDNode* MDD::goalAt(int level)
{
	if (level >= levels.size()) 
	{
		return nullptr; 
	}

	for (MDDNode* ptr: levels[level])
	{
		if (ptr->location == solver->goal_location && ptr->cost == level && ptr->direction == solver->goal_direction)
		{
			return ptr;
		}
	}
	return nullptr;
	// return levels[level][goal_location].get();
}

std::ostream& operator<<(std::ostream& os, const MDD& mdd)
{
	for (const auto& level : mdd.levels)
	{
		cout << "L" << level.front()->level << ": ";
		for (const auto& node : level)
		{
			cout << node->location << ",";
		}
		cout << endl;
	}
	return os;
}

MDDNode* SyncMDD::findInPruneLevel(int location, int direction, int level) const
{
	if (level < (int) prune_levels.size())
		for (auto it : prune_levels[level])
			if (it->location == location && it->direction == direction)
				return it;
	return nullptr;
}


SyncMDD::SyncMDD(const MDD& cpy) // deep copy of a MDD
{
	levels.resize(cpy.levels.size());
	auto root = new SyncMDDNode(cpy.levels[0].front()->location, cpy.levels[0].front()->direction, nullptr);
	levels[0].push_back(root);
	for (int t = 0; t < (int) levels.size() - 1; t++)
	{
		for (auto node = levels[t].begin(); node != levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.find((*node)->location,(*node)->direction, t);
			for (list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				SyncMDDNode* child = find((*cpyChild)->location,(*cpyChild)->direction, (*cpyChild)->level);
				if (child == nullptr)
				{
					child = new SyncMDDNode((*cpyChild)->location, (*cpyChild)->direction, (*node));
					levels[t + 1].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}

	}

	prune_levels.resize(cpy.prune_levels.size());
	auto prune_root = new MDDNode(cpy.prune_levels[0].front()->location, nullptr);
  	prune_root->cost = cpy.prune_levels.size() - 1;
	prune_levels[0].push_back(prune_root);
	//levels[0].insert(root->location,root)
	for (size_t t = 0; t < prune_levels.size() - 1; t++)
	{
		for (auto node = prune_levels[t].begin(); node != prune_levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.findInPruneLevel((*node)->location, (*node)->direction, (*node)->level);
			for (list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				MDDNode* child = findInPruneLevel((*cpyChild)->location,(*cpyChild)->direction, (*cpyChild)->level);
				if (child == nullptr)
				{
					child = new MDDNode((*cpyChild)->location, (*cpyChild)->direction, (*node));
					child->cost = (*cpyChild)->cost;
					prune_levels[child->level].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}

	}
}

// SyncMDDNode* SyncMDD::find(int location, int level) const
// {
// 	if (level < (int) levels.size())
// 		for (auto it : levels[level])
// 			if (it->location == location)
// 				return it;
// 	return nullptr;
// }

SyncMDDNode* SyncMDD::find(int location,int direction, int level) const
{
	if (level < (int) levels.size())
		for (auto it : levels[level])
			if (it->location == location && it->direction == direction)
				return it;
	return nullptr;
}

void SyncMDD::deleteNode(SyncMDDNode* node, int level)
{
	levels[level].remove(node);
	for (auto child = node->children.begin(); child != node->children.end(); ++child)
	{
		(*child)->parents.remove(node);
		if ((*child)->parents.empty())
			deleteNode(*child, level + 1);
	}
	for (auto parent = node->parents.begin(); parent != node->parents.end(); ++parent)
	{
		(*parent)->children.remove(node);
		if ((*parent)->children.empty())
			deleteNode(*parent, level - 1);
	}
}


void SyncMDD::clear()
{
	if (levels.empty())
		return;
	for (auto& level : levels)
	{
		for (auto& it : level)
			delete it;
	}
	levels.clear();
}


SyncMDD::~SyncMDD()
{
	clear();
}


MDD* MDDTable::getMDD(CBSNode& node, int id, size_t mdd_levels) //mdd_levels is the cost (or timesteps), id is agent id
{
	ConstraintsHasher c(id, &node);
	auto got = lookupTable[c.a].find(c);
	if (got != lookupTable[c.a].end())//find mdd for the same agent, same constraint and same cost
	{
		assert(got->second->levels.size() == mdd_levels);
		if (got->second->levels.size() == mdd_levels)
			return got->second;
		else
		{
			cout<<"error"<<std::endl;
			cout<<*(got->first.n)<<" "<<node<<std::endl;
			got->second->printNodes();
		}
	}
	releaseMDDMemory(id);

	clock_t t = clock();
	MDD* mdd = new MDD();
	ConstraintTable ct(initial_constraints[id]);
	ct.build(node, id);
	mdd->buildMDD(ct, mdd_levels, search_engines[id]);
	if (!lookupTable.empty())
	{
		lookupTable[c.a][c] = mdd;
	}
	accumulated_runtime += (double) (clock() - t) / CLOCKS_PER_SEC;
	return mdd;
}

double MDDTable::getAverageWidth(CBSNode& node, int agent, size_t mdd_levels)
{
	auto mdd = getMDD(node, agent, mdd_levels);
	double width = 0;
	for (const auto& level : mdd->levels)
		width += level.size();
	return width / mdd->levels.size();
}

void MDDTable::findSingletons(CBSNode& node, int agent, Path& path)
{
	auto mdd = getMDD(node, agent, path.size());
	for (size_t i = 0; i < mdd->levels.size(); i++)
	{
		
		int count = 0;
		unordered_set<int> loc; 
		for (auto mdd: mdd->levels[i])
		{
			if (loc.find(mdd->location) == loc.end())
			{
				loc.emplace(mdd->location);
				count++;
			}
		}
		path[i].mdd_width = count;
		//path[i].mdd_width = mdd->levels.size();
		path[i].mdd_raw_width = mdd->levels.size();
	}
		
	if (lookupTable.empty())
		delete mdd;
}

void MDDTable::releaseMDDMemory(int id)
{
	if (id < 0 || lookupTable.empty() || (int) lookupTable[id].size() < max_num_of_mdds)
		return;
	int minLength = MAX_TIMESTEP;
	for (auto mdd : lookupTable[id])
	{
		if ((int) mdd.second->levels.size() < minLength)
			minLength = mdd.second->levels.size();
	}
	for (auto mdd = lookupTable[id].begin(); mdd != lookupTable[id].end();)
	{
		if ((int) mdd->second->levels.size() == minLength)
		{
			delete mdd->second;
			mdd = lookupTable[id].erase(mdd);
			num_released_mdds++;
		}
		else
		{
			mdd++;
		}
	}
}

void MDDTable::clear()
{
	for (auto& mdds : lookupTable)
	{
		for (auto mdd : mdds)
		{
			delete mdd.second;
		}
	}
	lookupTable.clear();
}

// vector<MDDNode*> collectMDDlevel(MDD* mdd, int i)
// {
// 	std::vector<MDDNode*> loc2mdd;
// 	// unordered_map<int, MDDNode*> loc2mdd;
// 	for (MDDNode* it_0 : mdd->levels[i])
// 	{
// 		loc2mdd.push_back(it_0);
// 		// int loc = it_0->location;
// 		// loc2mdd[loc] = it_0;
// 	}
// 	return loc2mdd;
// }

// unordered_map<int, MDDNode*> collectMDDlevel(MDD* mdd, int i)
// {
// 	//std::vector<MDDNode*> loc2mdd;
// 	if (mdd->merged_levels.empty())
// 	{
// 		mdd->mergeLevels();
// 	}
// 	unordered_map<int, MDDNode*> loc2mdd;
// 	for (MDDNode* it_0 : mdd->merged_levels[i])
// 	{
// 		//loc2mdd.push_back(it_0);
// 		int loc = it_0->location;
// 		loc2mdd[loc] = it_0;
// 	}
// 	return loc2mdd;
// }

unordered_map<int, MDDNode*> collectRawMDDlevel(MDD* mdd, int i)
{
	unordered_map<int, MDDNode*> loc2mdd;
	for (MDDNode* it_0 : mdd->levels[i])
	{
		//loc2mdd.push_back(it_0);
		int loc = it_0->location;
		int dir = it_0->direction;
		loc2mdd[loc*4+dir] = it_0;
	}
	return loc2mdd;
}

unordered_map<int, MDDNode*> collectMDDlevel(MDD* mdd, int i)
{
	unordered_map<int, MDDNode*> loc2mdd;
	for (MDDNode* it_0 : mdd->levels[i])
	{
		//loc2mdd.push_back(it_0);
		int loc = it_0->location;
		int dir = it_0->direction;
		loc2mdd[loc*4+dir] = it_0;
	}
	return loc2mdd;
}

// unordered_map<int, MDDNode*> collectMDDleveltoList(MDD* mdd, int i)
// {
// 	//std::vector<MDDNode*> loc2mdd;
// 	unordered_map<int, MDDNode*> loc2mdd;
// 	for (MDDNode* it_0 : mdd->levels[i])
// 	{
// 		//loc2mdd.push_back(it_0);
// 		int loc = it_0->location;
// 		int dire = it_0->direction;
// 		loc2mdd[loc*4+dire] = it_0;
// 	}
// 	return loc2mdd;
// }

// unordered_map<int, list<MDDNode*>> collectMDDlevel(MDD* mdd, int i)
// {
// 	unordered_map<int, list<MDDNode*>> loc2mdd;
// 	for (MDDNode* it_0 : mdd->levels[i])
// 	{
// 		int loc = it_0->location;
// 		loc2mdd[loc].push_back(it_0);
// 	}
// 	return loc2mdd;
// }

// void MDD::mergeLevels()
// {
// 	merged_levels.resize(levels.size());
// 	auto start = new MDDNode(levels[0].front()->location,nullptr);
// 	start->cost = levels[0].front()->cost;
// 	start->level = levels[0].front()->level;
// 	merged_levels[0].push_back(start);
// 	for (int i = 1; i < levels.size(); i++)
// 	{
// 		unordered_map<int, MDDNode*> loc2mdd_from;
// 		for (MDDNode* it_0 : merged_levels[i-1])
// 		{
// 			int loc = it_0->location;
// 			loc2mdd_from[loc]=it_0;
// 		}
// 		unordered_map<int, list<MDDNode*>> loc2mdd_to;
// 		for (MDDNode* it_0 : levels[i])
// 		{
// 			int loc = it_0->location;
// 			loc2mdd_to[loc].push_back(it_0);
// 		}
//
//		//add nodes with unique location to level i
// 		for (auto mdd_to: loc2mdd_to)
// 		{
// 			int to_loc = mdd_to.first;
// 			auto new_node = new MDDNode(to_loc,nullptr);
// 			new_node->cost = mdd_to.second.front()->cost;
// 			new_node->level = mdd_to.second.front()->level;
// 			unordered_set<int> par_loc;
// 			for (auto node: mdd_to.second)
// 			{
// 				for (auto parent: node->parents)
// 				{
// 					par_loc.insert(parent->location);
// 				}
// 			}
// 			for (auto loc: par_loc)
// 			{
// 				new_node->parents.push_back(loc2mdd_from[loc]);
// 				loc2mdd_from[loc]->children.push_back(new_node);
// 			}
// 			merged_levels[i].push_back(new_node);
// 		}
// 	}
	
// }

// void MDD::updateMergedMdd()
// {
// 	if (merged_levels.empty())
// 		return mergeLevels();
// 	int old_height = merged_levels.size() - 1;

// 	//test
// 	previous_merged_levels.resize(merged_levels.size());
// 	previous_merged_levels = merged_levels;

// 	merged_levels.resize(levels.size());
// 	for (auto l: merged_levels)
// 	{
// 		for (auto level: l)
// 		{
// 			level->parents.clear();
// 			level->children.clear();
// 		}
// 	}
	// for (int i = 1; i < levels.size(); i++)
	// {
	// 	unordered_map<int, MDDNode*> loc2mdd_from;
	// 	for (MDDNode* it_0 : merged_levels[i-1])
	// 	{
	// 		int loc = it_0->location;
	// 		loc2mdd_from[loc]=it_0;
	// 	}
	// 	unordered_map<int, list<MDDNode*>> loc2mdd_to;
	// 	for (MDDNode* it_0 : levels[i])
	// 	{
	// 		int loc = it_0->location;
	// 		loc2mdd_to[loc].push_back(it_0);
	// 	}
//
		// //add nodes with unique location to level i
		// for (auto mdd_to: loc2mdd_to)
		// {
		// 	int to_loc = mdd_to.first;
		// 	if (i <= old_height)
		// 	{
		// 		unordered_map<int, MDDNode*> loc2mdd_check;
		// 		for (MDDNode* it_0 : merged_levels[i])
		// 		{
		// 			int loc = it_0->location;
		// 			loc2mdd_check[loc]=it_0;
		// 		}
		// 		if (loc2mdd_check.find(to_loc) != loc2mdd_check.end())
		// 		{
		// 			loc2mdd_check[to_loc]->cost = mdd_to.second.front()->cost;
		// 			loc2mdd_check[to_loc]->level = mdd_to.second.front()->level;
		// 			unordered_set<int> par_loc;
		// 			for (auto node: mdd_to.second)
		// 			{
		// 				for (auto parent: node->parents)
		// 				{
		// 					par_loc.insert(parent->location);
		// 				}
		// 			}
		// 			for (auto loc: par_loc)
		// 			{
		// 				loc2mdd_check[to_loc]->parents.push_back(loc2mdd_from[loc]);
		// 				loc2mdd_from[loc]->children.push_back(loc2mdd_check[to_loc]);
		// 			}
		// 			//merged_levels[i].push_back(loc2mdd_check[to_loc]);

		// 			continue;
		// 		}
		// 	}
	// 		auto new_node = new MDDNode(to_loc,nullptr);
	// 		new_node->cost = mdd_to.second.front()->cost;
	// 		new_node->level = mdd_to.second.front()->level;
	// 		unordered_set<int> par_loc;
	// 		for (auto node: mdd_to.second)
	// 		{
	// 			for (auto parent: node->parents)
	// 			{
	// 				par_loc.insert(parent->location);
	// 			}
	// 		}
	// 		for (auto loc: par_loc)
	// 		{
	// 			new_node->parents.push_back(loc2mdd_from[loc]);
	// 			loc2mdd_from[loc]->children.push_back(new_node);
	// 		}
	// 		merged_levels[i].push_back(new_node);
	// 	}
	// }
//
// 	// Delete useless nodes (nodes who don't have any children)
// 	for (int l = 0; l < old_height - 1; l++)
// 	{
// 		auto it = merged_levels[l].begin();
// 		while (it != merged_levels[l].end())
// 		{
// 			if ((*it)->children.empty())
// 			{
// 				it = merged_levels[l].erase(it);
// 			}
// 			else
// 			{
// 				it++;
// 			}
// 		}
// 	}
// }
