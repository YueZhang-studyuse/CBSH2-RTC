#include "CorridorReasoning.h"
#include "Conflict.h"
#include <memory>
#include "SpaceTimeAStar.h"
#include "SIPP.h"

//only modify the getTravelTime for corridor target conflict now, others pending modification

shared_ptr<Conflict> CorridorReasoning::run(const shared_ptr<Conflict>& conflict,
	const vector<Path*>& paths, const CBSNode& node)
{
	clock_t t = clock();
    shared_ptr<Conflict> corridor = nullptr;
    switch(strategy)
    {
        case corridor_strategy::NC:
            return corridor;
        case corridor_strategy::C:
            corridor = findCorridorConflict(conflict, paths, node);
            break;
        case corridor_strategy::PC:
            corridor = findCorridorConflict(conflict, paths, node);
            if (corridor == nullptr)
                corridor = findPseudoCorridorConflict(conflict, paths, node);
            break;
        case corridor_strategy::STC:
            corridor = findCorridorTargetConflict(conflict, paths, node);
            break;
        case corridor_strategy::GC:
        case corridor_strategy::DC:
            corridor = findCorridorTargetConflict(conflict, paths, node);
            if (corridor == nullptr)
                corridor = findPseudoCorridorConflict(conflict, paths, node);
            break;
    }
	accumulated_runtime += (double)(clock() - t) / CLOCKS_PER_SEC;
	return corridor;
}
//plan to do
//a function returns the pair of <leaving edge (through direction), leaving time>
//a new get travel time function that block the leaving edge, to calculate the time from start to the enter corridor direction
// above is clearer than only + 1 --verision 1
// also claerer than the previous corridor reasoning --version 0
// the version in the paper
shared_ptr<Conflict> CorridorReasoning::findCorridorConflict(const shared_ptr<Conflict>& conflict,
                                                             const vector<Path*>& paths, const CBSNode& node)
{
    int a[2] = { conflict->a1, conflict->a2 };
    int  agent, loc1, loc2, timestep;
    constraint_type type;
    tie(agent, loc1, loc2, timestep, type) = conflict->constraint1.back();
    int curr = -1;
    if (search_engines[0]->instance.getDegree(loc1) == 2)
    {
        curr = loc1;
        if (loc2 >= 0) //edge conflict
            timestep--;
    }
    else if (loc2 >= 0 && search_engines[0]->instance.getDegree(loc2) == 2)
        curr = loc2; // edge conflict, the comming vetex in corridor

    if (curr <= 0)
        return nullptr;

    int enter_time[2];
    for (int i = 0; i < 2; i++)
        enter_time[i] = getEnteringTime(*paths[a[i]], *paths[a[1 - i]], timestep);
    if (enter_time[0] > enter_time[1])
    {
        int temp = enter_time[0]; enter_time[0] = enter_time[1]; enter_time[1] = temp;
        temp = a[0]; a[0] = a[1]; a[1] = temp;
    }
    int enter_location[2];
	int enter_direction[2];
    for (int i = 0; i < 2; i++)
	{
		enter_location[i] = paths[a[i]]->at(enter_time[i]).location;
		enter_direction[i] = paths[a[i]]->at(enter_time[i]).direction;
	}
    if (enter_location[0] == enter_location[1])
        return nullptr;
    for (int i = 0; i < 2; i++)
    {
        bool found = false;
        for (int time = enter_time[i]; time < (int)paths[a[i]]->size() && !found; time++)
        {
            if (paths[a[i]]->at(time).location == enter_location[1 - i])
                found = true;
        }
        if (!found)
            return nullptr;
    }
    pair<int, int> edge; // one edge in the corridor
    int corridor_length = getCorridorLength(*paths[a[0]], enter_time[0], enter_location[1], edge);
    if (corridor_length < 2)
	{
        return nullptr;
	}
	//std::cout<<"length "<<corridor_length<<std::endl;
    int t3, t3_, t4, t4_;
    ConstraintTable ct1(initial_constraints[a[0]]);
    ct1.build(node, a[0]);
	//modify here, try to get the time agent leave this corridor
	//get the travel time to each direction exclude the corridor direciton
	//so need to first get the corridor direction
	//then check is the neighbor of this direction valid
	int moves_forward_offset[] = {-search_engines[0]->getInstanceCols(),1,search_engines[0]->getInstanceCols(),-1};
	int t3_temp = -1;
	for (int i = 0; i < 4; i++)
	{
		int temp = 0;
		if (i == enter_direction[1])
		{
			continue;
		}
		//need additional checking for valid direction pass here
		int neighbor = enter_location[1] + moves_forward_offset[i];
		if (!search_engines[0]->instance.validMove(enter_location[1],neighbor))
		{
			continue;
		}
		if (abs(i-enter_direction[1]) == 2)
		{
			temp = search_engines[a[0]]->getTravelTime(enter_location[1], i, ct1, MAX_TIMESTEP) + corridor_length + 1;
		}
		else
		{
			temp = search_engines[a[0]]->getTravelTime(enter_location[1], i, ct1, MAX_TIMESTEP) + corridor_length;
		}
		if (t3_temp == -1 || t3_temp > temp)
			t3_temp = temp;
	}
	//then fix the agent forward pass in this direciton, and then block this forward pass (as an edge constraint), and get the travel time for another agent
	//set min of the sum as the corridor constraint
    t3 = search_engines[a[0]]->getTravelTime(enter_location[1], -1, ct1, MAX_TIMESTEP); // the time from start to the end point for agent 1
    ct1.insert2CT(edge.first, edge.second, 0, MAX_TIMESTEP); // block the corridor in both directions
    ct1.insert2CT(edge.second, edge.first, 0, MAX_TIMESTEP);
    t3_ = search_engines[a[0]]->getTravelTime(enter_location[1], -1,  ct1, t3 + 2 * corridor_length + 1); //find the bypass
    ConstraintTable ct2(initial_constraints[a[1]]);
    ct2.build(node, a[1]);
    t4 = search_engines[a[1]]->getTravelTime(enter_location[0], -1 , ct2, MAX_TIMESTEP);
	int t4_temp = -1;
	for (int i = 0; i < 4; i++)
	{
		int temp = 0;
		if (i == enter_direction[0])
		{
			continue;
		}
		if (abs(i-enter_direction[0]) == 2)
		{
			temp = search_engines[a[1]]->getTravelTime(enter_location[0], i, ct1, MAX_TIMESTEP) + corridor_length + 1;
		}
		else
		{
			temp = search_engines[a[1]]->getTravelTime(enter_location[0], i, ct1, MAX_TIMESTEP) + corridor_length;
		}
		if (t4_temp == -1 || t4_temp > temp)
			t4_temp = temp;
	}
    ct2.insert2CT(edge.first, edge.second, 0, MAX_TIMESTEP); // block the corridor in both directions
    ct2.insert2CT(edge.second, edge.first, 0, MAX_TIMESTEP);
    t4_ = search_engines[a[1]]->getTravelTime(enter_location[0], -1, ct2, t3 + corridor_length + 1);

    if (abs(t3 - t4) <= corridor_length && t3_ > t3 && t4_ > t4)
    {
        //int t1 = std::min(t3_ - 1, t4 + corridor_length+1);
        //int t2 = std::min(t4_ - 1, t3 + corridor_length+1);
		int t1 = std::min(t3_ - 1, t4_temp);
        int t2 = std::min(t4_ - 1, t3_temp);
        list<Constraint> C1, C2;
        C1.emplace_back(a[0], enter_location[1], 0, t1, constraint_type::RANGE);
        C2.emplace_back(a[1], enter_location[0], 0, t2, constraint_type::RANGE);
        if (blocked(*paths[a[0]], C1.front()) &&
            blocked(*paths[a[1]], C2.front()))
        {
            shared_ptr<Conflict> corridor = make_shared<Conflict>();
            corridor->corridorConflict(a[0], a[1], C1, C2);
            return corridor;
        }
    }

    return nullptr;
}

shared_ptr<Conflict> CorridorReasoning::findPseudoCorridorConflict(const shared_ptr<Conflict>& conflict,
	const vector<Path*>& paths, const CBSNode& node)
{
	int  agent, loc1, loc2, timestep;
	int c_length = 0;
	constraint_type type;
	tie(agent, loc1, loc2, timestep, type) = conflict->constraint1.back();
	int endpoint1, endpoint2, lowerbound1, lowerbound2; // the timestep of the range constraint at the endpoint should be >= the lowerbound
	if (loc2 < 0) // vertex conflict
	{
		// if (paths[conflict->a1]->size() <= timestep + 1 || paths[conflict->a2]->size() <= timestep + 1)
		// 	return nullptr; // no need to modify
		//modify starts here
		//find the corridor near the t-3/t+3 of the current conflict
		endpoint1 = loc1;
		endpoint2 = loc1;
		for (int t = 1; t <= 2; t++)
		{
			if (timestep - t < 0 || paths[conflict->a1]->size() <= timestep+t || paths[conflict->a2]->size() <= timestep + t)
			{
				break;
			}
			if (paths[conflict->a1]->at(timestep - t).location != paths[conflict->a2]->at(timestep + t).location ||
			paths[conflict->a2]->at(timestep - t).location != paths[conflict->a1]->at(timestep + t).location)
			{
				break;
			}
			if (!paths[conflict->a1]->at(timestep - t).is_single() || 
			!paths[conflict->a1]->at(timestep).is_single() ||
			!paths[conflict->a1]->at(timestep + t).is_single() ||
			!paths[conflict->a2]->at(timestep - t).is_single() ||
			!paths[conflict->a2]->at(timestep).is_single() ||
			!paths[conflict->a2]->at(timestep + t).is_single())
			{
				break;
			}
			if (paths[conflict->a1]->at(timestep - t + 1).location == paths[conflict->a1]->at(timestep - t).location ||
			paths[conflict->a2]->at(timestep - t + 1).location == paths[conflict->a2]->at(timestep - t).location )
			{
				continue;
			}
			endpoint1 = paths[conflict->a1]->at(timestep + t).location;
			endpoint2 = paths[conflict->a1]->at(timestep - t + 1).location;
			lowerbound1 = timestep + t;
			lowerbound2 = timestep + t - 1;
			if (c_length == 0)
			{
				c_length = 1;
			}
			else
			{
				c_length+=2;
			}
		}
		if (endpoint1 == endpoint2)
		{
			return nullptr;
		}
		// if (paths[conflict->a1]->at(timestep - 1).location != paths[conflict->a2]->at(timestep + 1).location ||
		// 	paths[conflict->a2]->at(timestep - 1).location != paths[conflict->a1]->at(timestep + 1).location)
		// 	return nullptr; 
		// if (!paths[conflict->a1]->at(timestep - 1).is_single() || 
		// 	!paths[conflict->a1]->at(timestep).is_single() ||
		// 	!paths[conflict->a1]->at(timestep + 1).is_single() ||
		// 	!paths[conflict->a2]->at(timestep - 1).is_single() ||
		// 	!paths[conflict->a2]->at(timestep).is_single() ||
		// 	!paths[conflict->a2]->at(timestep + 1).is_single())
		// 	return nullptr;
		// endpoint1 = paths[conflict->a1]->at(timestep + 1).location;
		// endpoint2 = loc1;
		// lowerbound1 = timestep + 1;
		// lowerbound2 = timestep;
	}
	else // edge conflict
	{
		if (!paths[conflict->a1]->at(timestep - 1).is_single() ||
			!paths[conflict->a1]->at(timestep).is_single() ||
			!paths[conflict->a2]->at(timestep - 1).is_single() ||
			!paths[conflict->a2]->at(timestep).is_single())
			return nullptr;
		endpoint1 = loc2;
		endpoint2 = loc1;
		lowerbound1 = timestep;
		lowerbound2 = timestep;

		//add here, chech +1/-1 +2/-1
		for (int t = 1; t <=2; t++)
		{
			if (timestep - t - 1 < 0 || paths[conflict->a1]->size() <= timestep+t || paths[conflict->a2]->size() <= timestep + t)
			{
				break;
			}
			if (paths[conflict->a1]->at(timestep - t - 1).location != paths[conflict->a2]->at(timestep + t).location ||
			paths[conflict->a2]->at(timestep - t - 1).location != paths[conflict->a1]->at(timestep + t).location)
			{
				break;
			}
			if (!paths[conflict->a1]->at(timestep - t - 1).is_single() || 
			!paths[conflict->a1]->at(timestep + t).is_single() ||
			!paths[conflict->a2]->at(timestep - t - 1).is_single() ||
			!paths[conflict->a2]->at(timestep + t).is_single())
			{
				break;
			}
			if (paths[conflict->a1]->at(timestep - t).location == paths[conflict->a1]->at(timestep - t - 1).location ||
			paths[conflict->a2]->at(timestep - t).location == paths[conflict->a2]->at(timestep - t - 1).location )
			{
				continue;
				//break;
			}
			endpoint1 = paths[conflict->a1]->at(timestep + t).location;
			endpoint2 = paths[conflict->a1]->at(timestep - t - 1).location;
			lowerbound1 = timestep + t;
			lowerbound2 = timestep + t;
			c_length+=2;
		}
	}

	//up to here
	//need a different way to calculate
	int t[2] = {-1, -1}, tprime[2] = {-1, -1};
	ConstraintTable ct1(initial_constraints[conflict->a1]);
	ct1.build(node, conflict->a1);
	t[0] = search_engines[conflict->a1]->getTravelTime(endpoint1, -1,  ct1, MAX_TIMESTEP);
	if (t[0] + c_length < lowerbound2)
		return nullptr;
	ConstraintTable ct2(initial_constraints[conflict->a2]);
	ct2.build(node, conflict->a2);
	t[1] = search_engines[conflict->a2]->getTravelTime(endpoint2, -1,  ct2, MAX_TIMESTEP);
	if (t[1] + c_length < lowerbound1)
		return nullptr;
	//need to block the corridor for given time step
	int block_time = c_length;
	// if (block_time > 4)
	// {
	// 	block_time = 4;
	// }
	if (loc2 < 0)
	{
		ct1.insert2CT(loc1, timestep, block_time); // block the conflict location
		ct2.insert2CT(loc1, timestep, block_time); // block the conflict location
	}
	else
	{
		ct1.insert2CT(loc1, loc2, timestep, block_time);
		ct2.insert2CT(loc2, loc1, timestep, block_time);
	}
		
	//rotation: is it correct to only block the entry point?
	//ct1.insert2CT(endpoint1, 0, MAX_TIMESTEP); // TODO:: Is this correct? Can we block the entire horizon?
	tprime[0] = search_engines[conflict->a1]->getTravelTime(endpoint1, -1,  ct1, t[1] + c_length + 1);
	//std::cout<<tprime[0]<<std::endl;
	if (tprime[0] - 1 < lowerbound1)
		return nullptr;
	tprime[1] = search_engines[conflict->a2]->getTravelTime(endpoint2, -1,  ct2, t[0] + c_length  + 1);
	if (tprime[1] - 1 < lowerbound2)
		return nullptr;
	int t1 = std::min(tprime[0] - 1, t[1] + c_length);
	int t2 = std::min(tprime[1] - 1, t[0] + c_length);
	//auto t = make_pair(t1, t2);
	//end here
	// auto t = getTimeRanges(conflict->a1, conflict->a2, endpoint1, endpoint2, endpoint2, endpoint1,
	// 	lowerbound1, lowerbound2, c_length, node); // return (-1, -1) if doesn't exist

	//std::cout<<"pc test 2"<<t.first<<" "<<t.second<<std::endl;
	if (t1 >= 0)
	{
		shared_ptr<Conflict> corridor_conflict = make_shared<Conflict>();
		list<Constraint> C1, C2;
		C1.emplace_back(conflict->a1, endpoint1, 0, t1, constraint_type::RANGE);
		C2.emplace_back(conflict->a2, endpoint2, 0, t2, constraint_type::RANGE);
		corridor_conflict->corridorConflict(conflict->a1, conflict->a2, C1, C2);
		num_pesudo_corridors++;
		return corridor_conflict;
	}
	return nullptr;
}

//modify here now 
shared_ptr<Conflict> CorridorReasoning::findCorridorTargetConflict(const shared_ptr<Conflict>& conflict,
	const vector<Path*>& paths, const CBSNode& node)
{
	assert(conflict->constraint1.size() == 1);
	int  agent, loc1, loc2, timestep;
	constraint_type type;
	tie(agent, loc1, loc2, timestep, type) = conflict->constraint1.back();
	auto corridor = findCorridor(loc1, loc2); //get the corridor C (b and e included)
	if (corridor.empty()) //no corridor exists
		return nullptr;
	int corridor_length = (int)corridor.size() - 1; //corridor size -- -1 because begin and end is include
	int a[2] = { conflict->a1, conflict->a2 };
	int entry[2] = { -1, -1 }, exit[2] = { -1, -1 }, start[2] = {-1, -1}, goal[2] = { -1, -1 }; // store the corresponding node indices in the corridor
	int goal_time[2] = { -1, -1 };
	//entrying direction
	int corridor_front_direc = get_corridor_direction(corridor[1] - corridor.front());
	int corridor_back_direc = get_corridor_direction(corridor[corridor.size()-2] - corridor.back());

	//also need entry direction and exit direction for two agents
	//int entry_direction[2] = {-1,-1};
	//int exit_direction[2] = {-1,-1};

	for (int i = 0; i < 2; i++) //find two agents relevant information
	{
		for (int j = 0; j < (int)corridor.size(); j++)
		{
			if (paths[a[i]]->front().location == corridor[j]) // the start location is inside the corridor
			{
				start[i] = j;  // the jth in the coridor
				break;
			}
		}
		if (start[i] == -1) // the start location is not inside the corridor
		{
			//search from the collision time step to time step 0
			for (int t = min((int)paths[a[i]]->size(), timestep) - 1; t >= 0; t--) // find the entry point
			{
				if (paths[a[i]]->at(t).location == corridor.front())
				{
					entry[i] = 0; //entry has the same direction with corridor
					//entry_direction[i] = paths[a[i]]->at(t).direction;
					break;
				}
				else if (paths[a[i]]->at(t).location == corridor.back())
				{
					entry[i] = (int)corridor.size() - 1; //opposite direction with corridor
					//entry_direction[i] = paths[a[i]]->at(t).direction;
					break;
				}
			}
		}
		for (int j = 0; j < (int)corridor.size(); j++)
		{
			if (paths[a[i]]->back().location == corridor[j]) // the goal location is inside the corridor
			{
				goal[i] = j; //just represent goal is in jth in the corridor, we need to also consider goal direction
				goal_time[i] = (int)paths[a[i]]->size() - 1; 
				break;
			}
		}
		if (goal[i] == -1) //goal not in corridor
		{
			for (int t = timestep; t < (int)paths[a[i]]->size() - 1; t++) // find the exit point, search from conflict time step to the end
			{
				if (paths[a[i]]->at(t).location == corridor.front())
				{
					exit[i] = 0; //opposite direction with corridor
					//exit_direction = paths[a[i]]->at(t).direction;
					// goal[i] = 0;
					goal_time[i] = t;
					break;
				}
				else if (paths[a[i]]->at(t).location == corridor.back())
				{
					exit[i] = (int)corridor.size() - 1; //same direction with corridor
					//exit_direction = paths[a[i]]->at(t).direction;
					goal_time[i] = t;
					break;
				}
			}
		}
	} //above find the entry and exit
	if ((max(start[0], entry[0]) - max(start[1], entry[1])) * (max(goal[0], exit[0]) - max(goal[1], exit[1])) >= 0) // the start(entry) and goal(exit) locations need to be swaped
		return nullptr;
	list<Constraint> C1, C2;


	if (goal[0] >= 0 || goal[1] >= 0) // The goal location of one of the agents are inside the corridor
	{ 
		//return nullptr;
		// the is a corridor-target conflict no matter the two agents move in the same direction or not
		int middle_agent = (goal[0] >= 0) ? 0 : 1; // will add length constraints to this agent 
		if (start[1] == goal[1] && start[1] >= 0)  // if the start and goal locations of one agent are both (and, of course, inside the corridor). 
			middle_agent = 1; // We prioritize this agent to be the middle agent //both start and goal are inside corridor
		ConstraintTable ct1(initial_constraints[a[middle_agent]]);
		ct1.build(node, a[middle_agent]);
		ConstraintTable ct1_back(initial_constraints[a[middle_agent]]);
		ct1_back.build(node, a[middle_agent]);
		ConstraintTable ct2(initial_constraints[a[1 - middle_agent]]);
		ct2.build(node, a[1 - middle_agent]);
		
		auto t1 = search_engines[a[middle_agent]]->getTravelTime(corridor.front(),corridor_front_direc, ct1, MAX_TIMESTEP) - 1 + 2;
		ct1.insert2CT(corridor.front(), corridor[1], 0, MAX_TIMESTEP); // block the corridor in both directions
		ct1.insert2CT(corridor[1],corridor.front(), 0, MAX_TIMESTEP);
		auto t1_bypass = search_engines[a[middle_agent]]->getTravelTime(corridor.front(), corridor_front_direc, ct1, MAX_TIMESTEP) - 1;
		if (t1_bypass < t1)
			t1 = t1_bypass;
		int t2;
		if (entry[a[1-middle_agent]] == 0)
			t2 = search_engines[a[1 - middle_agent]]->getTravelTime(corridor.front(),corridor_front_direc, ct2, MAX_TIMESTEP);
		else //a2 enter in corridor back, so we calculate the leaving time
			t2 = search_engines[a[1 - middle_agent]]->getTravelTime(corridor.front(),-1, ct2, MAX_TIMESTEP) + 1;
		auto reach_goal_direct = get_corridor_direction(corridor[goal[middle_agent]]-corridor[goal[middle_agent]-1]);
		auto goal_time_with_dir = goal[middle_agent] + abs(reach_goal_direct-paths[a[middle_agent]]->back().direction)%3;
		auto l1 = max(t1,t2) + goal_time_with_dir;

		auto ll1 = l1;
		auto tt1 = t1;
		auto tt2 = t2;

		//a1 enter from corridor from end
		//first do not consider bypass
		// t1 = search_engines[a[middle_agent]]->getTravelTime(middle_agent_end, middle_end_direction, ct1, l1 - corridor_length + goal_time_with_dir) - 1 + 2;
		// ct1.insert2CT(corridor[entry[1-middle_agent]], corridor[entry[1-middle_agent]-dir], 0, MAX_TIMESTEP); // block the corridor in both directions
		// ct1.insert2CT(corridor[entry[1-middle_agent]-dir],corridor[entry[1-middle_agent]], 0, MAX_TIMESTEP);
		// auto t1_bypass = search_engines[a[middle_agent]]->getTravelTime(middle_agent_end, middle_end_direction, ct1, MAX_TIMESTEP) - 1;
		// if (t1_bypass < t1)
		// 	t1 = t1_bypass;
		// reach_goal_direct = get_corridor_direction(corridor[goal[middle_agent]]-corridor[goal[middle_agent]+dir]);
		// goal_time_with_dir = goal_time_without_dir_back + abs(reach_goal_direct-paths[a[middle_agent]]->back().direction)%2;
		// t2 = search_engines[a[1 - middle_agent]]->getTravelTime(middle_agent_end, middle_end_direction, ct2, l1 - corridor_length + goal_time_with_dir);
		// l1 = min(l1, max(t1, t2) + goal_time_with_dir);
		// if (l1 < (int)paths[a[middle_agent]]->size() - 1) // the length constraint for the left child node will not change the paths of the agents
		// 	return nullptr;
		// C1.emplace_back(a[middle_agent], corridor[goal[middle_agent]], -1, l1, constraint_type::GLENGTH);
		// C2.emplace_back(a[middle_agent], corridor[goal[middle_agent]], -1, l1, constraint_type::LEQLENGTH);

		t1 = search_engines[a[middle_agent]]->getTravelTime(corridor.back(), corridor_back_direc, ct1_back, l1 - corridor_length + goal_time_with_dir) - 1 + 1;
		ct1_back.insert2CT(corridor.back(),corridor[corridor.size()-2], 0, MAX_TIMESTEP); // block the corridor in both directions
		ct1_back.insert2CT(corridor[corridor.size()-2], corridor.back(), 0, MAX_TIMESTEP);
		t1_bypass = search_engines[a[middle_agent]]->getTravelTime(corridor.back(), corridor_back_direc, ct1_back, MAX_TIMESTEP) - 1;
		if (t1_bypass < t1)
			t1 = t1_bypass;
		reach_goal_direct = get_corridor_direction(corridor[goal[middle_agent]]-corridor[goal[middle_agent]+1]);
		goal_time_with_dir = corridor_length-goal[middle_agent] + abs(reach_goal_direct-paths[a[middle_agent]]->back().direction)%3;
		if (entry[a[1-middle_agent]] == (int)corridor.size() - 1)
			t2 = search_engines[a[1 - middle_agent]]->getTravelTime(corridor.back(), corridor_back_direc, ct2, l1 - corridor_length + goal_time_with_dir);
		else
			t2 = search_engines[a[1 - middle_agent]]->getTravelTime(corridor.back(), -1, ct2, l1 - corridor_length + goal_time_with_dir)+1;
		l1 = min(l1, max(t1, t2) + goal_time_with_dir);
		if (l1 < (int)paths[a[middle_agent]]->size() - 1) // the length constraint for the left child node will not change the paths of the agents
			return nullptr;
		C1.emplace_back(a[middle_agent], corridor[goal[middle_agent]], -1, l1, constraint_type::GLENGTH);
		C2.emplace_back(a[middle_agent], corridor[goal[middle_agent]], -1, l1, constraint_type::LEQLENGTH);

		//check bypass for agent 2
		int dir = max(goal[1 - middle_agent], exit[1 - middle_agent]) - max(start[1 - middle_agent], entry[1 - middle_agent]);
		assert(dir != 0); // the start and goal locations of the other agent cannot be the same, otherwise, the two agents didn't swap their relative locations
		dir = dir / abs(dir);
		int idx = max(exit[1 - middle_agent], goal[1 - middle_agent]); // the index of the exit/goal location
		auto edge = make_pair(corridor[idx], corridor[idx - dir]);
		ct2.insert2CT(edge.first, edge.second, 0, MAX_TIMESTEP); // block the corridor in both directions
		ct2.insert2CT(edge.second, edge.first, 0, MAX_TIMESTEP);
		auto l2 = search_engines[a[1 - middle_agent]]->getTravelTime(edge.first, -1, ct2, MAX_TIMESTEP) - 1;
		if (goal[1 - middle_agent] >= 0) // The goal location of the other agent is also inside the corridor
		{
			if (l2 < (int)paths[a[1 - middle_agent]]->size() - 1) // the length constraint below will not change the path of the agent
				return nullptr;
			C2.emplace_back(a[1 - middle_agent], edge.first, -1, l2, constraint_type::GLENGTH); // add length constraint to C2
		}
		else
		{
			if (goal_time[1 - middle_agent] >= l2) // the range constraint does not block the current path
				return nullptr;
			C2.emplace_back(a[1 - middle_agent], edge.first, 0, l2, constraint_type::RANGE); // add range constraint to C2
		}
		shared_ptr<Conflict> corridor_conflict = make_shared<Conflict>();
		corridor_conflict->corridorConflict(a[middle_agent], a[1 - middle_agent], C1, C2);
		return corridor_conflict;
	}
	else
	{
		int dir = exit[0] - max(start[0], entry[0]);
		assert(dir != 0); // the start and goal locations of the agent cannot be the same, otherwise, the two agents didn't swap their relative locations
		dir = dir / abs(dir);
		auto t = getTimeRanges(a[0], a[1], corridor[exit[0]], corridor[exit[1]], 
			corridor[exit[0] - dir], corridor[exit[1] + dir], goal_time[0], goal_time[1], corridor_length, node);

		if (t.first >= 0)
		{
			shared_ptr<Conflict> corridor_conflict = make_shared<Conflict>();
			C1.emplace_back(a[0], corridor[exit[0]], 0, t.first, constraint_type::RANGE);
			C2.emplace_back(a[1], corridor[exit[1]], 0, t.second, constraint_type::RANGE);
			corridor_conflict->corridorConflict(a[0], a[1], C1, C2);
			return corridor_conflict;
		}
	}
	return nullptr;
}



vector<int> CorridorReasoning::findCorridor(int loc1, int loc2)
{
    list<int> rst;
    if (search_engines[0]->instance.getDegree(loc1) == 2)
    {
        rst.push_back(loc1);
    }
    else if (loc2 >= 0 && search_engines[0]->instance.getDegree(loc2) == 2)
    {
        rst.push_back(loc2);
    }
    if (rst.empty())
        return vector<int>();

    auto root = rst.front();
    auto prev = root;
    auto curr = search_engines[0]->instance.getNeighbors(root).front();
    rst.push_front(curr);
    auto neighbors = search_engines[0]->instance.getNeighbors(curr); //east, west, south, north
    while (neighbors.size() == 2)
    {
        auto next = (neighbors.front() == prev)? neighbors.back() : neighbors.front();
        rst.push_front(next);
        prev = curr;
        curr = next;
        neighbors = search_engines[0]->instance.getNeighbors(next);
    }
    prev = root;
    curr = search_engines[0]->instance.getNeighbors(root).back();
    rst.push_back(curr);
    neighbors = search_engines[0]->instance.getNeighbors(curr);
    while (neighbors.size() == 2)
    {
        auto next = (neighbors.front() == prev) ? neighbors.back() : neighbors.front();
        rst.push_back(next);
        prev = curr;
        curr = next;
        neighbors = search_engines[0]->instance.getNeighbors(next);
    }

    // When k=2, it might just be a corner cell, which we do not want to recognize as a corridor
    /*if (rst.size() == 3 &&
        search_engines[0]->instance.getColCoordinate(rst.front()) != search_engines[0]->instance.getColCoordinate(rst.back()) &&
        search_engines[0]->instance.getRowCoordinate(rst.front()) != search_engines[0]->instance.getRowCoordinate(rst.back()))
    {
        rst.clear();
    }*/

    return vector<int>(rst.begin(), rst.end());
}


int CorridorReasoning::getEnteringTime(const vector<PathEntry>& path, const vector<PathEntry>& path2, int t) //agent 1 path, agent 2 path, conflict timestep
{
	if (t >= (int) path.size())
		t = (int) path.size() - 1; //target?
	int loc = path[t].location;
	int direction = path[t].direction;
	while (loc != path.front().location && loc != path2.back().location && search_engines[0]->instance.getDegree(loc) == 2)
	{
		t--;
		loc = path[t].location;
		direction = path[t].direction;
	}
	return t;
}


int CorridorReasoning::getCorridorLength(const vector<PathEntry>& path, int t_start, int loc_end, pair<int, int>& edge)
//a1 path, a1 enter time, a2 enter location, edge
{
	int curr = path[t_start].location;
	int next;
	int prev = -1;
	int length = 0; // distance to the start location
	int t = t_start;
	bool moveForward = true;
	bool updateEdge = false;
	while (curr != loc_end)
	{
		t++;
		next = path[t].location;
		if (next == curr) // wait
			continue;
		else if (next == prev) // turn around
			moveForward = !moveForward;
		if (moveForward)
		{
			if (!updateEdge)
			{
				edge = make_pair(curr, next);
				updateEdge = true;
			}
			length++;
		}
		else
			length--;
		prev = curr;
		curr = next;
	}
	return length;
}


pair<int, int> CorridorReasoning::getTimeRanges(int a1, int a2, int endpoint1, int endpoint2,
	int from1, int from2, int lowerbound1, int lowerbound2, int corridor_length, const CBSNode& node)
{
	int t[2] = {-1, -1}, tprime[2] = {-1, -1};

	ConstraintTable ct1(initial_constraints[a1]);
	ct1.build(node, a1);
	t[0] = search_engines[a1]->getTravelTime(endpoint1, -1,  ct1, MAX_TIMESTEP);
	if (t[0] + corridor_length < lowerbound2)
		return make_pair(-1, -1);
	ConstraintTable ct2(initial_constraints[a2]);
	ct2.build(node, a2);
	t[1] = search_engines[a2]->getTravelTime(endpoint2, -1,  ct2, MAX_TIMESTEP);
	if (t[1] + corridor_length < lowerbound1)
		return make_pair(-1, -1);
	//need to block the corridor for given time step
	//ct1.insert2CT(from1, 0, MAX_TIMESTEP); // block the corridor in both directions
	//rotation: is it correct to only block the entry point?
	ct1.insert2CT(from1, endpoint1, 0, MAX_TIMESTEP); // block the corridor in both directions
	ct1.insert2CT(endpoint1,from1, 0, MAX_TIMESTEP); // TODO:: Is this correct? Can we block the entire horizon?
	tprime[0] = search_engines[a1]->getTravelTime(endpoint1, -1,  ct1, t[1] + corridor_length + 1);
	//std::cout<<tprime[0]<<std::endl;
	if (tprime[0] - 1 < lowerbound1)
		return make_pair(-1, -1);
	ct2.insert2CT(from2, endpoint2, 0, MAX_TIMESTEP); // block the corridor in both directions
	ct2.insert2CT(endpoint2, from2, 0, MAX_TIMESTEP);
	tprime[1] = search_engines[a2]->getTravelTime(endpoint2, -1,  ct2, t[0] + corridor_length + 1);
	if (tprime[1] - 1 < lowerbound2)
		return make_pair(-1, -1);
	int t1 = std::min(tprime[0] - 1, t[1] + corridor_length + 1);
	int t2 = std::min(tprime[1] - 1, t[0] + corridor_length + 1);
	return make_pair(t1, t2);
}

bool CorridorReasoning::blocked(const Path& path, const Constraint& constraint)
{
	int a, loc, t1, t2;
	constraint_type type;
	tie(a, loc, t1, t2, type) = constraint;
	assert(type == constraint_type::RANGE);
	for (int t = t1; t < t2; t++)
	{
		if ((t >= (int)path.size() && loc == path.back().location) ||
			(t >= 0 && path[t].location == loc))
			return true;
	}
	return false;
}

int CorridorReasoning::get_corridor_direction(int edge_difference)
{
	if (edge_difference == 1)
	{
		return 1;
	}
	if (edge_difference == -1)
	{
		return 3;
	}
	if (edge_difference > 1)
	{
		return 0;
	}
	return 2;
}

