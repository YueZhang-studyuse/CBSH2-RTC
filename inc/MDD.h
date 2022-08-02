#pragma once

#include "SingleAgentSolver.h"


class MDDNode
{
public:
	MDDNode(int currloc, int currdirec, MDDNode* parent)
	{
		location = currloc; 
		//add direction here
		direction = currdirec;
		if (parent == nullptr)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
	}

	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		//add direction here
		direction = -1;
		if (parent == nullptr)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
	}

	int location;
	int level;
	int cost=0; // minimum cost of path traversing this MDD node

	//add directions
	int direction;

	bool operator==(const MDDNode& node) const
	{
		return (this->location == node.location) && (this->level == node.level) && (this->direction == node.direction);
	}


	list<MDDNode*> children;
	list<MDDNode*> parents;
};

class MDD
{
private:
	const SingleAgentSolver* solver;

public:
	vector<list<MDDNode*>> levels; //levels is the mdd tree

	bool buildMDD(const ConstraintTable& ct,
				  int num_of_levels, const SingleAgentSolver* solver);
	// bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels,
	// 	int start_location, const int* moves_offset, const std::vector<int>& my_heuristic, int map_size, int num_col);
	void printNodes() const;
	MDDNode* find(int location, int direction, int level) const;
	void deleteNode(MDDNode* node);
	void clear();
	// bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons) const;

	void increaseBy(const ConstraintTable& ct, int dLevel, SingleAgentSolver* solver);
	MDDNode* goalAt(int level);

	MDD() = default;
	MDD(const MDD& cpy);
	~MDD();
};

std::ostream& operator<<(std::ostream& os, const MDD& mdd);

class SyncMDDNode
{
public:
	SyncMDDNode(int currloc, int currdirec, SyncMDDNode* parent)
	{
		location = currloc;
		direction = currdirec;
		if (parent != nullptr)
		{
			//level = parent->level + 1;
			parents.push_back(parent);
		}
		//parent = NULL;
	}
	int location;
	int direction;
	//int level;

	bool operator==(const SyncMDDNode& node) const
	{
		return (this->location == node.location) && (this->direction == node.direction);
	}


	list<SyncMDDNode*> children;
	list<SyncMDDNode*> parents;
	list<const MDDNode*> coexistingNodesFromOtherMdds;

};


class SyncMDD
{
public:
	vector<list<SyncMDDNode*>> levels;

	SyncMDDNode* find(int location, int direction, int level) const;
	void deleteNode(SyncMDDNode* node, int level);
	void clear();

	explicit SyncMDD(const MDD& cpy);
	~SyncMDD();
};

class MDDTable
{
public:
	double accumulated_runtime = 0;  // runtime of building MDDs
	uint64_t num_released_mdds = 0; // number of released MDDs ( to save memory)

	MDDTable(const vector<ConstraintTable>& initial_constraints,
			 const vector<SingleAgentSolver*>& search_engines) :
			initial_constraints(initial_constraints), search_engines(search_engines) {}

	void init(int number_of_agents)
	{
		lookupTable.resize(number_of_agents);
	}
	~MDDTable() { clear(); }


	MDD* getMDD(CBSNode& node, int agent, size_t mdd_levels);
	void findSingletons(CBSNode& node, int agent, Path& path);
	double getAverageWidth(CBSNode& node, int agent, size_t mdd_levels);
	void clear();
private:
	int max_num_of_mdds = 10000;

	vector<unordered_map<ConstraintsHasher, MDD*,
			ConstraintsHasher::Hasher, ConstraintsHasher::EqNode>> lookupTable;

	const vector<ConstraintTable>& initial_constraints;
	const vector<SingleAgentSolver*>& search_engines;
	void releaseMDDMemory(int id);
};

//vector<MDDNode*> collectMDDlevel(MDD* mdd, int i);
unordered_map<int, MDDNode*> collectMDDlevel(MDD* mdd, int i);
