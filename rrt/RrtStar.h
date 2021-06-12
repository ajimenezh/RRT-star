#pragma once

#include "Trajectory.h"
#include "DubinTrajectory.h"
#include "Graph.h"
#include "ObstaclesData.h"

#include <set>
#include <functional>
#include <queue>

typedef std::pair<double, double> tPrioQueueKey;
typedef std::pair<tPrioQueueKey, int> tPrioQueueElem;

template<class T>
class RrtStar {
public:
	RrtStar(const Node& goal, const ObstaclesData& obstacles, double min_radius = 10.0) : 
		goal_(goal), min_radius_(min_radius), obstacles_(obstacles) {
	}

	RrtStar(const Node& start, const Node& goal, const ObstaclesData& obstacles, double min_radius = 10.0) : 
		start_(start), goal_(goal), obstacles_(obstacles), min_radius_(min_radius) {
	}

	void Solve();

	std::unique_ptr<T> ComputeTrajectory(const Node& from, const Node& to, double r);

	double LookAheadEstimate(const Node& v);

	double CostG(const Node& v);

	void SetLookAheadEstimate(const Node& v, double val);

	std::optional<Node> FindParent(const Node& v, const std::set<Node>& nodes, double r);

	void Extend(const Node& v, double r);

	void CullNeighbors(const Node& v, double r);

	void RewireNeighbors(const Node& v, double r);

	void ReduceInconsistency(double r);

	void UpdateLMC(const Node& v, double r);

	void SetG(int v_id, double val);

	Node RandomNode();

	Node Saturate(const Node& v, const Node& v_nearest, double delta);

	double ShrinkingBallRadius(int n);

	void DrawTrajectory(const Node& a, const Node& b, std::unique_ptr<Trajectory>& traj);

	void DrawGraph();

private:
	tPrioQueueKey QueueKey(const Node& v);

	bool IsInQueue(const Node& v);

private:

	Node goal_;

	Node start_;

	Node v_bot_;

	Graph<T> graph_;

	double min_radius_;

	std::priority_queue<tPrioQueueElem, 
		std::vector<tPrioQueueElem>, std::less<tPrioQueueElem> > q_;

	std::set<int> is_in_q_;

	std::map<int, double> lmc_;

	ObstaclesData obstacles_;

};

