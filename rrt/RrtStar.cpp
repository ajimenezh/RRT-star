#include "RrtStar.h"

#include "PngImage.h"
#include "lib_png_utils.h"

#include <random>

// const double INF = 1.0e8;
// const double EPS = 1.0e-4;

const int traj_n_points = 100;

template class RrtStar<DubinTrajectory>;

double fRand(double l, double r) {
	std::uniform_real_distribution<double> unif(l, r);
	std::random_device rd;
	std::mt19937 gen(rd()); 
	return unif(gen);
}

template <class T>
std::unique_ptr<T> RrtStar<T>::ComputeTrajectory(const Node& from, const Node& to, double r) {
	return T(r).CalcTrajectory(from, to);
}

template <class T>
double RrtStar<T>::LookAheadEstimate(const Node& v) {
	int id = graph_.Id(v);
	if (lmc_.find(id) != lmc_.end()) {
		return lmc_[id];
	}
	return INF;
	//return dist(v, goal_);
}

template <class T>
double RrtStar<T>::CostG(const Node& v) {
	int id = graph_.Id(v);
	if (id != -1) {
		return graph_.g_[id];
	}
	return INF;
}

template <class T>
void RrtStar<T>::SetLookAheadEstimate(const Node& v, double val) {
	int id = graph_.Id(v);
	lmc_[id] = val;
}

template <class T>
void RrtStar<T>::SetG(int v_id, double val) {
	graph_.g_[v_id] = val;
}

template <class T>
void RrtStar<T>::UpdateLMC(const Node& v, double r) {
	std::vector<int> neighbors = graph_.Neighbors_p(graph_.Id(v));

	CullNeighbors(v, r);

	double best = INF;
	int best_node = -1;
	T best_traj;
	for (auto u : neighbors) {
		std::unique_ptr<T> traj = ComputeTrajectory(v, graph_.GetNode(u), min_radius_);
		// Check if obstacles check is needed.
		if (traj->IsValid() && best > traj->Distance() + LookAheadEstimate(graph_.GetNode(u))
			&& !obstacles_.HasObstacle(traj->CalcTrajectoryPoints(traj_n_points))) {
			best = traj->Distance() + LookAheadEstimate(graph_.GetNode(u));
			best_node = u;
			best_traj = *traj.get();
		}
	}

	if (best_node != -1) {
		SetLookAheadEstimate(v, best);
		graph_.SetParent(graph_.GetNode(best_node), v, best_traj);
	}
}

template <class T>
std::optional<Node> RrtStar<T>::FindParent(const Node& v, const std::set<Node>& nodes, double r) {
	std::optional<Node> parent = std::nullopt;
	for (const auto& node : nodes) {
		std::unique_ptr<T> traj = ComputeTrajectory(v, node, min_radius_);
		
		//DrawTrajectory(v, node, traj);
		
		if (traj->IsValid() && traj->Distance() <= r &&
			LookAheadEstimate(v) > traj->Distance() + LookAheadEstimate(node)) {
			
			if (obstacles_.HasObstacle(traj->CalcTrajectoryPoints(traj_n_points))) {
				continue;
			}

			// Update parent 
			graph_.InsertNode(v);
			graph_.SetParent(node, v, *traj.get());
			parent = node;
			SetLookAheadEstimate(v, traj->Distance() + LookAheadEstimate(node));
		}
	}

	return parent;
}

template <class T>
void RrtStar<T>::Extend(const Node& v, double r) {
	std::set<Node> v_near = graph_.Near(v, r);

	std::optional<Node> parent = FindParent(v, v_near, r);

	if (!parent.has_value()) return;

	graph_.InsertNode(v);

	for (const auto& node : v_near) {
		int v_id = graph_.Id(v);
		int u_id = graph_.Id(node);

		std::unique_ptr<Trajectory> traj = ComputeTrajectory(v, node, min_radius_);
		
		//DrawTrajectory(v, node, traj);
		
		if (traj->IsValid() && !obstacles_.HasObstacle(traj->CalcTrajectoryPoints(traj_n_points))) {
			graph_.n_o_p_[v_id].insert(u_id);
			graph_.n_r_m_[u_id].insert(v_id);
		}

		std::unique_ptr<Trajectory> traj_inv = ComputeTrajectory(node, v, min_radius_);
		if (traj_inv->IsValid() && !obstacles_.HasObstacle(traj_inv->CalcTrajectoryPoints(traj_n_points))) {
			graph_.n_r_p_[u_id].insert(v_id);
			graph_.n_o_m_[v_id].insert(u_id);
		}
	}
}

template <class T>
void RrtStar<T>::CullNeighbors(const Node& v, double r) {
	int v_id = graph_.Id(v);
	for (const auto& u : graph_.n_r_p_[v_id]) {
		if (r < dist(v, graph_.GetNode(u)) &&
			graph_.GetParent(v_id) != u) {
			graph_.n_r_p_[v_id].erase(u);
			graph_.n_r_m_[u].erase(v_id);
		}
	}
}

template <class T>
void RrtStar<T>::RewireNeighbors(const Node& v, double r) {
	int v_id = graph_.Id(v);
	double eps = 0.1;
	if (CostG(v) - LookAheadEstimate(v) > eps) {
		CullNeighbors(v, r);
		for (int u : graph_.Neighbors_m(v_id)) if (u != graph_.GetParent(v_id)) {
			std::unique_ptr<T> traj = ComputeTrajectory(graph_.GetNode(u), v, min_radius_);
			// Check, maybe we can merge the two trajectories
			if (LookAheadEstimate(graph_.GetNode(u)) > traj->Distance() + LookAheadEstimate(v)
				&& !obstacles_.HasObstacle(traj->CalcTrajectoryPoints(traj_n_points))) {
				SetLookAheadEstimate(graph_.GetNode(u), traj->Distance() + LookAheadEstimate(v));
				graph_.SetParent(v_id, u, *traj.get());
				if (CostG(graph_.GetNode(u)) - LookAheadEstimate(graph_.GetNode(u)) > eps) {
					// verifyQueue(u)
				}
			}
		}
	}
}

template <class T>
void RrtStar<T>::ReduceInconsistency(double r) {
	while (!q_.empty() && (q_.top().first < QueueKey(v_bot_)
		|| LookAheadEstimate(v_bot_) != CostG(v_bot_)
		|| CostG(v_bot_) >= INF || IsInQueue(v_bot_))) {
		int v_id = q_.top().second;
		q_.pop();
		if (CostG(graph_.GetNode(v_id)) - LookAheadEstimate(graph_.GetNode(v_id)) > EPS) {
			UpdateLMC(graph_.GetNode(v_id), r);
			RewireNeighbors(graph_.GetNode(v_id), r);
		}
		SetG(v_id, LookAheadEstimate(graph_.GetNode(v_id)));
	}
}

template <class T>
tPrioQueueKey RrtStar<T>::QueueKey(const Node& v) {
	return std::make_pair(
		std::min(LookAheadEstimate(v), CostG(v)),
		CostG(v));
}

template <class T>
bool RrtStar<T>::IsInQueue(const Node& v) {
	return (is_in_q_.find(graph_.Id(v)) != is_in_q_.end());
}

template <class T>
void RrtStar<T>::Solve() {
	graph_.InsertNode(goal_);
	//graph_.InsertNode(start_);

	v_bot_ = start_;

	SetLookAheadEstimate(goal_, 0.0);

	int iter = 0;
	int max_iter = 1000;

	while (v_bot_ != goal_ && iter < max_iter) {
		iter++;
		double r = ShrinkingBallRadius(graph_.NNodes() + 1);

		//if (obstaclesHasChanged()) {
		//	updateObstacles();
		//}
		//if (robotIsMoving()) {
		//	v_bot = updateRobot(v_bot);
		//}
		Node v = RandomNode();
		const Node& v_nearest = graph_.Nearest(v);

		double delta = 10.0;
		if (dist(v, v_nearest) > delta) {
			v = Saturate(v, v_nearest, delta);
		}

		if (!obstacles_.IsObstacle(v.GetPoint())) {
			Extend(v, r);

			if (graph_.ContainsNode(v)) {
				RewireNeighbors(v, r);
				ReduceInconsistency(r);
			}
		}
	}

	DrawGraph();
}

template <class T>
Node RrtStar<T>::RandomNode() {
	double x;
	if (start_.x < goal_.x) {
		x = fRand(start_.x, goal_.x);
	}
	else {
		x = fRand(goal_.x, start_.x);
	}

	double y;
	if (start_.y < goal_.y) {
		y = fRand(start_.y, goal_.y);
	}
	else {
		y = fRand(goal_.y, start_.y);
	}

	double theta = fRand(0.0, 2*PI);

	return Node(x, y, theta);
}

template <class T>
Node RrtStar<T>::Saturate(const Node& v, const Node& v_nearest, double delta) {
	double ang = atan2(v.y - v_nearest.y, v.x - v_nearest.x);
	Node res = v;
	res.x = v_nearest.x + delta * cos(ang);
	res.y = v_nearest.y + delta * sin(ang);
	return res;
}

template <class T>
double RrtStar<T>::ShrinkingBallRadius(int n) {
	int d = 3;
	double lambda = 15.0;
	return lambda * pow(log(n) / n, 1.0 / d);
}

template <class T>
void RrtStar<T>::DrawTrajectory(const Node& a, const Node& b, std::unique_ptr<Trajectory>& traj) {
	if (!traj->IsValid()) {
		return;
	}
	
	auto points = traj->CalcTrajectoryPoints(traj_n_points);

	PngImage im = CreateWhiteImage(600, 600);
	im.SetDimensions(-5.0, 20.0, -5.0, 20.0);

	im.FillPoints(points, 1, 0, 0, 0);

	im.DrawLine(a.GetPoint(), a.theta, 35, 255, 0, 0);

	im.DrawLine(b.GetPoint(), b.theta, 35, 0, 255, 0);

	im.writeImage(_strdup("dubin_test_rsr.png"), _strdup(""));

}

template <class T>
void RrtStar<T>::DrawGraph() {
	std::vector<T> trajectories;
	for (const auto& v : graph_.Nodes()) {
		if (graph_.HasParent(v)) {
			const T& traj = graph_.GetParentTrajectory(v);
			trajectories.push_back(traj);
		}
	}

	int size = trajectories.empty() ? 0 : std::min(1000, (int) (1000000 / trajectories.size()));

	std::vector<std::vector<Point>> point_trajectories;
	double x_min = start_.x;
	double x_max = start_.x;
	double y_min = start_.y;
	double y_max = start_.y;
	for (const T& traj : trajectories) {
		auto points = traj.CalcTrajectoryPoints(size);
		point_trajectories.push_back(points);
		bool obs = obstacles_.HasObstacle(points);
		for (const auto& p : points) {
			x_min = std::min(x_min, p.x);
			x_max = std::max(x_max, p.x);
			y_min = std::min(y_min, p.y);
			y_max = std::max(y_max, p.y);
		}
	}

	//PngImage im = CreateWhiteImage(1000, 1000, x_min, x_max, y_min, y_max);
	PngImage im = CreateWhiteImage(1000, 1000);
	im.SetDimensions(x_min, x_max, y_min, y_max);
	obstacles_.DrawObstacles(im);

	for (const auto& points : point_trajectories) {
		im.FillPoints(points, 0, 0, 255, 0);
	}

	im.DrawPoint(start_.GetPoint(), 3, 255, 0, 0);
	im.DrawPoint(goal_.GetPoint(), 3, 255, 0, 0);

	im.writeImage(_strdup("dubin_test.png"), _strdup(""));
}