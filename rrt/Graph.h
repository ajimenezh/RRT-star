#pragma once

#include "Trajectory.h"
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <iterator>

const double INF = 1.0e8;

template<class T>
class Graph {
public:
	Graph(const std::vector<Node>& nodes) {
		nodes_ = nodes;
	}

	Graph() {}

	std::set<Node> Near(const Node& v, double r) {
		std::set<Node> result;
		for (const auto& it : nodes_) {
			if (dist(it, v) < r) {
				result.insert(it);
			}
		}
		return result;
	}

	const Node& Nearest(const Node& v) {
		const Node* result = nullptr;
		double best_dist = -1.0;
		for (const auto& it : nodes_) {
			if (result == nullptr || dist(it, v) < best_dist) {
				best_dist = dist(it, v);
				result = &it;
			}
		}

		return *result;
	}

	void InsertNode(const Node& v) {
		if (id_.find(v) != id_.end()) {
			return;
		}
		id_[v] = nodes_.size();
		nodes_.push_back(v);
		g_.push_back(INF);
		parent_[id_[v]] = -1;
	}

	void AddChildren(const Node& parent, const Node& v) {
		children_[id_[parent]].push_back(id_[v]);
		parent_[id_[v]] = id_[parent];
	}

	int Id(const Node& v) {
		if (id_.find(v) == id_.end()) {
			return -1;
		}
		return id_[v];
	}

	const Node& GetNode(int id) {
		return nodes_[id];
	}

	int GetParent(int id) {
		return parent_[id];
	}

	void SetParent(int parent, int child, T trajectory) {
		parent_[child] = parent;
		parent_trajectory_[child] = trajectory;
	}

	void SetParent(const Node& parent, const Node& child, T trajectory) {
		SetParent(Id(parent), Id(child), trajectory);
	}

	bool HasParent(const Node& v) {
		return Id(v) != -1 && parent_[Id(v)] != -1;
	}

	const T& GetParentTrajectory(const Node& v) {
		return parent_trajectory_[Id(v)];
	}

	std::vector<int> Neighbors_m(int id) {
		std::vector<int> s;

		std::set_union(n_o_m_[id].begin(), n_o_m_[id].end(),
			n_r_m_[id].begin(), n_r_m_[id].end(),
			std::back_inserter(s));

		return s;
	}

	std::vector<int> Neighbors_p(int id) {
		std::vector<int> s;

		std::set_union(n_o_p_[id].begin(), n_o_p_[id].end(),
			n_r_p_[id].begin(), n_r_p_[id].end(),
			std::back_inserter(s));

		return s;
	}

	bool ContainsNode(const Node& node) {
		return id_.find(node) != id_.end();
	}

	const std::vector<Node>& Nodes() {
		return nodes_;
	}

	int NNodes() {
		return nodes_.size();
	}

	std::map<int, std::set<int> > n_o_p_;
	std::map<int, std::set<int> > n_o_m_;
	std::map<int, std::set<int> > n_r_p_;
	std::map<int, std::set<int> > n_r_m_;

	std::vector<double> g_;

private:
	std::vector<Node> nodes_;
	std::map<Node, int> id_;
	std::map<int, int> parent_;
	std::map<int, T> parent_trajectory_;
	std::map<int, std::vector<int>> children_;

};

