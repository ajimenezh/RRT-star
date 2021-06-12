#pragma once
#include "Trajectory.h"
#include "MathUtils.h"

#include <memory>


const double PI = acos(-1.0);

class DubinTrajectory : public Trajectory {
public:
	DubinTrajectory() {
		valid_ = false;
	}

	DubinTrajectory(double _r_min) : r_min(_r_min) {
	}

	DubinTrajectory(double _r_min, const Point& l, const Point& r, const Node& a,
		const Point& pca, const Node& b, const Point& pcb, double length);

	std::vector<Point> CalcTrajectoryPoints(const Node& a, const Node& b, const int points = 1000);

	std::unique_ptr<DubinTrajectory> CalcTrajectory(const Node& a, const Node& b);

	std::vector<Point> CalcTrajectoryPoints(const int size = 1000) const;

	Point CalcCenterRight(const Node& p) {
		return { p.x + r_min * cos(p.theta - PI / 2.0),
			p.y + r_min * sin(p.theta - PI / 2.0) };
	}

	Point CalcCenterLeft(const Node& p) {
		return { p.x + r_min * cos(p.theta + PI / 2.0),
			p.y + r_min * sin(p.theta + PI / 2.0) };
	}

	double Distance();

	bool IsValid() const {
		return valid_;
	}

private:
	void CalcArcPoints(const Point& c, const Point& from, const Point& to, const Point& orientation, double angle, int size, std::vector<Point>& result) const;
private:
	double r_min;

	Point l_;
	Point r_; 
	Node a_;
	Point pca_; 
	Node b_; 
	Point pcb_;
	double length_;
	bool valid_ = true;
};

