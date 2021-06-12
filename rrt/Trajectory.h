#pragma once

#include <tuple>
#include "math.h"

#include "MathUtils.h"

const double EPS = 1.0e-2;

struct Node {
	double x;
	double y;
	double theta;

	Node(double x_, double y_, double theta_) : 
		x(x_), y(y_), theta(theta_) {}

	Node() {}

	bool operator==(const Node& other) const {
		return abs(x - other.x) < EPS && 
			abs(y - other.y) < EPS &&
			abs(theta - other.theta) < EPS;
	}

	bool operator!=(const Node& other) const {
		return !(*this == other);
	}

	Point GetPoint() const {
		return { x, y };
	}

	Point OrientationVector(double phi = 0.0) const {
		return { cos(theta + phi), sin(theta + phi) };
	}

	bool operator <(const Node& rhs) const {
		return std::tie(x, y, theta) < 
			std::tie(rhs.x, rhs.y, theta);
	}
};

class Trajectory {
public:
	virtual double Distance() = 0;

	virtual bool IsValid() const = 0;

	virtual std::vector<Point> CalcTrajectoryPoints(const Node& a, const Node& b, const int points) = 0;

	virtual std::vector<Point> CalcTrajectoryPoints(const int size) const = 0;
};

double dist(const Node& a, const Node& b);