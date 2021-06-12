#include "Trajectory.h"

double dist(const Node& a, const Node& b) {
	return sqrt((a.x - b.x) * (a.x - b.x) +
		(a.y - b.y) * (a.y - b.y));
}
