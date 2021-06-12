#include "DubinTrajectory.h"

#include "MathUtils.h"
#include <iostream>

DubinTrajectory::DubinTrajectory(double _r_min, const Point& l, const Point& r, const Node& a,
	const Point& pca, const Node& b, const Point& pcb, double length) :
		r_min(_r_min), l_(l), r_(r), a_(a), pca_(pca), b_(b), pcb_(pcb), length_(length) {}

std::unique_ptr<DubinTrajectory> DubinTrajectory::CalcTrajectory(const Node& a, const Node& b) {
	double best = 0.0;
	int ans = -1;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {

			Point pca = i == 0 ? CalcCenterRight(a) : CalcCenterLeft(a);
			Point pcb = j == 0 ? CalcCenterRight(b) : CalcCenterLeft(b);

			std::vector<std::optional<Line>> tangents = FindTangentLines({ pca, r_min }, { pcb, r_min });

			const auto& tangent = tangents[i * 2 + j];
			if (tangent.has_value()) {
				std::optional<Point> l = GetTangentPoint(tangent.value(), { pca, r_min });

				std::optional<Point> r = GetTangentPoint(tangent.value(), { pcb, r_min });

				if (l.has_value() && r.has_value()) {

					double dist = CalcDistance(l.value(), r.value());

					double angle_from = CalcAngle(l.value(), a.GetPoint(), a.OrientationVector(), pca);

					double angle_to = CalcAngle(r.value(), b.GetPoint(), b.OrientationVector(PI), pcb);

					double length = (angle_from + angle_to) * r_min + dist;

					if (ans == -1 || length < best) {
						best = length;
						ans = i * 2 + j;
					}
				}
			}
		}
	}

	if (ans == -1) return std::make_unique<DubinTrajectory>();

	int i = ans / 2;
	int j = ans % 2;
	Point pca = i == 0 ? CalcCenterRight(a) : CalcCenterLeft(a);
	Point pcb = j == 0 ? CalcCenterRight(b) : CalcCenterLeft(b);

	std::vector<std::optional<Line>> tangents = FindTangentLines({ pca, r_min }, { pcb, r_min });

	const auto& tangent = tangents[ans];
	std::optional<Point> l = GetTangentPoint(tangent.value(), { pca, r_min });

	std::optional<Point> r = GetTangentPoint(tangent.value(), { pcb, r_min });

	if (l.has_value() && r.has_value()) {

		double dist = CalcDistance(l.value(), r.value());

		double angle_from = CalcAngle(l.value(), a.GetPoint(), a.OrientationVector(), pca);

		double angle_to = CalcAngle(r.value(), b.GetPoint(), b.OrientationVector(PI), pcb);

		double length = (angle_from + angle_to) * r_min + dist;

		return std::make_unique<DubinTrajectory>(r_min, l.value(), r.value(), a, pca, b, pcb, length);
	}

	return std::make_unique<DubinTrajectory>();
}

std::vector<Point> DubinTrajectory::CalcTrajectoryPoints(const Node& a, const Node& b, const int size) {
	std::unique_ptr<DubinTrajectory> traj_opt = CalcTrajectory(a, b);

	return traj_opt->CalcTrajectoryPoints(size);
}

std::vector<Point> DubinTrajectory::CalcTrajectoryPoints(const int size) const {
	std::vector<Point> result;

	if (IsValid()) {
		const DubinTrajectory& traj = *this;

		double angle_from = CalcAngle(traj.l_, traj.a_.GetPoint(), traj.a_.OrientationVector(), traj.pca_);

		double angle_to = CalcAngle(traj.r_, traj.b_.GetPoint(), traj.b_.OrientationVector(PI), traj.pcb_);

		CalcArcPoints(traj.pca_, traj.l_, traj.a_.GetPoint(), traj.a_.OrientationVector(), angle_from, size / 2, result);
		CalcArcPoints(traj.pcb_, traj.r_, traj.b_.GetPoint(), traj.b_.OrientationVector(PI), angle_to, size / 2, result);

		for (int i = 0; i < size / 2; i++) {
			Point p = traj.l_ + (traj.r_ - traj.l_) * (1.0 * i / (size / 2 - 1));
			result.push_back(p);
		}
	}

	return result;
}

void DubinTrajectory::CalcArcPoints(const Point& c, const Point& from, const Point& to, 
	const Point& orientation, double angle, int size, std::vector<Point>& result) const {
	double start = atan2(from.y - c.y, from.x - c.x);
	double end = atan2(to.y - c.y, to.x - c.x);
	double cross_prod = CrossProduct(to - c, orientation);
	double sgn = cross_prod < 0.0 ? -1.0 : 1.0;
	for (int i = 0; i < size; i++) {
		double x = c.x + r_min * cos(start + sgn * angle * i / (size - 1));
		double y = c.y + r_min * sin(start + sgn * angle * i / (size - 1));
		result.push_back({ x, y });
	}
}

double DubinTrajectory::Distance() {
	return length_;
}
