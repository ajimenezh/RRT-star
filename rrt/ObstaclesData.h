#pragma once

#include <vector>

#include "MathUtils.h"
#include "PngImage.h"

class ObstaclesData {
public:
	ObstaclesData(int height, int width, double x_min, double x_max, double y_min, double y_max);

	void SetObstacle(int x, int y, bool val);

	bool IsObstacle(const Point& p) const;

	bool HasObstacle(const std::vector<Point>& v) const;

	PngImage DrawObstacles(PngImage& im);

	float GetColor(bool val);

private:
	std::vector<std::vector<bool> > data_;

	int height_;
	int width_;
	double x_min_;
	double x_max_;
	double y_min_;
	double y_max_;
};

ObstaclesData CreateObstaclesDataFromImage(const char* filename,
	double x_min, double x_max, double y_min, double y_max);