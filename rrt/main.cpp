#include <iostream>
#include <vector>
#include <set>
#include <map>

#include "Trajectory.h"
#include "DubinTrajectory.h"
#include "RrtStar.h"
#include "PngImage.h"
#include "lib_png_utils.h"
#include "ObstaclesData.h"

#include <iostream>


int main() {

	ObstaclesData obstacles = CreateObstaclesDataFromImage(
		_strdup("test_obstacles.png"), 0.0, 10.0, 0.0, 10.0);

	Node start(0.0, 0.0, 2.0);
	Node goal(10.0, 10.0, 2.0);
	RrtStar<DubinTrajectory> solver(start, goal, obstacles, 1.0);

	std::cout << "hello world" << std::endl;

	solver.Solve();

	return 0;
}