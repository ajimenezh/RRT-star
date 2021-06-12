#include "pch.h"

#include "PngImage.h"
#include "lib_png_utils.h"
#include "DubinTrajectory.h"

//TEST(DubinTrajectoryTest, RSR) {
//	DubinTrajectory traj(1.0);
//	Node n1 = { 0, 0, 1.0 };
//	Node n2 = { 2, 2, 2.6 };
//	auto points = traj.CalcTrajectoryPoints(n1, n2, 1000);
//
//	PngImage im = CreateWhiteImage(600, 600, -2.0, 5.0, -3.0, 5.0);
//
//	im.FillPoints(points, 1, 0, 0, 0);
//
//	im.DrawLine(n1.GetPoint(), n1.theta, 35, 255, 0, 0);
//
//	im.DrawLine(n2.GetPoint(), n2.theta, 35, 0, 255, 0);
//
//	im.writeImage(_strdup("dubin_test_rsr.png"), _strdup(""));
//}