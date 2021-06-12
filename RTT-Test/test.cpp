#include "pch.h"

#include "MathUtils.h"
#include <vector>

TEST(MathUtilsTest, TangentTwoCircles_4Tangents) {
	Circle c1 = Circle({ 0.0, 0.0 }, 1.0);
	Circle c2 = Circle({ 2.0, 2.0 }, 1.0);

	std::vector<std::optional<Line>> lines = FindTangentLines(c1, c2);

	EXPECT_EQ(lines.size(), 4);

	for (const auto& line : lines) {
		EXPECT_TRUE(line.has_value());
		EXPECT_FLOAT_EQ(Distance(c1, line.value()), c1.Radius());
		EXPECT_FLOAT_EQ(Distance(c2, line.value()), c2.Radius());
	}
}

TEST(MathUtilsTest, TangentTwoCircles_4Tangents_NoOrigin) {
	Circle c1 = Circle({ 10.0, 10.0 }, 1.0);
	Circle c2 = Circle({ 2.0, 2.0 }, 1.0);

	std::vector<std::optional<Line>> lines = FindTangentLines(c1, c2);

	EXPECT_EQ(lines.size(), 4);

	for (const auto& line : lines) {
		EXPECT_TRUE(line.has_value());
		EXPECT_FLOAT_EQ(Distance(c1, line.value()), c1.Radius());
		EXPECT_FLOAT_EQ(Distance(c2, line.value()), c2.Radius());
	}
}

TEST(MathUtilsTest, TangentTwoCircles_2Tangents) {
	Circle c1 = Circle({ 1.5, 2.0 }, 1.0);
	Circle c2 = Circle({ 2.0, 2.0 }, 1.0);

	std::vector<std::optional<Line>> lines = FindTangentLines(c1, c2);

	EXPECT_EQ(lines.size(), 4);

	int tot = 0;
	for (const auto& line : lines) {
		if (line.has_value()) {
			EXPECT_FLOAT_EQ(Distance(c1, line.value()), c1.Radius());
			EXPECT_FLOAT_EQ(Distance(c2, line.value()), c2.Radius());
			tot++;
		}
	}
	EXPECT_EQ(tot, 2);
}