#include "pch.h"

#include "PngImage.h"
#include "lib_png_utils.h"

TEST(LibPngTest, DrawRedImage) {
	int width = 500;
	int height = 300;

	PngImage image = createRed(width, height);

	int result = image.writeImage(_strdup("test.png"), _strdup("This is my test image"));
}

TEST(LibPngTest, ReadImage) {
	PngImage image = ReadPngImage(_strdup("test_obstacles.png"));

	EXPECT_EQ(image.Height(), 400);
	EXPECT_EQ(image.Width(), 500);
}
