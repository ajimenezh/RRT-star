#pragma once

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <png.h>
#include <string.h>
#include <vector>

#include "MathUtils.h"

class PngImage {
public:
	PngImage(int width, int height) : 
		width_(width), height_(height) {
		buffer.resize(width_ * height_);
	}

	PngImage() {}

	PngImage(int width, int height,
		double xl, double xr, double yl, double yr) : 
		width_(width), height_(height), xl_(xl), xr_(xr), yl_(yl), yr_(yr) {
		buffer.resize(width_ * height_);
	}

	~PngImage() {
	}

	void SetDimensions(double xl, double xr, double yl, double yr) {
		xl_ = xl;
		xr_ = xr;
		yl_ = yl; 
		yr_ = yr;
	}

	void set(int x, int y, float val) {
		buffer[y * width_ + x] = val;
	}

	void set(int x, int y, int r, int g, int b) {
		buffer[y * width_ + x] = b * 65536 + g * 256 + r;
	}

	void scale(int minVal, float maxVal) {
		// Scale buffer values between 0 and 1
		int count = width_ * height_;
		while (count) {
			count--;
			buffer[count] = (buffer[count] - minVal) / (maxVal - minVal);
		}
	}

	float get(int x, int y) const {
		return buffer[y * width_ + x];
	}

	int writeImage(char* filename, char* title);

	void Fill(int r, int g, int b) {
		for (int i = 0; i < width_; i++) {
			for (int j = 0; j < height_; j++) {
				set(i, j, r, g, b);
			}
		}
	}

	void FillPoints(const std::vector<Point>& points, int size, int r, int g, int b);
	
	void DrawLine(const Point& points, double m, int size, int r, int g, int b);

	void DrawPoint(const Point& point, int size, int r, int g, int b);

	int GetX(double x);

	int GetY(double y);

	int ReadPngInit(FILE* infile);

	int Height() {
		return height_;
	}

	int Width() {
		return width_;
	}

	float GetData(int x, int y) {
		return data_[x][y];
	}

	double XMin() const {
		return xl_;
	}

	double XMax() const {
		return xr_;
	}

	double YMin() const {
		return yl_;
	}

	double YMax() const {
		return yr_;
	}

private:
	//  B * 65536 + G * 256 + R
	void setRGB(png_byte* ptr, float val)
	{
		int v = (int)val;
		int b = v / 65536;
		v = v % 65536;
		int g = v / 256;
		int r = v % 256;

		ptr[0] = r; ptr[1] = g; ptr[2] = b;
	}

	void finalise(FILE* fp, png_structp png_ptr, png_infop info_ptr, png_bytep row);

	int width_;
	int height_;

	std::vector<float> buffer;

	double xl_;
	double xr_;
	double yl_;
	double yr_;

	std::vector<std::vector<float> > data_;
};

FILE* ReadFile(const char* filename);

PngImage ReadPngImage(const char* filename);

bool IsWhite(float val);