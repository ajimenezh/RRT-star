#include "ObstaclesData.h"
#include "PngImage.h"
#include "lib_png_utils.h"

const double EPS = 1.0e-6;

ObstaclesData::ObstaclesData(int height, int width, double x_min, double x_max, double y_min, double y_max) :
	height_(height), width_(width), x_min_(x_min), x_max_(x_max), 
	y_min_(y_min), y_max_(y_max) {
	data_.resize(height);
	for (int i = 0; i < height; i++) data_[i].resize(width_);
}

void ObstaclesData::SetObstacle(int x, int y, bool val) {
	data_[x][y] = val;
}

bool ObstaclesData::IsObstacle(const Point& p) const {
	if (p.x < x_min_ || p.x > x_max_) return false;
	if (p.y < y_min_ || p.y > y_max_) return false;

	int yy = height_ - (int)((p.y - y_min_) / (y_max_ - y_min_) * (height_ - 1)) - 1;
	int xx = (int)((p.x - x_min_) / (x_max_ - x_min_) * (width_ - 1));

	return data_[xx][yy];
}

bool ObstaclesData::HasObstacle(const std::vector<Point>& v) const {
	for (const auto& p : v) {
		if (IsObstacle(p)) {
			return true;
		}
	}
	return false;
}

PngImage ObstaclesData::DrawObstacles(PngImage& im) {
	int new_width = (x_max_ - x_min_) / (im.XMax() - im.XMin()) * im.Height();
	int new_height = (y_max_ - y_min_) / (im.YMax() - im.YMin()) * im.Width();

	double x_res = 1.0 * height_ / new_height;
	double y_res = 1.0 * width_ / new_width;

	int delta_x = (x_min_ - im.XMin()) / (im.XMax() - im.XMin()) * im.Width();
	int delta_y = (im.YMax() - y_max_) / (im.YMax() - im.YMin()) * im.Height();

	std::vector < std::vector<double > > partial_sum(new_width, std::vector< double >(height_));

	for (int x_idx = 0; x_idx < height_; x_idx++) {
		double cur_y = 0;
		int cur_y_int = 0;
		int j = 0;
		while (cur_y <= width_ - 1) {
			double sum = 0;
			double dif = std::min(cur_y_int + 1 - cur_y, y_res);
			sum += dif * data_[cur_y_int][x_idx];
			if (dif >= cur_y_int + 1 - cur_y) {
				cur_y_int++;
				cur_y = cur_y_int;
			}
			else {
				cur_y += dif;
			}

			int n = (int)(y_res - dif);
			for (int i = 0; i < n; i++) if (cur_y_int + i < width_) {
				sum += data_[cur_y_int + i][x_idx];
			}
			cur_y_int += n;
			cur_y += n;
			if (dif < y_res && cur_y_int < width_) {
				sum += (y_res - n - dif) * data_[cur_y_int][x_idx];
				cur_y += (y_res - n - dif);
			}
			//out.set(j++, i, GetColor((sum / y_res) > 0.5));
			partial_sum[j++][x_idx] += sum;
		}
	}

	for (int y_idx = 0; y_idx < new_width; y_idx++) {
		double cur_x = 0;
		int cur_x_int = 0;
		int i = 0;
		while (cur_x <= height_ - 1) {
			double sum = 0;
			double dif = std::min(cur_x_int + 1 - cur_x, x_res);
			sum += dif * partial_sum[y_idx][cur_x_int];
			if (dif >= cur_x_int + 1 - cur_x) {
				cur_x_int++;
				cur_x = cur_x_int;
			}
			else {
				cur_x += dif;
			}

			int n = (int)(y_res - dif);
			for (int i = 0; i < n; i++) if (cur_x_int + i < height_) {
				sum += partial_sum[y_idx][cur_x_int + i];
			}
			cur_x_int += n;
			cur_x += n;
			if (dif < x_res && cur_x_int < height_) {
				sum += (x_res - n - dif) * partial_sum[y_idx][cur_x_int];
				cur_x += (x_res - n - dif);
			}
			im.set(delta_x + y_idx, delta_y + i++, GetColor((sum / (y_res*x_res)) > 0.5));
		}
	}

	//for (int i = 0; i < height_; i++) {
	//	for (int j = 0; j < width_; j++) {
	//		out.set(j, i, GetColor(data_[i][j]));
	//	}
	//}

	int result = im.writeImage(_strdup("test.png"), _strdup("This is my test image"));
	return im;
}

float ObstaclesData::GetColor(bool val) {
	return val ? 0 : (1 << 24) - 1;
}
 
ObstaclesData CreateObstaclesDataFromImage(const char* filename,
	double x_min, double x_max, double y_min, double y_max) {
	PngImage image = ReadPngImage(filename);

	ObstaclesData obstacles(image.Height(), image.Width(), x_min, x_max, y_min, y_max);
	for (int i = 0; i < image.Height(); i++) {
		for (int j = 0; j < image.Width(); j++) {
			obstacles.SetObstacle(j, i, !IsWhite(image.GetData(i, j)));
		}
	}

	return obstacles;

}
