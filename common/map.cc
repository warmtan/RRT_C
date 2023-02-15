#include "map.h"

void Map::Init() {
  for (int i = 0; i < dimension_x_; ++i) {
    std::vector<Node> row;
    for (int j = 0; j < dimension_y_; ++j) {
      row.push_back(Node(Point(i, j)));
    }
    nodes_.push_back(row);
  }
  cv::Mat background(dimension_x_ * kMapGridSize, dimension_y_ * kMapGridSize,
                     CV_8UC3, cv::Scalar(255, 255, 255));
  background_ = background;
}

void Map::AddObstacle(const Obstacle& obstacle) {
  for (int i = 0; i < obstacle.dimension_x_; ++i) {
    for (int j = 0; j < obstacle.dimension_y_; ++j) {
      nodes_[obstacle.upper_left_x_ + i][obstacle.upper_left_y_ + j]
          .set_is_obstacle();
    }
  }
}

void Map::DrawObstacles() {
  for (int i = 0; i < dimension_x_; ++i) {
    for (int j = 0; j < dimension_y_; ++j) {
      if (nodes_[i][j].is_obstacle()) DrawNode(nodes_[i][j]);
    }
  }
}

void Map::DrawNode(Node node, cv::Scalar color,
                   int thickness) {
  int32_t x = node.point().x, y = node.point().y;
  cv::rectangle(background_, cv::Point(x * kMapGridSize, y * kMapGridSize),
                cv::Point((x + 1) * kMapGridSize, (y + 1) * kMapGridSize),
                color, thickness);
}
