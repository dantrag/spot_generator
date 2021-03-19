#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <random>
#include <chrono>
#include <unordered_map>

#include <QGraphicsRectItem>

const int kMinPoints = 4;
const int kMaxPoints = 8;
const double kMinCoord = 0.0;
const double kMaxCoord = 100.0;
const double kProbeRadius = 10.0;
std::random_device seeder;
std::mt19937 generator;

struct Point {
  Point() {}
  Point(double xcoord, double ycoord) : x(xcoord), y(ycoord) {}
  Point(const Point& copy) : Point(copy.x, copy.y) {}

  double x = 0.0;
  double y = 0.0;
};

inline double distance_sqr(const Point& p1, const Point& p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

inline double distance(const Point& p1, const Point& p2) {
  return std::sqrt(distance_sqr(p1, p2));
}

double distance(const Point& p, const Point& p1, const Point& p2) {
  auto t = std::max(0.0, std::min(1.0, ((p.x - p1.x) * (p2.x - p1.x) + (p.y - p1.y) * (p2.y - p1.y)) /
                                       distance_sqr(p1, p2)));
  Point projection(p1.x + t * (p2.x - p1.x),
                   p1.y + t * (p2.y - p1.y));
  return distance(p, projection);
}

// ==========================================
// Supporting methods for DSU data structure

int dsu_get(std::vector<int>& dsu, int v) {
  return (v == dsu[v]) ? v : dsu[v] = dsu_get(dsu, dsu[v]);
}

void dsu_unite(std::vector<int>& dsu, int a, int b) {
  a = dsu_get(dsu, a);
  b = dsu_get(dsu, b);
  if (std::rand() & 1) std::swap(a, b);
  if (a != b) dsu[a] = b;
}
// ==========================================

int image_dfs(const QImage& image, std::vector<std::vector<int>>& used_mask,
              int x, int y, int component_number) {
  int count = 1;
  used_mask[x][y] = component_number;
  std::vector<std::pair<int, int>> moves = {{0, -1}, {0, 1}, {1, 0}, {-1, 0}};
  for (auto move : moves) {
    if (image.valid(x + move.first, y + move.second) &&
        image.pixelColor(x + move.first, y + move.second) == image.pixelColor(x, y) &&
        !used_mask[x + move.first][y + move.second]) {
      count += image_dfs(image, used_mask, x + move.first, y + move.second, component_number);
    }
  }
  return count;
}

void removeScraps(QImage& image, QColor transparent_color) {
  int biggest_component = 0;
  int biggest_component_size = 0;
  int component_count = 0;
  std::vector<std::vector<int>> processed_mask(image.width(), std::vector<int>(image.height(), 0));

  for (int x = 0; x < image.width(); ++x)
    for (int y = 0; y < image.height(); ++y) {
      if (image.pixelColor(x, y) != transparent_color &&
          !processed_mask[x][y]) {
        auto connected_component_size = image_dfs(image, processed_mask, x, y, ++component_count);
        if (connected_component_size > biggest_component_size) {
          biggest_component_size = connected_component_size;
          biggest_component = component_count;
        }
      }
    }
  for (int x = 0; x < image.width(); ++x)
    for (int y = 0; y < image.height(); ++y) {
      if (image.pixelColor(x, y) != transparent_color) {
        if (processed_mask[x][y] != biggest_component) image.setPixelColor(x, y, transparent_color);
      }
    }
}

void MainWindow::generateSpot() {
  auto start_time = std::chrono::steady_clock::now();

  std::uniform_int_distribution<> vertex_distribution(kMinPoints, kMaxPoints);
  std::uniform_real_distribution<> coordinate_distribution(kMinCoord + 2 * kProbeRadius, kMaxCoord - 2 * kProbeRadius);

  // Generate graph vertices with certain separation between them
  std::vector<Point> vertices;
  int n = vertex_distribution(generator);
  double safe_distance = kMaxCoord / 2 / std::sqrt(n - 1);
  while (n--) {
    int iterations_left = 1000;
    bool added_point = false;
    while (iterations_left--) {
      auto new_point = Point(coordinate_distribution(generator),
                             coordinate_distribution(generator));
      bool collision_found = false;
      for (auto point : vertices) {
        if (distance(point, new_point) < safe_distance) {
          collision_found = true;
          break;
        }
      }
      if (!collision_found) {
        vertices.push_back(new_point);
        added_point = true;
        break;
      }
    }
    if (!added_point) break;
  }

  n = static_cast<int>(vertices.size());
  if (n < 2) return;

  // Generate edges, sort them according to length, and prepare to build MST
  std::map<double, std::pair<int, int>> sorted_edges;
  for (int v1 = 0; v1 < n - 1; ++v1) {
    for (int v2 = v1 + 1; v2 < n; ++v2) {
      sorted_edges[distance(vertices[v1], vertices[v2])] = std::make_pair(v1, v2);
    }
  }
  std::uniform_int_distribution<> edge_distribution(n - 1, n * (n - 1) / 2);
  int m = edge_distribution(generator);
  std::vector<std::pair<int, int>> edges, not_used_edges;
  std::vector<std::pair<int, int>> edge_candidates;
  for (auto edge : sorted_edges) edge_candidates.push_back(edge.second);

  std::vector<int> vertex_dsu(n);
  for (int i = 0; i < n; ++i) vertex_dsu[i] = i;

  // Build MST with Kruskal's algo
  for (size_t i = 0; i < edge_candidates.size() && static_cast<int>(edges.size()) < n - 1; ++i) {
    int v1 = edge_candidates[i].first;
    int v2 = edge_candidates[i].second;
    if (dsu_get(vertex_dsu, v1) != dsu_get(vertex_dsu, v2)) {
      dsu_unite(vertex_dsu, v1, v2);
      edges.push_back(edge_candidates[i]);
    } else not_used_edges.push_back(edge_candidates[i]);
  }

  // Add some number of extra edges to have previously selected m in total
  std::reverse(not_used_edges.begin(), not_used_edges.end());

  while (static_cast<int>(edges.size()) < m && !not_used_edges.empty()) {
    edges.push_back(not_used_edges.back());
    not_used_edges.pop_back();
  }

  // Build a kProbeRadius-separated complement -- final image
  int min_coordinate = std::floor(kMinCoord);
  int max_coordinate = std::ceil(kMaxCoord);
  QPixmap pixmap(max_coordinate - min_coordinate + 1, max_coordinate - min_coordinate + 1);
  QImage image1 = pixmap.toImage();
  QImage image2 = pixmap.toImage();
  for (int x = min_coordinate; x <= max_coordinate; ++x)
    for (int y = min_coordinate; y <= max_coordinate; ++y) {
      bool include = true;
      for (auto edge : edges) {
        if (distance(Point(x, y), vertices[edge.first], vertices[edge.second]) < 2 * kProbeRadius) {
          include = false;
          break;
        }
      }
      image1.setPixelColor(x - min_coordinate, y - min_coordinate, include ? Qt::blue : Qt::white);
      image2.setPixelColor(x - min_coordinate, y - min_coordinate, Qt::blue);
    }
  removeScraps(image1, Qt::white);

  // Concavity smoothing algorithm - currently in progress
  std::vector<std::pair<double, double>> radius_vectors;

  for (int i = 0; i < 60; ++i)
    radius_vectors.push_back(std::make_pair(kProbeRadius * std::cos(2 * M_PI / 60 * i),
                                            kProbeRadius * std::sin(2 * M_PI / 60 * i)));
  for (int x = min_coordinate; x <= max_coordinate; ++x)
    for (int y = min_coordinate; y <= max_coordinate; ++y) {
      bool include = false;
      for (int i = 0; i < 60; ++i) {
        Point circle(x + radius_vectors[i].first,
                     y + radius_vectors[i].second);
        if (image1.valid(static_cast<int>(round(circle.x)),
                         static_cast<int>(round(circle.y))) &&
            image1.pixelColor(static_cast<int>(round(circle.x)),
                              static_cast<int>(round(circle.y))) == Qt::blue) {
          include = true;
          break;
        }
      }
      image2.setPixelColor(x - min_coordinate,
                           y - min_coordinate, include ? Qt::white : Qt::red);
    }

  auto end_generation_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> generation_time = end_generation_time - start_time;

  ui->graphics_view->scene()->clear();
  backbone.clear();
  result.clear();
  complement.clear();

  // Render the backbone graph

  //ui->graphics_view->scene()->addRect(kMinCoord, kMinCoord, kMaxCoord - kMinCoord, kMaxCoord - kMinCoord, QPen(QBrush(Qt::blue), 1))->setOpacity(0.2);
  for (auto point : vertices) {
    auto ellipse = ui->graphics_view->scene()->addEllipse(point.x - 2, point.y - 2, 4, 4);
    ellipse->setVisible(ui->backbone_checkbox->isChecked());
    backbone.push_back(ellipse);
  }
  for (auto edge : edges) {
    const auto& p1 = vertices[edge.first];
    const auto& p2 = vertices[edge.second];
    auto line = ui->graphics_view->scene()->addLine(p1.x, p1.y, p2.x, p2.y,
                                                    QPen(QBrush(Qt::blue), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    line->setOpacity(0.2);
    line->setVisible(ui->backbone_checkbox->isChecked());
    backbone.push_back(line);
  }

  // Render the image
  auto pixmap_item1 = ui->graphics_view->scene()->addPixmap(QPixmap::fromImage(image1));
  pixmap_item1->setOpacity(0.2);
  pixmap_item1->moveBy(min_coordinate, min_coordinate);
  pixmap_item1->setVisible(ui->complement_checkbox->isChecked());
  complement.push_back(pixmap_item1);
  auto pixmap_item2 = ui->graphics_view->scene()->addPixmap(QPixmap::fromImage(image2));
  pixmap_item2->setOpacity(0.2);
  pixmap_item2->moveBy(min_coordinate, min_coordinate);
  result.push_back(pixmap_item2);

  auto end_total_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> total_time = end_total_time - start_time;
  ui->time_label->setText(QString("Generation time: %1 ms    total time: %2 ms").arg(generation_time.count() * 1000, 0, 'g', 3)
                                                                                .arg(total_time.count() * 1000, 0, 'g', 3));
}

void MainWindow::updateVisibility() {
  for (auto& item : backbone) {
    item->setVisible(ui->backbone_checkbox->isChecked());
  }
  for (auto& item : complement) {
    item->setVisible(ui->complement_checkbox->isChecked());
  }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent) , ui(new Ui::MainWindow) {
  generator = std::mt19937(seeder());

  ui->setupUi(this);
  ui->graphics_view->setScene(new QGraphicsScene());
  ui->graphics_view->scale(3, 3);

  connect(ui->next_button, &QPushButton::clicked, this, &MainWindow::generateSpot);
  connect(ui->backbone_checkbox, &QCheckBox::clicked, this, &MainWindow::updateVisibility);
  connect(ui->complement_checkbox, &QCheckBox::clicked, this, &MainWindow::updateVisibility);
}

MainWindow::~MainWindow() {
  delete ui;
}

