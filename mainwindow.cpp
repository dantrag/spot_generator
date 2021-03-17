#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <random>
#include <chrono>

#include <QGraphicsRectItem>

const int kMinPoints = 4;
const int kMaxPoints = 8;
const double kMinCoord = 0.0;
const double kMaxCoord = 100.0;
const double kProbeRadius = 20.0;
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

void MainWindow::generateSpot() {
  auto start_time = std::chrono::steady_clock::now();

  std::uniform_int_distribution<> vertex_distribution(kMinPoints, kMaxPoints);
  std::uniform_real_distribution<> coordinate_distribution(kMinCoord, kMaxCoord);

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
  int min_coordinate = std::floor(kMinCoord - 1 * kProbeRadius);
  int max_coordinate = std::ceil(kMaxCoord + 1 * kProbeRadius);
  QPixmap pixmap(max_coordinate - min_coordinate + 1, max_coordinate - min_coordinate + 1);
  QImage image1 = pixmap.toImage();
  QImage image2 = pixmap.toImage();
  for (int x = min_coordinate; x <= max_coordinate; ++x)
    for (int y = min_coordinate; y <= max_coordinate; ++y) {
      bool include = true;
      for (auto edge : edges) {
        if (distance(Point(x, y), vertices[edge.first], vertices[edge.second]) < kProbeRadius) {
          include = false;
          break;
        }
      }
      image1.setPixelColor(x - min_coordinate, y - min_coordinate, include ? Qt::white : Qt::blue);
    }

  // Concavity smoothing algorithm - currently in progress
  /*
  std::vector<std::pair<double, double>> radii;

  min_coordinate = std::ceil(kMinCoord - kProbeRadius);
  max_coordinate = std::floor(kMaxCoord + kProbeRadius);
  for (int i = 0; i < 60; ++i)
    radii.push_back(std::make_pair(kProbeRadius * std::cos(2 * M_PI / 60 * i),
                                   kProbeRadius * std::sin(2 * M_PI / 60 * i)));
  for (int x = min_coordinate; x <= max_coordinate; ++x)
    for (int y = min_coordinate; y <= max_coordinate; ++y) {
      bool include = true;
      for (int i = 0; i < 60; ++i) {
        Point circle(x + radii[i].first, y + radii[i].second);
        if (image1.pixelColor(static_cast<int>(round(circle.x + kProbeRadius)),
                              static_cast<int>(round(circle.y + kProbeRadius))) != Qt::blue) {
          include = false;
          break;
        }
      }
      image2.setPixelColor(x - min_coordinate, y - min_coordinate, include ? Qt::red : Qt::white);
    }

  min_coordinate = std::floor(kMinCoord - 2 * kProbeRadius);
  max_coordinate = std::ceil(kMaxCoord + 2 * kProbeRadius);

  */

  auto end_generation_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> generation_time = end_generation_time - start_time;

  ui->graphics_view->scene()->clear();

  // Render the backbone graph
  /*
  ui->graphics_view->scene()->addRect(kMinCoord, kMinCoord, kMaxCoord - kMinCoord, kMaxCoord - kMinCoord, QPen(QBrush(Qt::blue), 1))->setOpacity(0.2);
  for (auto point : vertices) {
    ui->graphics_view->scene()->addEllipse(point.x - 2, point.y - 2, 4, 4);
  }
  for (auto edge : edges) {
    const auto& p1 = vertices[edge.first];
    const auto& p2 = vertices[edge.second];
    ui->graphics_view->scene()->addLine(p1.x, p1.y, p2.x, p2.y, QPen(QBrush(Qt::blue), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))->setOpacity(0.2);
  }
  */

  // Render the image
  auto pixmap_item = ui->graphics_view->scene()->addPixmap(QPixmap::fromImage(image1));
  pixmap_item->setOpacity(0.4);
  pixmap_item->moveBy(min_coordinate, min_coordinate);

  auto end_total_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> total_time = end_total_time - start_time;
  ui->time_label->setText(QString("Generation time: %1 ms    total time: %2 ms").arg(generation_time.count() * 1000, 0, 'g', 3)
                                                                                .arg(total_time.count() * 1000, 0, 'g', 3));
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent) , ui(new Ui::MainWindow) {
  generator = std::mt19937(seeder());

  ui->setupUi(this);
  ui->graphics_view->setScene(new QGraphicsScene());
  ui->graphics_view->scale(3, 3);

  connect(ui->next_button, &QPushButton::clicked, this, &MainWindow::generateSpot);
}

MainWindow::~MainWindow() {
  delete ui;
}

