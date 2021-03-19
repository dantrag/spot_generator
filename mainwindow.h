#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 private:
  void generateSpot();
  void updateVisibility();
  std::vector<QGraphicsItem*> backbone;
  std::vector<QGraphicsPixmapItem*> complement;
  std::vector<QGraphicsPixmapItem*> result;

  Ui::MainWindow* ui;
};

#endif // MAINWINDOW_H
