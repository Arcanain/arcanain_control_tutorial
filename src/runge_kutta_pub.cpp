// ROS2のコア機能と標準メッセージ型をインクルード
#include "rclcpp/rclcpp.hpp"        // ROS2の基本的なノード機能を提供
#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include <fstream>
#include "arcanain_control_tutorial/matplotlibcpp.h"

using namespace Eigen;
using namespace std;
// 時間リテラル（例：500ms）を使うための名前空間を使用
using namespace std::chrono_literals;
namespace plt = matplotlibcpp;

// RungeKuttaPublisherクラスはrclcpp::Nodeクラスを継承している
class RungeKuttaPublisher : public rclcpp::Node
{
public:
  // コンストラクタ
  RungeKuttaPublisher()
  : Node("RungeKutta") // Nodeクラスのコンストラクタを呼び出し、ノード名を指定
  {
    InitialResponse();
    // 500ミリ秒ごとにtimer_callback関数を呼び出すタイマーを作成
    timer_ = this->create_wall_timer(10ms, std::bind(&RungeKuttaPublisher::timer_callback, this));
    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();
  }

private:
  
  // タイマーのコールバック関数
  void timer_callback()
  {
    current_time = this->get_clock()->now();
    double dt = (current_time - last_time).seconds();
    tt += dt;
    get_RungeKutta(dt);
    Y = C*X;
    ofs << tt << "," << Y(0, 0) << endl;
    store_data(tt, Y(0, 0));
    graph_output();
    RCLCPP_INFO(this->get_logger(), "%lf", tt);
    RCLCPP_INFO(this->get_logger(), "%lf", Y(0,0));
    last_time = current_time;
    if (tt > 10.0){
      rclcpp::shutdown();
    }
  }
  
  void get_RungeKutta(double dt) {
    MatrixXd k1 = A*X + B*u;
    MatrixXd k2 = A*(X + 0.5*k1*dt) + B*u;
    MatrixXd k3 = A*(X + 0.5*k2*dt) + B*u;
    MatrixXd k4 = A*(X + k3*dt) + B*u;
    MatrixXd k = (k1 + 2.0*k2 + 2.0*k3 + k4)*dt / 6.0;
    X = X + k;
  }

  void InitialResponse(){
    double  k = 1.0;
    double  m = 0.1;
    double  c = 0.1;
    A = Eigen::MatrixXd(2, 2);
    A(0, 0) = 0;
    A(0, 1) = 1;
    A(1, 0) = -k / m;
    A(1, 1) = -c / m;
    B = Eigen::MatrixXd(2, 1);
    B(0, 0) = 1;
    B(1, 0) = 1 / m;

    C = Eigen::MatrixXd(1, 2);
    C(0, 0) = 1;
    C(0, 1) = 0;

    D = Eigen::MatrixXd(1, 1);
    D(0, 0) = 0;

    X = Eigen::MatrixXd(2, 1);
    X(0, 0) = 10;
    X(1, 0) = 0;

    dX = Eigen::MatrixXd(2, 1);
    dX(0, 0) = 0;
    dX(1, 0) = 0;

    u = Eigen::MatrixXd(1, 1);
    u(0, 0) = 0;

    Y = Eigen::MatrixXd(1, 1);
    Y(0, 0) = 0;

    ofs.open("runge_kutta_results.csv");
    ofs << "time," << "y" << endl;
  }

  void store_data(double time, double value){
    // 時間と値を保存
    data_time.push_back(time);
    data_value.push_back(value);
  }

  void graph_output(){
    plt::clf();  // グラフをクリア
    plt::ion();  // インタラクティブモードを有効にする
    plt::plot(data_time, data_value);
    plt::pause(0.1);  // グラフを更新
    plt::draw();  // グラフを更新
  }

  double tt = 0.0;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd D;
  Eigen::MatrixXd X;
  Eigen::MatrixXd dX;
  Eigen::MatrixXd u;
  Eigen::MatrixXd Y;
  std::vector<double> data_time;
  std::vector<double> data_value;
  rclcpp::TimerBase::SharedPtr timer_;  // タイマーを保持するためのスマートポインタ
  rclcpp::Time current_time, last_time;
  ofstream ofs;
};

// main関数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // ROS2システムの初期化
  // MinimalPublisherノードのインスタンスを作成し、スピンすることでコールバック関数を実行可能にする
  rclcpp::spin(std::make_shared<RungeKuttaPublisher>());
  rclcpp::shutdown();  // ROS2システムのシャットダウン
  return 0;            // プログラム終了
}