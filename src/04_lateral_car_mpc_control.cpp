#include <casadi/casadi.hpp>
#include <cmath>
#include <iostream>
#include <vector>

#include "arcanain_control_tutorial/matplotlibcpp.h"
#include "rclcpp/rclcpp.hpp"

using namespace casadi;
namespace plt = matplotlibcpp;

class MPCNode : public rclcpp::Node
{
public:
  MPCNode()
  : Node("lateral_car_mpc_control")
  {
    // システムパラメータの設定
    dt = 0.1;
    nx = 2;
    nu = 1;
    N = 20;
    Q = casadi::DM::diag(casadi::DM({1.0, 1.0}));
    R = 0.01;
    P = 30 * casadi::DM::eye(nx);

    // 制約の設定
    xmin = casadi::DM({-5, -5});
    xmax = casadi::DM({5, 5});
    umin = -1;
    umax = 1;

    // システム行列の設定
    // double v = 30.0; // driving speed
    // A = casadi::DM({{1.0, 0.0}, {v * dt, 1.0}});
    // B = casadi::DM({dt, 0.5 * v * dt * dt});
    double v = 30.0; // driving speed
    A = casadi::DM(std::vector<std::vector<double>>{{1.0, 0.0}, {v * dt, 1.0}});
    B = casadi::DM(std::vector<std::vector<double>>{{dt}, {0.5 * v * dt * dt}});

    // 初期状態の設定
    xTrue = casadi::DM({0.0, 2.0});

    // 目標値
    xTarget = casadi::DM({0.0, 1.0});

    // MPCの実行タイマー
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(dt * 1000)), std::bind(&MPCNode::mpc_step, this));
  }

private:
  void mpc_step()
  {
    // MPC問題の解決
    casadi::DM uopt = solve_mpc(xTrue);
    RCLCPP_INFO_STREAM(this->get_logger(), "Optimal control: " << uopt);

    xTrue = mtimes(A, xTrue) + mtimes(B, uopt);
    x0_history.push_back(xTrue(0).scalar());
    x1_history.push_back(xTrue(1).scalar());

    // 結果の出力
    RCLCPP_INFO_STREAM(this->get_logger(), "Updated state: " << xTrue);

    // matplotlib-cppを使用したグラフの描画
    plt::clf();  // グラフをクリア
    plt::ion();  // インタラクティブモードを有効にする
    plt::plot(x0_history);
    plt::plot(x1_history);
    plt::title("State");
    plt::pause(0.5);  // グラフを更新

    plt::draw();  // グラフを更新
  }

  DM solve_mpc(DM xTrue)
  {
    // 最適化問題と最適化変数を定義
    casadi::Opti opti;
    casadi::MX x = opti.variable(nx, N + 1);
    casadi::MX u = opti.variable(nu, N);

    // 目的関数を定義
    casadi::MX cost = 0;
    for (int i = 0; i < N; ++i) {
      cost += mtimes(mtimes((x(Slice(), i) - xTarget).T(), Q), (x(Slice(), i) - xTarget)) +
        mtimes(mtimes(u(Slice(), i).T(), R), u(Slice(), i));
    }
    cost += mtimes(mtimes((x(Slice(), N) - xTarget).T(), P), (x(Slice(), N) - xTarget));
    opti.minimize(cost);

    // 制約条件を定義
    opti.subject_to(x(Slice(), 0) == xTrue);
    for (int i = 0; i < N; ++i) {
      opti.subject_to(x(Slice(), i + 1) == mtimes(A, x(Slice(), i)) + mtimes(B, u(i)));
      opti.subject_to(xmin <= x(Slice(), i));
      opti.subject_to(x(Slice(), i) <= xmax);
      opti.subject_to(umin <= u(i));
      opti.subject_to(u(i) <= umax);
    }
    opti.subject_to(xmin <= x(Slice(), N));
    opti.subject_to(x(Slice(), N) <= xmax);

    // 最適化問題を解く
    opti.solver("ipopt");
    casadi::OptiSol sol = opti.solve();

    // 結果を出力
    casadi::DM xopt = sol.value(x);
    casadi::DM uopt = sol.value(u(0));
    casadi::DM Jopt = sol.value(cost);

    RCLCPP_INFO_STREAM(this->get_logger(), "xopt: " << sol.value(x));
    RCLCPP_INFO_STREAM(this->get_logger(), "uopt: " << sol.value(u));
    RCLCPP_INFO_STREAM(this->get_logger(), "Jopt: " << sol.value(cost));

    return uopt;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  double dt;
  int nx, nu, N;
  casadi::DM A, B, Q, R, P, xTrue;
  casadi::DM xmin, xmax, umin, umax;
  casadi::DM xTarget;
  std::vector<double> x0_history, x1_history;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCNode>());
  rclcpp::shutdown();
  return 0;
}
