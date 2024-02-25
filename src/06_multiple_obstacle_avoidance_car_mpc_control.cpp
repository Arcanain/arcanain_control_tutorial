#include <casadi/casadi.hpp>
#include <cmath>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace casadi;

class MPCNode : public rclcpp::Node
{
public:
  MPCNode()
  : Node("multiple_obstacle_avoidance_car_mpc_control")
  {
    // システムパラメータの設定
    dt = 0.1;
    nx = 6;
    nu = 2;
    N = 10;
    P = 200 * casadi::DM::eye(nx);
    Q = casadi::DM::diag(casadi::DM({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
    R = casadi::DM::diag(casadi::DM({0.1, 0.1}));
    p = 200;

    // 制約の設定
    umin = casadi::DM({-1, -5});
    umax = casadi::DM({1, 5});

    // システム行列の設定
    double M = 1500;
    double I = 2500;
    double lf = 1.1;
    double lr = 1.6;
    double Kf = 55000;
    double Kr = 60000;
    double V = 20;

    double A53 = 2 * (Kf * lf + Kr * lr) / I;
    double A55 = -2 * (Kf * lf + Kr * lr) / (I * V);
    double A56 = -2 * (Kf * lf * lf + Kr * lr * lr) / I;
    double A63 = 2 * (Kf + Kr) / M;
    double A65 = -2 * (Kf + Kr) / (M * V);
    double A66 = -2 * (Kf * lf + Kr * lr) / (M * V);

    A = casadi::DM::eye(nx) + casadi::DM(
      {{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, A53, 0.0, A55, A56},
        {0.0, 0.0, A63, 0.0, A65, A66}}) * dt;

    double B51 = 2 * Kf / M;
    double B61 = 2 * Kf * lf / I;
    B = casadi::DM(
      {{0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 1.0},
        {B51, 0.0},
        {B61, 0.0}}) * dt;

    // 初期状態の設定
    xTrue = casadi::DM({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // 目標値
    xTarget = casadi::DM({6.0, 3.0, 0.0, 0.0, 0.0, 0.0});

    // 障害物の設定
    std::vector<casadi::DM> obs_pos_list = {{3.0, 0.0}, {4.0, 1.0}, {4.0, 2.0}};
    vehicle_diameter = 0.5;
    obs_diameter = 0.5;
    obs_r = vehicle_diameter + obs_diameter;

    // 実経路のPublish
    path_pub = this->create_publisher<nav_msgs::msg::Path>("multiple_obstacle_car_path", 50);
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // MPCの実行タイマー
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(dt * 1000)), std::bind(&MPCNode::mpc_step, this));

    // 静的な変換を送信するタイマー
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    send_static_transform();

    // 時刻の初期化
    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();

    // 座標変換とパス出力
    publish_transform_and_path(xTrue);
  }

private:
  void mpc_step()
  {
    // MPC制御の解
    casadi::DM uopt = solve_mpc(xTrue);

    // 状態の更新
    xTrue = mtimes(A, xTrue) + mtimes(B, uopt);

    // 座標変換とパス出力
    publish_transform_and_path(xTrue);

    // 結果の出力
    RCLCPP_INFO_STREAM(this->get_logger(), "Optimal control: " << uopt);
    RCLCPP_INFO_STREAM(this->get_logger(), "Updated state: " << xTrue);
  }

  DM solve_mpc(DM xTrue)
  {
    // 最適化問題と最適化変数を定義
    casadi::Opti opti;
    casadi::MX x = opti.variable(nx, N + 1);
    casadi::MX u = opti.variable(nu, N);
    // スラック変数の定義（障害物の数 × 時間ステップ数）
    int num_obstacles = obs_pos_list.size();
    casadi::MX delta = opti.variable(num_obstacles, N);

    // 目的関数を定義
    casadi::MX cost = 0;
    for (int i = 0; i < N; ++i) {
      cost += mtimes(mtimes((x(Slice(), i) - xTarget).T(), Q), (x(Slice(), i) - xTarget)) +
        mtimes(mtimes(u(Slice(), i).T(), R), u(Slice(), i));
    }
    cost += mtimes(mtimes((x(Slice(), N) - xTarget).T(), P), (x(Slice(), N) - xTarget));

    // ダイナミクス制約条件を定義
    opti.subject_to(x(Slice(), 0) == xTrue);
    for (int i = 0; i < N; ++i) {
      opti.subject_to(x(Slice(), i + 1) == mtimes(A, x(Slice(), i)) + mtimes(B, u(Slice(), i)));
      opti.subject_to(umin <= u(i));
      opti.subject_to(u(i) <= umax);
    }

    // 障害物制約条件を定義
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j < num_obstacles; ++j) {
        const auto & obs_pos = obs_pos_list[j];
        casadi::MX distance = mtimes(
          (x(Slice(0, 2), i) - obs_pos).T(), (x(
            Slice(
              0,
              2),
            i) - obs_pos)) - obs_r * obs_r;
        opti.subject_to(distance >= delta(j, i));      // 各障害物に対するスラック変数
        cost += delta(j, i) * p * delta(j, i);         // スラック変数に対するペナルティ項
      }
    }

    // 最適化問題を解く
    opti.minimize(cost);
    opti.solver("ipopt");
    casadi::OptiSol sol = opti.solve();

    // 結果を出力
    casadi::DM xopt = sol.value(x);
    casadi::DM uopt = sol.value(u(Slice(), 0));
    casadi::DM Jopt = sol.value(cost);

    RCLCPP_INFO_STREAM(this->get_logger(), "xopt: " << sol.value(x));
    RCLCPP_INFO_STREAM(this->get_logger(), "uopt: " << sol.value(u));
    RCLCPP_INFO_STREAM(this->get_logger(), "Jopt: " << sol.value(cost));

    return uopt;
  }

  void publish_transform_and_path(const casadi::DM & xTrue)
  {
    // 結果を格納
    x = xTrue(0).scalar();
    y = xTrue(1).scalar();
    th = xTrue(2).scalar();
    dx = xTrue(3).scalar();
    dy = xTrue(4).scalar();
    dth = xTrue(5).scalar();

    // 座標変換（odom to base_link）
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, th);  // ロール、ピッチ、ヨーをセット
    geometry_msgs::msg::Quaternion odom_quat_msg =
      tf2::toMsg(odom_quat);    // tf2::Quaternionからgeometry_msgs::msg::Quaternionに変換

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat_msg;

    odom_broadcaster->sendTransform(odom_trans);

    // パスを格納
    geometry_msgs::msg::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;
    this_pose_stamped.pose.orientation = odom_quat_msg;
    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = "odom";
    path.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);

    // パスを公開
    path_pub->publish(path);

    // 時刻を更新
    last_time = current_time;
  }

  void send_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->get_clock()->now();
    static_transform_stamped.header.frame_id = "map";
    static_transform_stamped.child_frame_id = "odom";
    static_transform_stamped.transform.translation.x = 0.0;
    static_transform_stamped.transform.translation.y = 0.0;
    static_transform_stamped.transform.translation.z = 0.0;
    static_transform_stamped.transform.rotation.x = 0.0;
    static_transform_stamped.transform.rotation.y = 0.0;
    static_transform_stamped.transform.rotation.z = 0.0;
    static_transform_stamped.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(static_transform_stamped);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  nav_msgs::msg::Path path;
  rclcpp::Time current_time, last_time;
  double dt;
  int nx, nu, N;
  casadi::DM A, B, Q, R, P, xTrue;
  casadi::DM umin, umax;
  casadi::DM xTarget;
  std::vector<casadi::DM> obs_pos_list;
  casadi::DM vehicle_diameter, obs_diameter, obs_r, p;
  double x, y, th, dx, dy, dth;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCNode>());
  rclcpp::shutdown();
  return 0;
}
