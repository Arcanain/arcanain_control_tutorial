#include <casadi/casadi.hpp>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

class OptiNode : public rclcpp::Node
{
public:
  OptiNode()
  : Node("casadi_opti_example")
  {
    // Optiインスタンスを作成
    casadi::Opti opti;

    // 変数を定義
    casadi::MX x1 = opti.variable();
    casadi::MX x2 = opti.variable();

    // 初期値を指定
    opti.set_initial(x1, 3);
    opti.set_initial(x2, 3);

    // 目的関数を定義
    casadi::MX obj = x1 * x1 + x2 * x2;
    opti.minimize(obj);

    // 制約条件を定義
    opti.subject_to(x1 * x2 >= 1);

    // 変数の範囲を定義
    opti.subject_to(opti.bounded(0, x1, 4));
    opti.subject_to(opti.bounded(0, x2, 4));

    // 最適化ソルバを設定
    opti.solver("ipopt", {{"ipopt.print_level", 5}});  // 出力レベルを設定

    // 最適化計算を実行
    casadi::OptiSol sol = opti.solve();
    eval_funcval = sol.value(obj);
    eval_vari_x1 = sol.value(x1);
    eval_vari_x2 = sol.value(x2);

    // 結果を出力
    RCLCPP_INFO_STREAM(this->get_logger(), "evaluation function value: " << eval_funcval);
    RCLCPP_INFO_STREAM(this->get_logger(), "x1: " << eval_vari_x1);
    RCLCPP_INFO_STREAM(this->get_logger(), "x2: " << eval_vari_x2);
  }

private:
  casadi::DM eval_funcval;
  casadi::DM eval_vari_x1;
  casadi::DM eval_vari_x2;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptiNode>());
  rclcpp::shutdown();
  return 0;
}
