#include <casadi/casadi.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace casadi;

// dx/dt = f(x,u)
MX f(const MX & x, const MX & u) {return vertcat(x(1), u - x(1));}

class CarRaceNode : public rclcpp::Node
{
public:
  CarRaceNode()
  : Node("car_race_node")
  {
    // Car race along a track
    // ----------------------
    // An optimal control problem (OCP),
    // solved with direct multiple-shooting.
    //
    // For more information see: http://labs.casadi.org/OCP

    int N = 100;  // number of control intervals

    Opti opti = Opti();  // Optimization problem

    Slice all;
    // ---- decision variables ---------
    MX X = opti.variable(2, N + 1);  // state trajectory
    auto pos = X(0, all);
    auto speed = X(1, all);
    MX U = opti.variable(1, N);  // control trajectory (throttle)
    MX T = opti.variable();      // final time

    // ---- objective          ---------
    opti.minimize(T);  // race in minimal time

    // ---- dynamic constraints --------
    MX dt = T / N;
    for (int k = 0; k < N; ++k) {
      MX k1 = f(X(all, k), U(all, k));
      MX k2 = f(X(all, k) + dt / 2 * k1, U(all, k));
      MX k3 = f(X(all, k) + dt / 2 * k2, U(all, k));
      MX k4 = f(X(all, k) + dt * k3, U(all, k));
      MX x_next = X(all, k) + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
      opti.subject_to(X(all, k + 1) == x_next);  // close the gaps
    }

    // ---- path constraints -----------
    opti.subject_to(speed <= 1 - sin(2 * casadi::pi * pos) / 2);  // track speed limit
    opti.subject_to(0 <= U <= 1);                                 // control is limited

    // ---- boundary conditions --------
    opti.subject_to(pos(0) == 0);    // start at position 0 ...
    opti.subject_to(speed(0) == 0);  // ... from stand-still
    opti.subject_to(pos(N) == 1);    // finish line at position 1

    // ---- misc. constraints  ----------
    opti.subject_to(T >= 0);  // Time must be positive

    // ---- initial values for solver ---
    opti.set_initial(speed, 1);
    opti.set_initial(T, 1);

    // ---- solve NLP              ------
    opti.solver("ipopt");                // set numerical backend
    casadi::OptiSol sol = opti.solve();  // actual solve

    eval_funcval = sol.value(T);
    eval_vari_speed = sol.value(speed);
    eval_vari_pos = sol.value(pos);

    RCLCPP_INFO_STREAM(this->get_logger(), "evaluation function value: " << eval_funcval);
    RCLCPP_INFO_STREAM(this->get_logger(), "x1: " << eval_vari_speed);
    RCLCPP_INFO_STREAM(this->get_logger(), "x2: " << eval_vari_pos);
  }

private:
  casadi::DM eval_funcval;
  casadi::DM eval_vari_speed;
  casadi::DM eval_vari_pos;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarRaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
