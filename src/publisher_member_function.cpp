// 必要なヘッダーファイルをインクルード
#include <chrono>      // 時間に関する標準ライブラリ
#include <functional>  // 関数オブジェクトに関する機能を提供
#include <memory>      // スマートポインタなどのメモリ管理機能を提供
#include <string>      // 文字列操作のための標準ライブラリ

// ROS2のコア機能と標準メッセージ型をインクルード
#include "rclcpp/rclcpp.hpp"        // ROS2の基本的なノード機能を提供
#include "std_msgs/msg/string.hpp"  // 文字列型のメッセージを定義

// 時間リテラル（例：500ms）を使うための名前空間を使用
using namespace std::chrono_literals;

// MinimalPublisherクラスはrclcpp::Nodeクラスを継承している
class MinimalPublisher : public rclcpp::Node
{
public:
  // コンストラクタ
  MinimalPublisher()
  : Node("minimal_publisher"),  // Nodeクラスのコンストラクタを呼び出し、ノード名を指定
    count_(0)                   // メッセージカウンタを0で初期化
  {
    // "topic"という名前のトピックに対してパブリッシャーを作成。キューサイズは10。
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // 500ミリ秒ごとにtimer_callback関数を呼び出すタイマーを作成
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  // タイマーのコールバック関数
  void timer_callback()
  {
    auto message = std_msgs::msg::String();  // 文字列メッセージを作成
    // メッセージの内容を設定（"Hello, world!"にカウンタ値を追加）
    message.data = "Hello, world! " + std::to_string(count_++);
    // メッセージをログに出力し、パブリッシュ
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);  // 作成したメッセージをパブリッシュ
  }
  rclcpp::TimerBase::SharedPtr timer_;  // タイマーを保持するためのスマートポインタ
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    publisher_;   // パブリッシャーを保持するスマートポインタ
  size_t count_;  // メッセージに付加するカウンタ
};

// main関数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // ROS2システムの初期化
  // MinimalPublisherノードのインスタンスを作成し、スピンすることでコールバック関数を実行可能にする
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();  // ROS2システムのシャットダウン
  return 0;            // プログラム終了
}
