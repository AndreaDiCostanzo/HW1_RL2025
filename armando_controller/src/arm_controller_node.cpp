#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
using namespace std::chrono_literals;

using std::placeholders::_1;


class ArmControllerNode : public rclcpp::Node
{
  public:
    ArmControllerNode()
    : Node("arm_controller_node"), count_(0), current_pos{0.0, 0.0, 0.0, 0.0}
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&ArmControllerNode::topic_callback, this, _1));
      publisherpc_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("position_controller/commands", 10);
      publishertc_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ArmControllerNode::timer_callback, this));
      this->declare_parameter("controller_type",0);
    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg)
    {

      RCLCPP_INFO(this->get_logger(), "\n J0 Pos: '%f'\n J1 Pos: '%f' \n J2 Pos: '%f'\n J3 Pos: '%f'\n", msg.position[0], msg.position[1], msg.position[2], msg.position[3]);
      for(int i=0; i<4;i++){
        current_pos[i] = msg.position[i];
      }
    }

    void timer_callback(){
      int choice_local = this->get_parameter("controller_type").as_int();
      bool reached_position = true;
      double epsilon = 1e-3;
      if (choice_local == 0){
        if(count_ <4){
          std_msgs::msg::Float64MultiArray message[4];
          message[0].data = {1.0, 0.0, 1.0, 0.0};
          message[1].data = {0.0, 1.0, 1.0, 1.0};
          message[2].data = {0.0, 0.0, 1.0, 2.0};
          message[3].data = {1.0, 1.0, 1.0, 1.0};

          //RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f', '%f', '%f'", message[count_].data[0], message[count_].data[1], message[count_].data[2], message[count_].data[3]);
          publisherpc_->publish(message[count_]);


          for(int i=0; i<4; i++){
            if(((current_pos[i] > message[count_].data[i] - epsilon)) && (current_pos[i] < (message[count_].data[i] + epsilon ))){
              reached_position = reached_position && true;
            }
            else{
              reached_position = reached_position && false;
            }
          }

          if(reached_position == true){
            //RCLCPP_INFO(this->get_logger(), "POSITION %d REACHED", count_);
            count_++;
          }
        }
        else{
          //RCLCPP_INFO(this->get_logger(), "ALL POSITIONS REACHED");
        }
      
      }
      else if(choice_local == 1){
        trajectory_msgs::msg::JointTrajectory message_tc;
        message_tc.joint_names={"j0", "j1", "j2", "j3"};
        trajectory_msgs::msg::JointTrajectoryPoint point[4];

        point[0].positions = {1.0, 0.0, 1.0, 0.0};
        point[0].velocities = {0.0, 0.0, 0.0, 0.0};
        point[0].time_from_start = rclcpp::Duration(15, 0);


        point[1].positions = {0.0, 1.0, 1.0, 1.0};
        point[1].velocities = {0.0, 0.0, 0.0, 0.0};
        point[1].time_from_start = rclcpp::Duration(30, 0);


        point[2].positions = {0.0, 0.0, 1.0, 2.0};
        point[2].velocities = {0.0, 0.0, 0.0, 0.0};
        point[2].time_from_start = rclcpp::Duration(45, 0);

        point[3].positions = {1.0, 1.0, 1.0, 1.0};
        point[3].velocities = {0.0, 0.0, 0.0, 0.0};
        point[3].time_from_start = rclcpp::Duration(60, 0);

        message_tc.points.push_back(point[0]);
        message_tc.points.push_back(point[1]);
        message_tc.points.push_back(point[2]);
        message_tc.points.push_back(point[3]);
        if(count_ < 1){
        publishertc_->publish(message_tc);
        count_++;
        }
      }  
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisherpc_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publishertc_;
    int count_;
    double current_pos[4];
    
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
