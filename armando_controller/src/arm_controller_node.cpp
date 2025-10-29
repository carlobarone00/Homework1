#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <vector>
#include <cmath>

using std::placeholders::_1;

class ArmControllerNode : public rclcpp::Node
{
public:
    ArmControllerNode(const std::string & choice_controll)
        : Node("arm_controller_node"),
          command_index_(0),
          choice_controll_(choice_controll),
          goal_reached_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Modalità di controllo scelta: '%s'", choice_controll_.c_str());

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&ArmControllerNode::jointStateCallback, this, _1));

        if(choice_controll_ == "0"){
            position_comm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands",
            10);
        }else{
            trajectory_comm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory",
            10);
        }

        command_sequence_ = {
            {0.0, 0.0, 0.2, 0.0},
            {0.0, 0.3, 0.2, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, -0.2, 0.0}
        };
        
        command_sequence_vel_ = {
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0}
        };

        auto timer_callback = [this]() {
        RCLCPP_INFO(this->get_logger(), "5 secondi passati — pubblico il primo comando...");
        publishCommand();
        };

        timer_ = this->create_wall_timer(std::chrono::seconds(5), timer_callback);
    

        RCLCPP_INFO(this->get_logger(), "Arm Controller Node started");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Stampare la posizione corrente delle giunture
        RCLCPP_INFO(this->get_logger(), "Joint positions:");
        for (size_t i = 0; i < msg->position.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  joint[%zu]: %.3f", i, msg->position[i]);
        }

        // Controllo se siamo vicini al target
        if (msg->position.size() >= command_sequence_[command_index_].size()) {
            double error_sum = 0.0;
            for (size_t i = 0; i < command_sequence_[command_index_].size(); ++i) {
                double err = std::fabs(msg->position[i] - command_sequence_[command_index_][i]);
                error_sum += err;
            }
            double avg_error = error_sum / command_sequence_[command_index_].size();

            const double tolerance = 0.0001;  // ad esempio 0.0001 rad
           if (avg_error < tolerance && !goal_reached_) {
                goal_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "Goal #%zu reached!", command_index_ + 1);

                // Avanza al prossimo comando solo se NON siamo all'ultimo
                if (command_index_ + 1 < command_sequence_.size()) {
                    command_index_++;
                    publishCommand();
                    goal_reached_ = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Tutti i goal sono stati raggiunti. Mi fermo.");
                }
            }
        }
    }

    void publishCommand()
    {
        if(choice_controll_ == "0"){
            std_msgs::msg::Float64MultiArray msg;
            msg.data = command_sequence_[command_index_];
            position_comm_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(),
                    "Published command #%zu: [%.2f, %.2f, %.2f, %.2f]",
                    command_index_ + 1,
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
        }else{
            trajectory_msgs::msg::JointTrajectory msg1;
        msg1.joint_names = {"j0", "j1", "j2", "j3"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = command_sequence_[command_index_];
        point.velocities = command_sequence_vel_[command_index_];
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);  // esempio: 2 secondi per raggiungere la posizione
        msg1.points.push_back(point);

        trajectory_comm_pub_->publish(msg1);

        RCLCPP_INFO(this->get_logger(),
                    "Published command #%zu: [%.2f, %.2f, %.2f, %.2f, Velocities: %.2f, %.2f, %.2f, %.2f]",
                    command_index_ + 1,
                    point.positions[0], point.positions[1], point.positions[2], point.positions[3],
                    point.velocities[0], point.velocities[1], point.velocities[2], point.velocities[3]);
        }
    }
    
    std::string choice_controll_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_comm_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_comm_pub_;
    std::vector<std::vector<double>> command_sequence_;
    std::vector<std::vector<double>> command_sequence_vel_;
    size_t command_index_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_reached_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string choice_controll = "default";
    if (argc > 1) {
        choice_controll = argv[1];  // Legge il valore passato dal launch file
    }

    auto node = std::make_shared<ArmControllerNode>(choice_controll); 
    rclcpp::spin(node); 
    rclcpp::shutdown(); 
    return 0;
}
