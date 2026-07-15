#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include <xarm_msgs/srv/move_home.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>
#include <xarm_msgs/srv/set_analog_io.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>
#include "block_sorter/srv/move_once.hpp"

#include <thread>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

std::vector<float> current_pose(6, 0);
int pose_ready = 0;

bool check_pose_reached(rclcpp::Node::SharedPtr node,
                        const std::vector<float> &target,
                        xarm_msgs::srv::MoveCartesian::Request &last_req) {
    long double sum = 0;
    if (pose_ready == 0) return true;
    sum += std::fabs(target[0] - current_pose[0]);
    sum += std::fabs(target[1] - current_pose[1]);
    sum += std::fabs(target[2] - current_pose[2]);
    std::this_thread::sleep_for(10ms);
    return sum > 0.1;
}

class MoveOnceNode : public rclcpp::Node {
public:
    MoveOnceNode() : Node("move_once") {
        up_height_ = static_cast<float>(this->declare_parameter<double>("up_height", 60.0));
        down_height_ = static_cast<float>(this->declare_parameter<double>("down_height", -24.0));
        place_clearance_ = static_cast<float>(this->declare_parameter<double>("place_clearance", 20.0));
        suction_voltage_ = static_cast<float>(this->declare_parameter<double>("suction_voltage", 5.0));

        /* Service clients for xarm */
        move_client_ = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
        valve_client_ = this->create_client<xarm_msgs::srv::SetDigitalIO>("/xarm/set_cgpio_digital");
        analog_client_ = this->create_client<xarm_msgs::srv::SetAnalogIO>("/xarm/set_cgpio_analog");
        go_home_client_ = this->create_client<xarm_msgs::srv::MoveHome>("/xarm/move_gohome");

        /* Publisher for sleep_sec */
        sleep_pub_ = this->create_publisher<std_msgs::msg::Float32>("/xarm/sleep_sec", 1);

        /* Subscribe to xarm state for feedback */
        state_sub_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
            "/xarm/robot_states", 10,
            std::bind(&MoveOnceNode::stateCallback, this, std::placeholders::_1));

        /* Advertise the MoveOnce service */
        service_ = this->create_service<block_sorter::srv::MoveOnce>(
            "/Move_Once",
            std::bind(&MoveOnceNode::commandCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "MoveOnce service ready, waiting for xarm services...");

        /* Wait for required services and go home */
        if (go_home_client_->wait_for_service(3s)) {
            auto go_req = std::make_shared<xarm_msgs::srv::MoveHome::Request>();
            go_req->speed = 20.0 / 57.0;
            go_req->acc = 1000;
            go_req->mvtime = 0;
            go_req->wait = false;
            go_home_client_->async_send_request(go_req);
        }
    }

private:
    void stateCallback(const xarm_msgs::msg::RobotMsg::SharedPtr msg) {
        current_pose[0] = msg->pose[0];
        current_pose[1] = msg->pose[1];
        current_pose[2] = msg->pose[2];
        current_pose[3] = msg->pose[3];
        current_pose[4] = msg->pose[4];
        current_pose[5] = msg->pose[5];
        pose_ready = 1;
    }

    void commandCallback(
        const block_sorter::srv::MoveOnce::Request::SharedPtr req,
        block_sorter::srv::MoveOnce::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "MoveOnce: pick(%.1f, %.1f) → place(%.1f, %.1f) angle=%.2f",
                    (double)req->x1, (double)req->y1, (double)req->x2, (double)req->y2, (double)req->angle);

        /* Publish sleep_sec */
        auto sleep_msg = std::make_unique<std_msgs::msg::Float32>();
        sleep_msg->data = 0.2f;
        sleep_pub_->publish(std::move(sleep_msg));

        /* 1. Move above pick point */
        auto pos_req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
        pos_req->acc = 2000;
        pos_req->speed = 200;
        pos_req->mvtime = 0;
        pos_req->radius = 20;
        pos_req->pose = {req->x1, req->y1, up_height_, 3.14f, 0, 0};
        pos_req->wait = false;
        move_client_->async_send_request(pos_req);

        /* 2. Move down to pick */
        pos_req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
        pos_req->acc = 2000;
        pos_req->speed = 200;
        pos_req->mvtime = 0;
        pos_req->radius = 20;
        pos_req->pose = {req->x1, req->y1, down_height_, 3.14f, 0, 0};
        pos_req->wait = false;
        move_client_->async_send_request(pos_req);
        rclcpp::sleep_for(std::chrono::milliseconds(1200));

        /* 3. Activate vacuum: digital IO port 1 ON + analog 5V */
        auto valve_req = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
        valve_req->ionum = 1;
        valve_req->value = 1;
        valve_client_->async_send_request(valve_req);
        auto analog_req = std::make_shared<xarm_msgs::srv::SetAnalogIO::Request>();
        analog_req->ionum = 1;
        analog_req->value = suction_voltage_;
        analog_client_->async_send_request(analog_req);
        std::this_thread::sleep_for(800ms);

        /* 4. Move up after pick */
        pos_req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
        pos_req->acc = 2000;
        pos_req->speed = 200;
        pos_req->mvtime = 0;
        pos_req->radius = 20;
        pos_req->pose = {req->x1, req->y1, up_height_, 3.14f, 0, 0};
        pos_req->wait = false;
        move_client_->async_send_request(pos_req);
        rclcpp::sleep_for(1s);

        /* 5. Move above place point */
        pos_req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
        pos_req->acc = 2000;
        pos_req->speed = 200;
        pos_req->mvtime = 0;
        pos_req->radius = 20;
        pos_req->pose = {req->x2, req->y2, up_height_ + place_clearance_, 3.14f, 0, req->angle};
        pos_req->wait = false;
        move_client_->async_send_request(pos_req);
        rclcpp::sleep_for(1s);

        /* 6. Move down to place */
        pos_req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
        pos_req->acc = 2000;
        pos_req->speed = 100;
        pos_req->mvtime = 0;
        pos_req->radius = 20;
        pos_req->pose = {req->x2, req->y2, down_height_, 3.14f, 0, req->angle};
        pos_req->wait = false;
        move_client_->async_send_request(pos_req);

        /* 7. Deactivate vacuum: digital IO port 1 OFF + analog 0V */
        std::this_thread::sleep_for(300ms);
        valve_req = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
        valve_req->ionum = 1;
        valve_req->value = 0;
        valve_client_->async_send_request(valve_req);
        analog_req = std::make_shared<xarm_msgs::srv::SetAnalogIO::Request>();
        analog_req->ionum = 1;
        analog_req->value = 0.0f;
        analog_client_->async_send_request(analog_req);
        std::this_thread::sleep_for(500ms);

        /* 8. Move up after place */
        pos_req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
        pos_req->acc = 2000;
        pos_req->speed = 200;
        pos_req->mvtime = 0;
        pos_req->radius = 20;
        pos_req->pose = {req->x2, req->y2, up_height_, 3.14f, 0, req->angle};
        pos_req->wait = false;
        move_client_->async_send_request(pos_req);

        res->result = true;
        RCLCPP_INFO(this->get_logger(), "MoveOnce completed successfully");
    }

    rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr move_client_;
    rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr valve_client_;
    rclcpp::Client<xarm_msgs::srv::SetAnalogIO>::SharedPtr analog_client_;
    rclcpp::Client<xarm_msgs::srv::MoveHome>::SharedPtr go_home_client_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sleep_pub_;
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr state_sub_;
    rclcpp::Service<block_sorter::srv::MoveOnce>::SharedPtr service_;
    float up_height_;
    float down_height_;
    float place_clearance_;
    float suction_voltage_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveOnceNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
