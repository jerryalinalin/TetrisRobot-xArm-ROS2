#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <color_detector/msg/detected_target_array.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include "block_sorter/srv/move_once.hpp"
#include "block_sorter/strategy.h"

#include <thread>
#include <chrono>
#include <ctime>
#include <stdexcept>

using namespace std::chrono_literals;

/* Local data for detected target (avoids depending on color_detector::target) */
struct TargetInfo {
    int x, y;
    double yaw;
    int sta;
    bool updown;
};

/* Globals for strategy.h */
int cnt[7] = {0};
std::vector<orderType> order;
std::vector<std::vector<orderType>> orders;

/* Mapping from color enum to name */
std::map<COLORS, std::string> colorTostring = {
    {PURPLE, "PURPLE"}, {GREEN, "GREEN"},   {BLUE, "BLUE"},
    {ORANGE, "ORANGE"}, {YELLOW, "YELLOW"}, {RED, "RED"},
    {BROWN, "BROWN"},   {WHITE, "WHITE"}};

class SorterNode : public rclcpp::Node {
public:
    SorterNode() : Node("sorter_node") {
        const auto pick_transform = this->declare_parameter<std::vector<double>>(
            "pick_transform",
            {0.6199788134234798, -0.0329665386415458, 20.3605039937090027,
             -0.0397976429099523, -0.6173058588885968, 446.5625774096370719});
        const auto place_transform = this->declare_parameter<std::vector<double>>(
            "place_transform",
            {-1.0694221236261483, 20.0825129441224810, 337.7265604465425213,
             -20.1716536574476351, -1.0901639344262297, -59.3888159450250086});
        const auto initial_pose = this->declare_parameter<std::vector<double>>(
            "initial_pose", {434.7, 130.2, 474.7, 3.14159, 0.0, 1.53});
        if (pick_transform.size() != 6 || place_transform.size() != 6 ||
            initial_pose.size() != 6) {
            throw std::runtime_error("workcell matrices and initial_pose must contain six values");
        }
        transform_mat_ = cv::Mat(2, 3, CV_64F, const_cast<double *>(pick_transform.data())).clone();
        transform2_mat_ = cv::Mat(2, 3, CV_64F, const_cast<double *>(place_transform.data())).clone();
        initial_pose_.assign(initial_pose.begin(), initial_pose.end());

        /* Use a dedicated callback group to isolate subscription from service calls */
        cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = cb_group_;

        move_client_ = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
        move_once_client_ = this->create_client<block_sorter::srv::MoveOnce>("/Move_Once");

        targets_sub_ = this->create_subscription<color_detector::msg::DetectedTargetArray>(
            "/detected_targets", 1,
            std::bind(&SorterNode::targetsCallback, this, std::placeholders::_1),
            sub_opt);

        /* Send initial position */
        RCLCPP_INFO(this->get_logger(), "Moving to initial position...");
        if (move_client_->wait_for_service(5s)) {
            auto req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
            req->acc = 3000;
            req->speed = 300;
            req->radius = 20;
            req->pose = initial_pose_;
            req->wait = false;
            move_client_->async_send_request(req);
            std::this_thread::sleep_for(3s);
        } else {
            RCLCPP_WARN(this->get_logger(), "/xarm/move_line not available");
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for detected targets...");
    }

private:
    bool busy_ = false;

    void targetsCallback(const color_detector::msg::DetectedTargetArray::SharedPtr msg) {
        if (busy_) return;
        busy_ = true;

        if (msg->targets.empty()) {
            RCLCPP_WARN(this->get_logger(), "No targets detected");
            busy_ = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received %zu targets, running strategy...",
                    msg->targets.size());

        /* Rebuild targetSet from message for position lookup */
        std::map<COLORS, std::vector<TargetInfo>> targetSet;
        for (auto &dt : msg->targets) {
            COLORS color = static_cast<COLORS>(dt.color_id);
            targetSet[color].push_back(
                TargetInfo{static_cast<int>(dt.x), static_cast<int>(dt.y),
                           dt.yaw, dt.sta, dt.updown});
        }

        /* Fill color counts and log */
        for (int i = 0; i < 7; i++) {
            cnt[i] = msg->color_counts[i];
        }
        for (auto &[k, v] : targetSet) {
            RCLCPP_INFO(this->get_logger(), "  %s: %zu",
                        colorTostring[k].c_str(), v.size());
        }

        /* Run Tetris strategy */
        order.clear();
        strategy();

        /* Save strategy visualization (only when plan exists) */
        if (!order.empty()) {
            cv::Mat grid_img = cv::Mat::zeros(14 * 50, 10 * 50, CV_8UC3);
            cv::Scalar colors[] = {
                cv::Scalar(0, 255, 255),   // YELLOW=0
                cv::Scalar(255, 0, 255),   // PURPLE
                cv::Scalar(0, 255, 0),     // GREEN
                cv::Scalar(255, 0, 0),     // BLUE
                cv::Scalar(0, 165, 255),   // ORANGE
                cv::Scalar(0, 75, 150),    // BROWN
                cv::Scalar(0, 0, 255),     // RED
            };
            for (size_t i = 0; i < order.size(); i++) {
                auto &d = order[i];
                /* Draw full tetromino shape: 4 cells per piece */
                int si = suck[d.id][d.way];
                for (int k = 0; k < 4; k++) {
                    int r = d.x - brick[d.id][d.way][si].dx + brick[d.id][d.way][k].dx;
                    int c = d.y - brick[d.id][d.way][si].dy + brick[d.id][d.way][k].dy;
                    cv::rectangle(grid_img,
                        cv::Rect(c * 50, r * 50, 50, 50),
                        colors[d.id % 7], cv::FILLED);
                }
                /* Draw order number at suction point */
                cv::putText(grid_img, std::to_string(i),
                    cv::Point(d.y * 50 + 15, d.x * 50 + 35),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 2);
            }
            for (int i = 0; i <= 14; i++)
                cv::line(grid_img, cv::Point(0, i*50), cv::Point(500, i*50),
                         cv::Scalar(100, 100, 100), 1);
            for (int j = 0; j <= 10; j++)
                cv::line(grid_img, cv::Point(j*50, 0), cv::Point(j*50, 700),
                         cv::Scalar(100, 100, 100), 1);
            std::time_t t = std::time(nullptr);
            char buf[64];
            std::strftime(buf, sizeof(buf), "/strategy_plan_%Y%m%d_%H%M%S.png", std::localtime(&t));
            std::string vis_path = "src/block_sorter/data" + std::string(buf);
            cv::imwrite(vis_path, grid_img);
            RCLCPP_INFO(this->get_logger(), "Strategy plan saved: %s", vis_path.c_str());
        }

        /* Execute orders */
        for (auto &d : order) {
            if (targetSet[(COLORS)d.id].empty()) {
                RCLCPP_WARN(this->get_logger(), "No more %s targets",
                            colorTostring[(COLORS)d.id].c_str());
                continue;
            }

            /* Get target position */
            TargetInfo &back = targetSet[(COLORS)d.id].back();
            double yaw = - back.yaw - d.way * M_PI / 2;
            targetSet[(COLORS)d.id].pop_back();

            /* Coordinate transform */
            cv::Mat pxs = (cv::Mat_<double>(3, 1) << back.x, back.y, 1.0);
            cv::Mat pos = transform_mat_ * pxs;
            cv::Mat pxs2 = (cv::Mat_<double>(3, 1) << d.x, d.y, 1.0);
            cv::Mat posend = transform2_mat_ * pxs2;

            /* Fill service request */
            auto req = std::make_shared<block_sorter::srv::MoveOnce::Request>();
            req->x1 = pos.at<double>(0, 0);
            req->y1 = pos.at<double>(1, 0);
            req->x2 = posend.at<double>(0, 0);
            req->y2 = posend.at<double>(1, 0);

            if ((COLORS)d.id == GREEN)  req->x1 += 1;
            if ((COLORS)d.id == ORANGE) req->y2 += 1;

            int tyaw = static_cast<int>(yaw * 100);
            yaw = tyaw / 100.0;
            if (yaw == 3.14 || yaw == -3.14) yaw = -3.13;
            req->angle = yaw;

            RCLCPP_INFO(this->get_logger(),
                        "Move: pick(%.1f, %.1f) → place(%.1f, %.1f) angle=%.2f",
                        (double)req->x1, (double)req->y1,
                        (double)req->x2, (double)req->y2, (double)req->angle);

            /* Call MoveOnce service (synchronous wait, no nested spin) */
            if (!move_once_client_->wait_for_service(5s)) {
                RCLCPP_ERROR(this->get_logger(), "/Move_Once not available");
                continue;
            }
            auto future = move_once_client_->async_send_request(req);
            auto status = future.wait_for(30s);
            if (status == std::future_status::ready) {
                auto result = future.get();
                RCLCPP_INFO(this->get_logger(), "Move result: %s",
                            result->result ? "OK" : "FAIL");
            } else {
                RCLCPP_ERROR(this->get_logger(), "MoveOnce service call timeout");
            }
        }

        RCLCPP_INFO(this->get_logger(), "All orders executed, returning to photo position...");
        auto home_req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
        home_req->acc = 3000;
        home_req->speed = 300;
        home_req->radius = 20;
        home_req->pose = initial_pose_;
        home_req->wait = false;
        move_client_->async_send_request(home_req);
        std::this_thread::sleep_for(3s);
        RCLCPP_INFO(this->get_logger(), "Waiting for next round...");
        busy_ = false;
    }

    rclcpp::Subscription<color_detector::msg::DetectedTargetArray>::SharedPtr targets_sub_;
    rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr move_client_;
    rclcpp::Client<block_sorter::srv::MoveOnce>::SharedPtr move_once_client_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    cv::Mat transform_mat_;
    cv::Mat transform2_mat_;
    std::vector<float> initial_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SorterNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
