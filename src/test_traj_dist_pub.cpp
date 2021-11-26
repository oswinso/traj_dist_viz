#include <random>

#include "ros/ros.h"
#include "traj_dist_viz/TrajDist.h"
#include "visualization_msgs/Marker.h"

class TestTrajDistPub {
 public:
    explicit TestTrajDistPub(ros::NodeHandle &nh);

 private:
    void pub_traj_dist(const ros::TimerEvent &e);

    ros::Publisher pub_;
    ros::Publisher pub2_;
    ros::Timer timer_;
    std::mt19937 gen_;
};

TestTrajDistPub::TestTrajDistPub(ros::NodeHandle &nh) : gen_{1234} {
    pub_ = nh.advertise<traj_dist_viz::TrajDist>("traj_dist", 1);
    pub2_ = nh.advertise<visualization_msgs::Marker>("marker", 1);
    timer_ = nh.createTimer(ros::Rate{1}, &TestTrajDistPub::pub_traj_dist, this);
}

void TestTrajDistPub::pub_traj_dist(const ros::TimerEvent &e) {
    std::uniform_int_distribution<size_t> dis{3, 12};
    std::uniform_real_distribution<double> offset{-0.05, 0.05};
    const size_t n_trajs_to_pub = dis(gen_);

    const auto now = ros::Time::now();
    traj_dist_viz::TrajDist msg{};
    msg.header.frame_id = "odom";
    msg.header.stamp = now;

    for (size_t ii = 0; ii < n_trajs_to_pub; ii++) {
        const double angle = ii * 0.1 + offset(gen_);
        nav_msgs::Path path{};
        path.header = msg.header;

        constexpr auto T = 100;
        for (size_t t = 0; t < T; t++) {
            geometry_msgs::PoseStamped pose{};
            pose.header.frame_id = path.header.frame_id;
            pose.header.stamp = path.header.stamp + ros::Duration{0.1 * t};

            const double length = 0.25 * static_cast<double>(t);
            pose.pose.position.x = length * std::cos(angle);
            pose.pose.position.y = length * std::sin(angle);
            path.poses.emplace_back(pose);
        }
        msg.paths.emplace_back(path);
    }

    ROS_INFO_STREAM("n_trajs_to_publish: " << n_trajs_to_pub);
    pub_.publish(msg);

    visualization_msgs::Marker marker_msg{};
    marker_msg.header = msg.header;
    marker_msg.ns = "lines";
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.orientation.w = 1;
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
    marker_msg.color.r = 1.0;

    std::uniform_real_distribution<double> alpha_dist{0.01, 0.99};
    marker_msg.color.a = alpha_dist(gen_);

    marker_msg.scale.x = 0.1;

    for (size_t i = 0; i < 100; ++i) {
        double y = 5 * std::sin(i / 100.0f * 2 * M_PI);
        double z = 5 * std::cos(i / 100.0f * 2 * M_PI);

        geometry_msgs::Point p;
        p.x = i - 50;
        p.y = y;
        p.z = z;
        marker_msg.points.push_back(p);
    }

    pub2_.publish(marker_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_traj_dist_pub");
    ros::NodeHandle pnh{"~"};
    TestTrajDistPub pub{pnh};
    ros::spin();
}
