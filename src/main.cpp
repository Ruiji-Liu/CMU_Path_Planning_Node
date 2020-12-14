#include "ros/ros.h"
#include "cmu_ekf/lines_org.h"
#include "cmu_ekf/line_polar.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"
#include "cmu_path_planning/path.h"
#include "visualization_msgs/Marker.h"

// Function Prototypes
void lineCallback(const cmu_ekf::lines_org::ConstPtr &msg);
double getYaw(const double w, const double x, const double y, const double z);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

// Helper Constants
constexpr uint8_t LEFT = cmu_ekf::line_polar::LEFT;
constexpr uint8_t CENTER = cmu_ekf::line_polar::CENTER;
constexpr uint8_t RIGHT = cmu_ekf::line_polar::RIGHT;

double x, y, yaw;
ros::Publisher pubPath, pubMarker;

struct polarPoint {
    double distance, theta;
};

void displayPoint(std::vector<geometry_msgs::Point> points, float r, float g, float b, int id) {
    uint32_t shape = visualization_msgs::Marker::POINTS;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();

    marker.ns = "shapes";
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.points = points;
    marker.lifetime = ros::Duration(1.0);
    pubMarker.publish(marker);
}

/**
 * @brief display a line in the rviz simulator
 * 
 * @param line the line to display
 * @param r the r value in rgb color scheme
 * @param g the g value in rgb color scheme
 * @param b the b value in rgb color scheme
 * @param id the ID to assign to the line
 */
void displayLine(cmu_ekf::line_polar line, float r, float g, float b, int id) {
    uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();

    marker.ns = "shapes";
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    // Calculate two points from which to build the line
    geometry_msgs::Point pt1;
    pt1.x = line.distance * std::cos(line.theta);
    pt1.y = line.distance * std::sin(line.theta);
    pt1.z = 0;

    geometry_msgs::Point pt2;


    marker.points.push_back(pt1);
    marker.points.push_back(pt2);

    // Set the thickness of the line (0-1)
    marker.scale.x = 0.15;

    // Set the color of the line
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.pose.orientation.w = 1;

    marker.lifetime = ros::Duration(1.0);
    pubMarker.publish(marker);
}


void lineCallback(const cmu_ekf::lines_org::ConstPtr &msg) {
    //Receive a left and right line, determine a middle line, choose points on that line
    cmu_ekf::line_polar middle_line;
    middle_line.theta = (msg->left.theta + msg->right.theta) / 2;
    //Left is negative, right is positive
    double ave_dist {(-msg->left.distance + msg->right.distance) / 2};
    middle_line.direction = ave_dist < 0 ? LEFT : RIGHT;
    middle_line.distance = std::abs(ave_dist);

    polarPoint mid;
    mid.distance = (msg->left.distance + msg->right.distance) / 2;
    double ave_theta {(msg->left.theta + msg->right.theta) / 2};
    mid.theta = middle_line.direction == RIGHT ? M_PI_2 - ave_dist : M_PI_2 - ave_dist + 180;

    std::vector<geometry_msgs::Point> points;

/**
 * 
 *          |                                 (x2, y2)> |_
 *          |                                           | `~_
 *          |                                           |    `~_ (r2, θ2)           r2 (c) = sqrt(r1^2 + dx^2)
 *          |                                           |       `~_                 θ2 = θ1 - tan^-1(dx / r1)
 *          |                                           |          `~_
 *          |    (r1,θ1)                                |             `~_           x2, y2 = r2cos(θ2), r2sin(θ2)
 * (x1,y1)> |~~~~~~~~~~~~~~~~* (0,0)                    |~~~~~~~~~~~~~~~~* (0,0)
 *          
 *                                         _
 *  | : line to follow, * : robot, ~~~ or   `~ : vector to point
 */

    // Every dx meters on this line, there will be a point
    double dx {0.5};
    unsigned numPoints {10};

    geometry_msgs::Point direct;
    direct.x = middle_line.distance * std::cos(middle_line.theta);
    direct.y = middle_line.distance * std::sin(middle_line.theta);

    points.push_back(direct);

    displayPoint(points, 1, 0, 0, 1);

    for(int i{0}; i < numPoints; i++) {
        polarPoint ptP;
        ptP.distance = std::sqrt((middle_line.distance * middle_line.distance) + (dx * dx));
        ptP.theta = middle_line.theta - std::atan(dx / middle_line.distance);
        geometry_msgs::Point ptC;
        ptC.x = ptP.distance * std::cos(middle_line.theta);
        ptC.y = ptP.distance * std::sin(middle_line.theta);
        points.push_back(ptC);
    }

    cmu_path_planning::path path;
    path.header = ros::Time::now();
    path.pts = points;
    pubPath.publish(path);
}

/**
 * @brief Find the yaw from a quaternion
 * 
 * @param w the w of the quaternion
 * @param x the x of the quaternion
 * @param y the y of the quaternion
 * @param z the z of the quaternion
 * 
 * @return the yaw calculated
 */
double getYaw(const double w, const double x, const double y, const double z) {
    tf::Quaternion quat;
    quat.setW(w);
    quat.setX(x);
    quat.setY(y);
    quat.setZ(z);

    return tf::getYaw(quat);
}

/**
 * @brief Set the global robot odometry variables
 * 
 * @param msg the Odometry message
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    geometry_msgs::Quaternion orientation = {msg->pose.pose.orientation};
    yaw = getYaw(orientation.w, orientation.x, orientation.y, orientation.z);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "path_planning");

    ros::NodeHandle n;

    ros::Subscriber subLines = n.subscribe<cmu_ekf::lines_org>("/ekf_lines", 1, lineCallback);
    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, odomCallback);
    pubPath = n.advertise<cmu_path_planning::path>("/path_planned", 1);
    pubMarker = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    ros::spin();

    return 0;
}