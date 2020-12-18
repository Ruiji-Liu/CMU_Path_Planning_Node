#include "ros/ros.h"
#include "CMU_EKF_Node/lines_org.h"
#include "CMU_EKF_Node/line_polar.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"
#include "CMU_Path_Planning_Node/path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

// Function Prototypes
void lineCallback(const CMU_EKF_Node::lines_org::ConstPtr &msg);
double getYaw(const double w, const double x, const double y, const double z);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

// Helper Constants
constexpr uint8_t LEFT = CMU_EKF_Node::line_polar::LEFT;
constexpr uint8_t CENTER = CMU_EKF_Node::line_polar::CENTER;
constexpr uint8_t RIGHT = CMU_EKF_Node::line_polar::RIGHT;

double x, y, yaw;
ros::Publisher pubPath, pubMarker, pubMarkers;

struct polarPoint {
    double distance, theta;
};

void displayPoints(std::vector<geometry_msgs::Point> points) {
    visualization_msgs::MarkerArray arr;
    std::vector<visualization_msgs::Marker> markers;
    
    for(int i {0}; i < points.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();

        marker.ns = "waypoints";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.05;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.pose.position.x = points.at(i).x + x;
        marker.pose.position.y = points.at(i).y + y;
        marker.pose.position.z = 0.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration(0);
        markers.push_back(marker);
    }
    arr.markers = markers;
    pubMarkers.publish(arr);
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
void displayLine(CMU_EKF_Node::line_polar line, float r, float g, float b, int id) {
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


void lineCallback(const CMU_EKF_Node::lines_org::ConstPtr &msg) {
    //Receive a left and right line, determine a middle line, choose points on that line
    CMU_EKF_Node::line_polar middle_line;

    //Left is negative, right is positive
    double ave_dist {(-msg->left.distance + msg->right.distance) / 2};
    middle_line.direction = ave_dist < 0 ? LEFT : RIGHT;
    middle_line.distance = std::abs(ave_dist);

    if (middle_line.direction == RIGHT) {
        middle_line.theta = (msg->right.theta + (M_PI_2 + (M_PI_2 - msg->left.theta))) / 2;
    } else {
        middle_line.theta = (msg->left.theta + (M_PI_2 + (M_PI_2 - msg->right.theta))) / 2;
    }

    std::cout << "Mid: " << middle_line.distance << " " << middle_line.theta << std::endl;

    geometry_msgs::Point original;
    original.x = 0;
    original.y = middle_line.direction == RIGHT ? -middle_line.distance : middle_line.distance;

    std::vector<geometry_msgs::Point> points;
    points.push_back(original);

    std::cout << "Point: " << original.x << " " << original.y << std::endl;

    // Every dx meters on this line, there will be a point
    double dxAbsolute {0.5};
    unsigned numPoints {10};

    double dx {dxAbsolute * std::sin(middle_line.theta)};
    double dy {dxAbsolute * std::abs(std::cos(middle_line.theta))};
    if((middle_line.direction == LEFT && middle_line.theta < M_PI_2) ||
       (middle_line.direction == RIGHT && middle_line.theta > M_PI_2)) {
            dy *= -1;
        }

    for(int i{1}; i < numPoints; i++) {
        geometry_msgs::Point pt;
        pt.x = original.x + (dx * i);
        pt.y = original.y + (dy * i);
        points.push_back(pt);
    }

    displayPoints(points);

    CMU_Path_Planning_Node::path path;
    path.header.stamp = ros::Time::now();
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

    ros::Subscriber subLines = n.subscribe<CMU_EKF_Node::lines_org>("/ekf_lines", 1, lineCallback);
    ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, odomCallback);
    pubPath = n.advertise<CMU_Path_Planning_Node::path>("/path_planned", 1);
    pubMarker = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    pubMarkers = n.advertise<visualization_msgs::MarkerArray>("visualization_waypoint", 1);

    ros::spin();

    return 0;
}