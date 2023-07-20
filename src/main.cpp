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
        marker.header.frame_id = "base_link";
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

        // marker.pose.position.x = points.at(i).x + x;
        // marker.pose.position.y = points.at(i).y + y;
        marker.pose.position.x = points.at(i).x;
        marker.pose.position.y = points.at(i).y;
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
    std::vector<geometry_msgs::Point> points;

    // Every dx meters on this line, there will be a point
    double dxAbsolute {0.5};
    unsigned numPoints {10};

    for(int i{1}; i < numPoints; i++) {
        geometry_msgs::Point left_pt;
        left_pt.x = dxAbsolute * i;

        // Get X,Y from the distance and theta
        double x {msg->left.distance * std::cos(msg->left.theta)};

        // Reverse the y, since left should be positive (but left theta is negative)
        double y {-msg->left.distance * std::sin(msg->left.theta)};

        // Get slope from this x,y and the theta (theta is normal to the slope)
        double slope {1 / std::tan(msg->left.theta)};
        double intercept {y - slope * x};

        // std::cout << "left: from distance " << msg->left.distance << " and theta " << msg->left.theta << " got x " << x << " and y " << y << " and slope " << slope << " and intercept " << intercept << std::endl;

        // Get the y value from the x value
        left_pt.y = slope * left_pt.x + intercept;

        geometry_msgs::Point right_pt;
        right_pt.x = dxAbsolute * i;
        x = msg->right.distance * std::cos(msg->right.theta);
        y = -msg->right.distance * std::sin(msg->right.theta);
        slope = 1 / std::tan(msg->right.theta);
        intercept = y - slope * x;
        right_pt.y = slope * right_pt.x + intercept;

        geometry_msgs::Point mid_pt;
        mid_pt.x = dxAbsolute * i;
        mid_pt.y = (left_pt.y + right_pt.y) / 2;

        points.push_back(mid_pt);
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
    pubMarkers = n.advertise<visualization_msgs::MarkerArray>("/visualization_waypoint", 1);

    ros::spin();

    return 0;
}