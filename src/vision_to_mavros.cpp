#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

Eigen::Vector3d p_mav;
Eigen::Quaterniond q_mav;
geometry_msgs::PoseStamped vision;

geometry_msgs::PoseStamped odomBias;
bool initFlag;

void vins_callback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg)
{
    // Dereference the shared pointer to access the actual message
    vision = *msg;
    if(!initFlag) {
        odomBias.pose.position.x = vision.pose.position.x;
        odomBias.pose.position.y = vision.pose.position.y;
        odomBias.pose.position.z = vision.pose.position.z;
        initFlag = true;
        ROS_WARN("Odom Bias Initialized, init position:");
        // Caution! Converting from millimeters to meters here!!
        ROS_WARN("x: %f, y: %f, z: %f",
                 odomBias.pose.position.x / 1000,
                 odomBias.pose.position.y / 1000,
                 odomBias.pose.position.z / 1000);
    }
    vision.pose.position.x -= odomBias.pose.position.x;
    vision.pose.position.y -= odomBias.pose.position.y;
    vision.pose.position.z -= odomBias.pose.position.z;
    vision.pose.position.x /= 1000;
    vision.pose.position.y /= 1000;
    vision.pose.position.z /= 1000;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh("~");

    ros::Subscriber slam_sub = nh.subscribe<geometry_msgs::PoseStamped>("odom", 100,vins_callback);

    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);
 
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    initFlag = 0;
    ros::Time last_request = ros::Time::now();
 
    while(ros::ok()){
        vision_pub.publish(vision);
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}