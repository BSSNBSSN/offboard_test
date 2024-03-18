#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <math.h>
#include <fstream>
#include <cstdlib> // 用于获取环境变量
#include <cmath>

#define FLIGHT_ALTITUDE 1.0f
#define RATE            100 // 频率 hz
#define CIRCLE_RADIUS   1 // 圆的半径 m
#define CIRCLE_DURATION 30  // 完成一圈所花费的时间
#define STEPS           (CIRCLE_DURATION * RATE)
#define VELOCITY_MAGNITUDE (2.0 * M_PI * CIRCLE_RADIUS / CIRCLE_DURATION)

mavros_msgs::State current_state;
mavros_msgs::PositionTarget path[STEPS];

void init_path()
{
    const double dt = 1.0 / RATE;
    const double angular_velocity = (2.0 * M_PI) / CIRCLE_DURATION; // 弧度每秒

    for (int i = 0; i < STEPS; i++) {
        double theta = angular_velocity * i * dt; // 角度
        path[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        path[i].type_mask = 0;

        path[i].position.x = CIRCLE_RADIUS * cos(theta);
        path[i].position.y = CIRCLE_RADIUS * sin(theta);
        path[i].position.z = FLIGHT_ALTITUDE;

        path[i].velocity.x = -VELOCITY_MAGNITUDE * sin(theta);
        path[i].velocity.y = VELOCITY_MAGNITUDE * cos(theta);
        path[i].velocity.z = 0;
        
        path[i].acceleration_or_force.x = -VELOCITY_MAGNITUDE * cos(theta);
        path[i].acceleration_or_force.y = -VELOCITY_MAGNITUDE * sin(theta);
        path[i].acceleration_or_force.z = 0;
    }
}


void print_trajectory()
{
    std::cout << "Trajectory Information:" << std::endl;
    for (int i = 0; i < STEPS; ++i)
    {
        std::cout << "Step " << i + 1 << ":" << std::endl;
        std::cout << "Position: (" << path[i].position.x << ", " << path[i].position.y << ", " << path[i].position.z << ")" << std::endl;
        std::cout << "Velocity: (" << path[i].velocity.x << ", " << path[i].velocity.y << ", " << path[i].velocity.z << ")" << std::endl;
    }
}

void write_traj_to_file() {
    // 获取 HOME 环境变量
    const char* home_path = std::getenv("HOME");
    if (home_path == NULL) {
        ROS_ERROR("Failed to get HOME environment variable");
        return;
    }
    // 将数据写入CSV文件
    std::ofstream file(std::string(home_path) + "/standard_circle.csv");
    if (file.is_open()) {
        for (size_t i = 0; i < STEPS; ++i) {
            file << path[i].position.x << "," << path[i].position.y << "," << path[i].position.z << "\n";
        }
        file.close();
        ROS_INFO("Data has been written to %s/standard_circle.csv", home_path);
    } else {
        ROS_ERROR("Unable to open position_data.csv");
    }
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    int i;

    ros::init(argc,argv,"figure_circle_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Publisher target_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

    ros::Rate rate(RATE);

    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCU...");
    }

    mavros_msgs::PositionTarget position_home;
    position_home.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_home.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    position_home.position.x = 0;
    position_home.position.y = 0;
    position_home.position.z = FLIGHT_ALTITUDE;
    position_home.velocity.x = 0;
    position_home.velocity.y = 0;
    position_home.velocity.z = 0;
    position_home.acceleration_or_force.x = 0;
    position_home.acceleration_or_force.y = 0;
    position_home.acceleration_or_force.z = 0;
    position_home.yaw = 0;
    position_home.yaw_rate = 0;

    init_path();
    print_trajectory();
    // write_traj_to_file();

    for(i=100;ros::ok() && i > 0; --i){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

HOME:
    ROS_WARN("Arming");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_mode_client.call(offb_set_mode);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arming_client.call(arm_cmd);

    while(ros::ok()){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
        if(current_state.mode == "OFFBOARD" && current_state.armed) break;
        else {
            std::cout << current_state.mode << ' ' << current_state.armed << std::endl;
        }
    }

    i = RATE * 5;
    while(ros::ok() && i>0){
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            goto HOME;
        }
        i--;
        target_local_pub.publish(position_home);
        ROS_INFO("Initializing position...");
        ros::spinOnce();
        rate.sleep();
    }

    i=0;
    ROS_INFO("following path");
    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            goto HOME;
        }
        target_local_pub.publish(path[i]);
        i++;
        if(i>=STEPS) i=0;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}