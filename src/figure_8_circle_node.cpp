
// source offboard/devel/setup.bash 
// rosrun offboard figure_8_circle_node 
// make px4_sitl gazebo
// roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"


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
#include <thread>
#include <chrono>

#define FLIGHT_ALTITUDE 1.0f
#define RATE            100  // 频率 hz
#define RADIUS          3   // 绕八运动的半径大小 m
#define CYCLE_S         10  // 完成一次绕八运动所花费的时间
#define STEPS           (CYCLE_S*RATE)

#define PI 3.141592653589793238327950

mavros_msgs::State current_state;
mavros_msgs::PositionTarget path[STEPS];


void init_path()
{
    int i;
    const double dt = 1.0/RATE;
    const double dadt = (2.0*PI)/CYCLE_S;   //角度相对于时间的一阶导数
    const double r = RADIUS;

    for(i=0;i<STEPS;i++)
    {
        path[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        path[i].type_mask = 0;

        
        double a = (-PI/2.0) + i*(2.0*PI/STEPS);
        double c = cos(a);
        double c2a = cos(2.0*a);
        double c4a = cos(4.0*a);
        double c2am3 = c2a - 3.0;
        double s = sin(a);
        double cc = c*c;
        double ss = s*s;
        double sspo = (s*s)+1.0;
        double ssmo = (s*s)-1.0;
        double sspos = sspo * sspo;

        path[i].position.x = (r*c) / sspo;
        path[i].position.y = -(r*c*s) / sspo;
        path[i].position.z = FLIGHT_ALTITUDE;

        path[i].velocity.x = -dadt*r*s*( ss + 2.0f*cc + 1.0f) / sspos;
        path[i].velocity.y = dadt*r*( ss*ss + ss + ssmo*cc) /sspos;
        path[i].velocity.z = 0;

        path[i].acceleration_or_force.x = -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/(c2am3*c2am3*c2am3);
        path[i].acceleration_or_force.y = dadt*dadt*r*((44.0*c2a) + c4a - 21.0)/(c2am3*c2am3*c2am3);
        path[i].acceleration_or_force.z = 0.0;

        // path[i].yaw = atan2(-path[i].velocity.x,path[i].velocity.y) + (PI/2.0f);

        printf("x:%7.3f y:%7.3f yaw:%7.1f\n",path[i].position.x,path[i].position.y,path[i].yaw*180.0f/PI);

    }
    for(i=0;i<STEPS;i++){
        double next = path[(i+1)%STEPS].yaw;
        double curr = path[i].yaw;
        if((next-curr) < -PI) next+=(2.0*PI);
        if((next-curr) > PI) next-=(2.0*PI);
        path[i].yaw_rate = (next-curr)/dt;
    }

    // 获取 HOME 环境变量
    const char* home_path = std::getenv("HOME");
    if (home_path == NULL) {
        ROS_ERROR("Failed to get HOME environment variable");
        return;
    }
    // 将数据写入CSV文件
    std::ofstream file(std::string(home_path) + "/standard8.csv");
    if (file.is_open()) {
        for (size_t i = 0; i < STEPS; ++i) {
            file << path[i].position.x << "," << path[i].position.y << "," << path[i].position.z << "\n";
        }
        file.close();
        ROS_INFO("Data has been written to %s/standard8.csv", home_path);
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

    ros::init(argc,argv,"figure_8_circle_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Publisher target_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
    
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state",10,state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local",10);
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    // ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/uav0/mavros/cmd/land");
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
    // ros::Publisher target_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav0/mavros/setpoint_raw/local",10);

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
    position_home.yaw = (-45.0f + 90.0f) * PI / 180.0f;
    position_home.yaw_rate = 0;

    init_path();

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
        // if(i==STEPS / 2) {
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        // }
        if(i>=STEPS) {
            i=0;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}