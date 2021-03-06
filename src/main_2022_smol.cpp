#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std;

enum Status
{
    SETUP = 0,
    READY,
    RUN,
    FINISH
};

enum Mode
{
    EMERGENCY = 0,
    NORMAL
};

enum resistanceState
{
    FIRST_SQUARE = 0,
    THIRD_SQUARE,
    FOURTH_SQUARE,
    FIFTH_SQUARE
};

// Global Variable Define Before Class Define

double mission_waitTime;
double waitTime_Normal;
vector<double> path_tracker_paramDefault;

// Class Define

class Path
{
private:
    double x;
    double y;
    double z;
    double w;
    int pathType;

public:
    Path(double x, double y, double z, double w, int pathType)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
        this->pathType = pathType;
    };
    void update(double x, double y, double z, double w, int pathType)
    {
        if (x != -1)
        {
            this->x = x;
        }
        if (y != -1)
        {
            this->y = y;
        }
        if (z != -1)
        {
            this->z = z;
        }
        if (w != -1)
        {
            this->w = w;
        }
        if (pathType != -1)
        {
            this->pathType = pathType;
        }
    }
    void printOut()
    {
        cout << x << " " << y << " " << z << " " << w << " " << pathType << endl;
    }
    double get_x()
    {
        return x;
    }
    double get_y()
    {
        return y;
    }
    double get_z()
    {
        return z;
    }
    double get_w()
    {
        return w;
    }
    int get_pathType()
    {
        return pathType;
    }
};

class missionPoint
{
private:
    int missionOrder;
    double x;
    double y;
    double z;
    double w;
    char missionType;
    int point;
    int whichHand = -1;
    double vl53Left = 0;
    double vl53Right = 0;
    double vl53_x_offset = 0;
    double vl53_y_offset = 0;

public:
    missionPoint(int missionOrder, double x, double y, double z, double w, char missionType, int point)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
        this->missionType = missionType;
        this->point = point;
        this->missionOrder = missionOrder;
    };
    void changeMissionType(char newMission)
    {
        missionType = newMission;
    }
    void update_VL53(int hand, double left_dis, double right_dis, double x_offset, double y_offset)
    {
        whichHand = hand;
        vl53Left = left_dis;
        vl53Right = right_dis;
        vl53_x_offset = x_offset;
        vl53_y_offset = y_offset;
    }
    void printOut()
    {
        cout << missionOrder << " " << x << " " << y << " " << z << " " << w << " " << missionType << " " << point << " " << whichHand << " " << vl53Left << " " << vl53Right << " " << vl53_x_offset << " " << vl53_y_offset << endl;
    }
    double get_x()
    {
        return x;
    }
    double get_y()
    {
        return y;
    }
    double get_z()
    {
        return z;
    }
    double get_w()
    {
        return w;
    }
    char get_missionType()
    {
        return missionType;
    }
    int get_point()
    {
        return point;
    }
    int get_missionOrder()
    {
        return missionOrder;
    }
    int get_vl53_hand()
    {
        return whichHand;
    }
    double get_vl53_left()
    {
        return vl53Left;
    }
    double get_vl53_right()
    {
        return vl53Right;
    }
    double get_vl53_x_offset()
    {
        return vl53_x_offset;
    }
    double get_vl53_y_offset()
    {
        return vl53_y_offset;
    }
};

class paramSetMission
{
private:
    int missionNum;
    double linear_max_velocity = -1;
    double linear_accelaration = -1;
    double linear_kp = -1;
    double linear_break_ratio = -1;
    double angular_max_velocity = -1;
    double angular_accelaration = -1;
    double angular_kp = -1;
    double angular_brake_distance = -1;
    double xy_tolerance = -1;
    double theta_tolerance = -1;
    double time_adjustment = -1;

public:
    paramSetMission(int mission)
    {
        missionNum = mission;
    }
    void printOut()
    {
        cout << missionNum << " " << linear_max_velocity << " " << linear_accelaration << " " << linear_kp << " " << linear_break_ratio << " " << angular_max_velocity << " " << angular_accelaration << " " << angular_kp << " " << angular_brake_distance << " " << xy_tolerance << " " << theta_tolerance << " " << time_adjustment << endl;
    }
    void updateParam(int whichParam, double adjustmentParam)
    {
        switch (whichParam)
        {
        case 1:
            linear_max_velocity = adjustmentParam;
            break;
        case 2:
            linear_accelaration = adjustmentParam;
            break;
        case 3:
            linear_kp = adjustmentParam;
            break;
        case 4:
            linear_break_ratio = adjustmentParam;
            break;
        case 5:
            angular_max_velocity = adjustmentParam;
            break;
        case 6:
            angular_accelaration = adjustmentParam;
            break;
        case 7:
            angular_kp = adjustmentParam;
            break;
        case 8:
            angular_brake_distance = adjustmentParam;
            break;
        case 9:
            xy_tolerance = adjustmentParam;
            break;
        case 10:
            theta_tolerance = adjustmentParam;
            break;
        case 11:
            time_adjustment = adjustmentParam;
            break;
        }
    }
    int get_missionNum()
    {
        return missionNum;
    }
    void setParam(ros::NodeHandle *nh)
    {
        if (linear_max_velocity != -1)
        {
            nh->setParam("path_tracker/linear_max_velocity", linear_max_velocity);
        }
        if (linear_accelaration != -1)
        {
            nh->setParam("path_tracker/linear_acceleration", linear_accelaration);
        }
        if (linear_kp != -1)
        {
            nh->setParam("path_tracker/linear_kp", linear_kp);
        }
        if (linear_break_ratio != -1)
        {
            nh->setParam("path_tracker/linear_brake_distance_ratio", linear_break_ratio);
        }
        if (angular_max_velocity != -1)
        {
            nh->setParam("path_tracker/angular_max_velocity", angular_max_velocity);
        }
        if (angular_accelaration != -1)
        {
            nh->setParam("path_tracker/angular_acceleration", angular_accelaration);
        }
        if (angular_kp != -1)
        {
            nh->setParam("path_tracker/angular_kp", angular_kp);
        }
        if (angular_brake_distance != -1)
        {
            nh->setParam("path_tracker/angular_brake_distance", angular_brake_distance);
        }
        if (xy_tolerance != -1)
        {
            nh->setParam("path_tracker/xy_tolerance", xy_tolerance);
        }
        if (theta_tolerance != -1)
        {
            nh->setParam("path_tracker/theta_tolerance", theta_tolerance);
        }
    }

    void correctMissionTime()
    {
        mission_waitTime = time_adjustment;
        if (time_adjustment == -1)
        {
            mission_waitTime = waitTime_Normal;
        }
    }
};

// Program Adjustment

// Adjustment Variable Define

const double INI_X_PURPLE = 0;
const double INI_Y_PURPLE = 0;
const double INI_Z_PURPLE = 0;
const double INI_W_PURPLE = 0;

const double INI_X_YELLOW = 0.832;
const double INI_Y_YELLOW = 0.257;
const double INI_Z_YELLOW = -0.263;
const double INI_W_YELLOW = 0.9646;

// Variable Define

int side_state; // 1 for yellow , 2 for purple
int run_state = 0;
bool feedback_activate;
double go_home_time;

int mission_num = 0;
int goal_num = 0;
int now_Status = SETUP;
int now_Mode = NORMAL;
int resistance_state = 0;

bool moving = false;
bool doing = false;
bool finish_mission = false;
bool going_home = false;
bool pid_closed = false;
bool mission_success = false;

double position_x;
double position_y;
double orientation_z;
double orientation_w;
double startMissionTime;
double armAngleBlue = 0;
double armAngleGreen = 0;
double armAngleRed = 0;

int total_point = 0;
int pico_point = 4;
int add_point = 0;

geometry_msgs::PoseStamped next_target;
std_msgs::Float32MultiArray next_docking_goal;
geometry_msgs::Pose2D next_correction;

vector<Path> path_List;
vector<missionPoint> mission_List;
vector<paramSetMission> param_List;
vector<int> delete_List;

tf::Quaternion bblue(0, 0, 0.237214, 0.971457);
tf::Quaternion ggreen(0, 0, 0.9259258, 0.388819);
tf::Quaternion rred(0, 0, -0.7071068, 0.7071068);
double machineAngleBlue = tf::getYaw(bblue);
double machineAngleGreen = tf::getYaw(ggreen);
double machineAngleRed = tf::getYaw(rred);

// Function Define

missionPoint getMission(int num)
{
    for (size_t i = 0; i < mission_List.size(); i++)
    {
        if (num == mission_List[i].get_missionOrder())
        {
            return mission_List[i];
        }
    }
}

missionPoint *getMissionPointer(int num)
{
    for (size_t i = 0; i < mission_List.size(); i++)
    {
        if (num == mission_List[i].get_missionOrder())
        {
            return &mission_List[i];
        }
    }
}

paramSetMission *getParamPointer(int num)
{
    for (size_t i = 0; i < param_List.size(); i++)
    {
        if (num == param_List[i].get_missionNum())
        {
            return &param_List[i];
        }
    }
}

void setVL53Update(int missionC, std_msgs::Float32MultiArray *next)
{
    next->data.clear();
    next->data.push_back(getMission(missionC).get_vl53_hand());
    next->data.push_back(getMission(missionC).get_vl53_left());
    next->data.push_back(getMission(missionC).get_vl53_right());
    next->data.push_back(getMission(missionC).get_vl53_x_offset());
    next->data.push_back(getMission(missionC).get_vl53_y_offset());
}

char getMissionChar(int num)
{
    return getMission(num).get_missionType();
}

void updateMissionChar(int num, char newMission)
{
    getMissionPointer(num)->changeMissionType(newMission);
}

int getMissionPoints(int num)
{
    return getMission(num).get_point();
}

double setAngleTurn(char type)
{
    double angle;
    if (type == 'B')
    {
        angle = armAngleBlue;
    }
    else if (type == 'G')
    {
        angle = armAngleGreen;
    }
    else if (type == 'R')
    {
        angle = armAngleRed;
    }
    else
    {
        angle = 0;
    }
    angle = angle / 3.1415 * 180;
    double angleTemp = angle / 60 - int(angle / 60);
    return -angleTemp * 60;
}

pair<double, double> changePurpleAngle(double ang_z, double ang_w, char which)
{
    pair<double, double> returnAngle;
    tf::Quaternion nnow(0, 0, ang_z, ang_w);
    double machineAngleNow = tf::getYaw(nnow);
    double machineAngleAdjust;
    machineAngleNow *= -1;
    if (which == 'A' || which == 'B' || which == 'a' || which == 'b' || which == 'c' || which == 'C')
    {
        machineAngleAdjust = machineAngleBlue;
    }
    else if (which == 'F' || which == 'G' || which == 'g' || which == 'h' || which == 'H')
    {
        machineAngleAdjust = machineAngleGreen;
    }
    else if (which == 'Q' || which == 'R' || which == 'q' || which == 'r' || which == 's' || which == 'S' || which == '!')
    {
        machineAngleAdjust = machineAngleRed;
    }
    machineAngleNow -= 2 * machineAngleAdjust;
    tf::Quaternion nnew = tf::createQuaternionFromYaw(machineAngleNow);
    returnAngle.first = nnew.getZ();
    returnAngle.second = nnew.getW();
    return returnAngle;
}

void setMissionParam(int which, ros::NodeHandle *nh)
{
    param_List[param_List.size() - 1].setParam(nh);
    for (size_t i = 0; i < param_List.size() - 1; i++)
    {
        if (param_List[i].get_missionNum() == which)
        {
            param_List[i].setParam(nh);
            break;
        }
    }
}

void setMissionTime(int which)
{
    mission_waitTime = waitTime_Normal;
    for (size_t i = 0; i < param_List.size() - 1; i++)
    {
        if (param_List[i].get_missionNum() == which)
        {
            param_List[i].correctMissionTime();
            break;
        }
    }
}

void checkDeleteList()
{
    bool checkFinish = false;

    cout << "Mission No." << path_List[goal_num].get_pathType() << " Check Delete List :" << endl;

    cout << "Delete List Before: ";
    for (size_t i = 0; i < delete_List.size(); i++)
    {
        cout << "No." << delete_List[i] << " ";
    }
    cout << endl;

    while (1)
    {
        if (delete_List.size() == 0 || checkFinish)
        {
            break;
        }
        for (size_t i = 0; i < delete_List.size(); i++)
        {
            if (path_List[goal_num].get_pathType() == delete_List[i])
            {
                cout << "Delete Mission No." << path_List[goal_num].get_pathType() << endl;
                goal_num++;
                mission_num = goal_num;
                while (path_List[goal_num].get_pathType() == 0)
                {
                    goal_num++;
                }
                delete_List.erase(delete_List.begin() + i);
                break;
            }
            if (i == delete_List.size() - 1)
            {
                checkFinish = true;
            }
        }
    }

    cout << "Delete List After: ";
    for (size_t i = 0; i < delete_List.size(); i++)
    {
        cout << "No." << delete_List[i] << " ";
    }
    cout << endl;
    cout << endl;
}

// Node Handling Class Define

class mainProgram
{
public:
    // Callback Function Define

    void position_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        position_x = msg->pose.pose.position.x;
        position_y = msg->pose.pose.position.y;
        orientation_z = msg->pose.pose.orientation.z;
        orientation_w = msg->pose.pose.orientation.w;
    }

    void emergency_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        // if (msg->data)
        // {
        //     now_Mode = EMERGENCY;
        //     std_msgs::Bool publisher;
        //     publisher.data = true;
        //     _StopOrNot.publish(publisher);
        // }
        // else
        // {
        //     now_Mode = NORMAL;
        //     std_msgs::Bool publisher;
        //     publisher.data = false;
        //     _StopOrNot.publish(publisher);
        // }
    }

    void moving_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data && moving && now_Status > READY)
        {
            if (goal_num == path_List.size() - 1 && path_List[goal_num].get_pathType() != 0 && getMissionChar(path_List[goal_num].get_pathType()) == 'X')
            {
                now_Status = FINISH;
            }
            else
            {
                moving = false;
                if (path_List[goal_num].get_pathType() != 0 && getMissionChar(path_List[goal_num].get_pathType()) == 'X')
                {
                    mission_num++;
                    goal_num++;
                    while (path_List[goal_num].get_pathType() == 0)
                    {
                        goal_num++;
                    }
                    checkDeleteList();
                    next_target.pose.position.x = path_List[goal_num].get_x();
                    next_target.pose.position.y = path_List[goal_num].get_y();
                    next_target.pose.orientation.z = path_List[goal_num].get_z();
                    next_target.pose.orientation.w = path_List[goal_num].get_w();
                    next_target.header.frame_id = "map";
                    next_target.header.stamp = ros::Time::now();
                    setMissionParam(path_List[goal_num].get_pathType(), &nh);
                    setVL53Update(path_List[goal_num].get_pathType(), &next_docking_goal);
                    _docking.publish(next_docking_goal);
                    _target.publish(next_target);
                    moving = true;
                    ROS_INFO("[%d] Going to Mission No.%d : Moving to x:[%.3f] y:[%.3f]", goal_num, path_List[goal_num].get_pathType(), path_List[goal_num].get_x(), path_List[goal_num].get_y());
                    cout << endl;
                }
                else
                {
                    doing = true;
                    std_msgs::Char mm;
                    mm.data = getMissionChar(path_List[goal_num].get_pathType());
                    _arm.publish(mm);
                    ROS_INFO("Mission Type Publish : [ %c ]", mm.data);
                    cout << endl;
                    startMissionTime = ros::Time::now().toSec();
                    setMissionTime(path_List[goal_num].get_pathType());
                }
            }
        }
    }

    void resistance_callback(const std_msgs::Int64::ConstPtr &msg) // Team Purple 470 ohm / Team Yellow 1000 ohm / Red Cross 4700 ohm
    {
        switch (resistance_state)
        {
        case FIRST_SQUARE:
            if (msg->data == 0)
            {
                ROS_INFO("FIRST_SQUARE - No");
                resistance_state = 1;
            }
            else if ((msg->data == 470 && side_state == 2) || (msg->data == 1000 && side_state == 1))
            {
                ROS_INFO("FIRST_SQUARE - Our Side");
                delete_List.push_back(path_List[goal_num].get_pathType() + 2);
                delete_List.push_back(path_List[goal_num].get_pathType());
                resistance_state = 2;
            }
            else if (msg->data == 4700)
            {
                ROS_INFO("FIRST_SQUARE - Red Cross");
                updateMissionChar(path_List[goal_num].get_pathType() + 2, 'u');
                getParamPointer(path_List[goal_num].get_pathType() + 2)->updateParam(11, 1);
                delete_List.push_back(path_List[goal_num].get_pathType());
                resistance_state = 2;
            }
            break;
        case THIRD_SQUARE:
            if (msg->data == 0)
            {
                ROS_INFO("THIRD_SQUARE - No");
                resistance_state = 2;
                delete_List.push_back(path_List[goal_num].get_pathType() - 2);
            }
            else if ((msg->data == 470 && side_state == 2) || (msg->data == 1000 && side_state == 1))
            {
                ROS_INFO("THIRD_SQUARE - Our Side");
                resistance_state = 2;
                delete_List.push_back(path_List[goal_num].get_pathType() - 2);
            }
            else if (msg->data == 4700)
            {
                ROS_INFO("THIRD_SQUARE - Red Cross");
                resistance_state = 2;
                updateMissionChar(path_List[goal_num].get_pathType() - 2, 'u');
                getParamPointer(path_List[goal_num].get_pathType() - 2)->updateParam(11, 1);
            }
            break;
        case FOURTH_SQUARE:
            if (msg->data == 0)
            {
                ROS_INFO("FOURTH_SQUARE - No");
                resistance_state = 3;
            }
            else if ((msg->data == 470 && side_state == 2) || (msg->data == 1000 && side_state == 1))
            {
                ROS_INFO("FOURTH_SQUARE - Our Side");
                resistance_state = 4;
                delete_List.push_back(path_List[goal_num].get_pathType() + 1);
                delete_List.push_back(path_List[goal_num].get_pathType() + 2);
            }
            else if ((msg->data == 470 && side_state == 1) || (msg->data == 1000 && side_state == 2))
            {
                ROS_INFO("FOURTH_SQUARE - Enemy Side");
                resistance_state = 4;
                updateMissionChar(path_List[goal_num].get_pathType() + 1, 'u');
                getParamPointer(path_List[goal_num].get_pathType() + 1)->updateParam(11, 1);
                delete_List.push_back(path_List[goal_num].get_pathType() + 3);
            }
            break;
        case FIFTH_SQUARE:
            if (msg->data == 0)
            {
                ROS_INFO("FIFTH_SQUARE - No");
                resistance_state = 4;
            }
            else if ((msg->data == 470 && side_state == 2) || (msg->data == 1000 && side_state == 1))
            {
                ROS_INFO("FIFTH_SQUARE - Our Side");
                resistance_state = 4;
                delete_List.push_back(path_List[goal_num].get_pathType() + 2);
            }
            else if ((msg->data == 470 && side_state == 1) || (msg->data == 1000 && side_state == 2))
            {
                ROS_INFO("FIFTH_SQUARE - Enemy Side");
                resistance_state = 4;
                delete_List.push_back(path_List[goal_num].get_pathType() + 1);
            }
            break;
        }
        cout << endl;
        mission_waitTime = 0;
        mission_success = true;
        ROS_INFO("Mission Success !");
        cout << endl;
    }

    void feedback_callback(const std_msgs::Int64::ConstPtr &msg)
    {
        if (feedback_activate)
        {
            // mission_waitTime = 0;
            if (msg->data)
            {
                pico_point += msg->data;
                ROS_INFO("Mission Success !");
            }
            else
            {
                ROS_INFO("Mission Failed !");
            }
            cout << endl;
        }
    }

    void point_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        add_point = msg->data;
    }

    bool givePath_callback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
    {
        cout << "Mission Path Giving ..." << endl;
        res.plan.poses.clear();
        while (1)
        {
            geometry_msgs::PoseStamped next;
            next.pose.position.x = path_List[mission_num].get_x();
            next.pose.position.y = path_List[mission_num].get_y();
            next.pose.orientation.z = path_List[mission_num].get_z();
            next.pose.orientation.w = path_List[mission_num].get_w();
            cout << "Path : " << path_List[mission_num].get_x() << " " << path_List[mission_num].get_y() << " " << path_List[mission_num].get_z() << " " << path_List[mission_num].get_w() << " " << endl;
            res.plan.poses.push_back(next);
            if (path_List[mission_num].get_pathType() == 0)
            {
                mission_num++;
            }
            else
            {
                break;
            }
        }
        cout << endl;
        return true;
    }

    bool start_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        if (now_Status > READY)
        {
            now_Status = SETUP;
            mission_num = 0;
            goal_num = 0;
            moving = false;
            doing = false;
            finish_mission = false;
            total_point = 0;
            pico_point = 4;
            add_point = 0;
            resistance_state = 0;
        }
        else
        {
            run_state = 1;
        }

        std_srvs::Empty srv;
        if (_teraStart.call(srv))
        {
        }

        return true;
    }

    ros::NodeHandle nh;

    // ROS Topics Publishers
    ros::Publisher _target = nh.advertise<geometry_msgs::PoseStamped>("target", 1000);         // Publish goal to controller
    ros::Publisher _StopOrNot = nh.advertise<std_msgs::Bool>("Stopornot", 1000);               // Publish emergency state to controller
    ros::Publisher _arm = nh.advertise<std_msgs::Char>("arm_go_where", 1000);                  // Publish mission to mission
    ros::Publisher _time = nh.advertise<std_msgs::Float32>("total_Time", 1000);                // Publish total Time
    ros::Publisher _pubpoint = nh.advertise<std_msgs::Int32>("/total_point", 1000);            // Publish total Point
    ros::Publisher _docking = nh.advertise<std_msgs::Float32MultiArray>("docking_goal", 1000); // Publish vl53 goal
    ros::Publisher _shutdown = nh.advertise<std_msgs::Bool>("shutdown", 1000);                 // Publish PID status to base

    // ROS Topics Subscribers
    // ros::Subscriber _globalFilter = nh.subscribe<nav_msgs::Odometry>("global_filter", 1000, &mainProgram::position_callback, this);               // Get position from localization
    ros::Subscriber _globalFilter = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 1000, &mainProgram::position_callback, this); // Get position from localization Lu
    ros::Subscriber _haveObsatcles = nh.subscribe<std_msgs::Bool>("have_obstacles", 1000, &mainProgram::emergency_callback, this);                   // Get emergency state from lidar
    ros::Subscriber _FinishOrNot = nh.subscribe<std_msgs::Bool>("Finishornot", 1000, &mainProgram::moving_callback, this);                           // Get finish moving state from controller
    ros::Subscriber _feedback = nh.subscribe<std_msgs::Int64>("feedback", 1000, &mainProgram::feedback_callback, this);                              // Get feedfback from mission
    ros::Subscriber _subpoint = nh.subscribe<std_msgs::Int32>("/add_point", 1000, &mainProgram::point_callback, this);                               // Get Tera's Point
    ros::Subscriber _resistance = nh.subscribe<std_msgs::Int64>("how_much_resistance", 1000, &mainProgram::resistance_callback, this);               // Get Resistance from mission

    // ROS Service Server
    ros::ServiceServer _MissionPath = nh.advertiseService("MissionPath", &mainProgram::givePath_callback, this);  // Path giving Service
    ros::ServiceServer _RunState = nh.advertiseService("/Pico_startRunning", &mainProgram::start_callback, this); // Start Signal Service

    // ROS Service Client
    ros::ServiceClient _teraStart = nh.serviceClient<std_srvs::Empty>("/Tera_startRunning"); // Call Tera startRunning Service
};

// Main Program

int main(int argc, char **argv)
{
    // ROS initial
    ros::init(argc, argv, "Main_Node_smol");

    // Node Handling Class Initialize

    mainProgram mainClass;
    ros::Time initialTime = ros::Time::now();
    std_msgs::Float32 timePublish;
    std_msgs::Int32 pointPublish;

    // Main Node Update Frequency

    ros::Rate rate(20);

    ifstream inFile;
    string value;
    string line;
    string field;
    string packagePath = ros::package::getPath("main_2022_smol");
    string filename_mission;
    string filename_path;
    string filename_param;
    mainClass.nh.getParam("file_name_mission", filename_mission);
    mainClass.nh.getParam("file_name_path", filename_path);
    mainClass.nh.getParam("file_name_param", filename_param);
    int waitCount = 0;

    while (ros::ok())
    {
        switch (now_Mode)
        {
        case NORMAL:
            switch (now_Status)
            {
            case SETUP:

                mainClass.nh.getParam("side_state", side_state);

                if (side_state == 1)
                {
                    position_x = INI_X_YELLOW;
                    position_y = INI_Y_YELLOW;
                    orientation_z = INI_Z_YELLOW;
                    orientation_w = INI_W_YELLOW;
                }
                else if (side_state == 2)
                {
                    position_x = INI_X_PURPLE;
                    position_y = INI_Y_PURPLE;
                    orientation_z = INI_Z_PURPLE;
                    orientation_w = INI_W_PURPLE;
                }

                // Script Reading

                double next_x;
                double next_y;
                double next_z;
                double next_w;
                char next_m;
                int next_p;
                int next_o;
                int next_vl1;
                double next_vl2;
                double next_vl3;
                double next_vl4;
                double next_vl5;

                cout << endl;
                inFile.open(packagePath + "/include/" + filename_mission);
                cout << "Mission Point CSV File << " << filename_mission << " >> ";
                if (inFile.fail())
                {
                    cout << "Could Not Open !" << endl;
                }
                else
                {
                    cout << "Open Successfully !" << endl;
                }
                cout << endl;
                getline(inFile, line);
                while (getline(inFile, line))
                {
                    istringstream sin(line);

                    getline(sin, field, ',');
                    next_o = atoi(field.c_str());
                    // cout << next_o << endl;

                    getline(sin, field, ',');
                    next_x = atof(field.c_str());
                    // cout << next_x << " ";

                    getline(sin, field, ',');
                    next_y = atof(field.c_str());
                    // cout << next_y << " ";

                    getline(sin, field, ',');
                    next_z = atof(field.c_str());
                    // cout << next_z << " ";

                    getline(sin, field, ',');
                    next_w = atof(field.c_str());
                    // cout << next_w << " ";

                    getline(sin, field, ',');
                    const char *cstr = field.c_str();
                    char b = *cstr;
                    next_m = b;
                    // cout << next_m << " ";

                    getline(sin, field, ',');
                    next_p = atoi(field.c_str());
                    // cout << next_p << " ";

                    missionPoint nextPoint(next_o, next_x, next_y, next_z, next_w, next_m, next_p);

                    getline(sin, field, ',');
                    next_vl1 = atoi(field.c_str());
                    // cout << next_vl1 << " ";

                    if (next_vl1 != -1)
                    {
                        getline(sin, field, ',');
                        next_vl2 = atof(field.c_str());
                        // cout << next_vl2 << " ";

                        getline(sin, field, ',');
                        next_vl3 = atof(field.c_str());
                        // cout << next_vl3 << " ";

                        getline(sin, field, ',');
                        next_vl4 = atof(field.c_str());
                        // cout << next_vl4 << " ";

                        getline(sin, field, ',');
                        next_vl5 = atof(field.c_str());
                        // cout << next_vl5 << " ";

                        nextPoint.update_VL53(next_vl1, next_vl2, next_vl3, next_vl4, next_vl5);
                    }
                    // cout << endl;

                    mission_List.push_back(nextPoint);
                }

                for (size_t i = 0; i < mission_List.size(); i++)
                {
                    mission_List[i].printOut();
                }
                cout << endl;

                inFile.close(); // --------------------------------------------- Change CSV Line ---------------------------------------------

                inFile.open(packagePath + "/include/" + filename_path);
                cout << "Path CSV File << " << filename_path << " >> ";
                if (inFile.fail())
                {
                    cout << "Could Not Open !" << endl;
                }
                else
                {
                    cout << "Open Successfully !" << endl;
                }
                cout << endl;
                while (getline(inFile, line))
                {
                    istringstream sin(line);

                    getline(sin, field, ',');
                    next_o = atoi(field.c_str());
                    // cout << next_o << " ";

                    if (next_o)
                    {
                        next_x = getMission(next_o).get_x();
                        // cout << next_x << " ";
                        next_y = getMission(next_o).get_y();
                        // cout << next_y << " ";
                        next_z = getMission(next_o).get_z();
                        // cout << next_z << " ";
                        next_w = getMission(next_o).get_w();
                        // cout << next_w << endl;
                    }
                    else
                    {
                        getline(sin, field, ',');
                        next_x = atof(field.c_str());
                        // cout << next_x << " ";

                        getline(sin, field, ',');
                        next_y = atof(field.c_str());
                        // cout << next_y << " ";

                        getline(sin, field, ',');
                        next_z = atof(field.c_str());
                        // cout << next_z << " ";

                        getline(sin, field, ',');
                        next_w = atof(field.c_str());
                        // cout << next_w << endl;
                    }

                    Path nextMission(next_x, next_y, next_z, next_w, next_o);
                    path_List.push_back(nextMission);

                    // if (side_state == 1)
                    // {
                    //     Path nextMission(next_x, next_y, next_z, next_w, next_o);
                    //     path_List.push_back(nextMission);
                    // }
                    // else if (side_state == 2)
                    // {
                    //     Path nextMission(next_x, 3 - next_y, changePurpleAngle(next_z, next_w, getMissionChar(next_o)).first, changePurpleAngle(next_z, next_w, getMissionChar(next_o)).second, next_o);
                    //     path_List.push_back(nextMission);
                    // }
                    // cout << next_x << " " << next_y << " " << next_z << " " << next_w << " " << next_m << endl;
                }

                for (size_t i = 0; i < path_List.size(); i++)
                {
                    path_List[i].printOut();
                }
                cout << endl;

                inFile.close(); // --------------------------------------------- Change CSV Line ---------------------------------------------

                inFile.open(packagePath + "/include/" + filename_param);
                cout << "Mission Setparam CSV File << " << filename_param << " >> ";
                if (inFile.fail())
                {
                    cout << "Could Not Open !" << endl;
                }
                else
                {
                    cout << "Open Successfully !" << endl;
                }
                cout << endl;

                double next_pp;
                getline(inFile, line);
                while (getline(inFile, line))
                {
                    istringstream sin(line);

                    getline(sin, field, ',');
                    next_o = atoi(field.c_str());
                    // cout << next_o << " ";

                    paramSetMission next_psm(next_o);
                    param_List.push_back(next_psm);

                    for (int i = 1; i <= 11; i++)
                    {
                        getline(sin, field, ',');
                        next_pp = atof(field.c_str());
                        // cout << "[" << next_pp << "] ";

                        if (next_pp != 0)
                        {
                            param_List.at(param_List.size() - 1).updateParam(i, next_pp);
                        }
                    }
                    // cout << endl;
                }

                for (size_t i = 0; i < param_List.size(); i++)
                {
                    param_List[i].printOut();
                }
                cout << endl;

                mainClass.nh.getParam("mission_waitTime", waitTime_Normal);
                mainClass.nh.getParam("feedback_activate", feedback_activate);
                mainClass.nh.getParam("go_home_time", go_home_time);

                now_Status = READY;
                break;

            case READY:
                if (run_state)
                {
                    run_state = 0;
                    std_msgs::Bool bb;
                    bb.data = false;
                    mainClass._shutdown.publish(bb);
                    if (path_List.size() == 0)
                    {
                        now_Status = FINISH;
                    }
                    else
                    {
                        double next_param;
                        mainClass.nh.getParam("path_tracker/linear_max_velocity", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/linear_acceleration", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/linear_kp", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/linear_brake_distance_ratio", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/angular_max_velocity", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/angular_acceleration", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/angular_kp", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/angular_brake_distance", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/xy_tolerance", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        mainClass.nh.getParam("path_tracker/theta_tolerance", next_param);
                        path_tracker_paramDefault.push_back(next_param);
                        paramSetMission defaultMission(-1);
                        param_List.push_back(defaultMission);
                        for (int i = 1; i < 11; i++)
                        {
                            param_List.at(param_List.size() - 1).updateParam(i, path_tracker_paramDefault[i - 1]);
                        }

                        initialTime = ros::Time::now();
                        while (path_List[goal_num].get_pathType() == 0)
                        {
                            goal_num++;
                        }
                        checkDeleteList();
                        next_target.pose.position.x = path_List[goal_num].get_x();
                        next_target.pose.position.y = path_List[goal_num].get_y();
                        next_target.pose.orientation.z = path_List[goal_num].get_z();
                        next_target.pose.orientation.w = path_List[goal_num].get_w();
                        next_target.header.frame_id = "map";
                        next_target.header.stamp = ros::Time::now();
                        setMissionParam(path_List[goal_num].get_pathType(), &mainClass.nh);
                        setVL53Update(path_List[goal_num].get_pathType(), &next_docking_goal);
                        mainClass._docking.publish(next_docking_goal);
                        mainClass._target.publish(next_target);
                        moving = true;
                        ROS_INFO("[%d] Going to Mission No.%d : Moving to x:[%.3f] y:[%.3f]", goal_num, path_List[goal_num].get_pathType(), path_List[goal_num].get_x(), path_List[goal_num].get_y());
                        cout << endl;
                    }
                    now_Status = RUN;
                }
                else
                {
                    if (waitCount++ > 20)
                    {
                        ROS_INFO("Pico Waiting Now...");
                        cout << endl;
                        waitCount = 0;
                    }
                }
                break;

            case RUN:

                if (moving && !doing)
                {
                    // Moving to Target Point
                }
                else if (doing && !moving)
                {
                    if (ros::Time::now().toSec() - startMissionTime < mission_waitTime)
                    {
                        // Doing Mission Wait Time
                    }
                    else
                    {
                        doing = false;

                        // if (mission_waitTime != 0)
                        // {
                        //     mission_success = false;
                        //     ROS_INFO("Mission Time %.1f seconds is Up !", mission_waitTime);
                        // }

                        // if (mission_success)
                        // {
                        //     pico_point += getMissionPoints(path_List[goal_num].get_pathType());
                        //     mission_success = false;
                        // }

                        if (goal_num == path_List.size() - 1)
                        {
                            now_Status = FINISH;
                        }
                        else
                        {
                            mission_num++;
                            goal_num++;
                            while (path_List[goal_num].get_pathType() == 0)
                            {
                                goal_num++;
                            }
                            checkDeleteList();
                            next_target.pose.position.x = path_List[goal_num].get_x();
                            next_target.pose.position.y = path_List[goal_num].get_y();
                            next_target.pose.orientation.z = path_List[goal_num].get_z();
                            next_target.pose.orientation.w = path_List[goal_num].get_w();
                            next_target.header.frame_id = "map";
                            next_target.header.stamp = ros::Time::now();
                            setMissionParam(path_List[goal_num].get_pathType(), &mainClass.nh);
                            setVL53Update(path_List[goal_num].get_pathType(), &next_docking_goal);
                            mainClass._docking.publish(next_docking_goal);
                            mainClass._target.publish(next_target);
                            moving = true;
                            ROS_INFO("[%d] Going to Mission No.%d : Moving to x:[%.3f] y:[%.3f]", goal_num, path_List[goal_num].get_pathType(), path_List[goal_num].get_x(), path_List[goal_num].get_y());
                            cout << endl;
                        }
                    }
                }

                if (ros::Time::now().toSec() - initialTime.toSec() > go_home_time && !going_home)
                {
                    going_home = true;
                    mission_num = path_List.size() - 1;
                    goal_num = path_List.size() - 1;
                    next_target.pose.position.x = path_List[goal_num].get_x();
                    next_target.pose.position.y = path_List[goal_num].get_y();
                    next_target.pose.orientation.z = path_List[goal_num].get_z();
                    next_target.pose.orientation.w = path_List[goal_num].get_w();
                    next_target.header.frame_id = "map";
                    next_target.header.stamp = ros::Time::now();
                    setMissionParam(path_List[goal_num].get_pathType(), &mainClass.nh);
                    setVL53Update(path_List[goal_num].get_pathType(), &next_docking_goal);
                    mainClass._docking.publish(next_docking_goal);
                    mainClass._target.publish(next_target);
                    ROS_INFO("Time to Go Home !!! : Moving to x:[%.3f] y:[%.3f]", path_List[goal_num].get_x(), path_List[goal_num].get_y());
                    cout << endl;
                }

                timePublish.data = ros::Time::now().toSec() - initialTime.toSec();
                mainClass._time.publish(timePublish);

                total_point = add_point + pico_point;
                pointPublish.data = total_point;
                mainClass._pubpoint.publish(pointPublish);

                if (ros::Time::now().toSec() - initialTime.toSec() > 99.85)
                {
                    now_Status = FINISH;
                    ROS_INFO("Time Up ! Close All Things !");
                    cout << endl;
                }

                break;

            case FINISH:
                if (!finish_mission)
                {
                    timePublish.data = ros::Time::now().toSec() - initialTime.toSec();
                    ROS_INFO("Mission Time: %f", timePublish.data);
                    mainClass._time.publish(timePublish);

                    total_point = pico_point + add_point + 20;
                    pointPublish.data = total_point;
                    ROS_INFO("Total Point: %d", pointPublish.data);
                    mainClass._pubpoint.publish(pointPublish);
                    cout << endl;

                    ROS_INFO("Finish All Mission");
                    finish_mission = true;
                    cout << endl;
                }

                if (ros::Time::now().toSec() - initialTime.toSec() > 100)
                {
                    if (!pid_closed)
                    {
                        std_msgs::Bool turnoff;
                        turnoff.data = true;
                        mainClass._shutdown.publish(turnoff);
                        pid_closed = true;
                    }

                    std_msgs::Char cc;
                    cc.data = '@';
                    mainClass._arm.publish(cc);
                }

                total_point = pico_point + add_point + 20;
                pointPublish.data = total_point;
                mainClass._pubpoint.publish(pointPublish);

                break;
            }
            break;

        case EMERGENCY:
            ROS_INFO("Emergency State");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
