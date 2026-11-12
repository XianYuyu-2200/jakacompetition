#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "jaka_msgs/srv/set_io.hpp"

#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jkerr.h"
#include "jaka_planner/jktypes.h"

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <string>
#include <map>
#include <thread>
using namespace std;

JAKAZuRobot robot;
const double PI = 3.1415926;
// bool in_pos;
// int ret_preempt;
// int ret_inPos;

typedef rclcpp_action::Server<control_msgs::action::FollowJointTrajectory> Server;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

// Map error codes to messages
map<int, string> mapErr = {
    {2,   "ERR_FUCTION_CALL_ERROR"},
    {-1,  "ERR_INVALID_HANDLER"},
    {-2,  "ERR_INVALID_PARAMETER"},
    {-3,  "ERR_COMMUNICATION_ERR"},
    {-4,  "ERR_KINE_INVERSE_ERR"},
    {-5,  "ERR_EMERGENCY_PRESSED"},
    {-6,  "ERR_NOT_POWERED"},
    {-7,  "ERR_NOT_ENABLED"},
    {-8,  "ERR_DISABLE_SERVOMODE"},
    {-9,  "ERR_NOT_OFF_ENABLE"},
    {-10, "ERR_PROGRAM_IS_RUNNING"},
    {-11, "ERR_CANNOT_OPEN_FILE"},
    {-12, "ERR_MOTION_ABNORMAL"}
};

// Determine if the robot has reached the target position.
bool jointStates(const JointValue &joint_pose)
{
    // RobotStatus robotstatus;
    JointValue joint_position;
    // robot.get_robot_status(&robotstatus);
    robot.get_joint_position(&joint_position);

    bool joint_state = true;
    for (int i = 0; i < 6; i++)
    {
        // double actual_deg = robotstatus.joint_position[i] * 180.0 / PI;
        double actual_deg = joint_position.jVal[i] * 180.0 / PI;
        double target_deg = joint_pose.jVal[i] * 180.0 / PI;
        // Within +/- 0.2 degrees tolerance
        bool ret = (target_deg - 0.2 < actual_deg) && (actual_deg < target_deg + 0.2);
        joint_state = joint_state && ret;
    }

    RCLCPP_INFO(rclcpp::get_logger("jointStates"), 
                "Whether the robot has reached the target position: %d", joint_state);
    return joint_state;
}

// Hand-drag teleop. Point button on the flange toggles the NPN gripper.
bool g_teleop = false;
bool g_gripper_open = false;
int g_last_point = 0;
bool g_point_ready = false;
chrono::steady_clock::time_point g_last_toggle =
    chrono::steady_clock::now() - chrono::seconds(1);
chrono::steady_clock::time_point g_last_poll =
    chrono::steady_clock::now() - chrono::seconds(1);

bool set_tool_do(int index, bool on)
{
    int ret = robot.set_digital_output(IO_TOOL, index, on ? TRUE : FALSE);
    if (ret != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("gripper"),
                     "工具端 DO_%d=%d 失败, ret=%d", index + 1, on ? 1 : 0, ret);
        return false;
    }
    return true;
}

// DO_1 (index 0) holds open, DO_2 (index 1) holds close. Never both.
bool gripper_set_open(bool open)
{
    if (open)
    {
        if (!set_tool_do(1, false) || !set_tool_do(0, true))
            return false;
    }
    else
    {
        if (!set_tool_do(0, false) || !set_tool_do(1, true))
            return false;
    }
    g_gripper_open = open;
    RCLCPP_INFO(rclcpp::get_logger("gripper"),
                open ? "夹爪张开 (DO_1=1, DO_2=0)" : "夹爪闭合 (DO_2=1, DO_1=0)");
    return true;
}

void init_gripper_latch()
{
    BOOL open_do = FALSE;
    BOOL close_do = FALSE;
    int r0 = robot.get_digital_output(IO_TOOL, 0, &open_do);
    int r1 = robot.get_digital_output(IO_TOOL, 1, &close_do);
    g_gripper_open = (r0 == 0 && open_do && !(r1 == 0 && close_do));
    RCLCPP_INFO(rclcpp::get_logger("gripper"),
                "夹爪当前视为%s (DO_1=%d, DO_2=%d). 末端 Point 按一下切换.",
                g_gripper_open ? "张开" : "闭合",
                r0 == 0 ? (int)open_do : -1,
                r1 == 0 ? (int)close_do : -1);
}

// tio_key[1] is the flange Point button: level 0/1, or a click counter.
void poll_point_button()
{
    auto now = chrono::steady_clock::now();
    if (now - g_last_poll < chrono::milliseconds(40))
        return;
    g_last_poll = now;

    RobotStatus status{};
    if (robot.get_robot_status(&status) != 0)
        return;

    int point = status.tio_key[1];
    if (!g_point_ready)
    {
        g_last_point = point;
        g_point_ready = true;
        RCLCPP_INFO(rclcpp::get_logger("flange"),
                    "末端按键 free=%d point=%d pause=%d",
                    status.tio_key[0], status.tio_key[1], status.tio_key[2]);
        return;
    }
    if (point == g_last_point)
        return;

    RCLCPP_INFO(rclcpp::get_logger("flange"),
                "末端按键 free=%d point=%d pause=%d",
                status.tio_key[0], point, status.tio_key[2]);
    bool click = point != 0 && (g_last_point == 0 || point > g_last_point);
    g_last_point = point;
    if (!click)
        return;
    if (now - g_last_toggle < chrono::milliseconds(250))
        return;
    g_last_toggle = now;
    gripper_set_open(!g_gripper_open);
}

void drag_mode_callback(const shared_ptr<std_srvs::srv::SetBool::Request> request,
                        shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    BOOL enable = request->data ? TRUE : FALSE;
    int ret = robot.drag_mode_enable(enable);
    if (ret != 0)
    {
        response->success = false;
        response->message = "drag_mode_enable failed: " + to_string(ret);
        RCLCPP_ERROR(rclcpp::get_logger("teleop"), "%s", response->message.c_str());
        return;
    }
    g_teleop = request->data;
    response->success = true;
    response->message = request->data ? "手拖遥操作已打开" : "手拖遥操作已关闭";
    RCLCPP_INFO(rclcpp::get_logger("teleop"), "%s", response->message.c_str());
}

// Servo trajectories and drag mode cannot run together.
struct DragPause
{
    bool resume;
    DragPause() : resume(g_teleop)
    {
        if (!resume)
            return;
        robot.drag_mode_enable(FALSE);
        RCLCPP_INFO(rclcpp::get_logger("goalCb"), "轨迹开始, 暂时退出手拖");
    }
    ~DragPause()
    {
        if (!resume)
            return;
        robot.servo_move_enable(FALSE);
        robot.drag_mode_enable(TRUE);
        RCLCPP_INFO(rclcpp::get_logger("goalCb"), "轨迹结束, 恢复手拖");
    }
};

// Handle a new FollowJointTrajectory goal
void goalCb(const shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    DragPause pause_drag;

    // Enable servo mode
    robot.servo_move_enable(true);

    // Retrieve the trajectory from the goal
    auto goal = goal_handle->get_goal();
    const auto &traj = goal->trajectory;
    int point_num = traj.points.size();
    RCLCPP_INFO(rclcpp::get_logger("goalCb"), "number of points: %d", point_num);

    if (point_num == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("goalCb"), "Trajectory has no points. Aborting goal.");
        goal_handle->abort(make_shared<control_msgs::action::FollowJointTrajectory::Result>());
        return;
    }

    float lastDuration = 0.0;
    JointValue joint_pose;

    for (int i = 1; i < point_num; i++)
    {
        // Grab the positions from the i-th trajectory point
        for (int j = 0; j < 6; j++) {
            joint_pose.jVal[j] = traj.points[i].positions[j];
        }

        // Convert time_from_start to a float seconds
        float Duration = static_cast<float>(traj.points[i].time_from_start.sec) +
                         static_cast<float>(traj.points[i].time_from_start.nanosec) * 1e-9;

        // Calculate time delta relative to previous point
        float dt = Duration - lastDuration;
        lastDuration = Duration;

        // step_num matches old ROS1 logic: step_num = dt / 0.008
        int step_num = static_cast<int>(dt / 0.008f);
        step_num = max(step_num, 1);

        int sdk_res = robot.servo_j(&joint_pose, MoveMode::ABS, step_num);
        if (sdk_res != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo_j Motion Failed");
        }

        RCLCPP_INFO(rclcpp::get_logger("goalCb"), "The return status of servo_j: %d", sdk_res);
        RCLCPP_INFO(rclcpp::get_logger("goalCb"), 
                    "For point no.: %d, Accepted joint angle: %f %f %f %f %f %f, dt=%f, step_num=%d", i,
                    joint_pose.jVal[0], joint_pose.jVal[1], joint_pose.jVal[2],
                    joint_pose.jVal[3], joint_pose.jVal[4], joint_pose.jVal[5],
                    dt, step_num);

        // // Check if the action was canceled in the middle
        // if (goal_handle->is_canceling())
        // {
        //     // stop motion
        //     robot.motion_abort();
        //     robot.servo_move_enable(false);
        //     RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo Mode Disable, motion canceled");
        //     auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        //     goal_handle->canceled(result);
        //     return;
        // }
    }

    // Wait until the robot is actually at the final position, or until canceled
    while (rclcpp::ok())
    {
        if (jointStates(joint_pose))
        {
            robot.servo_move_enable(false);
            RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo Mode Disable: Target Reached");
            RCLCPP_INFO(rclcpp::get_logger("goalCb"), 
                        "==============Motion stops or reaches the target position==============");
            break;
        }

        // if (goal_handle->is_canceling())
        // {
        //     robot.motion_abort();
        //     robot.servo_move_enable(false);
        //     RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo Mode Disable");
        //     RCLCPP_INFO(rclcpp::get_logger("goalCb"), 
        //                 "==============Motion stops or was canceled==============");
        //     auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        //     goal_handle->canceled(result);
        //     return;
        // }

        rclcpp::sleep_for(chrono::milliseconds(500));
    }

    // // After processing all points, check if the goal was canceled
    // if (goal_handle->is_canceling()) {
    //     robot.motion_abort();
    //     robot.servo_move_enable(false);
    //     RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo Mode Disabled, Motion Canceled");
    //     auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    //     goal_handle->canceled(result);
    //     return;
    // }

    // If we get here, it succeeded
    auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    goal_handle->succeed(result);
    rclcpp::sleep_for(chrono::milliseconds(500));
}

// Publish the robot's joint states to /joint_states (for RViz / MoveIt feedback)
void joint_states_callback(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &joint_states_pub)
{
    sensor_msgs::msg::JointState joint_msg;
    // RobotStatus robotstatus;
    JointValue joint_position;
    // robot.get_robot_status(&robotstatus);
    robot.get_joint_position(&joint_position);

    for (int i = 0; i < 6; i++)
    {
        // joint_msg.position.push_back(robotstatus.joint_position[i]);
        joint_msg.position.push_back(joint_position.jVal[i]);
        joint_msg.name.push_back("joint_" + to_string(i+1));
    }
    joint_msg.header.stamp = rclcpp::Clock().now();
    joint_states_pub->publish(joint_msg);
}

void sigintHandler(int /*sig*/) {
    rclcpp::shutdown();
}

// Tool DO for the real gripper. Pendant (工具端, NPN): DO_1 opens, DO_2 closes.
// Same SetIO contract as jaka_driver/set_io, but on the planner's existing SDK
// login — jaka_driver cannot be started alongside this node.
bool set_io_callback(const shared_ptr<jaka_msgs::srv::SetIO::Request> request,
                     shared_ptr<jaka_msgs::srv::SetIO::Response> response)
{
    IOType type;
    switch (request->type)
    {
        case 0: type = IO_CABINET; break;
        case 1: type = IO_TOOL; break;
        case 2: type = IO_EXTEND; break;
        default:
            response->ret = 0;
            response->message = "Invalid IO type";
            return false;
    }

    int ret;
    if (request->signal == "digital")
    {
        BOOL digital_value = request->value ? TRUE : FALSE;
        ret = robot.set_digital_output(type, request->index, digital_value);
    }
    else if (request->signal == "analog")
    {
        ret = robot.set_analog_output(type, request->index, request->value);
    }
    else
    {
        response->ret = 0;
        response->message = "Invalid signal type";
        return false;
    }

    if (ret == 0)
    {
        response->ret = 1;
        response->message = "set IO has been executed";
        return true;
    }
    response->ret = 0;
    auto it = mapErr.find(ret);
    response->message = "error occurred:" + (it == mapErr.end() ? to_string(ret) : it->second);
    return false;
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    auto node = rclcpp::Node::make_shared("moveit_server");

    // Read parameters
    string default_ip = "10.5.5.100";
    string default_model = "zu3";
    string robot_ip = node->declare_parameter("ip", default_ip);
    string robot_model = node->declare_parameter("model", default_model);
    bool teleop = node->declare_parameter("teleop", false);

    // Connect to robot
    robot.login_in(robot_ip.c_str(), false);
    rclcpp::Rate rate(125);

    // Turn off servo at startup
    robot.servo_move_enable(false);
    rclcpp::sleep_for(chrono::milliseconds(500));

    // Filter param
    robot.servo_move_use_joint_LPF(0.5);

    // Power on + enable
    robot.power_on();
    rclcpp::sleep_for(chrono::seconds(8));
    robot.enable_robot();
    rclcpp::sleep_for(chrono::seconds(4));

    // Publisher for /joint_states
    joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Gripper: tool DO. UI DO_1 (index 0) = open, DO_2 (index 1) = close, both NPN.
    auto set_io_service = node->create_service<jaka_msgs::srv::SetIO>(
        "jaka_planner/set_io", &set_io_callback);
    (void)set_io_service;
    RCLCPP_INFO(rclcpp::get_logger("moveit_server"),
                "tool IO service /jaka_planner/set_io ready (NPN DO_1 open, DO_2 close)");

    init_gripper_latch();
    auto drag_service = node->create_service<std_srvs::srv::SetBool>(
        "jaka_planner/drag_mode", &drag_mode_callback);
    (void)drag_service;
    if (teleop)
    {
        int ret = robot.drag_mode_enable(TRUE);
        if (ret == 0)
        {
            g_teleop = true;
            RCLCPP_INFO(rclcpp::get_logger("teleop"),
                        "手拖遥操作已打开: 可以直接掰机械臂. 末端 Point 一下张开, 再一下闭合.");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("teleop"),
                         "drag_mode_enable 失败, ret=%d. Point 键仍会切换夹爪.", ret);
        }
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("teleop"),
                    "未开手拖. 需要时: ros2 service call /jaka_planner/drag_mode std_srvs/srv/SetBool \"{data: true}\"");
    }

    // Create Action Server for FollowJointTrajectory
    auto moveit_server = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        node,
        "/jaka_" + robot_model + "_controller/follow_joint_trajectory",
        // Goal callback
        [](const rclcpp_action::GoalUUID &uuid,
           shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
            RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "Received goal request");
            (void)uuid; // Avoid unused parameter warning
            (void)goal; // Avoid unused parameter warning
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // // Cancel callback
        // [](const shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
        //     (void)goal_handle;  // Avoid unused parameter warning
        //     RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "Received cancel request");
        //     return rclcpp_action::CancelResponse::ACCEPT;
        // },
        nullptr,  // Cancel callback removed
        // Execute callback
        [](const shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
            RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "Executing goal");
            goalCb(goal_handle); 
        }
    );

    RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "==================Moveit Start==================");

    // Main spin loop
    while (rclcpp::ok())
    {
        // Publish joint states at ~125Hz
        joint_states_callback(joint_states_pub);
        poll_point_button();
        rate.sleep();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
