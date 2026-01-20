#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("pib_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "right_arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    
    // Enable approximate IK solutions (required for 5-DOF robot)
    arm.setGoalPositionTolerance(0.01);       // 1cm position tolerance
    arm.setGoalOrientationTolerance(3.14159); // Allow any orientation (180 degrees)
    arm.setPlanningTime(10.0);                // Give more time to find solution

    //name of the target pose
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success)
    // {
    //     RCLCPP_INFO(node->get_logger(), "Planning to pose1 successful, executing...");
    //     arm.execute(plan1);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning to pose1 failed");
    // }   
    //---------------------------------------------------------

    // Position Goal (5-DOF robot cannot satisfy full 6D pose)
    arm.setStartStateToCurrentState();
    arm.setPositionTarget(-0.595, -0.118, 0.508);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1) {
        arm.execute(plan1);
    }


    rclcpp::shutdown();
    spinner.join();
    return 0;
}