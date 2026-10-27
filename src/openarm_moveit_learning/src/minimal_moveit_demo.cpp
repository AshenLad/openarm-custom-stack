#include <memory>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

namespace
{

    constexpr char kGroupName[] = "left_arm";

    // 精度容差（按实际机器人调整）
    constexpr double kPlanPosTolerance = 5e-3; // 执行前 FK 校验：规划落点 vs 目标
    constexpr double kExecPosTolerance = 5e-3; // 执行后位置误差
    constexpr double kExecOriTolerance = 0.05; // 执行后姿态误差（约 2.9 deg）

    // 关节目标 demo：joint1 +0.1 rad → 规划 → 打印轨迹信息 → 执行
    int run_joint_demo(
        moveit::planning_interface::MoveGroupInterface &move_group,
        rclcpp::Logger logger)
    {
        const std::string group_name = move_group.getName();

        auto current_state = move_group.getCurrentState(2.0);
        if (!current_state)
        {
            RCLCPP_ERROR(logger, "Failed to get current robot state");
            return 1;
        }

        const auto *joint_group =
            current_state->getRobotModel()->getJointModelGroup(group_name);
        if (!joint_group)
        {
            RCLCPP_ERROR(logger, "Group '%s' not found in robot model", group_name.c_str());
            return 1;
        }

        std::vector<double> positions;
        current_state->copyJointGroupPositions(joint_group, positions);
        // 7 自由度机械臂的断言，按你的机器人调整
        if (positions.size() != 7)
        {
            RCLCPP_ERROR(logger, "Unexpected joint count: %zu", positions.size());
            return 1;
        }

        positions[0] += 0.1;
        RCLCPP_INFO(logger, "Target joint1: %.6f rad", positions[0]);

        move_group.setStartStateToCurrentState();
        if (!move_group.setJointValueTarget(positions))
        {
            RCLCPP_ERROR(logger, "Joint target is invalid or out of bounds");
            return 1;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = move_group.plan(plan);
        if (!plan_result)
        {
            RCLCPP_ERROR(logger, "Planning failed, error code: %d", plan_result.val);
            return 1;
        }

        const auto &traj = plan.trajectory.joint_trajectory;
        if (traj.points.empty())
        {
            RCLCPP_ERROR(logger, "Plan succeeded but trajectory is empty");
            return 1;
        }

        const auto &first = traj.points.front();
        const auto &last = traj.points.back();
        const auto &names = traj.joint_names;

        RCLCPP_INFO(logger, "Planning succeeded!");
        RCLCPP_INFO(logger, "Trajectory points: %zu", traj.points.size());
        RCLCPP_INFO(logger, "Planning computation time: %.3f s", plan.planning_time);
        RCLCPP_INFO(logger, "Trajectory duration: %.3f s",
                    rclcpp::Duration(last.time_from_start).seconds());

        if (first.positions.size() == names.size() &&
            last.positions.size() == names.size())
        {
            for (std::size_t i = 0; i < names.size(); ++i)
            {
                RCLCPP_INFO(logger, "%s: start=%.6f, goal=%.6f",
                            names[i].c_str(), first.positions[i], last.positions[i]);
            }
        }

        if (!first.velocities.empty() && !last.velocities.empty() &&
            !first.accelerations.empty() && !last.accelerations.empty())
        {
            RCLCPP_INFO(logger, "Joint1 velocity: start=%.6f, end=%.6f rad/s",
                        first.velocities[0], last.velocities[0]);
            RCLCPP_INFO(logger, "Joint1 acceleration: start=%.6f, end=%.6f rad/s^2",
                        first.accelerations[0], last.accelerations[0]);
        }

        RCLCPP_INFO(logger, "Starting trajectory execution");
        const auto exec_result = move_group.execute(plan);
        if (!exec_result)
        {
            RCLCPP_ERROR(logger, "Execution failed, error code: %d", exec_result.val);
            return 1;
        }
        RCLCPP_INFO(logger, "Execution succeeded!");
        return 0;
    }

    // 位姿目标 demo：末端 z +1 cm → 规划 → FK 校验 → 执行 → 实测精度
    int run_pose_demo(
        moveit::planning_interface::MoveGroupInterface &move_group,
        rclcpp::Logger logger)
    {
        const auto eef_link = move_group.getEndEffectorLink();
        if (eef_link.empty())
        {
            RCLCPP_ERROR(logger, "No end effector link configured");
            return 1;
        }

        auto current_state = move_group.getCurrentState(2.0);
        if (!current_state)
        {
            RCLCPP_ERROR(logger, "Failed to get current robot state");
            return 1;
        }

        const auto pose = move_group.getCurrentPose(eef_link);
        if (pose.header.frame_id.empty())
        {
            RCLCPP_ERROR(logger, "getCurrentPose returned empty frame (state not ready?)");
            return 1;
        }

        RCLCPP_INFO(logger, "Pose frame: %s", pose.header.frame_id.c_str());
        RCLCPP_INFO(logger, "Position: x=%.4f, y=%.4f, z=%.4f",
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        RCLCPP_INFO(logger, "Orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
                    pose.pose.orientation.x, pose.pose.orientation.y,
                    pose.pose.orientation.z, pose.pose.orientation.w);

        // 目标：当前位姿沿 z 抬高 1 cm（姿态不变，姿态误差只反映跟踪精度）
        auto target_pose = pose;
        target_pose.pose.position.z += 0.01;

        move_group.setStartStateToCurrentState();
        if (!move_group.setPoseTarget(target_pose, eef_link))
        {
            RCLCPP_ERROR(logger, "Failed to set pose target");
            return 1;
        }

        RCLCPP_INFO(logger, "Target position: x=%.4f, y=%.4f, z=%.4f",
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z);

        moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
        const auto plan_result = move_group.plan(pose_plan);
        if (!plan_result)
        {
            RCLCPP_ERROR(logger, "Pose planning failed, error code: %d", plan_result.val);
            return 1;
        }

        const auto &traj = pose_plan.trajectory.joint_trajectory;
        RCLCPP_INFO(logger, "Pose planning succeeded (points: %zu, planning time: %.3f s)",
                    traj.points.size(), pose_plan.planning_time);

        if (traj.points.empty() ||
            traj.joint_names.size() != traj.points.back().positions.size())
        {
            RCLCPP_ERROR(logger, "Invalid planned trajectory");
            return 1;
        }

        // ---- 执行前：用正运动学验证规划终点是否落点达标 ----
        moveit::core::RobotState final_state(*current_state);
        final_state.setVariablePositions(traj.joint_names, traj.points.back().positions);
        final_state.update();

        const Eigen::Vector3d fk_pos =
            final_state.getGlobalLinkTransform(eef_link).translation();

        // FK 结果在模型坐标系下，把目标位置也换算到该坐标系再比较
        Eigen::Vector3d target_pos(
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z);
        const std::string model_frame =
            current_state->getRobotModel()->getModelFrame();
        if (target_pose.header.frame_id != model_frame)
        {
            bool frame_found = false;
            const Eigen::Isometry3d &frame_tf =
                current_state->getFrameTransform(target_pose.header.frame_id, &frame_found);
            if (!frame_found)
            {
                RCLCPP_ERROR(logger, "Frame '%s' not found in robot model",
                             target_pose.header.frame_id.c_str());
                return 1;
            }
            target_pos = frame_tf * target_pos;
        }

        const double plan_error = (fk_pos - target_pos).norm();
        RCLCPP_INFO(logger, "Planned FK position: x=%.6f, y=%.6f, z=%.6f (error: %.6f m)",
                    fk_pos.x(), fk_pos.y(), fk_pos.z(), plan_error);

        if (plan_error > kPlanPosTolerance)
        {
            RCLCPP_ERROR(logger,
                         "Planned solution deviates from target by %.6f m (> %.6f), "
                         "aborting before execution",
                         plan_error, kPlanPosTolerance);
            return 1;
        }

        // ---- 执行 ----
        RCLCPP_INFO(logger, "Starting pose target execution");
        const auto exec_result = move_group.execute(pose_plan);
        if (!exec_result)
        {
            RCLCPP_ERROR(logger, "Pose execution failed, error code: %d", exec_result.val);
            return 1;
        }
        RCLCPP_INFO(logger, "Pose execution succeeded!");

        // ---- 执行后：实测精度 ----
        const auto final_pose = move_group.getCurrentPose(eef_link);
        if (final_pose.header.frame_id != target_pose.header.frame_id)
        {
            RCLCPP_ERROR(logger, "Pose frames do not match (%s vs %s)",
                         final_pose.header.frame_id.c_str(),
                         target_pose.header.frame_id.c_str());
            return 1;
        }

        const auto &p = final_pose.pose.position;
        const auto &g = target_pose.pose.position;
        const double position_error =
            Eigen::Vector3d(p.x - g.x, p.y - g.y, p.z - g.z).norm();

        const Eigen::Quaterniond q_final(
            final_pose.pose.orientation.w, final_pose.pose.orientation.x,
            final_pose.pose.orientation.y, final_pose.pose.orientation.z);
        const Eigen::Quaterniond q_target(
            target_pose.pose.orientation.w, target_pose.pose.orientation.x,
            target_pose.pose.orientation.y, target_pose.pose.orientation.z);
        const double orientation_error =
            q_final.normalized().angularDistance(q_target.normalized());

        RCLCPP_INFO(logger, "Final position: x=%.6f, y=%.6f, z=%.6f", p.x, p.y, p.z);
        RCLCPP_INFO(logger, "Position error: %.6f m", position_error);
        RCLCPP_INFO(logger, "Orientation error: %.6f rad", orientation_error);

        if (position_error > kExecPosTolerance ||
            orientation_error > kExecOriTolerance)
        {
            RCLCPP_ERROR(logger,
                         "Execution accuracy out of tolerance: position %.6f m (limit %.6f), "
                         "orientation %.6f rad (limit %.6f)",
                         position_error, kExecPosTolerance,
                         orientation_error, kExecOriTolerance);
            return 1;
        }

        RCLCPP_INFO(logger, "Accuracy check PASSED");
        return 0;
    }

    // 选择要运行的 demo：-p demo:=pose（默认）或 -p demo:=joint
    std::string select_demo(const rclcpp::Node::SharedPtr &node)
    {
        // automatically_declare_parameters_from_overrides(true) 已声明命令行传入的参数，
        // 未传入时才 declare，避免重复声明抛异常
        if (node->has_parameter("demo"))
        {
            std::string name;
            node->get_parameter("demo", name);
            return name;
        }
        return node->declare_parameter<std::string>("demo", "pose");
    }

    int run_demo(const rclcpp::Node::SharedPtr &node)
    {
        const auto logger = node->get_logger();

        moveit::planning_interface::MoveGroupInterface move_group(node, kGroupName);
        RCLCPP_INFO(logger, "Planning group: %s", move_group.getName().c_str());
        RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
        RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

        const std::string demo = select_demo(node);

        if (demo == "state")
        {
            RCLCPP_INFO(logger, "Checking current RobotState...");

            auto state = move_group.getCurrentState(2.0);

            if (!state)
            {
                RCLCPP_ERROR(logger, "RobotState unavailable after 2 seconds");
                return 1;
            }

            RCLCPP_INFO(logger, "RobotState received successfully");
            return 0;
        }
        if (demo == "ik_fail")
        {
            const auto eef_link = move_group.getEndEffectorLink();

            if (eef_link.empty())
            {
                RCLCPP_ERROR(logger, "End effector link is empty");
                return 1;
            }

            // 获取当前末端位姿
            auto target = move_group.getCurrentPose(eef_link);

            // 故意设置明显不可达的目标
            target.pose.position.x += 10.0;

            RCLCPP_INFO(logger,
                        "Testing unreachable target: x=%.3f, y=%.3f, z=%.3f",
                        target.pose.position.x,
                        target.pose.position.y,
                        target.pose.position.z);

            move_group.setStartStateToCurrentState();
            move_group.setPlanningTime(3.0);

            if (!move_group.setPoseTarget(target, eef_link))
            {
                RCLCPP_ERROR(logger, "Failed to set pose target");
                return 1;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto result = move_group.plan(plan);

            if (!result)
            {
                RCLCPP_INFO(logger,
                            "Expected planning failure, error code: %d",
                            result.val);
                return 0;
            }

            RCLCPP_WARN(logger, "Unexpectedly found a plan");
            return 1;
        }
        if (demo == "pose")
        {
            return run_pose_demo(move_group, logger);
        }
        if (demo == "joint")
        {
            return run_joint_demo(move_group, logger);
        }

        RCLCPP_ERROR(logger, "Unknown demo '%s' (expected 'pose' or 'joint')", demo.c_str());
        return 1;
    }

} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("minimal_moveit_demo", options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    int ret = 1;
    try
    {
        ret = run_demo(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Demo failed: %s", e.what());
    }

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return ret;
}
