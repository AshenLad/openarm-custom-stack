import rclpy  # ROS 2 Python 库
from rclpy.node import Node  # 节点基类
from rclpy.action import ActionClient  # Action 客户端
from control_msgs.action import FollowJointTrajectory  # 轨迹 Action 类型
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from action_msgs.msg import GoalStatus  # Action 最终状态的常量


class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')  # 本程序的节点名

        self.action_name = (
            '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.joint_names = [
            f'openarm_left_joint{i}' for i in range(1, 8)
        ]  # 左臂 joint1 到 joint7
        self.current_positions = None  # 尚未收到完整的七关节位置
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        self.client = ActionClient(
            self,  # 客户端属于当前节点
            FollowJointTrajectory,  # 使用轨迹 Action 类型
            self.action_name,  # 联系这个入口
        )
        
    def joint_state_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))  # 建立名字到位置的字典

        for name in self.joint_names:
            if name not in joint_positions:
                return  # 本条消息缺少需要的位置，等待下一条

        first_sample = self.current_positions is None
        self.current_positions = [
            joint_positions[name] for name in self.joint_names
        ]  # 按左臂 joint1 到 joint7 的顺序保存

        if first_sample:
            for name, position in zip(self.joint_names, self.current_positions):
                self.get_logger().info(f'{name}={position:.4f} rad')

            self.build_trajectory()  # 七个位置齐全后，构造一次轨迹
            self.send_goal()  # 再发送一次任务请求

    def build_trajectory(self):
        target_positions = self.current_positions.copy()  # 单独建立目标位置
        target_positions[0] += 0.05  # 第一个名字是 joint1，增加0.05弧度

        point = JointTrajectoryPoint()  # 创建一个轨迹点
        point.positions = target_positions  # 这个点的七个目标位置
        point.time_from_start.sec = 3  # 从轨迹开始算，3秒到达
        point.time_from_start.nanosec = 0  # 本次没有不足1秒的部分

        self.trajectory = JointTrajectory()  # 创建整条轨迹
        self.trajectory.joint_names = self.joint_names.copy()  # 固定关节顺序
        self.trajectory.points = [point]  # 本次轨迹只有一个目标点

        self.get_logger().info(
            f'轨迹关节：{self.trajectory.joint_names}'
        )
        self.get_logger().info(
            f'目标位置：{point.positions}'
        )
        self.get_logger().info(
            f'到达时间：{point.time_from_start.sec} 秒'
        )
        
    def send_goal(self):
        goal = FollowJointTrajectory.Goal()  # 创建任务请求
        goal.trajectory = self.trajectory  # 把已构造的轨迹装入请求

        self.get_logger().info('发送 Goal')
        self.send_future = self.client.send_goal_async(goal)  # 异步发送
        self.send_future.add_done_callback(
            self.goal_response_callback  # 收到接受/拒绝回复后调用
        )

    def goal_response_callback(self, future):
        self.goal_handle = future.result()  # 取得任务句柄

        if self.goal_handle.accepted:
            self.get_logger().info('Goal 已接受')
            self.result_future = self.goal_handle.get_result_async()  # 请求最终结果
            self.result_future.add_done_callback(
                self.result_callback  # 任务结束、结果到达后执行
            )
        else:
            self.get_logger().warning('Goal 被拒绝')

    def result_callback(self, future):
        response = future.result()  # 取得完整的结果回复
        result = response.result  # 其中的轨迹执行结果

        self.get_logger().info(f'最终状态：{response.status}')
        self.get_logger().info(f'轨迹错误码：{result.error_code}')
        self.get_logger().info(f'轨迹说明：{result.error_string!r}')

        if (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().info('执行成功')
        else:
            self.get_logger().error('任务未成功，请检查最终状态和轨迹说明')


def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS 2
    node = TrajectoryActionClient()  # 创建节点和客户端

    try:
        found = node.client.wait_for_server(timeout_sec=5.0)  # 最多等待5秒

        if found:
            node.get_logger().info(f'找到服务端：{node.action_name}')
            rclpy.spin(node)
        else:
            node.get_logger().error(f'等待服务端超时：{node.action_name}')
    finally:
        node.destroy_node()  # 释放节点及其客户端
        rclpy.shutdown()  # 关闭本程序的 ROS 2 环境