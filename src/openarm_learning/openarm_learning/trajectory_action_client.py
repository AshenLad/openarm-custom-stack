import array
from time import monotonic
from urllib import response

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
        self.current_positions: list[float] | None = None  # 尚未收到完整的七关节位置

        self.done = False
        self.send_future = None  # 尚未发送，没有待等的回复
        self.goal_reply_deadline = None  # 尚未开始回复计时
        self.result_deadline = None  # 尚未开始等待最终结果
        self.result_future = None    # 尚未请求最终结果
        self.cancel_future = None    # 尚未发送取消请求
        self.cancel_result_deadline = None  # 尚未发送取消请求，不启动收尾计时
        
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

        first_sample = self.current_positions is None #True or Flase
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
        point.time_from_start.sec = 3  # 从轨迹开始算，x秒到达
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

        self.get_logger().info('发送 Goal，等待接受或拒绝回复（最多5秒）')
        self.goal_reply_deadline = monotonic() + 5.0  # 发送时设定期限
        self.send_future = self.client.send_goal_async( # 异步发送，返回一个Future实例
            goal,
            feedback_callback=self.feedback_callback,  # 收到过程反馈时调用
            )  
        self.send_future.add_done_callback(
            self.goal_response_callback  # 收到接受/拒绝回复后调用
        )
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback  # 从消息包装中取出反馈内容
        name = self.joint_names[0]  # 本次只观察 openarm_left_joint1

        if name not in feedback.joint_names:
            return  # 这条反馈没有目标关节

        index = feedback.joint_names.index(name)  # 按名字找到对应下标
        arrays = (
            feedback.desired.positions,
            feedback.actual.positions,
            feedback.error.positions,
        )
        if any(index >= len(values) for values in arrays):
            return  # 本条反馈的位置字段不完整

        desired = feedback.desired.positions[index]
        actual = feedback.actual.positions[index]
        error = feedback.error.positions[index]

        self.get_logger().info(
            f'反馈 {name}：期望={desired:.4f} rad，'
            f'实际={actual:.4f} rad，误差={error:.4f} rad',
            throttle_duration_sec=0.5,  # 最多约每0.5秒打印一次，避免刷屏
        )

    def goal_response_callback(self, future):
        self.goal_handle = future.result()  # 取得任务句柄

        if self.goal_handle.accepted:
            self.get_logger().info('Goal 已接受')
            
            self.result_deadline = monotonic() + 5.0 #5秒取消
            self.get_logger().info('等待执行结果（最多5秒）')
            
            self.result_future = self.goal_handle.get_result_async()  # 请求最终结果
            self.result_future.add_done_callback(
                self.result_callback  # 任务结束、结果到达后执行
            )
        else:
            self.get_logger().warning('Goal 被拒绝')
            self.done = True

    def cancel_response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'取消回复码：{response.return_code}')
        self.get_logger().info(
            f'进入取消流程的任务数：{len(response.goals_canceling)}'
        )  # 列表里记录了接受取消请求的任务
    
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
        elif response.status == GoalStatus.STATUS_CANCELED:  # 状态5：任务已取消
            self.get_logger().info('任务已取消')
        else:
            self.get_logger().error('任务未成功，请检查最终状态和轨迹说明')

        self.done = True  # 收到最终结果后，才让主循环退出


def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS 2
    node = TrajectoryActionClient()  # 创建节点和客户端

    try:
        found = node.client.wait_for_server(timeout_sec=5.0)  # 最多等待5秒

        if found:
            node.get_logger().info(f'找到服务端：{node.action_name}')
            
            state_deadline = monotonic()+5.0
            node.get_logger().info('等待完整关节状态（最多5秒）')
            
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)

                if(
                    node.current_positions is None
                   and monotonic() >= state_deadline
                   ):
                    node.get_logger().error('等待完整关节状态超时，未发送 Goal')
                    node.done = True
                    
                if (
                    node.send_future is not None  # 已发送 Goal
                    and not node.send_future.done()  # 回复仍未到达
                    and monotonic() >= node.goal_reply_deadline  # 期限已到
                ):
                    node.get_logger().error(
                        '等待 Goal 回复超时：任务状态未知，不自动重发'
                    )
                    node.done = True
                    
                if (
                    not node.done  # 任务处理仍在进行
                    and node.result_deadline is not None  # 已开始结果计时
                    and node.result_future is not None  # 已请求最终结果
                    and not node.result_future.done()  # 最终结果还没到
                    and monotonic() >= node.result_deadline  # 等待期限已到
                ):
                    node.result_deadline = None  # 关闭计时，避免重复请求取消
                    node.get_logger().warning('等待执行结果超时，发送取消请求')
                    node.cancel_result_deadline = monotonic() + 5.0
                    node.cancel_future = node.goal_handle.cancel_goal_async()
                    node.cancel_future.add_done_callback(
                        node.cancel_response_callback  # 继续使用已有取消回复回调
                    )
                    
                if (
                    not node.done  # 最终结果回调尚未结束本次任务处理
                    and node.cancel_result_deadline is not None  # 收尾计时已启动
                    and node.result_future is not None  # 已请求最终结果
                    and not node.result_future.done()  # 最终结果仍未到达
                    and monotonic() >= node.cancel_result_deadline  # 收尾期限已到
                ):
                    node.get_logger().error(
                        '取消后等待最终结果超时：任务状态未知，客户端退出'
                    )
                    node.done = True  # 结束客户端等待，由 finally 清理资源
            
            if node.done:
                node.get_logger().info(f'任务处理结束，客户端退出')
            
        else:
            node.get_logger().error(f'等待服务端超时：{node.action_name}')
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()  # 释放节点及其客户端
        if rclpy.ok():
            rclpy.shutdown()  # 关闭本程序的 ROS 2 环境