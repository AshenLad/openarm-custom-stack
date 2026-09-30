import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped


class TFMonitor(Node):

    def __init__(self):
        super().__init__('tf_monitor')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.timer = self.create_timer(
            1.0,
            self.transform_object_pose
        )

    def transform_object_pose(self):

        # 假装这是 GDRNPP 输出的物体位姿
        object_pose = PoseStamped()

        # GDRNPP 的 Pose 属于 optical frame
        object_pose.header.frame_id = 'fake_camera_color_optical_frame'

        # 这里先用时间戳 0：
        # 表示查询最新可用 TF，方便当前实验
        object_pose.header.stamp = Time().to_msg()

        # optical frame:
        # +X 右，+Y 下，+Z 前
        object_pose.pose.position.x = 0.2
        object_pose.pose.position.y = -0.1
        object_pose.pose.position.z = 0.7

        # 假设物体姿态与 optical frame 相同
        object_pose.pose.orientation.x = 0.0
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.0
        object_pose.pose.orientation.w = 1.0

        try:
            object_pose_world = self.tf_buffer.transform(
                object_pose,
                'world'
            )

            p = object_pose_world.pose.position
            q = object_pose_world.pose.orientation

            self.get_logger().info(
                f'object in world: '
                f'position=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), '
                f'quaternion=({q.x:.3f}, {q.y:.3f}, '
                f'{q.z:.3f}, {q.w:.3f})'
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform pose: {ex}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = TFMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()