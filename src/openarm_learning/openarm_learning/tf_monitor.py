import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

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

        # 假装这是 GDRNPP 输出的物体 Pose
        object_pose = PoseStamped()

        # 这个 Pose 是在哪个坐标系下表达的？
        object_pose.header.frame_id = 'openarm_left_grasp_frame'

        #实验代码
        # future_time = self.get_clock().now() + Duration(seconds=2.0)
        # object_pose.header.stamp = future_time.to_msg()

        # 物体位于 grasp frame 的 X 正方向 10 cm
        object_pose.pose.position.x = 0.1
        object_pose.pose.position.y = 0.0
        object_pose.pose.position.z = 0.0

        # 物体姿态和 grasp frame 完全一致
        object_pose.pose.orientation.x = 0.0
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.0
        object_pose.pose.orientation.w = 1.0

        try:
            # # world <- grasp
            # transform = self.tf_buffer.lookup_transform(
            #     'world',
            #     object_pose.header.frame_id,
            #     Time()
            # )

            # # world <- object
            # object_pose_world = do_transform_pose_stamped(
            #     object_pose,
            #     transform
            # )
            
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