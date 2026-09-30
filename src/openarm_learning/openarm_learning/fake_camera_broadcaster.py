import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class FakeCameraBroadcaster(Node):

    def __init__(self):
        super().__init__('fake_camera_broadcaster')

        # 创建静态 TF 广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # 构造一条 parent -> child 的 Transform
        camera_tf = TransformStamped()

        camera_tf.header.stamp = self.get_clock().now().to_msg()

        # parent frame
        camera_tf.header.frame_id = 'openarm_left_grasp_frame'

        # child frame
        camera_tf.child_frame_id = 'fake_camera_link'

        # 暂时让 camera_link 与 grasp_frame 完全重合
        camera_tf.transform.translation.x = 0.0
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.0

        camera_tf.transform.rotation.x = 0.0
        camera_tf.transform.rotation.y = 0.0
        camera_tf.transform.rotation.z = 0.0
        camera_tf.transform.rotation.w = 1.0

        # 发布这条静态 TF
        self.tf_broadcaster.sendTransform(camera_tf)

        self.get_logger().info(
            'Published static TF: '
            'openarm_left_grasp_frame -> fake_camera_link'
        )
        
        
        #构建optical frame
        optical_tf = TransformStamped()

        optical_tf.header.stamp = self.get_clock().now().to_msg()

        # parent frame
        optical_tf.header.frame_id = 'fake_camera_link'

        # child frame
        optical_tf.child_frame_id = 'fake_camera_color_optical_frame'

        optical_tf.transform.translation.x = 0.0
        optical_tf.transform.translation.y = 0.0
        optical_tf.transform.translation.z = 0.0

        optical_tf.transform.rotation.x = -0.5
        optical_tf.transform.rotation.y = 0.5
        optical_tf.transform.rotation.z = -0.5
        optical_tf.transform.rotation.w = 0.5

        # 发布这条静态 TF
        self.tf_broadcaster.sendTransform(optical_tf)

        self.get_logger().info(
            'Published static TF: '
            'fake_camera_link -> fake_camera_color_optical_frame'
        )
        


def main(args=None):
    rclpy.init(args=args)

    node = FakeCameraBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()