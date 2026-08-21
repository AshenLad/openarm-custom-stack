// Pure TCP geometry helpers shared by mtc_pregrasp.cpp and the unit tests.
//
// The official OpenArm pinch-gripper grasp frame is a fixed transform from the
// end-effector base link:
//
// In the current generated v2.0 model the base-frame shift makes this
// translation [-0.00143, 0, -0.138] m. mtc_pregrasp resolves the transform
// from RobotModel, while the helpers below remain useful for tests.
//
// setIKFrame() consumes exactly this T_link_tcp (NOT its inverse). MoveIt Task
// Constructor computes the world TCP as
//
//   T_world_tcp = T_world_ee * T_ee_tcp
//
// so a target pose is a world->TCP pose and the IK solves
//
//   T_world_ee = target * T_ee_tcp.inverse().
//
// Passing T_ee_tcp.inverse() instead would silently move the effective TCP by
// the full 138 mm lever arm after orientation, producing exactly the kind of
// multi-cm XY error this project is chasing. The tests pin this direction.

#ifndef OPENARM_MTC_TCP_GEOMETRY_HPP
#define OPENARM_MTC_TCP_GEOMETRY_HPP

#include <Eigen/Geometry>

#include <algorithm>

namespace openarm_mtc {

// Use the same dimensionless near-square contract as the RGB-D detector.
// A metric tolerance (for example the perception motion threshold) is not a
// shape classifier: it labels the same measured object differently as its
// scale changes and previously caused vision to stabilize yaw modulo 90 deg
// while MTC generated only the two modulo-180 grasp candidates.
inline int planar_symmetry_order(double length, double width,
                                 double square_aspect_ratio_max) {
  const double longer = std::max(length, width);
  const double shorter = std::min(length, width);
  if (shorter <= 0.0 || square_aspect_ratio_max < 1.0) {
    return 0;
  }
  return longer / shorter <= square_aspect_ratio_max ? 4 : 2;
}

// A rectangular object is intentionally grasped across its shorter side. For
// a near-square RGB-D OBB either edge may be labelled L/W from one frame to the
// next, so min(L,W) systematically underestimates the aperture.
inline double measured_grasp_width(double length, double width,
                                   int symmetry_order) {
  if (length <= 0.0 || width <= 0.0) {
    return 0.0;
  }
  return symmetry_order == 4 ? 0.5 * (length + width)
                             : std::min(length, width);
}

// Position control needs a small commanded aperture reduction to create
// preload after the fingers meet the object. This is gripper compliance, not
// an assumed object size; the input width remains the live RGB-D measurement.
inline double preloaded_grasp_width(double measured_width,
                                    double aperture_preload) {
  if (measured_width <= 0.0 || aperture_preload < 0.0) {
    return 0.0;
  }
  return std::max(0.0, measured_width - aperture_preload);
}

inline double grasp_tcp_z_offset(double object_height,
                                 double tip_below_grasp_frame,
                                 double support_clearance) {
  if (object_height <= 0.0 || tip_below_grasp_frame < 0.0 ||
      support_clearance < 0.0) {
    return 0.0;
  }
  return std::max(
      0.0, tip_below_grasp_frame + support_clearance - 0.5 * object_height);
}

// The object detector and the box detector are independent RGB-D estimates.
// Their Z errors must never be able to command a fingertip below the support
// safety plane.  Keep the object-centred grasp when it is already conservative,
// otherwise raise the TCP to the support-derived hard floor.
inline double safe_grasp_tcp_z(double measured_object_center_z,
                               double object_height, double support_top,
                               double tip_below_grasp_frame,
                               double support_clearance) {
  const double object_based =
      measured_object_center_z +
      grasp_tcp_z_offset(object_height, tip_below_grasp_frame,
                         support_clearance);
  const double support_floor =
      support_top + tip_below_grasp_frame + support_clearance;
  return std::max(object_based, support_floor);
}

inline double landing_object_center_z(double support_top,
                                      double object_height,
                                      double surface_clearance,
                                      double grasp_tcp_z_offset,
                                      double tip_below_grasp_frame,
                                      double gripper_clearance) {
  const double object_surface_floor =
      support_top + 0.5 * object_height + surface_clearance;
  // GeneratePlacePose preserves the attached object's transform, so the
  // landing TCP is object_center_z + grasp_tcp_z_offset for this vertical task.
  const double gripper_floor_as_object_center =
      support_top + tip_below_grasp_frame + gripper_clearance -
      grasp_tcp_z_offset;
  return std::max(object_surface_floor, gripper_floor_as_object_center);
}

// Build T_ee_tcp from xyz + fixed-axis RPY (R = Rx(roll) * Ry(pitch) * Rz(yaw)).
inline Eigen::Isometry3d make_tcp_transform(const Eigen::Vector3d& xyz,
                                            const Eigen::Vector3d& rpy) {
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
  tcp.translation() = xyz;
  tcp.linear() =
      (Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()))
          .toRotationMatrix();
  return tcp;
}

// Grasp orientation used by the planning stages: the grasp frame +X axis is
// the approach axis, grasp_pitch = pi/2 points it along world -Z, and
// world_yaw rotates the whole frame about world Z.
inline Eigen::Isometry3d vertical_grasp_orientation(double roll, double pitch,
                                                    double world_yaw) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() =
      (Eigen::AngleAxisd(world_yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return result;
}

// At pitch=pi/2 the grasp +X axis is vertical, so ordinary quaternion/RPY yaw
// extraction is singular and returns numerical noise. With roll=0 the grasp
// +Z axis remains horizontal and directly encodes the commanded world yaw.
inline double vertical_grasp_yaw(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 2), rotation(0, 2));
}

// World-space offset of the TCP from the ee_base_link origin when the arm is
// held at the given vertical grasp orientation:
//
//   p_tcp_world = R_world_ee * t_ee_tcp
//
// where R_world_ee = R_world_tcp * R_ee_tcp^T. This is what setIKFrame/IK
// compensate exactly. A notable consequence (pinned by the unit tests): at
// pitch = pi/2 the 138 mm lever stays along world -Z for every yaw, with only
// the small radial offset rotating about world Z. A TCP-direction
// bug applies the transform twice (R_ee_tcp^2 = diag(-1,1,-1) is a 180 deg
// rotation about Y) and flips that Z lever into a ~11.6 cm XY error that
// rotates with the grasp yaw - the error class measured on the real robot.
inline Eigen::Vector3d tcp_lever_in_world(double pitch, double world_yaw,
                                          const Eigen::Vector3d& t_ee_tcp,
                                          const Eigen::Vector3d& rpy_ee_tcp) {
  const Eigen::Matrix3d r_world_tcp =
      vertical_grasp_orientation(0.0, pitch, world_yaw).linear();
  const Eigen::Matrix3d r_world_ee =
      r_world_tcp * make_tcp_transform(t_ee_tcp, rpy_ee_tcp).linear().transpose();
  return r_world_ee * t_ee_tcp;
}

}  // namespace openarm_mtc

#endif  // OPENARM_MTC_TCP_GEOMETRY_HPP
