// Pure-math unit tests for the TCP / setIKFrame chain (tcp_geometry.hpp).
//
// Pins the invariant that is the whole point of the PreGrasp deviation audit:
//
//   setIKFrame(pose, link) consumes T_link_tcp, NOT its inverse.
//
// The IK solves
//   T_world_ee = T_world_tcp_target * T_ee_tcp.inverse()
// and the forward chain reproduces
//   T_world_tcp = T_world_ee * T_ee_tcp == T_world_tcp_target.
//
// Passing T_ee_tcp.inverse() instead moves the effective TCP by the 138 mm
// lever arm projected through the grasp orientation, which is exactly the
// multi-cm XY error this project measured (X~5 cm, Y~10 cm). The inverse-
// regression test below proves that error class numerically so the failure
// mode cannot be reintroduced silently.

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "tcp_geometry.hpp"

namespace openarm_mtc {
namespace {

const double kPi = M_PI;

// Current generated OpenArm v2.0 grasp frame. The xacro shifts the EE base
// frame by +20.5 mm, so the original -21.93 mm CAD coordinate becomes
// -1.43 mm. MTC now reads this from RobotModel; the constant only exercises
// the pure transform helpers in this file.
const Eigen::Vector3d kTcpXyz(-0.00143, 0.0, -0.138);
const Eigen::Vector3d kTcpRpy(kPi, kPi / 2.0, kPi);

TEST(PlanarSymmetry, UsesScaleIndependentAspectRatioContract) {
  EXPECT_EQ(planar_symmetry_order(0.040, 0.038, 1.30), 4);
  EXPECT_EQ(planar_symmetry_order(0.057, 0.046, 1.30), 4);
  EXPECT_EQ(planar_symmetry_order(0.078, 0.038, 1.30), 2);
  EXPECT_EQ(planar_symmetry_order(0.120, 0.100, 1.30), 4);
  EXPECT_EQ(planar_symmetry_order(0.120, 0.080, 1.30), 2);
  EXPECT_EQ(planar_symmetry_order(0.0, 0.040, 1.30), 0);
}

TEST(GraspWidth, UsesShortSideForRectangleAndMeanForNearSquare) {
  EXPECT_NEAR(measured_grasp_width(0.080, 0.030, 2), 0.030, 1e-12);
  EXPECT_NEAR(measured_grasp_width(0.036, 0.033, 4), 0.0345, 1e-12);
  EXPECT_DOUBLE_EQ(measured_grasp_width(0.0, 0.033, 4), 0.0);
}

TEST(TcpGeometry, GraspAperturePreloadDoesNotAssumeObjectSize) {
  EXPECT_NEAR(preloaded_grasp_width(0.036, 0.002), 0.034, 1e-12);
  EXPECT_DOUBLE_EQ(preloaded_grasp_width(0.001, 0.002), 0.0);
  EXPECT_DOUBLE_EQ(preloaded_grasp_width(0.036, -0.001), 0.0);
}

TEST(VerticalClearance, DerivesGraspAndLandingHeightFromGeometry) {
  EXPECT_NEAR(grasp_tcp_z_offset(0.041, 0.032, 0.008), 0.0195, 1e-12);
  EXPECT_DOUBLE_EQ(grasp_tcp_z_offset(0.100, 0.032, 0.008), 0.0);
  EXPECT_NEAR(safe_grasp_tcp_z(0.2685, 0.041, 0.247, 0.032, 0.008),
              0.288, 1e-12);
  // A bad object Z estimate cannot pull the gripper through the support floor.
  EXPECT_NEAR(safe_grasp_tcp_z(0.250, 0.041, 0.247, 0.032, 0.008),
              0.287, 1e-12);
  // Five millimetres of upward-only collision padding is added to the desired
  // 8 mm clearance before this helper is called.
  EXPECT_NEAR(safe_grasp_tcp_z(0.2685, 0.041, 0.247, 0.032, 0.013),
              0.293, 1e-12);
  EXPECT_NEAR(landing_object_center_z(0.247, 0.041, 0.005, 0.0195,
                                      0.032, 0.008),
              0.2725, 1e-12);
  // A tall object whose centre target would put the fingers too low is raised.
  EXPECT_NEAR(landing_object_center_z(0.247, 0.100, 0.005, 0.0,
                                      0.032, 0.008),
              0.302, 1e-12);
}

TEST(MakeTcpTransform, OfficialGraspFrameTranslationAndRotation) {
  const Eigen::Isometry3d tcp = make_tcp_transform(kTcpXyz, kTcpRpy);
  EXPECT_NEAR(tcp.translation().x(), -0.00143, 1e-12);
  EXPECT_NEAR(tcp.translation().y(), 0.0, 1e-12);
  EXPECT_NEAR(tcp.translation().z(), -0.138, 1e-12);

  // R = Rx(pi) * Ry(pi/2) * Rz(pi). Verified columns:
  //   R * (1,0,0) = ( 0, 0,-1)
  //   R * (0,1,0) = ( 0, 1, 0)
  //   R * (0,0,1) = ( 1, 0, 0)
  const Eigen::Vector3d ex = tcp.linear() * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d ey = tcp.linear() * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d ez = tcp.linear() * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(ex.x(), 0.0, 1e-12);
  EXPECT_NEAR(ex.y(), 0.0, 1e-12);
  EXPECT_NEAR(ex.z(), -1.0, 1e-12);
  EXPECT_NEAR(ey.x(), 0.0, 1e-12);
  EXPECT_NEAR(ey.y(), 1.0, 1e-12);
  EXPECT_NEAR(ey.z(), 0.0, 1e-12);
  EXPECT_NEAR(ez.x(), 1.0, 1e-12);
  EXPECT_NEAR(ez.y(), 0.0, 1e-12);
  EXPECT_NEAR(ez.z(), 0.0, 1e-12);
}

TEST(VerticalGraspOrientation, PointsApproachAxisAlongWorldMinusZ) {
  // With pitch = pi/2 the grasp +X axis (the approach axis) must point along
  // world -Z regardless of the world yaw.
  for (const double yaw : {0.0, 0.3, kPi / 2.0, -1.1, kPi}) {
    const Eigen::Matrix3d r =
        vertical_grasp_orientation(0.0, kPi / 2.0, yaw).linear();
    const Eigen::Vector3d approach = r * Eigen::Vector3d::UnitX();
    EXPECT_NEAR(approach.x(), 0.0, 1e-9);
    EXPECT_NEAR(approach.y(), 0.0, 1e-9);
    EXPECT_NEAR(approach.z(), -1.0, 1e-9);
    EXPECT_NEAR(std::atan2(std::sin(vertical_grasp_yaw(r) - yaw),
                           std::cos(vertical_grasp_yaw(r) - yaw)),
                0.0, 1e-9);
  }
}

TEST(TcpIkChain, ForwardChainReproducesTargetXYAcrossYaws) {
  // The target TCP pose has the object XY; IK then solves for the ee link and
  // the forward chain must land back on the very same world TCP.
  const Eigen::Vector3d object_xy(0.300, -0.020, 0.0);
  const double object_z = 0.280;
  const double pregrasp_z = object_z + 0.07;
  for (const double yaw : {0.0, 0.4, kPi / 2.0, -0.7, 2.4}) {
    const Eigen::Isometry3d target =
        vertical_grasp_orientation(0.0, kPi / 2.0, yaw);
    Eigen::Isometry3d target_pose = target;
    target_pose.translation() = Eigen::Vector3d(
        object_xy.x(), object_xy.y(), pregrasp_z);

    const Eigen::Isometry3d ee_to_tcp =
        make_tcp_transform(kTcpXyz, kTcpRpy);
    // The IK transform actually used by the stages / raw_ik_diagnostic.
    const Eigen::Isometry3d world_ee = target_pose * ee_to_tcp.inverse();
    // Forward chain: FK(q) gives T_world_ee, TCP = T_world_ee * T_ee_tcp.
    const Eigen::Isometry3d recovered = world_ee * ee_to_tcp;

    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(recovered.translation()[axis], target_pose.translation()[axis],
                  1e-9)
          << "yaw=" << yaw << " axis=" << axis;
    }
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        EXPECT_NEAR(recovered.linear()(row, col), target_pose.linear()(row, col),
                    1e-9)
            << "yaw=" << yaw << " r=" << row << " c=" << col;
      }
    }
    // The whole point: target TCP XY equals object XY, exactly.
    EXPECT_NEAR(recovered.translation().x(), object_xy.x(), 1e-12);
    EXPECT_NEAR(recovered.translation().y(), object_xy.y(), 1e-12);
  }
}

TEST(TcpIkChain, InverseRegressionMovesTcpByLeverArm) {
  // Regression guard: if setIKFrame were given T_ee_tcp.inverse() instead of
  // T_ee_tcp, the IK would solve
  //   T_world_ee = target * (T_ee_tcp.inverse()).inverse() = target * T_ee_tcp
  // and the physical TCP (T_world_ee * T_ee_tcp) is displaced from the target
  // by the 138 mm lever arm projected through the grasp orientation.
  const Eigen::Isometry3d ee_to_tcp = make_tcp_transform(kTcpXyz, kTcpRpy);
  const double object_z = 0.280;
  for (const double yaw : {0.0, 0.4, kPi / 2.0, -0.7}) {
    const Eigen::Isometry3d target =
        vertical_grasp_orientation(0.0, kPi / 2.0, yaw);
    Eigen::Isometry3d target_pose = target;
    target_pose.translation() = Eigen::Vector3d(0.300, 0.0, object_z);

    const Eigen::Isometry3d wrong_world_ee = target_pose * ee_to_tcp;
    const Eigen::Vector3d wrong_tcp =
        (wrong_world_ee * ee_to_tcp).translation();
    const Eigen::Vector3d error = wrong_tcp - target_pose.translation();

    // The XY error is on the same order as the 138 mm lever: well beyond the
    // 5 cm / 10 cm class measured on the real robot. A regression can never
    // masquerade as "a small TCP offset".
    EXPECT_GT(error.head<2>().norm(), 0.05)
        << "yaw=" << yaw << " XY error=" << error.head<2>().norm();
    // And it is orientation-dependent: no constant x/y_offset patch can be
    // the correct fix (that is exactly the banned workaround).
    const Eigen::Vector3d error_at_zero_yaw = [&] {
      const Eigen::Isometry3d t0 =
          vertical_grasp_orientation(0.0, kPi / 2.0, 0.0);
      Eigen::Isometry3d p0 = t0;
      p0.translation() = Eigen::Vector3d(0.300, 0.0, object_z);
      return ((p0 * ee_to_tcp) * ee_to_tcp).translation() -
             p0.translation();
    }();
    if (std::abs(yaw) > 1e-6) {
      EXPECT_GT((error.head<2>() - error_at_zero_yaw.head<2>()).norm(), 0.02)
          << "yaw=" << yaw;
    }
  }
}

TEST(TcpLeverInWorld, VerticalGraspKeepsLeverMostlyVertical) {
  // At pitch = pi/2 the 13.8 cm lever stays along world -Z for EVERY yaw; only
  // the small 21.9 mm radial offset rotates about world Z with constant
  // magnitude. So a correct TCP chain can never, by itself, produce the
  // measured 5 cm / 10 cm XY offset at a vertical grasp.
  for (const double yaw : {0.0, 0.4, kPi / 2.0, -1.0, 2.4}) {
    const Eigen::Vector3d lever =
        tcp_lever_in_world(kPi / 2.0, yaw, kTcpXyz, kTcpRpy);
    EXPECT_NEAR(lever.z(), -0.138, 1e-9) << "yaw=" << yaw;
    EXPECT_NEAR(lever.head<2>().norm(), 0.00143, 1e-9) << "yaw=" << yaw;
  }
}

TEST(TcpLeverInWorld, WrongChainTurnsLeverIntoXyError) {
  // A TCP-direction bug applies the transform twice (R_ee^2 = diag(-1,1,-1)
  // is a 180 deg rotation about Y), which flips the 13.8 cm Z lever into an
  // XY error of ~11.6 cm that ROTATES with the grasp yaw:
  //   yaw=0     -> ( -13.657 cm,      0 cm)
  //   yaw=pi/2  -> (       0 cm, -13.657 cm)
  // The real-robot measurement (X~5 cm, Y~10 cm) is exactly this error class.
  const Eigen::Isometry3d ee_to_tcp = make_tcp_transform(kTcpXyz, kTcpRpy);
  const double object_z = 0.280;
  const auto wrong_xy_error = [&](double yaw) {
    const Eigen::Isometry3d target =
        vertical_grasp_orientation(0.0, kPi / 2.0, yaw);
    Eigen::Isometry3d target_pose = target;
    target_pose.translation() = Eigen::Vector3d(0.300, 0.0, object_z);
    return ((target_pose * ee_to_tcp) * ee_to_tcp).translation() -
           target_pose.translation();
  };
  const Eigen::Vector3d err0 = wrong_xy_error(0.0);
  EXPECT_NEAR(err0.x(), -0.13657, 0.001);
  EXPECT_NEAR(err0.y(), 0.0, 0.001);
  const Eigen::Vector3d err90 = wrong_xy_error(kPi / 2.0);
  EXPECT_NEAR(err90.x(), 0.0, 0.001);
  EXPECT_NEAR(err90.y(), -0.13657, 0.001);
}

}  // namespace
}  // namespace openarm_mtc
