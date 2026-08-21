import numpy as np

from openarm_aruco_vision.aruco_detector_node import rotation_matrix_to_quaternion


def test_identity_rotation():
    assert np.allclose(
        rotation_matrix_to_quaternion(np.eye(3)),
        (0.0, 0.0, 0.0, 1.0),
    )


def test_half_turn_x():
    rotation = np.diag([1.0, -1.0, -1.0])
    quaternion = np.asarray(rotation_matrix_to_quaternion(rotation))
    assert np.allclose(np.abs(quaternion), (1.0, 0.0, 0.0, 0.0))
