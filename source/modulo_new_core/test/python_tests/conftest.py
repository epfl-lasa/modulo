import clproto
import pytest
import rclpy.clock
import state_representation as sr


@pytest.fixture
def cart_state():
    return sr.CartesianState().Random("test", "ref")


@pytest.fixture
def joint_state():
    return sr.JointState().Random("robot", 3)


@pytest.fixture
def clock():
    return rclpy.clock.Clock()


@pytest.fixture
def parameters():
    return {"bool": [True, sr.ParameterType.BOOL, False],
            "bool_array": [[True, False], sr.ParameterType.BOOL_ARRAY, [False]],
            "int": [1, sr.ParameterType.INT, 2],
            "int_array": [[1, 2], sr.ParameterType.INT_ARRAY, [2]],
            "double": [1.0, sr.ParameterType.DOUBLE, 2.0],
            "double_array": [[1.0, 2.0], sr.ParameterType.DOUBLE_ARRAY, [2.0]],
            "string": ["1", sr.ParameterType.STRING, "2"],
            "string_array": [["1", "2"], sr.ParameterType.STRING_ARRAY, ["2"]]
            }


@pytest.fixture
def state_parameters():
    return {"cartesian_state": [sr.CartesianState().Random("test"), sr.StateType.CARTESIAN_STATE,
                                clproto.CARTESIAN_STATE_MESSAGE],
            "cartesian_pose": [sr.CartesianPose().Random("test"), sr.StateType.CARTESIAN_POSE,
                               clproto.CARTESIAN_POSE_MESSAGE],
            "joint_state": [sr.JointState().Random("test", 3), sr.StateType.JOINT_STATE,
                            clproto.JOINT_STATE_MESSAGE],
            "joint_positions": [sr.JointPositions().Random("test", 3), sr.StateType.JOINT_POSITIONS,
                                clproto.JOINT_POSITIONS_MESSAGE]
            }
