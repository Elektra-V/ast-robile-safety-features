import pytest
from unittest.mock import Mock, patch
from src.behaviors import Rotate
from geometry_msgs.msg import Twist
from py_trees.common import Status as pt_Status


@pytest.fixture
def rotate_behavior():
    return Rotate()


def test_initialization(rotate_behavior):
    assert rotate_behavior.topic_name == "/cmd_vel"
    assert rotate_behavior.ang_vel == 1.0


@patch('src.behaviors.Rotate.logger')
def test_setup(mock_logger, rotate_behavior):
    mock_node = Mock()
    assert rotate_behavior.setup(node=mock_node)
    mock_logger.info.assert_called_with("[ROTATE] setting up rotate behavior")

    with pytest.raises(KeyError):
        rotate_behavior.setup()


@patch('src.behaviors.Rotate.publisher')
def test_update(mock_publisher, rotate_behavior):
    mock_node = Mock()
    rotate_behavior.setup(node=mock_node)
    status = rotate_behavior.update()
    mock_publisher.publish.assert_called_once()
    args, _ = mock_publisher.publish.call_args
    twist_msg = args[0]
    assert twist_msg.angular.z == rotate_behavior.ang_vel
    assert status == pt_Status.RUNNING


def test_terminate(rotate_behavior):
    mock_node = Mock()
    rotate_behavior.setup(node=mock_node)
    rotate_behavior.terminate(new_status=pt_Status.SUCCESS)
