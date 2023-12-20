import unittest
from unittest.mock import Mock, patch

from src import Rotate, Twist, StopMotion, BatteryStatus2bb, LaserScan2bb


class TestRotate(unittest.TestCase):

    def setUp(self):
        # Mock the ROS node
        self.mock_node = Mock()

        # Initialize the Rotate behavior with mock parameters
        self.rotate = Rotate(name="test_rotate", topic_name="test_cmd_vel", ang_vel=1.0)
        self.rotate.setup(node=self.mock_node)

    @patch('behaviors.Rotate.publisher')
    def test_setup(self, mock_publisher):
        # Test if a publisher is created during setup
        self.rotate.setup(node=self.mock_node)
        mock_publisher.create_publisher.assert_called_once_with(Twist, "test_cmd_vel", 10)

    @patch('behaviors.Rotate.publisher')
    def test_update_publishes_twist(self, mock_publisher):
        # Test if the correct Twist message is published in update
        self.rotate.update()
        expected_twist = Twist()
        expected_twist.angular.z = 1.0
        mock_publisher.publish.assert_called_with(expected_twist)

    @patch('behaviors.Rotate.publisher')
    def test_terminate_publishes_stop_twist(self, mock_publisher):
        # Test if a stop Twist message is published on terminate
        self.rotate.terminate(new_status=None)
        expected_twist = Twist()
        mock_publisher.publish.assert_called_with(expected_twist)


class TestStopMotion(unittest.TestCase):

    def setUp(self):
        # Mock the ROS node
        self.mock_node = Mock()

        # Initialize the StopMotion behavior with mock parameters
        self.stop_motion = StopMotion(name="test_stop_motion", topic_name="test_cmd_vel")
        self.stop_motion.setup(node=self.mock_node)

    @patch('behaviors.StopMotion.publisher')
    def test_setup(self, mock_publisher):
        # Test if a publisher is created during setup
        self.stop_motion.setup(node=self.mock_node)
        mock_publisher.create_publisher.assert_called_once_with(Twist, "test_cmd_vel", 10)

    @patch('behaviors.StopMotion.publisher')
    def test_update_publishes_stop_twist(self, mock_publisher):
        # Test if a stop Twist message is published in update
        self.stop_motion.update()
        expected_twist = Twist()
        mock_publisher.publish.assert_called_with(expected_twist)


class TestBatteryStatus2bb(unittest.TestCase):

    def setUp(self):
        self.battery_status = BatteryStatus2bb(battery_voltage_topic_name="test_battery_voltage", threshold=30.0)
        self.battery_status.blackboard = Mock()

    def test_low_battery_warning(self):
        # Simulate different battery levels and check for low battery warning
        self.battery_status.blackboard.get.return_value = 25.0  # Low battery
        self.battery_status.update()
        self.assertTrue(self.battery_status.blackboard.battery_low_warning)

        self.battery_status.blackboard.get.return_value = 35.0  # Normal battery
        self.battery_status.update()
        self.assertFalse(self.battery_status.blackboard.battery_low_warning)


class TestLaserScan2bb(unittest.TestCase):

    def setUp(self):
        self.laser_scan = LaserScan2bb(topic_name="test_scan", safe_range=0.25)
        self.laser_scan.blackboard = Mock()

    def test_collision_detection(self):
        # Simulate laser scan data for collision detection
        self.laser_scan.blackboard.get.return_value = [0.2, 0.3, 0.4]  # Collision detected
        self.laser_scan.update()
        self.assertTrue(self.laser_scan.blackboard.collision_detected)

        self.laser_scan.blackboard.get.return_value = [0.5, 0.6, 0.7]  # No collision
        self.laser_scan.update()
        self.assertFalse(self.laser_scan.blackboard.collision_detected)


if __name__ == '__main__':
    unittest.main()