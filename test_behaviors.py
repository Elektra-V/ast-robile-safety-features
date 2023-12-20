import unittest
from unittest.mock import Mock, patch

from behaviors import BatteryStatus2bb


## A sample python unit test
class TestBareBones(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        self.assertEqual(1, 2, "1!=1")

class TestBatteryStatus2bb(unittest.TestCase):
    def setUp(self):
        # Mock the superclass __init__ method to not perform any action
        with patch('py_trees_ros.subscribers.ToBlackboard.__init__', return_value=None):
            self.battery_status = BatteryStatus2bb(battery_voltage_topic_name="test_battery_voltage", threshold=30.0)

        # Manually set up the blackboard attribute
        self.battery_status.blackboard = Mock()
        self.battery_status.blackboard.battery = 100.0
        self.battery_status.blackboard.get.side_effect = lambda key: {'battery': 25.0}.get(key, None)

    def test_low_battery_warning(self):
        # Test for low battery warning
        self.battery_status.update()
        self.assertTrue(self.battery_status.blackboard.battery_low_warning)

        # Resetting the side effect for normal battery level
        self.battery_status.blackboard.get.side_effect = lambda key: 35.0 if key == 'battery' else None
        self.battery_status.update()
        self.assertFalse(self.battery_status.blackboard.battery_low_warning)

if __name__ == '__main__':
    unittest.main()