#!/usr/bin/env python3

# import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis 
    """
    def __init__(self, name="rotate platform",
                 topic_name="/cmd_vel",
                 ang_vel=1.0):
        # inherit all the class variables from the parent class and make it a behavior
        super(Rotate, self).__init__(name)

        # TODO: initialise class variables
        self.topic_name = topic_name
        self.ang_vel = ang_vel
        self.publisher = None

    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 

        # TODO: setup any necessary publishers or subscribers

        ### YOUR CODE HERE ###

        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        """Rotates the robot at the maximum allowed angular velocity.
        Note: The actual behaviour is implemented here.

        """
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # TODO: implement the primary function of the behavior and decide which status to return 
        # based on the structure of your behavior tree

        # Hint: to return a status, for example, SUCCESS, pt.common.Status.SUCCESS can be used

        ### YOUR CODE HERE ###
        twist = Twist()
        twist.angular.z = self.ang_vel
        self.publisher.publish(twist)
        return pt.common.Status.RUNNING


    def terminate(self, new_status):
        """Trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")

        # TODO: implement the termination of the behavior, i.e. what should happen when the behavior 
        # finishes its execution

        ### YOUR CODE HERE ###
        if self.publisher:
            stop_twist = Twist()
            self.publisher.publish(stop_twist)
        return super().terminate(new_status)

class StopMotion(pt.behaviour.Behaviour):
    """Stops the robot when it is controlled using a joystick or with a cmd_vel command
    """
    
    # TODO: based on previous eexample, implement the behavior to stop the robot when it is controlled 
    # by sending a cmd_vel command (eg: teleop_twist_keyboard)

    ### YOUR CODE HERE ###
    def __init__(self, name="Stop Motion", topic_name="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        self.topic_name = topic_name
        self.publisher = None

    def setup(self, **kwargs):
        self.logger.info("[STOP MOTION] setting up stop motion behavior")
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 

        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        return True
    
    def update(self):
        stop_twist = Twist()
        self.publisher.publish(stop_twist)
        return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        return super().terminate(new_status)

class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Checks the battery status
    """
    def __init__(self, battery_voltage_topic_name: str="/battery_voltage",
                 name: str='Battery2BB',
                 threshold: float=30.0):
        super(BatteryStatus2bb, self).__init__(name=name,
                                               topic_name=battery_voltage_topic_name,
                                               topic_type=Float32,
                                               blackboard_variables={'battery': 'data'},
                                               initialise_variables={'battery': 100.0},
                                               clearing_policy=pt.common.ClearingPolicy.NEVER,
                                               qos_profile=ptr.utilities.qos_profile_unlatched())
        
        self.threshold = threshold
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.WRITE)


    def update(self):
        """Calls the parent to write the raw data to the blackboard and then check against the
        threshold to determine if a low warning flag should also be updated.
        """
        self.logger.info('[BATTERY] update: running battery_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        """check battery voltage level stored in self.blackboard.battery. By comparing with 
        threshold value, update the value of self.blackboad.battery_low_warning
        """

        # TODO: based on the battery voltage level, update the value of self.blackboard.battery_low_warning
        # and return the status of the behavior based on your logic of the behavior tree

        ### YOUR CODE HERE ###
        super().update()
        battery_level = self.blackboard.battery
        low_battery_warning = battery_level < self.threshold
        self.blackboard.battery_low_warning = low_battery_warning
        self.logger.debug(f"[BATTERY STATUS] Battery level: {battery_level}, Low warning: {low_battery_warning}")
        return pt.common.Status.SUCCESS if battery_level else pt.common.Status.FAILURE

class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Checks the laser scan measurements to avoid possible collisions.
    """
    def __init__(self, topic_name: str="/scan",
                 name: str='Scan2BB',
                 safe_range: float=0.25):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan':'ranges'},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                depth=10))
        
        # TODO: initialise class variables and blackboard variables
        ### YOUR CODE HERE ###
        self.safe_range = safe_range
        self.collision_detected = False
        self.blackboard.register_key(key='collision_detected', access=pt.common.Access.WRITE)
        self.blackboard.register_key(key='laser_scan', access=pt.common.Access.READ)

    def update(self):
        # TODO: impletment the update function to check the laser scan data and update the blackboard variable
        ### YOUR CODE HERE ###
        super().update()
        ranges = self.blackboard.laser_scan
        if not ranges:
            return pt.common.Status.FAILURE

        self.collision_detected = any(distance < self.safe_range for distance in ranges if distance is not None)
        self.blackboard.collision_detected = self.collision_detected

        return pt.common.Status.SUCCESS if self.collision_detected else pt.common.Status.FAILURE