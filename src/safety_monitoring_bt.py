#!/usr/bin/env python3

import py_trees as pt
from behaviors import *
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys


def create_root() -> pt.behaviour.Behaviour:
    """Structures a behavior tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """

    # we define the root node
    root = pt.composites.Parallel(name="root",
                                  policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    

    ### we create a sequence node called "Topics2BB" and a selector node called "Priorities"
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)

    battery_status = BatteryStatus2bb()
    laser_scan = LaserScan2bb()
    rotate = Rotate()
    stop_motion = StopMotion()

    ### we create an "Idle" node, which is a running node to keep the robot idle
    idle = pt.behaviours.Running(name="Idle")
    
    """
    TODO:  The first and second level of the tree structure is defined above, but please
    define the rest of the tree structure.

    Class definitions for your behaviours are provided in behaviours.py; you also need to fill out
    the behaviour implementations!

    HINT: Some behaviors from pt.behaviours may be useful to use as well.
    """

    ### YOUR CODE HERE ###

    battery_low_condition = pt.decorators.Condition(
        name="Battery Low?",
        child=rotate,
        status=lambda: battery_status.blackboard.get('battery_low_warning')
    )
    
    collision_condition = pt.decorators.Condition(
        name="Colliding?",
        child=stop_motion,
        condition=lambda: laser_scan.blackboard.get('collision_detected')
    )


    # TODO: construct the behavior tree structure using the nodes and behaviors defined above
    # HINT: for reference, the sample tree structure in the README.md file might be useful

    topics2BB.add_children([battery_status, laser_scan])
    priorities.add_children([collision_condition, battery_low_condition, idle])
    root.add_children([topics2BB, priorities])
    return root

def main():
    """
    This is the main function that initializes and executes the behavior tree.
    It first initializes the ROS client library, then creates the root of the behavior tree.
    The tree is then set up with a timeout of 30.0 seconds.
    If the setup fails due to a timeout or a keyboard interrupt, the tree and the ROS client library are shut down and the program exits.
    If the setup is successful, the tree starts ticking with a frequency of 100ms.
    The function then enters a loop where it waits for callbacks from the ROS client library.
    If a keyboard interrupt or an external shutdown exception occurs, the loop is broken.
    Finally, the tree and the ROS client library are shut down.
    """
    rclpy.init(args=None)

    root = create_root() # Create the root of the behavior tree
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        # If setup fails due to a timeout, log the error, shut down the tree and the ROS client library, and exit the program
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # If setup is interrupted by the user, log the interruption, shut down the tree and the ROS client library, and exit the program
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=100)  # Start the tree ticking with a frequency of 100ms  

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Finally, shut down the tree and the ROS client library
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
