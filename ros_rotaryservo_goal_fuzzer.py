from hrim_actuator_rotaryservo_msgs.msg import *
from hrim_generic_msgs.msg import Header
from hypothesis.strategies import floats, text
from hypothesis import given, Verbosity, settings
from ros2node.api import get_node_names
import rclpy
import argparse

def check_node_alive(asker_node, alive_node):
    nodes = get_node_names(node=asker_node)
    node_names = list(map((lambda x: x.name), nodes))
    if alive_node in node_names:
        return True
    else:
        return False


@settings(verbosity=Verbosity.verbose)
@given(floats(), floats(), floats())
def fuzz_goal_message_all(node, target_node, pub, position, velocity, effort):
    message = GoalRotaryServo()
    message.control_type = GoalRotaryServo.CONTROL_TYPE_POSITION_VELOCITY_EFFORT
    message.position = position
    message.velocity = velocity
    message.effort = effort
    message.header = Header()
    pub.publish(message)
    assert check_node_alive(node, target_node) is True

@settings(verbosity=Verbosity.verbose)
@given(floats())
def fuzz_goal_message_position(node, target_node, pub, position):
    message = GoalRotaryServo()
    message.control_type = GoalRotaryServo.CONTROL_TYPE_POSITION
    message.position = position
    message.header = Header()
    pub.publish(message)
    assert check_node_alive(node, target_node) is True

@settings(verbosity=Verbosity.verbose)
@given(floats(), floats())
def fuzz_goal_message_position_velocity(node, target_node, pub, position, velocity):
    message = GoalRotaryServo()
    message.control_type = GoalRotaryServo.CONTROL_TYPE_POSITION_VELOCITY
    message.position = position
    message.velocity = velocity
    message.header = Header()
    pub.publish(message)
    assert check_node_alive(node, target_node) is True


@settings(verbosity=Verbosity.verbose)
@given(floats(), floats())
def fuzz_goal_message_position_effort(node, target_node, pub, position, effort):
    message = GoalRotaryServo()
    message.control_type = GoalRotaryServo.CONTROL_TYPE_POSITION_EFFORT
    message.position = position
    message.effort = effort
    message.header = Header()
    pub.publish(message)
    assert check_node_alive(node, target_node) is True


@settings(verbosity=Verbosity.verbose)
@given(floats())
def fuzz_goal_message_velocity(node, target_node, pub, velocity):
    message = GoalRotaryServo()
    message.control_type = GoalRotaryServo.CONTROL_TYPE_VELOCITY
    message.velocity = velocity
    message.header = Header()
    pub.publish(message)
    assert check_node_alive(node, target_node) is True

@settings(verbosity=Verbosity.verbose)
@given(floats(), floats())
def fuzz_goal_message_velocity_effort(node, target_node, pub, velocity, effort):
    message = GoalRotaryServo()
    message.control_type = GoalRotaryServo.CONTROL_TYPE_VELOCITY_EFFORT
    message.velocity = velocity
    message.effort = effort
    message.header = Header()
    pub.publish(message)
    assert check_node_alive(node, target_node) is True

@settings(verbosity=Verbosity.verbose)
@given(floats())
def fuzz_goal_message_effort(node, target_node, pub, effort):
    message = GoalRotaryServo()
    message.control_type = GoalRotaryServo.CONTROL_TYPE_EFFORT
    message.effort = effort
    message.header = Header()
    pub.publish(message)
    assert check_node_alive(node, target_node) is True

@settings(verbosity=Verbosity.verbose, max_examples=50000)
@given(text())
def fuzz_header_frameid_goal(node, target_node, pub, frame_id):
    message = GoalRotaryServo()
    message.control_type = GoalRotaryServo.CONTROL_TYPE_POSITION
    message.position = 0.0
    message.header = Header()
    message.header.frame_id = frame_id
    try:
    	pub.publish(message)
    except:
       pass
    assert check_node_alive(node, target_node) is True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('topic', help='Target topic for performing fuzzing')
    parser.add_argument('target_node', help='Target node for performing fuzzing')
    args = parser.parse_args()
    rclpy.init()

    node = rclpy.create_node('fuzzer_node')
    
    publisher = node.create_publisher(GoalRotaryServo, args.topic)
    fuzz_goal_message_all(publisher)
    fuzz_goal_message_position(publisher)
    fuzz_goal_message_position_velocity(publisher)
    fuzz_goal_message_position_effort(publisher)
    fuzz_goal_message_velocity(publisher)
    fuzz_goal_message_velocity_effort(publisher)
    fuzz_goal_message_effort(publisher)
    fuzz_header_frameid_goal(node, args.target_node, publisher)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()





