#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from pathlib import Path
from enum import IntEnum
from dataclasses import dataclass
from rustypot import Scs0009PyController
from teleop_interfaces.msg import HandPositions


class Finger(IntEnum):
    INDEX = 0
    MIDDLE = 1
    PINKY = 2
    THUMB = 3


@dataclass(frozen=True)
class ServoOffsets:
    servo1: float
    servo2: float


# Mapping from Finger enum to servo IDs
FINGER_TO_SERVOS = {
    Finger.INDEX: (1, 2),
    Finger.MIDDLE: (3, 4),
    Finger.PINKY: (5, 6),
    Finger.THUMB: (7, 8)
}


class AmazingHandController(Node):
    def __init__(self):
        super().__init__('amazing_hand_controller')

        # Load configuration
        self.load_config()

        # Initialize AmazingHand hardware connection
        self.init_hardware()

        # Create publishers and subscribers
        self.setup_ros_interfaces()

        self.get_logger().info('AmazingHand Controller initialized')

    def load_config(self) -> None:
        """Load configuration from YAML file"""
        config_path = Path(__file__).parent / 'config.yaml'

        if not config_path.exists():
            raise FileNotFoundError(f'Config file not found: {config_path}')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        if config is None:
            raise ValueError(f'Config file is empty: {config_path}')

        if 'com_port' not in config:
            raise KeyError('Required config key "com_port" not found in config.yaml')

        self.com_port = config['com_port']

        # Convert middle_pos list to dict with named offsets
        middle_pos_list = config.get('middle_pos', [])
        self.middle_pos = {
            Finger.INDEX: ServoOffsets(middle_pos_list[0], middle_pos_list[1]),
            Finger.MIDDLE: ServoOffsets(middle_pos_list[2], middle_pos_list[3]),
            Finger.PINKY: ServoOffsets(middle_pos_list[4], middle_pos_list[5]),
            Finger.THUMB: ServoOffsets(middle_pos_list[6], middle_pos_list[7])
        }

        self.get_logger().info(f'Loaded config: COM port = {self.com_port}')
        self.get_logger().info(f'Loaded config: Middle positions = {self.middle_pos}')

    def init_hardware(self) -> None:
        """Initialize connection to AmazingHand hardware"""
        self.controller = self.get_serial_connection(self.com_port)
        self.get_logger().info('Connected, configuring servos...')
        
        # Enable torque (position) mode at 20% for all servos
        servos = list(range(1, 9))  # 8 servos with IDs 1-8
        self.controller.write_torque_enable(servos, [True]*8)
        self.controller.write_max_torque_limit(servos, [20.0]*8)
        self.get_logger().info('Servos configured with 20% torque limit')
    
    def get_serial_connection(self, com_port) -> 'Scs0009PyController':
        """Initialize serial connection to the AmazingHand hardware
        
        Args:
            com_port (str): The serial port to connect to.
        
        Returns:
            Scs0009PyController: The initialized controller object.
        """
        self.get_logger().info(f'Connecting to AmazingHand on {com_port}')
        return Scs0009PyController(
            serial_port=com_port,
            baudrate=1000000,
            timeout=0.05,
        )

    def setup_ros_interfaces(self) -> None:
        """Setup ROS publishers and subscribers"""
        # Initialize last known positions (start at 0-point: open and centered)
        self.last_positions = {
            Finger.INDEX: (0.0, 0.0),    # (flexion, splay)
            Finger.MIDDLE: (0.0, 0.0),
            Finger.PINKY: (0.0, 0.0),
            Finger.THUMB: (0.0, 0.0)
        }

        # Create 10Hz timer for commanding servos
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create subscriber for finger position topic
        self.subscription = self.create_subscription(
            HandPositions,
            'finger_positions_fractional',
            self.finger_positions_callback,
            10
        )

    def timer_callback(self):
        """Timer callback to command servos at 10Hz"""
        # Command all 4 fingers using last known positions
        for finger in [Finger.INDEX, Finger.MIDDLE, Finger.PINKY, Finger.THUMB]:
            flexion, splay = self.last_positions[finger]
            self.command_finger(finger, flexion, splay)

    def finger_positions_callback(self, msg):
        """Callback for receiving finger position commands from topic.

        Message contains 5 fingers (index, middle, ring, pinky, thumb).
        We drop ring finger data and update last_positions for the other 4.
        """
        # Update last known positions (drop ring finger)
        self.last_positions[Finger.INDEX] = (msg.index.flexion, msg.index.splay)
        self.last_positions[Finger.MIDDLE] = (msg.middle.flexion, msg.middle.splay)
        self.last_positions[Finger.PINKY] = (msg.pinky.flexion, msg.pinky.splay)
        self.last_positions[Finger.THUMB] = (msg.thumb.flexion, msg.thumb.splay)

    # temp notes:
    # read position data from some topic at some frequency
    # use a timer callback
    # this topic will have position data for 5 fingers
    # we drop 1 finger (ring finger)
    # map the 4 fingers to 8 servos
    #     use the AmazingHand_Demo.py as reference for control commands
    #     offset by the middle_pos calibration data
    #     clamp the servo values before sending for safety
    # send position commands to servos using the controller object

    # for the topic's data format
    #     each finger has 2 DOF
    #     I think we just use 100% to 0% to -100% range for each finger DOF
    #     No that doesnt work because flexion should have 0% at fully extended
    #     But for splay, the range should be -100% to 100% with 0% in middle
    #     For this right-handed system, with the palm downwards
    #         -100% means the finger moves left
    #            0% means the finger is in middle position
    #         +100% means the finger moves right
    #     We should use floats: -1.0 to 1.0 for splay, and 0.0 to 1.0 for flexion

    # Additional implementation details:
    # - Define custom message with named fields for 5 fingers (index, middle, ring, pinky, thumb)
    # - 10Hz timer callback frequency
    # - On topic timeout: hold last known position
    # - Start at 0-point position on initialization
    # - No filtering in this node (handled upstream)
    # - drop the ring finger data
    # - Servo mapping: Index[1,2], Middle[3,4], Pinky[5,6], Thumb[7,8]
    
    # control translation details
    # open finger: servos -35 deg, 35 deg
    # close finger: servos 90 deg, -90 deg
    # splay left: -15 deg, 65 deg
    # splay right: -65 deg, 15 deg
    
    #   This is a differential drive mechanism. Each finger has two motors working together where:
    #   The mapping appears to be:
    #   - Sum (m1 + m2) controls splay (side-to-side movement)
    #   - Difference (m2 - m1) controls flexion (open/close)
    #
    #   Let me verify with your values:
    #   Open (-35, 35):
    #   - Sum: -35 + 35 = 0 → no splay
    #   - Diff: 35 - (-35) = 70 → extended/open
    #
    #   Close (90, -90):
    #   - Sum: 90 + (-90) = 0 → no splay
    #   - Diff: -90 - 90 = -180 → flexed/closed
    #
    #   Splay left (-15, 65):
    #   - Sum: -15 + 65 = 50 → splay left
    #   - Diff: 65 - (-15) = 80 → partially extended
    #
    #   Splay right (-65, 15):
    #   - Sum: -65 + 15 = -50 → splay right
    #   - Diff: 15 - (-65) = 80 → partially extended
    #
    #   So to convert from your normalized values to motor commands:
    #   # flexion ∈ [0.0, 1.0], splay ∈ [-1.0, 1.0]
    #   m1 = splay_angle + flexion_angle
    #   m2 = -splay_angle + flexion_angle
    
    # another data format option for the topic is to do angles
    # this makes more sense because its less likely to cause collisions
    #     with the hand's fingers into itself
    # but this is more complex
    # so we will name this topic "finger_positions_fractional"
    # and develop the other one later
    
    def convert_finger_fractionals_to_servo_radians(self, finger, flexion, splay):
        """Convert normalized finger positions to servo angles in radians.

        Args:
            finger (Finger): Finger enum value [INDEX, MIDDLE, PINKY, THUMB]
            flexion (float): Flexion amount [0.0=open, 1.0=closed]
            splay (float): Splay amount [-1.0=left, 0.0=middle, 1.0=right]

        Returns:
            tuple: (m1_radians, m2_radians) for the two servos
        """

        # Map flexion [0, 1] to base motor angles
        # flexion=0 (open): -35 deg, flexion=1 (closed): 90 deg
        flexion_angle = -35 + flexion * 125  # ranges from -35 to 90 degrees

        # Map splay [-1, 1] to differential offset
        # This creates the side-to-side movement
        splay_angle = splay * 50  # ranges from -50 to 50 degrees

        # Apply differential drive mechanism
        # m1 and m2 work together: splay creates differential, flexion is common
        m1_deg = flexion_angle - splay_angle
        m2_deg = -flexion_angle - splay_angle

        # Apply middle_pos calibration offset
        m1_deg += self.middle_pos[finger].servo1
        m2_deg += self.middle_pos[finger].servo2

        # Convert to radians
        m1_rad = np.deg2rad(m1_deg)
        m2_rad = np.deg2rad(m2_deg)

        return (m1_rad, m2_rad)

    def command_finger(self, finger, flexion, splay, speed=7):
        """Send position commands to a finger's servos.

        Args:
            finger (Finger): Finger enum value [INDEX, MIDDLE, PINKY, THUMB]
            flexion (float): Flexion amount [0.0=open, 1.0=closed]
            splay (float): Splay amount [-1.0=left, 0.0=middle, 1.0=right]
            speed (int): Servo speed (default 7)
        """
        # Get servo IDs for this finger
        servo1_id, servo2_id = FINGER_TO_SERVOS[finger]

        # Convert fractional inputs to radians (with middle_pos offsets applied)
        m1_rad, m2_rad = self.convert_finger_fractionals_to_servo_radians(finger, flexion, splay)

        # Send speed commands
        self.controller.write_goal_speed(servo1_id, speed)
        self.controller.write_goal_speed(servo2_id, speed)

        # Send position commands
        self.controller.write_goal_position(servo1_id, m1_rad)
        self.controller.write_goal_position(servo2_id, m2_rad)






def main(args=None):
    rclpy.init(args=args)
    node = AmazingHandController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
