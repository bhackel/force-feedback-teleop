#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import re
import yaml
from pathlib import Path
from enum import IntEnum
from teleop_interfaces.msg import HandPositions, FingerPosition


class Finger(IntEnum):
    THUMB = 0
    INDEX = 1
    MIDDLE = 2
    RING = 3
    PINKY = 4


class LucidglovesController(Node):
    def __init__(self):
        super().__init__('lucidgloves_controller')

        # Load configuration
        self.load_config()

        # Initialize serial connection
        self.init_serial()

        # Create ROS interfaces
        self.setup_ros_interfaces()

        # Create timer to read serial data
        self.timer = self.create_timer(0.001, self.read_serial_data)  # 1000Hz (matches firmware LOOP_TIME=1ms)

        self.get_logger().info('Lucidgloves Controller initialized')

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
        self.baud_rate = config.get('baud_rate', 115200)

        self.get_logger().info(f'Loaded config: COM port = {self.com_port}, baud = {self.baud_rate}')

    def init_serial(self) -> None:
        """Initialize serial connection to lucidgloves"""
        self.serial = serial.Serial(
            port=self.com_port,
            baudrate=self.baud_rate,
            timeout=0.1
        )
        self.get_logger().info(f'Connected to lucidgloves on {self.com_port}')

    def setup_ros_interfaces(self) -> None:
        """Setup ROS publishers"""
        # Create publisher for finger positions
        self.publisher = self.create_publisher(
            HandPositions,
            'finger_positions_fractional',
            10
        )
        self.get_logger().info('Publisher created for finger_positions_fractional')

    def read_serial_data(self):
        """Read and parse data from serial port"""
        if self.serial.in_waiting > 0:
            try:
                # Read line from serial (format: A123B456C789D012E345F678G890P123...\n)
                line = self.serial.readline().decode('utf-8').strip()

                if line:
                    finger_data = self.parse_lucidgloves_data(line)
                    if finger_data:
                        # Publish the data
                        self.publish_finger_data(finger_data)

            except Exception as e:
                self.get_logger().error(f'Error reading serial: {e}')

    def parse_lucidgloves_data(self, data_string: str) -> dict:
        """Parse lucidgloves Alpha encoding format

        Format: A%dB%dC%dD%dE%d...(AB)%d(BB)%d(CB)%d(DB)%d(EB)%d...\n
        - A-E: flexion values for thumb, index, middle, ring, pinky
        - (AB)-(EB): splay values for thumb, index, middle, ring, pinky

        Returns:
            dict: {
                Finger.THUMB: {'flexion': int, 'splay': int},
                Finger.INDEX: {'flexion': int, 'splay': int},
                Finger.MIDDLE: {'flexion': int, 'splay': int},
                Finger.RING: {'flexion': int, 'splay': int},
                Finger.PINKY: {'flexion': int, 'splay': int}
            }
        """
        finger_data = {}

        # Parse flexion values (A-E)
        flexion_matches = {
            Finger.THUMB: re.search(r'A(\d+)', data_string),
            Finger.INDEX: re.search(r'B(\d+)', data_string),
            Finger.MIDDLE: re.search(r'C(\d+)', data_string),
            Finger.RING: re.search(r'D(\d+)', data_string),
            Finger.PINKY: re.search(r'E(\d+)', data_string)
        }

        # Parse splay values (AB-EB)
        splay_matches = {
            Finger.THUMB: re.search(r'\(AB\)(\d+)', data_string),
            Finger.INDEX: re.search(r'\(BB\)(\d+)', data_string),
            Finger.MIDDLE: re.search(r'\(CB\)(\d+)', data_string),
            Finger.RING: re.search(r'\(DB\)(\d+)', data_string),
            Finger.PINKY: re.search(r'\(EB\)(\d+)', data_string)
        }

        # Build the finger data dict
        for finger in [Finger.THUMB, Finger.INDEX, Finger.MIDDLE, Finger.RING, Finger.PINKY]:
            flexion = int(flexion_matches[finger].group(1)) if flexion_matches[finger] else 0
            splay = int(splay_matches[finger].group(1)) if splay_matches[finger] else 0

            finger_data[finger] = {
                'flexion': flexion,
                'splay': splay
            }

        return finger_data

    def normalize_values(self, flexion_raw: int, splay_raw: int) -> tuple:
        """Convert raw lucidgloves values to normalized values

        Args:
            flexion_raw: Raw flexion value from lucidgloves (0-4095)
            splay_raw: Raw splay value from lucidgloves (0-4095)

        Returns:
            tuple: (flexion, splay) where flexion is 0.0-1.0 and splay is -1.0 to 1.0
        """
        ANALOG_MAX = 4095  # ESP32 analog max value

        # Normalize flexion to 0.0-1.0
        flexion = flexion_raw / ANALOG_MAX

        # Normalize splay from 0-4095 to -1.0 to 1.0
        # Center point is ANALOG_MAX/2
        splay = (splay_raw - ANALOG_MAX / 2) / (ANALOG_MAX / 2)

        # Clamp values to valid ranges
        flexion = max(0.0, min(1.0, flexion))
        splay = max(-1.0, min(1.0, splay))

        return flexion, splay

    def publish_finger_data(self, finger_data: dict) -> None:
        """Publish finger data to ROS topic

        Args:
            finger_data: Dict with finger data from parse_lucidgloves_data
        """
        msg = HandPositions()

        # Mapping from Finger enum to message field names
        finger_to_field = {
            Finger.THUMB: 'thumb',
            Finger.INDEX: 'index',
            Finger.MIDDLE: 'middle',
            Finger.RING: 'ring',
            Finger.PINKY: 'pinky'
        }

        # Convert and populate each finger
        for finger in [Finger.THUMB, Finger.INDEX, Finger.MIDDLE, Finger.RING, Finger.PINKY]:
            flexion, splay = self.normalize_values(
                finger_data[finger]['flexion'],
                finger_data[finger]['splay']
            )

            finger_pos = FingerPosition()
            finger_pos.flexion = flexion
            finger_pos.splay = splay

            # Set the appropriate field in the message
            setattr(msg, finger_to_field[finger], finger_pos)

        # Publish the message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LucidglovesController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
