import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from sonar3d.api.interface_sonar_api import set_acoustics, describe_response, enable_multicast
import socket
import struct
import numpy as np
import sys
from pathlib import Path
import math
import zlib
import snappy
import importlib
from google.protobuf import descriptor_pool, message_factory

# Ensure local wlsonar package is on the path (vendored under the repo)
_wlsonar_src = Path(__file__).resolve().parents[1] / 'wlsonar' / 'src'
if _wlsonar_src.exists() and str(_wlsonar_src) not in sys.path:
    sys.path.insert(0, str(_wlsonar_src))

from wlsonar import range_image_to_xyz
from wlsonar.range_image_protocol import (
    unpackb,
    RangeImage,
    BitmapImageGreyscale8,
    BadIDError,
    CRCMismatchError,
    ExtraDataError,
    UnknownProtobufTypeError,
    Packet,
    ProtocolVersion,
)

class TimerNode(Node):

    # Multicast group and port used by the Sonar 3D-15
    MULTICAST_GROUP = '224.0.0.96'
    PORT = 4747

    # The maximum possible packet size for Sonar 3D-15 data
    BUFFER_SIZE = 65535

    def __init__(self):
        super().__init__('timer_node')
        
        # Declare parameters
        self.declare_parameter('IP', '192.168.194.96')# '192.168.194.96' is the fallback ip, to change this, edit the launchfile.
        self.declare_parameter('speed_of_sound', 1491)    # setting this takes ~20s
        self.declare_parameter('local_interface_ip', '0.0.0.0')
        self.declare_parameter('imu_proto_module', '')

        self.sonar_ip = self.get_parameter('IP').get_parameter_value().string_value
        self.sonar_speed_of_sound = self.get_parameter('speed_of_sound').get_parameter_value().integer_value
        self.local_interface_ip = self.get_parameter('local_interface_ip').get_parameter_value().string_value
        self.imu_proto_module = self.get_parameter('imu_proto_module').get_parameter_value().string_value

        # Create a timer that calls the timer_callback every sample_time seconds 
        sample_time = 0.01          # sample time in seconds
        self.create_timer(sample_time, self.timer_callback)
        self.get_logger().info(f'Timer Node initialized with {1/sample_time} Hz')

        if self.imu_proto_module:
            try:
                importlib.import_module(self.imu_proto_module)
                self.get_logger().info(
                    f"Loaded IMU proto module: {self.imu_proto_module}"
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Failed to load IMU proto module '{self.imu_proto_module}': {e}"
                )

        # Create a publisher that publishes the point cloud data
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, 'sonar_point_cloud', 10)
        self.image_publisher_ = self.create_publisher(Image, 'sonar_range_image', 10)
        self.imu_publisher_ = self.create_publisher(Imu, 'sonar_imu', 10)

        # Enable the acoustics on the sonar
        resp = set_acoustics(self.sonar_ip, True)
        self.get_logger().info(f'Enabling acoustics response: {describe_response(self.sonar_ip, resp)}')

        resp = enable_multicast(self.sonar_ip)
        self.get_logger().info(f'Enabling multicast response: {describe_response(self.sonar_ip, resp)}')

        # Set up a UDP socket with multicast membership
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.PORT))

        joined_ip = self._join_multicast_group()

        self.get_logger().info(
            f"Joined multicast {self.MULTICAST_GROUP}:{self.PORT} via interface {joined_ip}"
        )
        self.get_logger().info(f"Listening for Sonar 3D-15 RIP packets on {self.MULTICAST_GROUP}:{self.PORT}...")

        if self.sonar_ip != "":
            self.get_logger().info(f"Filtering packets from IP: {self.sonar_ip}")


    def timer_callback(self):

        data, addr = self.sock.recvfrom(self.BUFFER_SIZE)

        # If SONAR_IP is configured, and this doesn't match the known Sonar IP, skip it.
        if not (addr[0] == self.sonar_ip or addr[0] == '192.168.194.96'):
            self.get_logger().info(f"Received packet from {addr[0]}. Data was received from an IP that does not match the declared SONAR_IP ({self.sonar_ip}), so the packet will be skipped.")
            return

        try:
            msg_obj = unpackb(
                data,
                known_message_types=(RangeImage, BitmapImageGreyscale8),
                max_packet_size=self.BUFFER_SIZE,
            )
        except UnknownProtobufTypeError as e:
            # Attempt to decode unknown Any messages (e.g., IMU orientation)
            any_msg = self._extract_any_from_rip_packet(data)
            if any_msg is None:
                return
            self._handle_any_message(any_msg)
            return
        except (BadIDError, CRCMismatchError, ExtraDataError, ValueError) as e:
            # Unknown message types may surface as ValueError via different module copies
            if "Unknown message type in Range Image Protocol Packet" in str(e):
                any_msg = self._extract_any_from_rip_packet(data)
                if any_msg is None:
                    return
                self._handle_any_message(any_msg)
                return
            self.get_logger().warning(f"Failed to decode RIP packet: {e}")
            return

        if isinstance(msg_obj, RangeImage):
            # Convert the RangeImage message to voxel data
            pts = range_image_to_xyz(msg_obj)

            # Create a PointCloud2 message
            # Create the msg header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()  # Use ROS2 time
            header.frame_id = 'sonar_frame'

            cloud_msg = point_cloud2.create_cloud_xyz32(header, pts)

            # Publish the PointCloud2 message
            self.pointcloud_publisher_.publish(cloud_msg)

            # Publish the raw range image
            img_msg = Image()
            img_msg.header = header
            img_msg.height = msg_obj.height
            img_msg.width = msg_obj.width
            img_msg.encoding = '32FC1'
            img_msg.is_bigendian = False
            img_msg.step = msg_obj.width * 4
            range_image = (np.array(msg_obj.image_pixel_data, dtype=np.uint32) * msg_obj.image_pixel_scale).astype(np.float32)
            img_msg.data = range_image.tobytes()
            self.image_publisher_.publish(img_msg)
        elif isinstance(msg_obj, BitmapImageGreyscale8):
            # Currently not published as a separate topic
            return

    def _join_multicast_group(self):
        group = socket.inet_aton(self.MULTICAST_GROUP)

        # Try configured interface first
        candidate_ip = self.local_interface_ip
        if candidate_ip:
            try:
                mreq = group + socket.inet_aton(candidate_ip)
                self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                return candidate_ip
            except OSError as e:
                self.get_logger().warning(
                    f"Failed to join multicast on {candidate_ip}: {e}. Trying auto-detect."
                )

        # Auto-detect local IP that can reach sonar
        auto_ip = self._detect_local_ip(self.sonar_ip)
        if auto_ip:
            mreq = group + socket.inet_aton(auto_ip)
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            return auto_ip

        raise OSError("No valid local interface IP found for multicast join")

    def _detect_local_ip(self, target_ip: str) -> str | None:
        try:
            tmp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tmp.connect((target_ip, 9))
            local_ip = tmp.getsockname()[0]
            tmp.close()
            return local_ip
        except Exception:
            return None

    def _extract_any_from_rip_packet(self, data: bytes):
        if len(data) < 12:
            return None

        magic = data[:4]
        if magic not in (ProtocolVersion.RIP1.bytes(), ProtocolVersion.RIP2.bytes()):
            return None

        total_length = struct.unpack('<I', data[4:8])[0]
        if total_length < 12 or total_length > len(data):
            return None

        payload = data[8: total_length - 4]
        crc_received = struct.unpack('<I', data[total_length - 4: total_length])[0]
        crc_calculated = zlib.crc32(data[: total_length - 4]) & 0xffffffff
        if crc_calculated != crc_received:
            return None

        if magic == ProtocolVersion.RIP2.bytes():
            try:
                payload = snappy.uncompress(payload)
            except Exception:
                return None

        packet = Packet()
        try:
            packet.ParseFromString(payload)
        except Exception:
            return None

        if not packet.msg.IsInitialized():
            return None

        return packet.msg

    def _handle_any_message(self, any_msg):
        type_name = any_msg.TypeName()
        msg = self._decode_any_message(any_msg)
        if msg is None:
            self.get_logger().warning(
                f"RIP packet contains unknown message type '{type_name}', no decoder available."
            )
            return

        if type_name.endswith('ImuOrientation'):
            self._publish_imu(msg)
            return

        self.get_logger().debug(f"Unhandled RIP Any message type: {type_name}")

    def _decode_any_message(self, any_msg):
        type_name = any_msg.TypeName()
        try:
            desc = descriptor_pool.Default().FindMessageTypeByName(type_name)
        except KeyError:
            return None

        msg_cls = message_factory.MessageFactory().GetPrototype(desc)
        msg = msg_cls()
        try:
            msg.ParseFromString(any_msg.value)
        except Exception:
            return None
        return msg

    def _publish_imu(self, imu_msg):
        # Attempt to map fields from internal IMU message to sensor_msgs/Imu
        orientation = self._extract_quaternion(imu_msg)
        angular_velocity = self._extract_vector3(imu_msg, ['angular_velocity', 'gyro'])
        linear_acceleration = self._extract_vector3(imu_msg, ['linear_acceleration', 'acceleration'])

        if orientation is None:
            self.get_logger().warning('IMU message missing orientation, skipping publish.')
            return

        imu_out = Imu()
        imu_out.header.stamp = self.get_clock().now().to_msg()
        imu_out.header.frame_id = 'sonar_frame'

        imu_out.orientation.x = orientation[0]
        imu_out.orientation.y = orientation[1]
        imu_out.orientation.z = orientation[2]
        imu_out.orientation.w = orientation[3]

        if angular_velocity is not None:
            imu_out.angular_velocity.x = angular_velocity[0]
            imu_out.angular_velocity.y = angular_velocity[1]
            imu_out.angular_velocity.z = angular_velocity[2]

        if linear_acceleration is not None:
            imu_out.linear_acceleration.x = linear_acceleration[0]
            imu_out.linear_acceleration.y = linear_acceleration[1]
            imu_out.linear_acceleration.z = linear_acceleration[2]

        self.imu_publisher_.publish(imu_out)

    def _extract_quaternion(self, msg):
        # Direct quaternion fields
        quat = self._extract_vector4(msg, ['orientation', 'quaternion'])
        if quat is not None:
            return quat

        # Individual fields
        for prefix in ['orientation', 'quat', 'q']:
            values = self._extract_components(msg, prefix, ['x', 'y', 'z', 'w'])
            if values is not None:
                return values

        # Roll/pitch/yaw
        rpy = self._extract_components(msg, '', ['roll', 'pitch', 'yaw'])
        if rpy is not None:
            roll, pitch, yaw = rpy
            return self._rpy_to_quaternion(roll, pitch, yaw)

        return None

    def _extract_vector3(self, msg, names):
        for name in names:
            if hasattr(msg, name):
                sub = getattr(msg, name)
                values = self._extract_components(sub, '', ['x', 'y', 'z'])
                if values is not None:
                    return values
            values = self._extract_components(msg, name, ['x', 'y', 'z'])
            if values is not None:
                return values
        return None

    def _extract_vector4(self, msg, names):
        for name in names:
            if hasattr(msg, name):
                sub = getattr(msg, name)
                values = self._extract_components(sub, '', ['x', 'y', 'z', 'w'])
                if values is not None:
                    return values
            values = self._extract_components(msg, name, ['x', 'y', 'z', 'w'])
            if values is not None:
                return values
        return None

    def _extract_components(self, msg, prefix, components):
        values = []
        for comp in components:
            name = f"{prefix}_{comp}" if prefix else comp
            if hasattr(msg, name):
                values.append(getattr(msg, name))
            else:
                return None
        return tuple(values)

    def _rpy_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return (x, y, z, w)

        
        
    

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()