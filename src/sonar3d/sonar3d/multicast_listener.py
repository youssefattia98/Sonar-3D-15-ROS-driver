import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import socket
import os
import datetime
from pathlib import Path
import math
import yaml
from ament_index_python.packages import get_package_share_directory

from wlsonar import Sonar3D, range_image_to_xyz
from wlsonar.range_image_protocol import (
    unpackb,
    RangeImage,
    BitmapImageGreyscale8,
    ImuBatch,
    BadIDError,
    CRCMismatchError,
    ExtraDataError,
    UnknownProtobufTypeError,
)
from sonar3d.imu_conversion import imu_batch_to_messages


def set_acoustics(ip, enabled):
    try:
        Sonar3D(ip).set_acoustics_enabled(bool(enabled))
        return True, None
    except Exception as e:
        return False, str(e)


def enable_multicast(ip):
    try:
        Sonar3D(ip).set_udp_multicast()
        return True, None
    except Exception as e:
        return False, str(e)


def describe_response(_ip, response):
    ok, err = response
    if ok:
        return 'success'
    return f'failed ({err})' if err else 'failed'

class TimerNode(Node):

    # Multicast group and port used by the Sonar 3D-15
    MULTICAST_GROUP = '224.0.0.96'
    PORT = 4747

    # The maximum possible packet size for Sonar 3D-15 data
    BUFFER_SIZE = 65535

    @staticmethod
    def _load_default_params_from_yaml() -> dict:
        defaults = {}
        try:
            share_dir = Path(get_package_share_directory('sonar3d'))
            params_file = share_dir / 'config' / 'sonar3d.params.yaml'
            if not params_file.exists():
                return defaults

            with params_file.open('r', encoding='utf-8') as f:
                content = yaml.safe_load(f) or {}

            node_params = {}
            if isinstance(content, dict):
                timer_node_cfg = content.get('timer_node', {})
                if isinstance(timer_node_cfg, dict):
                    node_params = timer_node_cfg.get('ros__parameters', {})

                if not node_params:
                    wildcard_cfg = content.get('/**', {})
                    if isinstance(wildcard_cfg, dict):
                        node_params = wildcard_cfg.get('ros__parameters', {})

            if isinstance(node_params, dict):
                defaults = node_params
        except Exception as e:
            # Keep node startup robust if config file is missing/malformed
            print(f"[sonar3d] Failed to load default params YAML: {e}")

        return defaults

    def __init__(self):
        super().__init__('timer_node')

        yaml_defaults = self._load_default_params_from_yaml()
        default_ip = str(yaml_defaults.get('IP', os.getenv('SONAR3D_IP', ''))).strip()
        default_speed_of_sound = int(yaml_defaults.get('speed_of_sound', 1491))
        default_local_interface_ip = str(yaml_defaults.get('local_interface_ip', '0.0.0.0')).strip()
        default_sync_time = bool(yaml_defaults.get('sync_time_on_startup', True))
        default_configure_speed = bool(yaml_defaults.get('configure_speed_of_sound', True))
        default_enable_imu = bool(yaml_defaults.get('enable_imu', True))
        default_correct_clock = bool(yaml_defaults.get('correct_sonar_clock_offset', True))
        default_sonar_frame = str(yaml_defaults.get('sonar_frame', 'sonar_frame')).strip()
        default_imu_frame = str(yaml_defaults.get('imu_frame', 'sonar_imu_frame')).strip()
        
        # Declare parameters
        self.declare_parameter('IP', default_ip)
        self.declare_parameter('speed_of_sound', default_speed_of_sound)    # setting this takes ~20s
        self.declare_parameter('local_interface_ip', default_local_interface_ip)
        self.declare_parameter('sync_time_on_startup', default_sync_time)
        self.declare_parameter('configure_speed_of_sound', default_configure_speed)
        self.declare_parameter('enable_imu', default_enable_imu)
        self.declare_parameter('correct_sonar_clock_offset', default_correct_clock)
        self.declare_parameter('sonar_frame', default_sonar_frame)
        self.declare_parameter('imu_frame', default_imu_frame)

        self.sonar_ip = self.get_parameter('IP').get_parameter_value().string_value.strip()
        self.sonar_speed_of_sound = self.get_parameter('speed_of_sound').get_parameter_value().integer_value
        self.local_interface_ip = self.get_parameter('local_interface_ip').get_parameter_value().string_value
        self.sync_time_on_startup = self.get_parameter('sync_time_on_startup').value
        self.configure_speed_of_sound = self.get_parameter('configure_speed_of_sound').value
        self.enable_imu = self.get_parameter('enable_imu').value
        self.correct_sonar_clock_offset = self.get_parameter(
            'correct_sonar_clock_offset'
        ).value
        self.sonar_frame = self.get_parameter('sonar_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.sonar_clock_offset_ns = None
        self.last_sonar_timestamp_ns = None

        # Create a timer that calls the timer_callback every sample_time seconds 
        sample_time = 0.01          # sample time in seconds
        self.create_timer(sample_time, self.timer_callback)
        self.get_logger().info(f'Timer Node initialized with {1/sample_time} Hz')

        # Create a publisher that publishes the point cloud data
        # Retain only the newest cloud. TRANSIENT_LOCAL offered by the publisher
        # is compatible with both VOLATILE and TRANSIENT_LOCAL subscribers.
        pointcloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pointcloud_publisher_ = self.create_publisher(
            PointCloud2, 'sonar_point_cloud', pointcloud_qos
        )
        self.imu_publisher_ = self.create_publisher(
            Imu, '/sonr/imu/data_raw', 100
        )
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_imu_transform()

        if self.sonar_ip:
            self._configure_sonar()

            # Enable the acoustics on the sonar
            resp = set_acoustics(self.sonar_ip, True)
            self.get_logger().info(f'Enabling acoustics response: {describe_response(self.sonar_ip, resp)}')

            resp = enable_multicast(self.sonar_ip)
            self.get_logger().info(f'Enabling multicast response: {describe_response(self.sonar_ip, resp)}')
        else:
            self.get_logger().warning(
                "Parameter 'IP' is empty. Skipping REST configuration and accepting packets from any source IP. "
                "Set SONAR3D_IP or pass --ros-args -p IP:=<sonar_ip> to enable filtering/configuration."
            )

        # Set up a UDP socket with multicast membership
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.PORT))

        joined_ip = self._join_multicast_group()

        self.get_logger().info(
            f"Joined multicast {self.MULTICAST_GROUP}:{self.PORT} via interface {joined_ip}"
        )
        self.get_logger().info(f"Listening for Sonar 3D-15 RIP packets on {self.MULTICAST_GROUP}:{self.PORT}...")

        if self.sonar_ip:
            self.get_logger().info(f"Filtering packets from IP: {self.sonar_ip}")

    def _configure_sonar(self):
        """Apply settings which must be correct before timestamped data arrives."""
        try:
            sonar = Sonar3D(self.sonar_ip)
        except Exception as e:
            self.get_logger().warning(
                f"Could not connect to sonar configuration API: {e}"
            )
            return

        if self.sync_time_on_startup:
            try:
                time_status = sonar.get_time_status()
                if time_status.ntp_synced:
                    self.get_logger().info(
                        f"Sonar clock is synchronized to {time_status.ntp_synced_to}"
                    )
                else:
                    sonar.set_time_manual(datetime.datetime.now(datetime.timezone.utc))
                    self.get_logger().info("Synchronized sonar clock to this computer (UTC)")
            except Exception as e:
                self.get_logger().warning(f"Could not synchronize sonar clock: {e}")

        if self.configure_speed_of_sound and self.sonar_speed_of_sound > 0:
            try:
                configured_speed = float(sonar.get_speed_of_sound())
                target_speed = float(self.sonar_speed_of_sound)
                if abs(configured_speed - target_speed) > 0.5:
                    sonar.set_speed_of_sound(target_speed)
                    self.get_logger().info(
                        f"Set sonar speed of sound to {target_speed:.1f} m/s "
                        f"(was {configured_speed:.1f} m/s)"
                    )
                else:
                    self.get_logger().info(
                        f"Sonar speed of sound is {configured_speed:.1f} m/s"
                    )
            except Exception as e:
                self.get_logger().warning(
                    f"Could not configure sonar speed of sound: {e}"
                )

        if self.enable_imu:
            try:
                sonar.set_output_imu_batch_enabled(True)
                self.get_logger().info('Enabled sonar ImuBatch output')
            except Exception as e:
                self.get_logger().warning(
                    f"Could not enable sonar ImuBatch output: {e}"
                )

    def _publish_imu_transform(self):
        """Publish the internal IMU position relative to the point-cloud origin."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.sonar_frame
        transform.child_frame_id = self.imu_frame

        # Water Linked specifies (-22, -46, -3) mm in sonar FRD. In ROS FLU
        # this is (-22, +46, +3) mm. Both FLU frames have aligned axes.
        transform.transform.translation.x = -0.022
        transform.transform.translation.y = 0.046
        transform.transform.translation.z = 0.003
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)


    def timer_callback(self):

        data, addr = self.sock.recvfrom(self.BUFFER_SIZE)
        received_at = self.get_clock().now()

        # If SONAR_IP is configured and this doesn't match the known Sonar IP, skip it.
        if self.sonar_ip and addr[0] != self.sonar_ip:
            self.get_logger().info(f"Received packet from {addr[0]}. Data was received from an IP that does not match the declared SONAR_IP ({self.sonar_ip}), so the packet will be skipped.")
            return

        try:
            msg_obj = unpackb(
                data,
                known_message_types=(RangeImage, BitmapImageGreyscale8, ImuBatch),
                max_packet_size=self.BUFFER_SIZE,
            )
        except UnknownProtobufTypeError:
            # Future protocol messages can be safely ignored by this driver.
            return
        except (BadIDError, CRCMismatchError, ExtraDataError, ValueError) as e:
            self.get_logger().warning(f"Failed to decode RIP packet: {e}")
            return

        if isinstance(msg_obj, RangeImage):
            # Convert the RangeImage message to voxel data
            raw_pts = range_image_to_xyz(msg_obj)
            # Publish in ROS FLU by mirroring sensor FRD lateral/vertical axes:
            # x: forward (unchanged), y: right->left, z: down->up.
            # The Water Linked converter returns None for pixels without a
            # valid range.  Those pixels are expected and must not terminate
            # the entire ROS node.
            pts = []
            for point in raw_pts:
                if point is None:
                    continue
                try:
                    x, y, z = point
                    x, y, z = float(x), float(y), float(z)
                except (TypeError, ValueError):
                    continue
                if not all(math.isfinite(value) for value in (x, y, z)):
                    continue
                pts.append((x, -y, -z))

            # Create a PointCloud2 message
            # Create the msg header
            header = Header()
            # Preserve the acquisition time supplied by the sonar. Stamping here
            # with the callback time would include network and processing latency.
            timestamp_ns = self._sonar_timestamp_ns(msg_obj.header.timestamp)
            corrected_timestamp_ns = timestamp_ns + (self.sonar_clock_offset_ns or 0)
            header.stamp.sec, header.stamp.nanosec = divmod(
                corrected_timestamp_ns, 1_000_000_000
            )
            header.frame_id = self.sonar_frame

            cloud_msg = point_cloud2.create_cloud_xyz32(header, pts)

            # Publish the PointCloud2 message
            self.pointcloud_publisher_.publish(cloud_msg)

        elif isinstance(msg_obj, BitmapImageGreyscale8):
            # The driver intentionally does not publish 2D sonar images.
            return
        elif isinstance(msg_obj, ImuBatch):
            try:
                if msg_obj.timestamp:
                    self._update_sonar_clock_offset(
                        msg_obj.timestamp[-1], received_at.nanoseconds
                    )
                messages = imu_batch_to_messages(
                    msg_obj,
                    self.imu_frame,
                    self.sonar_clock_offset_ns or 0,
                )
            except ValueError as e:
                self.get_logger().warning(str(e))
                return
            for imu_msg in messages:
                self.imu_publisher_.publish(imu_msg)

    @staticmethod
    def _sonar_timestamp_ns(timestamp):
        return timestamp.seconds * 1_000_000_000 + timestamp.nanos

    def _update_sonar_clock_offset(self, timestamp, received_at_ns):
        """Map the sonar UTC clock onto ROS time using minimum observed delay.

        The newest sample in each IMU batch has the smallest and most stable
        acquisition-to-reception delay. Taking the minimum observed offset
        removes the sonar clock error without adding callback or point-cloud
        processing latency to the acquisition timestamps.
        """
        if not self.correct_sonar_clock_offset:
            return

        timestamp_ns = self._sonar_timestamp_ns(timestamp)
        rebooted = (
            self.last_sonar_timestamp_ns is not None
            and timestamp_ns + 5_000_000_000 < self.last_sonar_timestamp_ns
        )
        observed_offset_ns = received_at_ns - timestamp_ns

        if rebooted:
            self.sonar_clock_offset_ns = None
            self.get_logger().warning(
                'Sonar timestamp jumped backwards; resetting clock synchronization'
            )

        previous_offset_ns = self.sonar_clock_offset_ns
        if previous_offset_ns is None or observed_offset_ns < previous_offset_ns:
            self.sonar_clock_offset_ns = observed_offset_ns
            if previous_offset_ns is None:
                self.get_logger().info(
                    'Sonar-to-ROS clock offset initialized to '
                    f'{observed_offset_ns / 1e9:+.6f} s'
                )

        self.last_sonar_timestamp_ns = timestamp_ns

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
            if not target_ip:
                return None
            tmp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tmp.connect((target_ip, 9))
            local_ip = tmp.getsockname()[0]
            tmp.close()
            return local_ip
        except Exception:
            return None

def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = TimerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C is a normal shutdown, including while the offline sonar's
        # REST endpoint is still being contacted during node construction.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
