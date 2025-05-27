import numpy as np
from quaternion import from_euler_angles, as_euler_angles

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
    VehicleCommand,
    VehicleLandDetected,
    VehicleAttitude,
)

from martian_mines_msgs.msg import ENULocalOdometry


def ned_to_enu(n, e, d):
    return (float(e), float(n), float(-d))


def enu_to_ned(e, n, u):
    return (float(n), float(e), float(-u))


def ned_to_enu_heading(heading):
    return heading + (np.pi / 2)


def enu_to_ned_heading(heading):
    return heading - (np.pi / 2)


def frd_to_flu_quaternion(x, y, z, w):
    q_enu = np.quaternion(float(w), float(x), float(-y), float(-z))
    q_90 = from_euler_angles((0, 0, np.pi / 2))

    result = q_90 * q_enu
    return (result.x, result.y, result.z, result.w)

def heading_from_quaternion(x, y, z, w):
    q = np.quaternion(float(w), float(x), float(y), float(z))
    angles = as_euler_angles(q)

    return angles[0]

def offboard_command(func):
    def wrapper(self, *args, **kwargs):
        if self.is_in_offboard:
            return func(self, *args, **kwargs)

    return wrapper


class Offboard:
    HEARTBEAT_THRESHOLD = 10

    def __init__(self, node: Node) -> None:
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.node = node

        # Publishers
        self._pub_trajectory_setpoint = self.node.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", px4_qos
        )
        self._pub_offboard_control_mode = self.node.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", px4_qos
        )
        self._pub_vehicle_command = self.node.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", px4_qos
        )

        self._pub_enu_local_position = self.node.create_publisher(
            ENULocalOdometry, "enu_local_odometry", px4_qos
        )

        # Subscribers
        self._sub_vehicle_local_position = self.node.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.vehicle_local_position_cb,
            px4_qos,
        )
        self._sub_vehicle_attitude = self.node.create_subscription(
            VehicleAttitude,
            "fmu/out/vehicle_attitude",
            self.vehicle_attitude_cb,
            px4_qos,
        )
        self._sub_vehicle_status = self.node.create_subscription(
            VehicleStatus,
            "fmu/out/vehicle_status",
            self.vehicle_status_cb,
            px4_qos,
        )
        self._sub_vehicle_land_detected = self.node.create_subscription(
            VehicleLandDetected,
            "fmu/out/vehicle_land_detected",
            self.vehicle_land_detected_cb,
            px4_qos,
        )

        self._vehicle_local_position = None
        self._vehicle_attitude = None
        self._vehicle_status = VehicleStatus()
        self._vehicle_land_detected = None
        self._enu_local_position = None

        self.timer = self.node.create_timer(0.1, self.timer_callback)
        self._heartbeat_counter = 0

    @property
    def enu_local_odom(self) -> ENULocalOdometry:
        return self._enu_local_position
    
    @property
    def is_ready(self) -> bool:
        return self._heartbeat_counter >= self.HEARTBEAT_THRESHOLD

    @property
    def is_armed(self) -> bool:
        return self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
    
    @property
    def is_in_offboard(self):
        return self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def vehicle_local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self._vehicle_local_position = msg

        if self._vehicle_attitude is None:
            return

        enu = ENULocalOdometry()
        enu.x, enu.y, enu.z = ned_to_enu(msg.x, msg.y, msg.z)
        enu.vx, enu.vy, enu.vz = ned_to_enu(msg.vx, msg.vy, msg.vz)
        enu.qx, enu.qy, enu.qz, enu.qw = frd_to_flu_quaternion(
            self._vehicle_attitude.q[1],
            self._vehicle_attitude.q[2],
            self._vehicle_attitude.q[3],
            self._vehicle_attitude.q[0],
        )
        enu.heading = heading_from_quaternion(enu.qx, enu.qy, enu.qz, enu.qw)

        self._pub_enu_local_position.publish(enu)
        self._enu_local_position = enu

    def vehicle_attitude_cb(self, msg: VehicleAttitude) -> None:
        self._vehicle_attitude = msg

    def vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    def vehicle_land_detected_cb(self, msg: VehicleLandDetected) -> None:
        self._vehicle_land_detected = msg

    def takeoff(self, height):
        if self._enu_local_position is None:
            return

        self.fly_point(self._enu_local_position.x, self._enu_local_position.y, height)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_home(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def set_hold_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=2.0)

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def is_takeoff_finished(self, height, epsilon=0.1):
        if self._enu_local_position is None:
            return False
        
        return self.is_point_reached(
            self._enu_local_position.x, self._enu_local_position.y, height, epsilon=epsilon
        )

    def is_point_reached(self, x, y, z, epsilon=0.1):
        if self._enu_local_position is None:
            return False
        
        distance = np.linalg.norm(
            [
                x - self._enu_local_position.x,
                y - self._enu_local_position.y,
                z - self._enu_local_position.z,
            ]
        )
        return distance < epsilon

    def is_landed(self):
        if self._vehicle_land_detected is None:
            return False
        
        return self._vehicle_land_detected.landed

    @offboard_command
    def fly_point(self, x, y, z, heading=None):
        if self._enu_local_position is None:
            return

        msg = TrajectorySetpoint()
        msg.position = enu_to_ned(x, y, z)
        msg.yaw = enu_to_ned_heading(heading if heading else self._enu_local_position.heading)
        msg.timestamp =int(self.node.get_clock().now().nanoseconds / 1000)
        self._pub_trajectory_setpoint.publish(msg)

    @offboard_command
    def fly_velocity(self, vx, vy, vz, heading=None):
        if self._enu_local_position is None:
            return
        
        msg = TrajectorySetpoint()
        msg.position = [float("NaN"), float("NaN"), float("NaN")]
        msg.velocity = enu_to_ned(vx, vy, vz)
        msg.yaw = enu_to_ned_heading(heading if heading else self._enu_local_position.heading)
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self._pub_trajectory_setpoint.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        if self._heartbeat_counter < self.HEARTBEAT_THRESHOLD:
            self._heartbeat_counter += 1

    @offboard_command
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self._pub_offboard_control_mode.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self._pub_vehicle_command.publish(msg)
