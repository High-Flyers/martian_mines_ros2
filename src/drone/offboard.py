import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, ParamSetResponse, ParamSetRequest
from std_msgs.msg import Float64
import numpy as np
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose


def point_to_pose_stamped(x, y, z, frame_id: str ="map") -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    return msg


def pose_stamped_to_point(pose: PoseStamped):
    return pose.pose.position.x, pose.pose.position.y, pose.pose.position.z


class Offboard(Node):
    def __init__(self) -> None:
        super().__init__('offboard_node')
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_setpoint_local = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 1)
        self.pub_setpoint_velocity = self.create_publisher(TwistStamped, 'mavros/setpoint_velocity/cmd_vel', 1)

        # Subscribers
        self.sub_local_pos = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.callback_local_pos, 10)
        self.sub_extended_state = self.create_subscription(ExtendedState, 'mavros/extended_state', self.callback_extended_state, 10)
        self.sub_rel_alt = self.create_subscription(Float64, 'mavros/global_position/rel_alt', self.callback_rel_alt, 10)

        # Services
        self.client_arming = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.client_set_mode = self.create_client(SetMode, 'mavros/set_mode')
        self.client_param_set = self.create_client(ParamSet, 'mavros/param/set')

        # State Variables
        self.local_pos = PoseStamped()
        self.extended_state = ExtendedState()
        self.rel_alt: float = 0.0

        self.get_logger().info("Offboard node initialized")

    def start(self):
        self.arm()
        self.set_offboard_mode()

    def takeoff(self, height):
        curr_position = self.local_pos.pose.position
        self.fly_point(curr_position.x, curr_position.y, height)


    def arm(self):
        if not self.client_arming.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service not available')
            return False

        request = CommandBool.Request()
        request.value = True
        future = self.client_arming.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def disarm(self):
        if not self.client_arming.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service not available')
            return False

        request = CommandBool.Request()
        request.value = False
        future = self.client_arming.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success


    def land(self):
        request = SetMode.Request()
        request.custom_mode = "AUTO.LAND"
        future = self.client_set_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent

    def return_home(self):
        request = SetMode.Request()
        request.custom_mode = "AUTO.RTL"
        future = self.client_set_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent
    
    def set_mission_mode(self):
        request = SetMode.Request()
        request.custom_mode = "AUTO.MISSION"
        future = self.client_set_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent
    
    def set_hold_mode(self):
        request = SetMode.Request()
        request.custom_mode = "AUTO.LOITER"
        future = self.client_set_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent
    
    def set_offboard_mode(self):
        request = SetMode.Request()
        request.custom_mode = "OFFBOARD"
        future = self.client_set_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent
    
    def set_precision_landing_mode(self):
        request = SetMode.Request()
        request.custom_mode = "AUTO.PRECLAND"
        future = self.client_set_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent
    
    def is_takeoff_finished(self, height, epsilon=0.1):
        curr_position = self.local_pos.pose.position
        return self.is_point_reached(curr_position.x, curr_position.y, height, epsilon=epsilon)
    
    def is_point_reached(self, x, y, z, frame_id="map", epsilon=0.1):
        pose = point_to_pose_stamped(x, y, z, frame_id)

        if frame_id != "map":
            pose = self.tf_buffer.transform(pose, "map", rclpy.time.Time()) # not sure if it's correct

        x,y,z = pose_stamped_to_point(pose)
        cx, cy, cz = pose_stamped_to_point(self.local_pos)
        distance = np.linalg.norm([x - cx, y - cy, z - cz])
        return distance < epsilon

    def is_landed(self):
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def fly_point(self, x, y, z, frame_id="map"):
        pose = point_to_pose_stamped(x, y, z, frame_id)
        if frame_id != "map":
            pose = self.tf_buffer.transform(pose, "map", rclpy.time.Time()) # not sure if it's correct
        
        self.pub_setpoint_local.publish(pose)

    def fly_velocity(self, vx, vy, vz, yaw_rate=0.0):
        msg = TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate # it wasnt connected to anything in the original code
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = "map"

        self.pub_setpoint_velocity.publish(msg)

    def callback_local_pos(self, msg: PoseStamped):
        self.local_pos = msg

    def callback_extended_state(self, msg: ExtendedState):
        self.extended_state = msg

    def callback_rel_alt(self, msg: Float64):
        self.rel_alt = msg.data

    def set_param(self, name: str, value) -> ParamSetResponse:
        request = ParamSetRequest()
        request.param_id = name

        if isinstance(value, int):
            request.value.integer = value
        elif isinstance(value, float):
            request.value.real = value
        else:
            raise ValueError(f"Parameter {name} value must be int or float! Got {type(value)}")
        
        future = self.client_param_set.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
def main(args=None):
    rclpy.init(args=args)
    offboard = Offboard()

    rclpy.spin(offboard)

    offboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
