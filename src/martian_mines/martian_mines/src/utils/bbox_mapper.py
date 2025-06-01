import tf2_ros
import pyrr
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import do_transform_vector3
from geometry_msgs.msg import Vector3Stamped
from martian_mines_msgs.msg import BoundingBoxLabeled

class BBoxMapper:
    def __init__(self, camera_info_msg, node, base_frame="start_pose", camera_frame="camera_link"):
        self.node = node
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info_msg)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.base_frame = base_frame
        self.camera_frame = camera_frame

    def set_base_frame(self, base_frame):
        self.base_frame = base_frame

    def set_camera_frame(self, camera_frame):
        self.camera_frame = camera_frame

    def get_transform(self, transform_time):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, transform_time) # transform time to fix
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().warn(f"Failed to get transform: {e}")
            return None

    def bbox_to_ground_position(self, bbox: BoundingBoxLabeled, transform_time):
        ray_camera_frame = self.camera_model.projectPixelTo3dRay((bbox.bbox.center.position.x, bbox.bbox.center.position.y))
        transform = self.get_transform(transform_time) # transform time to fix
        if transform:
            vector = Vector3Stamped()
            vector.vector.x = ray_camera_frame[0]
            vector.vector.y = ray_camera_frame[1]
            vector.vector.z = ray_camera_frame[2]
            camera_position = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            )
            transformed_ray = do_transform_vector3(vector, transform)
            ray_direction = (transformed_ray.vector.x, transformed_ray.vector.y, transformed_ray.vector.z)
            ray_local_frame = pyrr.ray.create(camera_position, ray_direction)
            plane = pyrr.plane.create()
            intersection_point = pyrr.geometric_tests.ray_intersect_plane(ray_local_frame, plane)
            if intersection_point is not None:
                return intersection_point
            self.node.get_logger().warn("The line is parallel to the plane, cannot get intersection.")

        return None
