import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os
import json

from pathlib import Path
from datetime import datetime

from .drone.mavlink_telemetry import MavlinkTelemetry
from .utils.streamer import Streamer

FPS = 30
RESOLUTION = (1280, 720)

TELEMETRY_CONNECTION_STRING = "udpin:0.0.0.0:14550"

datetime_str = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")
DATA_STORE_DIR = Path(f"data/test_data/{datetime_str}")

STREAM_ADDRESS = "tcp://10.42.0.1:5555"


class DataCollector(Node):
    def __init__(self, stream=True, no_save=True, no_telem=False, no_display=False):
        super().__init__("data_collector")

        """Collects telemetry data alongside video capture. Press Ctrl+C to stop.
        
        :param stream: bool, determines if the video stream should be enabled.
        :param no_save: bool, determines if the video capture should be saved.
        :param no_telem: bool, determines if telemetry data should be gathered.
        :param no_display: bool, determines if video capture should be displayed.
        """
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        self.pipeline.start(config)

        self.stream = stream
        self.no_save = no_save
        self.no_telem = no_telem
        self.no_display = no_display

        if self.stream:
            self.streamer = Streamer(adress=STREAM_ADDRESS, sending_fps=15, jpeg_quality=70)

        if not self.no_telem:
            self.telemetry = MavlinkTelemetry()
            self.telemetry.connect(TELEMETRY_CONNECTION_STRING)
            self.telemetry.start()

        if not self.no_save:
            os.makedirs(DATA_STORE_DIR, exist_ok=True)
            self.color_out = cv2.VideoWriter(
                str(DATA_STORE_DIR / "color_out.mp4"), 0x7634706D, FPS, RESOLUTION
            )
            self.depth_out = cv2.VideoWriter(
                str(DATA_STORE_DIR / "depth_out.mp4"), 0x7634706D, FPS, RESOLUTION
            )

        self.data = []

        self.timer = self.create_timer(1.0 / FPS, self.capture_frame)

    def capture_frame(self):
        """Captures video frames and telemetry data periodically."""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        if self.stream:
            frame_to_send = cv2.resize(color_image, (640, 360))
            self.streamer.add_frame_to_send(frame_to_send)

        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        if not self.no_telem:
            telem_data = self.telemetry.get_telem_data()
            self.get_logger().info(str(telem_data))

        if not self.no_save:
            self.color_out.write(color_image)
            self.depth_out.write(depth_colormap)
            if not self.no_telem:
                self.data.append(telem_data)

        if not self.no_display:
            cv2.imshow("RealSense Color", color_image)
            cv2.waitKey(1)
            cv2.imshow("RealSense Depth", depth_colormap)
            cv2.waitKey(1)

    def cleanup(self):
        """Cleans up resources before exiting."""
        self.pipeline.stop()
        cv2.destroyAllWindows()

        if not self.no_save:
            with open(DATA_STORE_DIR / "data.json", "w") as file:
                json.dump(self.data, file)
            self.color_out.release()
            self.depth_out.release()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--stream", action="store_true", help="stream color image")
    parser.add_argument("--no-telem", action="store_true", help="do not gather telemetry data")
    parser.add_argument("--no-save", action="store_true", help="do not store captured video")
    parser.add_argument("--no-display", action="store_true", help="do not display video capture")
    args = parser.parse_args()

    node = DataCollector(
        stream=args.stream,
        no_save=args.no_save,
        no_telem=args.no_telem,
        no_display=args.no_display,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
