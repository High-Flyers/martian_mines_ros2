import queue
import requests
import rclpy
from rclpy.node import Node


class Uploader(Node):
    """
    Class for uploading some data to the server.

    params:
    url: string specifying the server address (should contain the endpoint).
    upload_interval: float specifying the wait time between requests, default to 0.5.
    """

    def __init__(self, url: str, upload_interval: float = 0.5) -> None:
        super().__init__('uploader_node')
        self._url = url
        self._data_queue = queue.Queue()
        self._upload_interval = upload_interval
        
        # Set up a timer to periodically check the queue and process the data
        self.create_timer(self._upload_interval, self._process_queue)

    def add(self, item) -> None:
        """Add an item to the sending queue."""
        self._data_queue.put(item, block=False)

    def _process_queue(self):
        """Check and process the queue periodically."""
        if not self._data_queue.empty():
            data = self._data_queue.get()
            self._send(data)
            self._data_queue.task_done()

    def _send(self, data: dict) -> requests.Response:
        """Send the data to the server."""
        try:
            response = requests.post(self._url, json=data)
            return response
        except requests.RequestException as e:
            self.get_logger().error(f"Failed to send data: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    
    # Example URL and uploader initialization
    url = 'http://example.com/upload'
    uploader = Uploader(url)

    # Keep the node running
    rclpy.spin(uploader)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
