import queue
import requests
import threading
import time


class Uploader:
    """
    Class for uploading some data to the server.

    params:
    url: string specifying the server address (should contain the endpoint).
    upload_interval: float specifying the wait time between requests, default to 0.5.
    """

    def __init__(self, url: str, upload_interval: float = 0.5) -> None:
        self._url = url
        self._data_queue = queue.Queue()
        self._upload_interval = upload_interval
        
        # Set up a timer to periodically check the queue and process the data
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.start()

    def add(self, item) -> None:
        """Add an item to the sending queue."""
        self._data_queue.put(item, block=False)

    def _run(self):
        """Run the periodic queue processing."""
        while not self._stop_event.is_set():
            self._process_queue()
            time.sleep(self._upload_interval)

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
            print(f"Failed to send data: {e}")
            return None

    def stop(self):
        """Stop the uploader."""
        self._stop_event.set()
        self._thread.join()


def main():
    # Example URL and uploader initialization
    url = 'http://example.com/upload'
    uploader = Uploader(url)

    # Example of adding data to the uploader
    uploader.add({'key': 'value'})

    # Keep the uploader running for a while to demonstrate functionality
    try:
        time.sleep(10)
    finally:
        uploader.stop()


if __name__ == '__main__':
    main()
