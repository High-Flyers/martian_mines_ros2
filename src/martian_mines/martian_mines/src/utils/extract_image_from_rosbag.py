import argparse
import rclpy
import rosbag2_py
import cv2
from cv_bridge import CvBridge
import os


class ImageFromBag:
    def __init__(self, bag_path, output_path):
        rclpy.init()
        self.bridge = CvBridge()

        # Wyciągnięcie nazwy pliku z ścieżki do baga
        bag_filename = os.path.splitext(os.path.basename(bag_path))[0]

        # Tworzenie czytnika dla ROS 2 bag
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions()
        reader.open(storage_options, converter_options)

        # Pobranie metadanych i dostępnych tematów
        topics_info = reader.get_all_topics_and_types()
        topic_to_read = "/color/image_raw"

        available_topics = [topic.name for topic in topics_info]
        if topic_to_read not in available_topics:
            print(f"Temat {topic_to_read} nie znaleziony w bagfile.")
            return

        out = None

        while reader.has_next():
            topic, msg, t = reader.read_next()
            if topic == topic_to_read:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg)
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                    if out is None:
                        # Pobranie rozmiarów obrazu dla zapisu wideo
                        height, width, channels = cv_image.shape
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        out = cv2.VideoWriter(os.path.join(output_path, bag_filename + '.mp4'), fourcc, 15.0,
                                              (width, height))

                    # Dodanie klatki do wideo
                    out.write(cv_image)
                    print(f"Dodano klatkę do wideo")

                except Exception as e:
                    print(f"Błąd konwersji obrazu: {e}")

        if out:
            out.release()
            print("Wideo zapisane pomyślnie")

        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Ekstrakcja obrazów z pliku ROS 2 bag.')
    parser.add_argument('bag_path', type=str, help='Ścieżka do pliku ROS 2 bag')
    parser.add_argument('output_path', type=str, help='Ścieżka do katalogu wyjściowego')
    args = parser.parse_args()

    try:
        ImageFromBag(args.bag_path, args.output_path)
    except KeyboardInterrupt:
        print("Przerwano działanie programu")
