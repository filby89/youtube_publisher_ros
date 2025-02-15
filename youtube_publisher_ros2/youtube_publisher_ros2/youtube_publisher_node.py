import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import youtube_dl

class YouTubePublisherNode(Node):
    def __init__(self):
        super().__init__('youtube_publisher_node')
        self.declare_parameter('youtube_link', '')
        youtube_link = self.get_parameter('youtube_link').value
        if not youtube_link:
            self.get_logger().error("Parameter 'youtube_link' not set.")
            rclpy.shutdown()
            return
        
        self.get_logger().info(f"Extracting video URL from: {youtube_link}")
        video_url = self.get_video_url(youtube_link)
        if not video_url:
            self.get_logger().error("Failed to extract video URL.")
            rclpy.shutdown()
            return
        
        self.get_logger().info(f"Opening video stream: {video_url}")
        self.cap = cv2.VideoCapture(video_url)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video stream.")
            rclpy.shutdown()
            return
        
        self.publisher_ = self.create_publisher(Image, '/rgb/image_raw', 10)
        self.bridge = CvBridge()
        self.frame_idx = 0
        # Create a timer to publish at 30 Hz.
        self.timer = self.create_timer(1.0/30.0, self.publish_frame)

    def get_video_url(self, link):
        ydl_opts = {
            'format': 'best',
            'quiet': True,
        }
        try:
            with youtube_dl.YoutubeDL(ydl_opts) as ydl:
                info = ydl.extract_info(link, download=False)
                video_url = info.get('url', None)
                return video_url
        except Exception as e:
            self.get_logger().error(f"Error extracting video URL: {e}")
            return None

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Video stream ended or frame not received.")
            self.cap.release()
            self.timer.cancel()
            return
        
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = str(self.frame_idx)
            self.publisher_.publish(img_msg)
            self.frame_idx += 1
        except Exception as e:
            self.get_logger().error(f"Error converting frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YouTubePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
