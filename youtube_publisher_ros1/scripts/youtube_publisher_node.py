#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import youtube_dl

def get_video_url(link):
    ydl_opts = {
        'format': 'best',
        'quiet': True,
    }
    with youtube_dl.YoutubeDL(ydl_opts) as ydl:
        info = ydl.extract_info(link, download=False)
        # Some extractors return a list, so try to get the first URL if needed.
        video_url = info.get('url', None)
        return video_url

def main():
    rospy.init_node('youtube_publisher_node')
    
    # Get the YouTube link from a private parameter (e.g., _youtube_link:=...)
    youtube_link = rospy.get_param('~youtube_link', '')
    if not youtube_link:
        rospy.logerr("Parameter '~youtube_link' not set.")
        return

    rospy.loginfo("Extracting video URL from: %s", youtube_link)
    video_url = get_video_url(youtube_link)
    if not video_url:
        rospy.logerr("Failed to extract video URL.")
        return

    rospy.loginfo("Opening video stream: %s", video_url)
    cap = cv2.VideoCapture(video_url)
    if not cap.isOpened():
        rospy.logerr("Failed to open video stream.")
        return

    pub = rospy.Publisher('/rgb/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(30)  # Publish at 30 Hz

    rospy.loginfo("Publishing video frames...")
    frame_idx = 0
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Video stream ended or frame not received.")
            break
        try:
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = rospy.Time.now()  # Set the current time
            img_msg.header.frame_id = str(frame_idx)  # Optionally set a frame id
            pub.publish(img_msg)

            frame_idx += 1

        except Exception as e:
            rospy.logerr("Error converting frame: %s", e)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    main()
