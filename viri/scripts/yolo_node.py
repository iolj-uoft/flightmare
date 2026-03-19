#!/usr/bin/env python3
import rospy
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

def main():
    rospy.init_node("ultralytics_yolo_node", anonymous=False)

    # params
    model_path = rospy.get_param("~model_path", "/home/austin/Desktop/catkin_ws/src/flightmare/viri/model/best.pt")
    image_topic = rospy.get_param("~image_topic", "/hummingbird/chaser_drone/camera")
    out_topic = rospy.get_param("~out_image_topic", "ultralytics/detection/image")

    bridge = CvBridge()
    rospy.loginfo("Loading YOLO model: %s", model_path)
    model = YOLO(model_path)

    pub = rospy.Publisher(out_topic, Image, queue_size=1)

    def callback(msg: Image):
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # OpenCV BGR
        except Exception as e:
            rospy.logwarn("cv_bridge failed: %s", e)
            return

        # ultralytics expects RGB numpy array
        rgb = cv_img[:, :, ::-1]  # BGR->RGB

        # run detection
        results = model(rgb, imgsz=640)  # adjust imgsz if needed
        r = results[0]

        # annotated image as numpy (RGB)
        annotated = r.plot()  # returns RGB image (numpy)
        if annotated is None:
            annotated = rgb

        # convert back to BGR for CvBridge (if you want to keep bgr8)
        annotated_bgr = annotated[:, :, ::-1]

        out_msg = bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
        out_msg.header = msg.header
        pub.publish(out_msg)

    rospy.Subscriber(image_topic, Image, callback, queue_size=1, buff_size=2**24)
    rospy.loginfo("YOLO node subscribed to %s, publishing to %s", image_topic, out_topic)
    rospy.spin()

if __name__ == "__main__":
    main()