#!/usr/bin/env python3
import rospy
import message_filters
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np

def main():
    rospy.init_node("ultralytics_yolo_stereo_node", anonymous=False)

    # Params
    model_path = rospy.get_param("~model_path", "/home/austin/Desktop/catkin_ws/src/flightmare/viri/model/best.pt")
    image_left_topic = rospy.get_param("~image_left_topic", "/chaser_drone/camera_left")
    image_right_topic = rospy.get_param("~image_right_topic", "/chaser_drone/camera_right")
    out_image_left_topic = rospy.get_param("~out_image_left_topic", "/ultralytics/detection/image_left_annotated")
    # NEW: right annotated image topic
    out_image_right_topic = rospy.get_param("~out_image_right_topic", "/ultralytics/detection/image_right_annotated")
    
    # NEW: Topic to publish the raw pixel coordinates to your stereo estimator
    bbox_topic = rospy.get_param("~bbox_topic", "/ultralytics/detection/target_centers")

    bridge = CvBridge()
    rospy.loginfo("Loading YOLO model: %s", model_path)
    model = YOLO(model_path)

    img_pub_left = rospy.Publisher(out_image_left_topic, Image, queue_size=1)
    # NEW: publisher for right annotated image
    img_pub_right = rospy.Publisher(out_image_right_topic, Image, queue_size=1)
    
    # Using Float32MultiArray to easily send [x_left, y_left, x_right, y_right]
    # without needing to compile a custom ROS message just yet.
    bbox_pub = rospy.Publisher(bbox_topic, Float32MultiArray, queue_size=1)

    # The Synchronized Callback
    def sync_callback(msg_left, msg_right):
        try:
            cv_img_left = bridge.imgmsg_to_cv2(msg_left, desired_encoding="bgr8")
            cv_img_right = bridge.imgmsg_to_cv2(msg_right, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge failed: %s", e)
            return

        rgb_left = cv_img_left[:, :, ::-1]
        rgb_right = cv_img_right[:, :, ::-1]

        # 1. BATCH DETECTION: Pass both images at once for faster inference
        results = model([rgb_left, rgb_right], imgsz=640, verbose=False)
        res_left = results[0]
        res_right = results[1]

        x_l, y_l, x_r, y_r = -1.0, -1.0, -1.0, -1.0

        # 2. EXTRACT LEFT CENTER
        if len(res_left.boxes) > 0:
            # Grab the highest confidence box: xyxy format is [x_min, y_min, x_max, y_max]
            box_l = res_left.boxes[0].xyxy[0].cpu().numpy() 
            x_l = (box_l[0] + box_l[2]) / 2.0
            y_l = (box_l[1] + box_l[3]) / 2.0

        # 3. EXTRACT RIGHT CENTER
        if len(res_right.boxes) > 0:
            box_r = res_right.boxes[0].xyxy[0].cpu().numpy()
            x_r = (box_r[0] + box_r[2]) / 2.0
            y_r = (box_r[1] + box_r[3]) / 2.0

        # 4. PUBLISH COORDINATES
        # Only publish if YOLO successfully found the target in BOTH cameras
        if x_l != -1.0 and x_r != -1.0:
            coord_msg = Float32MultiArray()
            coord_msg.data = [x_l, y_l, x_r, y_r]
            bbox_pub.publish(coord_msg)

        # 5. VISUALIZATION — publish BOTH left and right annotated frames
        annotated_left = res_left.plot() 
        if annotated_left is None:
            annotated_left = rgb_left
        annotated_bgr_left = annotated_left[:, :, ::-1]
        out_msg_left = bridge.cv2_to_imgmsg(annotated_bgr_left, encoding="bgr8")
        out_msg_left.header = msg_left.header # Maintain the original timestamp
        img_pub_left.publish(out_msg_left)

        annotated_right = res_right.plot()
        if annotated_right is None:
            annotated_right = rgb_right
        annotated_bgr_right = annotated_right[:, :, ::-1]
        out_msg_right = bridge.cv2_to_imgmsg(annotated_bgr_right, encoding="bgr8")
        out_msg_right.header = msg_right.header
        img_pub_right.publish(out_msg_right)

    # --- Setup Time Synchronization ---
    sub_left = message_filters.Subscriber(image_left_topic, Image, buff_size=2**24)
    sub_right = message_filters.Subscriber(image_right_topic, Image, buff_size=2**24)
    
    # ApproximateTimeSynchronizer links the frames together. 
    # slop=0.05 means timestamps can be off by max 50ms and still be considered a matched pair.
    ts = message_filters.ApproximateTimeSynchronizer([sub_left, sub_right], queue_size=5, slop=0.05)
    ts.registerCallback(sync_callback)

    rospy.loginfo("YOLO Stereo Node running and publishing to %s and %s", out_image_left_topic, out_image_right_topic)
    rospy.spin()

if __name__ == "__main__":
    main()