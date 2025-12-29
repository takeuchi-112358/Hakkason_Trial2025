#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import os

rospy.init_node("face_tracking_smile_node")

SMILE_DETECTION_DISTANCE = 0.8
MOVEMENT_THRESHOLD = 1.0
LOST_TRACKING_THRESHOLD = 10

bridge = CvBridge()

# 出力用Publisher
pub_color = rospy.Publisher("/camera/color/image_tracked", Image, queue_size=1)
pub_smile = rospy.Publisher("/smile_status", String, queue_size=10)
pub_depth_value = rospy.Publisher("/face/depth", Float32, queue_size=10)
pub_movement_status = rospy.Publisher("/movement_status", String, queue_size=10)
pub_detection_status = rospy.Publisher("/detection_status", String, queue_size=10)

# グローバル変数
color_image = None
depth_image = None
depth_scale = 0.001  # RealSenseのデフォルト

def color_callback(msg):
    global color_image
    try:
        color_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr_throttle(10, f"カラー画像変換エラー: {e}")

def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, "16UC1")
    except Exception as e:
        rospy.logerr_throttle(10, f"深度画像変換エラー: {e}")

# ROSトピックをサブスクライブ
rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)

rospy.loginfo("RealSense ROSトピックからの画像を待機中...")
rospy.loginfo("別ターミナルで以下を実行してください:")
rospy.loginfo("  roslaunch realsense2_camera rs_camera.launch align_depth:=true")
rospy.sleep(3.0)

# Haar Cascade
def find_haarcascade(filename):
    candidates = [
        os.path.join(os.path.expanduser("~"), "haarcascades", filename),
        f"/usr/share/opencv4/haarcascades/{filename}",
        f"/usr/share/opencv/haarcascades/{filename}",
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None

face_path = find_haarcascade("haarcascade_frontalface_default.xml")
smile_path = find_haarcascade("haarcascade_smile.xml")

if not face_path:
    rospy.logerr("顔検出XMLが見つかりません")
    exit(1)

face_cascade = cv2.CascadeClassifier(face_path)
smile_cascade = cv2.CascadeClassifier(smile_path) if smile_path else None

rospy.loginfo("Face cascade: OK")
if smile_path:
    rospy.loginfo("Smile cascade: OK")

# トラッカー
def create_tracker():
    try:
        return cv2.TrackerCSRT_create()
    except:
        try:
            return cv2.legacy.TrackerCSRT_create()
        except:
            try:
                return cv2.TrackerMOSSE_create()
            except:
                return cv2.legacy.TrackerMOSSE_create()

tracker = create_tracker()
tracking = False
frame_count = 0
lost_count = 0

rate = rospy.Rate(15)
rospy.loginfo("メインループ開始")

try:
    while not rospy.is_shutdown():
        if color_image is None or depth_image is None:
            if frame_count == 0:
                rospy.logwarn_throttle(5, "画像待機中... RealSenseカメラノードは起動していますか？")
            rate.sleep()
            continue

        frame_count += 1
        
        if frame_count == 1:
            rospy.loginfo(f"画像受信開始: {color_image.shape}")
        
        img = color_image.copy()
        depth = depth_image.copy()

        if not tracking:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 5, minSize=(30,30))
            
            if len(faces) > 0:
                x, y, w, h = [int(v) for v in faces[0]]
                tracker = create_tracker()
                tracker.init(img, (x, y, w, h))
                tracking = True
                lost_count = 0
                rospy.loginfo("顔検出・追跡開始")
            else:
                cv2.putText(img, "No face detected", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            success, bbox = tracker.update(img)
            
            if success and bbox is not None:
                x, y, w, h = [int(v) for v in bbox]
                
                # 範囲チェック
                if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]:
                    x = max(0, min(x, img.shape[1] - 1))
                    y = max(0, min(y, img.shape[0] - 1))
                    w = min(w, img.shape[1] - x)
                    h = min(h, img.shape[0] - y)
                    
                    cx, cy = x + w//2, y + h//2
                    
                    if 0 <= cx < depth.shape[1] and 0 <= cy < depth.shape[0]:
                        depth_raw = depth[cy, cx]
                        
                        if depth_raw > 0:
                            d = depth_raw * depth_scale
                            pub_depth_value.publish(Float32(d))
                            
                            # 移動判定
                            if d >= MOVEMENT_THRESHOLD:
                                pub_movement_status.publish("you can move now")
                                label = f"Move {d:.2f}m"
                                col = (0, 0, 255)
                            else:
                                pub_movement_status.publish("stop")
                                label = f"STOP {d:.2f}m"
                                col = (255, 255, 0)
                            
                            # 笑顔検出
                            if d <= SMILE_DETECTION_DISTANCE and smile_cascade:
                                pub_detection_status.publish("detecting smile")
                                roi = img[y:y+h, x:x+w]
                                if roi.size > 0:
                                    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                                    smiles = smile_cascade.detectMultiScale(roi_gray, 1.8, 20, minSize=(15,15))
                                    if len(smiles) > 0:
                                        pub_smile.publish("smiling")
                                        label = f"SMILE! {d:.2f}m"
                                        col = (0, 255, 0)
                            
                            cv2.rectangle(img, (x, y), (x+w, y+h), col, 2)
                            cv2.putText(img, label, (x, y-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                            lost_count = 0
                        else:
                            cv2.putText(img, "Invalid depth", (x, y-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128,128,128), 2)
            else:
                lost_count += 1
                if lost_count > LOST_TRACKING_THRESHOLD:
                    tracking = False
                    lost_count = 0
                    rospy.loginfo("追跡ロスト・再検出モード")

        # 情報表示
        info = f"Frame:{frame_count} | {'Tracking' if tracking else 'Detecting'}"
        cv2.putText(img, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # ROSトピックに配信
        try:
            pub_color.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        except Exception as e:
            rospy.logerr_throttle(10, f"画像配信エラー: {e}")

        # 画面表示
        try:
            cv2.imshow("Face Tracking", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("ユーザーによる終了")
                break
        except:
            pass

        rate.sleep()

except KeyboardInterrupt:
    rospy.loginfo("Ctrl+C 終了")
finally:
    cv2.destroyAllWindows()
    rospy.loginfo("終了しました")
