import cv2
import rospy
import numpy as np
import time
import requests
import threading

from std_msgs.msg import String


class Human_detect:
    def __init__(self):
        self.cap = cv2.VideoCapture(7)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.start_time = time.time()
        _, self.img = self.cap.read()
        self.pub = rospy.Publisher('human_detect', String, queue_size=10)
        t1 = threading.Thread(target=self.video_capture)
        t1.start()

    def video_capture(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            start_time = time.time()
            _, img = self.cap.read()
            # img = cv2.imread('../test_img/000001.jpg')
            img_str = cv2.imencode('.jpg', img)[1].tostring()
            img_uni = unicode(img_str, encoding='latin-1')
            payload = {'image': img_uni}
            r = requests.post("http://192.168.1.2:9000/", data=payload)
            duration = time.time() - start_time
            return_str = str(r.text)
            if return_str != "No one in the camera":
                index = return_str.find(']]')
                rectangles = eval(return_str[:(index + 2)])
                label = eval(return_str[(index + 2):])
                if len(rectangles):
                    for rectangle in rectangles:
                        cv2.rectangle(img, (int(rectangle[0]), int(rectangle[1])),
                                      (int(rectangle[2]), int(rectangle[3])), (255, 0, 0), 1)
                    cv2.putText(img, label[0], (int(rectangle[0]), int(rectangle[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0))
                    print label[0]
                    human_msg = String()
                    human_msg.data = label[0]
                    self.pub.publish(human_msg)
                else:
                    print "No one in the camera"
            else:
                print return_str
            cv2.imshow("capture", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    rospy.init_node('human_detect', anonymous=True)
    tmp = Human_detect()

if __name__ == '__main__':
    main()