import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from autoware_msgs.msg import VehicleCmd
import mediapipe as mp

cap = cv2.VideoCapture(0)  # kamerayı aç
mp_Hands = mp.solutions.hands  #
hands = mp_Hands.Hands()
mpDraw = mp.solutions.drawing_utils
finger_Coord = [(8, 6), (12, 10), (16, 14), (20, 18)]  #her parmağin en uç ve en alt koordinarı (uç,alt)
thumb_Coord = (4,2) # baş parmak için koordinatlar

class ikra:
  def __init__(self):
    rospy.init_node("cv_camera",anonymous=True) #node'a isim verildi
    rospy.Subscriber('/simulator/camera_node/image/compressed', CompressedImage,self.callback)  #aracın kamerasını kullanabilmek için
    self.pub = rospy.Publisher('vehicle_cmd',VehicleCmd, queue_size = 100) #araca yön verebilmek için 

  def callback(self, image_msg): #araç kamerası için callback
    vehicle_command = VehicleCmd()

    # hand tracking
    while True:
      success, image = cap.read()
      RGB_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      results = hands.process(RGB_image)
      multiLandMarks = results.multi_hand_landmarks

      if multiLandMarks:
          handList = []
          for handLms in multiLandMarks:
              mpDraw.draw_landmarks(image, handLms, mp_Hands.HAND_CONNECTIONS)
              for idx, lm in enumerate(handLms.landmark):
                h, w, c = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                handList.append((cx, cy))
          for point in handList:
              cv2.circle(image, point, 10, (255, 255, 0), cv2.FILLED)

          upCount = 0
          for coordinate in finger_Coord:
              if handList[coordinate[0]][1] < handList[coordinate[1]][1]: 
                  upCount += 1
          if handList[thumb_Coord[0]][0] > handList[thumb_Coord[1]][0]:
              upCount += 1

          # el harerketine  göre araca hareket verme
          if upCount ==1:  # düz ilerle
            cv2.putText(image, "DUZ", (150,150), cv2.FONT_HERSHEY_PLAIN, 8, (0,255,0), 12)
            vehicle_command.twist_cmd.twist.linear.x = 3.0
            vehicle_command.twist_cmd.twist.angular.z = 0
            self.pub.publish(vehicle_command)
          elif upCount == 0:  # dur
            cv2.putText(image, "DUR", (150,150), cv2.FONT_HERSHEY_PLAIN, 8, (0,255,0), 12)
            vehicle_command.twist_cmd.twist.linear.x = 0
            self.pub.publish(vehicle_command)
          elif upCount == 2:  # sola dön
            cv2.putText(image, "SOL", (150,150), cv2.FONT_HERSHEY_PLAIN, 8, (0,255,0), 12)
            vehicle_command.twist_cmd.twist.linear.x = 1
            vehicle_command.twist_cmd.twist.angular.z = 1.5
            self.pub.publish(vehicle_command)
          elif upCount == 3:  # sağa dön
            cv2.putText(image, "SAG", (150,150), cv2.FONT_HERSHEY_PLAIN, 8, (0,255,0), 12)
            vehicle_command.twist_cmd.twist.linear.x = 1
            vehicle_command.twist_cmd.twist.angular.z = -1.5
            self.pub.publish(vehicle_command)

      cv2.imshow("Frame", image)
      cv2.waitKey(1)

if __name__ =='__main__':
  t = ikra()
  rospy.spin()
  cv2.destroyAllWindows()