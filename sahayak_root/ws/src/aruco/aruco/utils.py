import cv2
import numpy as np
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker = np.zeros((300, 300), np.uint8)
cv2.aruco.drawMarker(aruco_dict, 0, 300, marker, 1)
cv2.imwrite('marker.png', marker)
