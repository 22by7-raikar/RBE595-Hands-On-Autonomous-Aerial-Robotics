import cv2
import numpy as np

img1 = cv2.imread('/home/anuj/AerialRobotics/apairaikar_p3a/new_data/imgs/render0.5_0.5_0.5_1.jpg')
img2 = cv2.imread('/home/anuj/AerialRobotics/apairaikar_p3a/new_data/imgs/render0.5_0.5_0.5_10.jpg')
dst = cv2.addWeighted(img1, 0.5, img2, 0.5, 0)

cv2.imwrite("\home\anuj\Desktop\blended_image.jpg",dst)
cv2.imshow('Blended_image',dst)

cv2.waitKey(0)
cv2.destroyAllWindows()