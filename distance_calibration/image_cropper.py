import cv2
for i in range(1,13):
	image = cv2.imread(str(i)+".jpg")[220:480]
	cv2.imwrite(str(i)+"_cropped.jpg",image)

