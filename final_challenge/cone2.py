import cv2
import numpy as np

def inpath(x,y):
    if y > 245:
        if y > 0.734*x +9.728 and y > -0.734*x+480:
            return True
        else:
            return False
    else:
        return False

img = cv2.imread('cone.jpg')
img = cv2.line(img,(0,480),(320,245),(0,0,255),1)
img = cv2.line(img,(640,480),(320,245),(0,0,255),1)
x1 = np.random.randint(0,640)
y1 = np.random.randint(245,480)
img = cv2.circle(img,(x1,y1),5,(0,0,255),-1)
print(inpath(x1,y1))

cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()