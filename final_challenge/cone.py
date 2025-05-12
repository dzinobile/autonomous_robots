import cv2
import numpy as np

# Load the image
image = cv2.imread("cone.jpg")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Threshold to isolate white objects using HSV range
lower_white = np.array([0, 0, 184])
upper_white = np.array([255, 255, 255])
mask = cv2.inRange(hsv, lower_white, upper_white)

# Invert mask to isolate dark (non-white) gap
gap_mask = cv2.bitwise_not(mask)

# Find edges in the gap area
edges = cv2.Canny(gap_mask, 50, 150)

# Hough Line Transform to detect lines
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=160, maxLineGap=10)

# Draw and extract line equations
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        # Calculate line equation: y = mx + b
        if x2 != x1:
            m = (y2 - y1) / (x2 - x1)
            b = y1 - m * x1
            print(f"Line equation: y = {m:.2f}x + {b:.2f}")
        else:
            print(f"Vertical line at x = {x1}")
        cv2.imshow("Detected Gap Lines", image)
        cv2.waitKey(0)

# Show results

cv2.destroyAllWindows()
