import cv2
import numpy as np

print("OpenCV version:", cv2.__version__)
print("NumPy version:", np.__version__)

# Create a simple image
img = np.zeros((300, 300, 3), dtype=np.uint8)
cv2.putText(img, "OpenCV Works!", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

# Save the image
cv2.imwrite("/home/amar/madmax/output/test_image.jpg", img)
print("Image saved to /home/amar/madmax/output/test_image.jpg")
