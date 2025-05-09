
from matplotlib import image 
from matplotlib import pyplot as plt 
  
# to read the image stored in the working directory 
data = image.imread('sunset-1404452-640x480.jpg') 
  
# to draw a point on co-ordinate (200,300) 
plt.plot(200, 350, marker='v', color="white") 
plt.imshow(data) 
plt.show() 
