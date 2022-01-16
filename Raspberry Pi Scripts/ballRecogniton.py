import cv2
from PIL import Image, ImageEnhance

myImage = Image.open('/Users/srimanachanta/Documents/CODING/Talon540/RobotCode-v14/Raspberry Pi Scripts/test.jpeg') #Change this to your path. add to GITIGNORE
enhancer = ImageEnhance.Contrast(myImage)

imageContrastFactor = 10
contrasted_image = enhancer.enhance(imageContrastFactor)

contrasted_image.show()
myImage.show()

