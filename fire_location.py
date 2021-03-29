import math
import numpy as np

#inputting positions of the the rocket
xpos = 0
ypos = 0
zpos = 1000

#inputting the rotation of the rocket
yaw = 180    
pitch = -10
roll = 180

#inputting the pixel position of the fire
xpic = 100
ypic = 30

#converting angles of -180 - 180 to angles of 0 - 360
def threesixty(angle):
    if(angle < 0): # checks if angle is between -180 and 0
        angle = abs(angle) + 180 #changes angle to be between 0-360
        return angle
    else:
        return angle

#redefines the rotations as 0-360
yaw = threesixty(yaw)
pitch = threesixty(pitch)
roll = threesixty(roll)

#finds the distance from the center of the lens in each direction on the plane of the ground  
def length(angle, height):
   
    angle = angle -180 #finds the angle from the the downward direction
   
    if(angle >= 30 or angle <= -30): #checks that the angle is within the feild of view of the camera
       
        print("Angle over 30 degrees")
       
    else:
   
        xypos3 = [] #creates a array
   
        #finds the angle of the feild of view in relation to the angle of the rocket to the ground
        angle1 = 30-angle
        angle2 = 30+angle
       
        #finds the distance from the middle of the image to the edge, on the ground
        xypos1 = np.tan(np.radians(angle1)) * height
        xypos2 = np.tan(np.radians(angle2)) * height
       
        #adds distances to the array
        xypos3.append(xypos1)
        xypos3.append(xypos2)
       
        return xypos3 #returns array

#calls the functions and creates arrays for x and y
x = length(pitch, zpos)
y = length(yaw, zpos)    

#fixes rotation of the image in the x,y plane
def orientationCorrection(roll, x, y):
   
    #declares two variables to store the adjusted values
    adjustedx = []
    adjustedy = []
   
    #finds the components of each directon in the x and y on global coordinate system
    xcomponentx1 = np.cos(np.radians(roll)) * x[0]
    ycomponentx1 = np.sin(np.radians(roll)) * x[0]
   
    xcomponentx2 = np.cos(np.radians(roll)) * x[1]
    ycomponentx2 = np.sin(np.radians(roll)) * x[1]
   
    ycomponenty1 = np.cos(np.radians(roll)) * y[0]
    xcomponenty1 = np.sin(np.radians(roll)) * y[0]
   
    ycomponenty2 = np.cos(np.radians(roll)) * y[1]
    xcomponenty2 = np.sin(np.radians(roll)) * y[1]
   
    #adds the summed compomemts to the arrays
    adjustedx.append(xcomponentx1 - xcomponenty1)
    adjustedx.append(xcomponentx2 - xcomponenty2)
   
    adjustedy.append(ycomponenty1 + ycomponentx1)
    adjustedy.append(ycomponenty2 + ycomponentx2)
   
    #adds both sides to produce a distance across the width (x) and hight (y) of the image
    adjustedx.append(adjustedx[0]+adjustedx[1])
    adjustedy.append(adjustedy[0]+adjustedy[1])
   
    #returns the arrays
    return adjustedx, adjustedy

#defines two arrays from the function
adjustedx, adjustedy  = orientationCorrection(roll, x, y)

#defines the point on the image on our global 2d coordiate system
def positionInPicture(adjustedx,adjustedy,xpic,ypic,xpos,ypos):
   
    #finds the width and height of a pixle
    numx = adjustedx[2] / 160
    numy = adjustedy[2] / 120
   
    if(adjustedx[2] < 0 and adjustedy[2] < 0 ): #if the direction is is negative for both axis
       
        #finds displacment in global x and y
        actualx = xpos - (numx * xpic) - (0.5 * numx)
        actualy = ypos - (numy * ypic) - (0.5 * numy)
       
    elif(adjustedx[2] < 0 and adjustedy[2] > 0): #if the direction is positive in y and negative in x
       
        #finds displacment in global x and y
        actualx = xpos - (numx * xpic) - (0.5 * numx)
        actualy = ypos + (numy * ypic) + (0.5 * numy)
       
    elif(adjustedx[2] > 0 and adjustedy[2] > 0): #if the direcion is positive in both axis
       
        #finds displacment in global x and y
        actualx = xpos + (numx * xpic) + (0.5 * numx)
        actualy = ypos + (numy * ypic) + (0.5 * numy)
       
    elif(adjustedx[2] > 0 and adjustedy[2] < 0): #if the directionis positive in x and negative in y
       
        #finds displacment in global x and y
        actualx = xpos + (numx * xpic) + (0.5 * numx)
        actualy = ypos - (numy * ypic) - (0.5 * numy)
       
    else:
       
        "error"
     
    return actualx, actualy #returns actual values of x and y in world space

print(x,y)
print(orientationCorrection(roll, x, y))
print(positionInPicture(adjustedx, adjustedy, xpic, ypic, xpos, ypos))
