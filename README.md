# RobotCode-v14
**RobotCode for Talon540 v14**

## **Plan of Action *(as of 01/15/2022)*:**

### Shooting Mechanics - Automatically aim turret at Hubs
**Aiming and tracking team:**
1. Sriman Achanta
2. 
3. 

 * Use limelight to automatically track shooter at top hub retro reflectors.
 * Limelight will be added directly to the shooter
 * Limelight will extrapolate distance from hubs and create an optimal angle for the shooter in accordance with the data
    * Create a parabola and required speed for optimal chances of landing the shot in the TOP hub.
    
### Ball Detection - Use camera and Raspberry Pi to find balls
**Ball tracking and navigation team:**
1. Ayush Pal
2. 
3. 
~~Camera and Contrast Idea~~
**Track balls using Video**
 * Take video input from Raspberry Pi
	 * Use a mask and contur detection to differentiate balls from the outline of the arena.
	 * Outline this ball from the rest of the arena and remove the background of the arena.
		 * Isolate the ball using this and get ball color.
			 * If its our color find out if it is moving by tracking it over a set period of time (500 ms). Then mark that ball as moving and ignore it. 
			 * If its not moving turn the video input into numerical data and find out how to move the robot to that positon in terms of *Java* functions and motor controls. 

### Climb Mechanic - TBD
**Climbing team:**
1. 
2. 
3. 
