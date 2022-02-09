# RobotCode-v14 "Cyclops"
**RobotCode for Talon540 v14**

## **Plan of Action *(as of 01/31/2022)*:**

### Shooting Mechanics - Automatically aim turret at Hubs
**Aiming and tracking team:**
1. Sriman Achanta
2. Mihir Pokhriyal

 * Use limelight to automatically track shooter at top hub retro reflectors.
 * Limelight will be added directly to the shooter, at a fixed vertical angle.
 * Limelight will extrapolate distance from hubs: 
	* Determines ideal angle to fire shot
	* Determines ideal flywheel speed
		* Interpolated from the ball velocity needed- WILL NEED TESTING
 * Sets motors accordingly 
    
### Ball Detection and Intake/Index - Use camera and Raspberry Pi to find balls
**Ball tracking and navigation team:**
1. Ayush Pal
2. Philip Naveen
3. Aayush Kulkarni 
4. Sam Gibbs

**Track balls using Video**
 * Take video input from Raspberry Pi
	 * Use a mask and contur detection to differentiate balls from the outline of the arena.
	 * Outline this ball from the rest of the arena and remove the background of the arena.
		 * Isolate the ball using this and get ball color.
			 * If its our color find out if it is moving by tracking it over a set period of time (500 ms). Then mark that ball as moving and ignore it. 
			 * If its not moving turn the video input into numerical data and find out how to move the robot to that positon in terms of *Java* functions and motor controls. 
 * Determine necessary intake motor controls
	 * Bouncing balls *flyswatter/catcher mechanism* (idk ask Ojas/Zaid)
	 * When to switch from intake to index and vice versa

### Climb Mechanics - TBD from CAD
**Climbing team:**
1. Iuri Vintonyak
2. Jinwon Cha
3. Rishav Sen

### Main Drive Code
1. Brandon Fecht
