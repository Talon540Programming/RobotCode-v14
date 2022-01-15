# RobotCode-v14
RobotCode for Talon540 v14

Plan of Action (as of 01/15/2022):
Use Lidar to gauge bot's position to a specific location in the field.

Shooting Mechanics - Automatically aim turret at 
* Use limelight to automatically track shooter at top hub retro reflecters.
* Limelight will be added directly to the shooter
  * Limelight will extrapalte distance from hubs and create an optimal angle for the shooter in ocordance with the data
    * Create a parabola and required speed for optimal chances of landing the shot in the TOP hub.
    
Ball Detection - Use camera and Raspberry Pi to find balls
* Use Camera and Raspberry Pi to detect balls.
* Take 2 Images
  * Test if the two images are the same. If they are:
    * Pump Contrast
    * Make out whether something is a ball or not and the color
    * Give directions to move to RoboRio

Climb Mechanic - TBD
