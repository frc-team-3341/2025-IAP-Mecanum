Nov. 13 Creation of the Original DriveTrain and MecanumDrive Files:
These two files are the core files of this subsystem. TankDrive.java is responsible for deciding how the controller should affect the speed of each wheel. TankDrive.java also calls methods/functions from DriveTrain.java. DriveTrain.java contains methods that both directly sets wheel speeds to each motor and that also figures out in what direction each wheel should move using the MecanumDriveKinematics library. 

Nov. 13 Updating RobotContainer.java and Robot.java:
These two files help with initializing the DriveTrain and MecanumDrive classes, as well as initializing the XboxController class and setting the port for the controller to be port 0 of the computer. 

Nov. 15 Update DriveTrain with the Correct SparkMax Libraries and Fixing of the applydeadband method:
Orignally, we were using the wrong SparkMax Libraries and classes that wouldn't work with the SparkMax motor controllers on the chassis. So, we had to change the code in DriveTrain.java to use the correct classes and we had to downlaod the REVLib Vendordep.

Nov. 15  Addition of the resetNavX method:
This is necessary to keep make sure that the angle of the NavX gets resetted in order to avoid creating any confusions when controlling the chassis for field oriented control. The Studica2025 Vendordep was also added to obtain the classes necessary for obtaining the NavX angle. 

Dec. 1 Addition of Simulation Files and Changes to execute method in MecanumDrive.java:
Since our robot wasn't finished, we decided to add try out robot simulation on AdvantageScope to see if the code was working correctly. A few files, including the simulation files for robot simulation and the robot.json file to replicate the robot on AdvantageScope, were added as well. After testing the robot, we realized that the direction the wheels were spinning on the simulated chassis was the opposite of what it should have been when laterally moving the robot. So, we updated MecanumDrive.java to make the wheels do the opposite of what they were doing when laterally moving the robot. 

Dec. 11. Adding Pathplanner Control to the Robot
In this commit, I added multiple different methods in DriveTrain.java and created a new file caleld AutoDrive.java to be able to smoothly drive the chassis using the pathplanner paths. 

Dec. 13. Creation of Methods that involve getting the robot to show up on AdvantageScope
Even despite adding the simulation files and changes on Dec 1, the robot still was not displaying. So this commit helped get all of the necessary data and angles from the controller and transferred this into the movement a simulated chassis on AdvantageScope. One of the main methods added was SimulationPeriodic inside of DriveTrain.java, which is resonsible for transferring the speeds and directions of each wheel into the simulated chassis and getting wheel position off of that. 
