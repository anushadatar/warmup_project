# FA20 Computational Robotics Warmup Project
For this warmup project, I followed the specifications of [this assignment document](https://comprobo20.github.io/assignments/warmup_project) to implement a variety of behaviors in python using ROS and a simulated mobile robot. 

## Implemented Behaviors
- [Robot Teleop](#robot-teleop)
- [Drive in a Square](#drive-in-a-square)
- [Follow a Wall](#follow-a-wall)
- [Follow a Person](#follow-a-person)
- [Avoid Objects](#avoid-objects)
- [Finite State Controller](#finite-state-control)

## [Robot Teleop](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/teleop.py)
### Problem Description
Write a program to control the robot using the keyboard.

### Strategy and Solution
Creating the teleop program required both getting keyboard input and translating that keyboard input into robot movement.

I leveraged [existing skeleton code](https://comprobo20.github.io/assignments/warmup_project) to get non-blocking keyboard input using termios, tty, select, and sys.stdin. The node constantly gets the next keystroke, confirms that its value does not correspond to quitting the program, and then executes on the action associated with the input key. Then, when I initialized the node, I created a class attribute for each potential direction with the velocity message associated with the keystroke and the associated robot direction. The node's `run()` function gets the most recent keystroke and publishes the message associated with the keystroke. The table below shows each keystroke, message definition, and action.

| **Keystroke** 	| **Message**                                	| **Outcome**     	|
|---------------	|--------------------------------------------	|-----------------	|
| w             	| Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))  	| Drive Forwards  	|
| a             	| Twist(Vector3(0, 0, 0), Vector3(0, 0, -1)) 	| Turn Left       	|
| s             	| Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))  	| Turn Right      	|
| d             	| Twist(Vector3(-1, 0, 0), Vector3(0, 0, 0)) 	| Drive Backwards 	|
| no input      	| Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))  	| Stop            	|

### Design Decisions and Debugging
Here, the major design decision I had to make involved how to store the messages associated with the keys and execute on them. While I considered using more sophisticated data structures, like a map of key values and message values, I decided a set of conditional statements within the `run()` function was the appropriate level of program complexity given the simplicity of this specific program.

### Demonstration
![Teleop Demo](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/teleop.gif "Teleop Gif")

## [Drive in a Square](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/drive_square.py)
### Problem Description
The robot should autonomously travel in a 1 meter by 1 meter square.
### Strategy and Solution
As a first pass, I implemented this program using timing. In the [timing-based solution](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/drive_square.py), I measured how long it would take the robot to drive 1 meter and turn 90 degrees, and I hardcoded the velocity messages I needed to publish to drive in a square.

Afterwards, I developed a solution using odometry data. I created a class attribute to store the robot's position, and I wrote a callback for the Odometry data subscriber that constantly updated the robot's position. By tracking the position and publishing velocity messages accordingly, I drive the robot in a 1x1 square.

#### State Controller
Driving in a square requires combining two unique states- driving forward and turning right. The robot starts by storing its initial position as a reference, and then it drives until its position exceeds the initial position and the side length. Then, it sets its current orientation as a reference, and then turns until its orientation exceeds the initial orientation and 90 degrees. Then, it stores its new, current position as a reference and repeats this process.
The diagrams below display how the state tracker and its execution work.
![Drive Square Diagrams](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/draw_square_diagram.jpg "Drive Square Diagrams")

### Design Decisions and Debugging
One initial decision I made was to implement this program using odometry data instead of timing data. While this was definitely more work, I felt that it was appropriate because I wanted my square to be more accurate and metrics-driven than what the timing-based approach could offer. I also had to design the state controller to be as simple as possible while still allowing for the appropriate functionality - I wanted to make sure I was maintaing only the variables I needed while still performing as expected.

### Demonstration
![Drive Square Demo](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/drive_square.gif "Drive Square Gif")

## [Follow a Wall](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/wall_follower.py)
### Problem Description
The robot should visualize and drive parallel to the closest wall.
### Strategy and Solution
I leveraged laser scan data to find and track the nearest wall, and I used odometry data to track the position of the robot so that I could accurately place a marker at the location of the wall. After checking both sides of the robot's scanning window (`window`), I store any point that falls within double the following distance `2*follow_distance` in an array of scanned points (`scan_view`) that I arrange such that the range of angles stored are from `[-window, window]`. 
After ensuring that there is a wall visible, I sort `scan_ranges` by value so that I can find the point on the wall that is closest to the robot. I then calculate the difference between the robot's following distance and this closest point to find the error for a proportional controller to adjust the steering angle of the robot. I also visualized the scanned values as a marker for visualization and debugging. The diagram below shows the relationship between the scanning window, the `scan_view` array, and the calculated error.

![Wall Follower Diagram](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/wall_follow.jpg "Wall Follower Diagrams")

### Design Decisions and Debugging
The main design decision I made here involved how to sample the points received through the laser scan. While I explored sampling multiple points at specified angles or averaging the entire scan to find the most comprehensive possible distance, I found sampling the nearest point to be the most robust and flexible solution.

### Demonstration
![Wall Follower Demo](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/wall_follow.gif "Wall Follower Gif")

## [Follow a Person](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/person_follower.py)
### Problem Description
The robot should follow the closest person at a specified distance.
### Strategy and Solution
I used the center of mass approach to implement person following. This process requires three discrete steps: finding the person, visualizing the center of mass in rviz, and then approaching the person. To find the person, I subscribed to the laser scan data input. For any angle with valid data, I computed the cartesian distance between the point on the data point recorded by the laser scanner and the robot. I kept a rolling average of the center point in terms of the x (`x_center`) and y (`y_center`) values. To visualize the center of mass, I placed a marker to represent the center of mass, and I offset its position by the coordinates of the robot so that it would appear in the proper location. To approach the person, I calculated the difference between the perceived center of mass and the robot's position and tuned a set of proportional controllers to adjust the velocity and angle of the robot to follow the object at the specified distance.

### Design Decisions and Debugging
The main design considerations here involved the difficulty of tuning the controller when using a strategy as error-prone as center of mass. I often got distance values that were much larger or smaller than expected, and I had to carefully tune my parameters to make my program as robust as possible. 

### Demonstration
![Person Follower Demo](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/follow_person.gif "Person Follower Gif")

## [Avoid Objects](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/avoid_obstacles.py)
### Problem Description
The robot should continuously move forward while avoiding obstacles in its path.
### Strategy and Solution
I used the potential fields method to implement obstacle avoidance. To do so, I subscribed to the laser scan data stream and computed the cartesian distance between the point on the data point recorded by the laser scanner and the robot. I kept a rolling sum of the total forces in terms of the x (`total_x_value`) and y (`total_y_value`) directions, and then subtracted this sum from the total forward force ('force_threshold). The diagram below shows the overall strategy.

![Avoid Obstacles Diagram](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/potential_fields.jpg "Avoid Obstacles Diagram")

From there, I used a set of manually-tuned proportional controllers to adjust the forward velocity and angle of the robot according to the value of the overall negative force.

### Design Decisions and Debugging
The main design decision I had to make here was to determine the appropriate thresholds and offsets necessary for the positive and negative forces so that I could ensure that the robot continued moving forward and appropriately avoided obstacles. I determined this value manually through trial error.

### Demonstration
![Avoid Obstacles Demo](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/avoid_obstacles.gif "Avoid Obstacles Gif")

## [Finite State Control](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/finite_state_controller.py)
### Problem Description
The robot should combine and transition between multiple behaviors using a finite state controller. I chose to combine driving in a square and following a wall.
### Strategy and Solution
The finite state controller node subscribes to both laser scan and odometry input data. If the inbound laser scan includes a wall (detection performed the same way as in the wall follower node) the callback sets `wall_visible` callback to True so that the robot can start visualizing and following the wall. If `wall_visible` is not True, the robot uses the odometry data to track the robot's movement in a square. 

### Design Decisions and Debugging
One critical design decision I had to make involved handling the case when a robot is driving in a square, encounters a wall, follows the wall, and then starts driving in a square again. I had to decide whether the robot should start a new square with a new reference position from the end of the wall it followed or if it should use its previous reference position to complete its original square. I decided that starting a new square would make more sense in the context of driving in a square in pursuit of a wall to follow rather than staying in place, so I created a state variable called `started_square_drive` that kept track of if there was already a square drive in place and set it to False every time a wall becomes visible and True every time the robot starts driving in a square again.
### Overall Behavior
The robot drives in a square until it encounters a wall (or continuously if there are no walls visible), follows the wall and creates a marker for it until it is no longer visible, and then drives in a square again.

### State Combination and Transition
My finite state controller used two state variables - `go_straight_state` and `wall_visible`. In general, when `wall_visible` is true, the robot should follow the visible wall. Otherwise, the robot should execute on driving in a square such `go_straight_state` is True while it moves forward and False while it turns right. As the node runs and processes incoming data, it analyzes the incoming laser scan to see if a wall is visible and uses odometry data to track the location of the wall and the progress on driving in a square. The table below shows the relationship between the state variables and the robot's behavior.
| **wall_visible** 	| **go_straight_state** 	| **Outcome**                    	|
|------------------	|-----------------------	|--------------------------------	|
| True             	| True                  	| Follow entire Wall             	|
| True             	| False                 	| Follow entire wall             	|
| False            	| True                  	| Drive straight for side length 	|
| False            	| False                 	| Turn 90 degrees                	|
The state diagram below displays these states.
![Finite State Controller Diagram](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/fsc_diagram.jpg "Finite State Controller Diagrams")

### Demonstration
![Finite State Controller Demo](https://github.com/anushadatar/warmup_project/blob/master/report_visuals/fsc.gif "Finite State Controller Gif")


## Overall Code Structure
In general, each of my programs is an independent ROS node. Each node's filename should correspond to its function, and each node's class name should match the filename and use capital letters and underscores to separate words (for example, `do_something.py` should contain a node called `Do_Something_Node`). Each node's initialization function should contain publishers and subscribers named for their functionality, and it should also contain any parameters used by multiple functions. These parameter definitions should either be self-explanatory or documented with a comment, and they should be organized by the functionality they correspond to (i.e. ROS setup should go together, proportional control constants should go together, etc.). Each subscriber should have a callback function that updates the appropriate class variable; as these are called often, they should stay lightweight unless they are directly related to robot state (like in the case of the finite state controller). Each node should also have a run function that executes other, smaller functions to realize expected behavior. Unless they are general utility functions (like functions that convert between coordinate systems or calculate distances), they should use class attributes instead of parameters.

When executed individually, a node file should create an instance of its contained node and run it. Calling the run function should be the only function call necessary to realize the behavior specified by the node name.

## Reflection
### Challenges
There are three challenges I faced while working on this assignment.
1. Defining problems.
In general, I struggled with understanding what an appropriate definition of the problem statement and a complete solution was. In general, I was never quite sure what "good enough" was - was it acceptable for the wall follower to only work on one side? What was an acceptable tolerance for the accuracy of the following distance for the person followers? Having to come up with my own definitions of acceptable performance was a new and interesting challenge for me.

2. Investing time in workflows vs. working on projects.
I know that specific development improvements, such as better visualizations or dynamic parameter reconfiguration, require time upfront to implement but deliver value in future productivity. That being said, I had trouble justifying working on such tooling when I could be working on algorithms or tuning instead. Moving forward, I hope to have a better mindset about the importance of writing good tools.

3. General ROS/Gazebo errors.
I faced several issues with my ROS installation and Gazebo. While many of them had solid workarounds, I did have an unfortunately-timed Gazebo issue where the best solution I could find was to reset the world every single time I did another test; if I did not do that, the robot model would spin in a circle indefinitely. While characterizing errors and finding workarounds took time and energy, it was also an opportunity to learn more about ROS and Gazebo and a reminder to always allocate time for unforeseen issues.,

### Further Extensions
There are three major areas in which I would improve my project if I had more time.
1. Development and debugging tools.

Adding features to my programs such as dynamic parameter adjustment and richer visualizations would empower me to have a finer, more productive debugging process. Paradoxically, I often felt as though implementing structures that would simplify my workflow and increase my productivity would take too much time in and of itself. In holding that mindset, I fail to recognize that improving workflows is an investment. With more time, the first thing I would do is put structures in place to improve my workflow.

2. Algorithms.

For almost all of these programs, the use of more sophisticated algorithms would improve performance and be a productive learning experience. For example, my person-following algorithm could use more intelligent tracking than simply following the center of mass (like using a clustering algorithm), or my wall-following algorithm could have more nicely tracked the wall using RANSAC.

3. Tuning.

I tuned all of the proportional controller constants in my programs by hand over a finite amount of time. Had I had more time, I could have spent more time adding more constants (such as integral and derivative parameters) and spent more time manually tuning my programs and creating structures to introduce some automation. This would improve the performance of my existing algorithms and help me practice refining my work.

### Key Takeaways
There are three key takeaways from this assignment that I can bring to future robotics programming projects.

1. Start with a clear definition of the problem and of an acceptable solution.

Assignment definitions can be deceptively simple. For example, the requirement that a robot follow a wall can have a variety of meanings, as it does not specify the magnitude of turn it must handle, the range at which it should detect or follow walls, or what an acceptable level of performance is. After all, a robotics program is not a sorting algorithm; there is not necessarily a perfectly optimal solution that works in every case. As a result of this ambiguity, my problem-solving process was fairly nonlinear; I simply added features, and then I felt confused about if I had done a sufficient amount of work and then had to go back, define the problem, evaluate my solution, and then modify it to meet the requirements I created. If I had started with a well-defined problems statement and a clear (yet flexible) definition of a complete program, I would have saved a lot of time and energy.

2. Investing time in visualization is worth it.

Many of the bugs I handled while working on this project were due to my failure to understand the coordinate frame I was working in or the nature of the transformation I was using. Initially, I tried to debug my program through print statements, which proved exhausting and confusing. Tools like rviz helped a lot, and I wish I had gotten comfortable with using it sooner.

3. Document as you go.

One mistake I made during this project was saving the majority of documentation of functionality to the very end of the project (especially for deliverables like recorded rosbags). I justified this by noting that my project would be most finished closer to the deadline, and I should document the most final version of the project. This proved problematic when I had issues with gazebo the night before the project was due, and also created a lot of work that was fairly easy to deprioritize in favor of spending time tuning the program before worrying about recording. Realistically, however, recording a rosbag is a trivial task, and taking the time to do it while working is a great way to stay on top of documentation requirements and to hold a record of previous robot performance to compare improved programs to.
