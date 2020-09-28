# FA20 Computational Robotics Warmup Project
For this warmup project, I followed the specifications of [this assignment document](https://comprobo20.github.io/assignments/warmup_project) to implement a variety of behaviors in python using ROS and a simulated mobile robot. 

## [Robot Teleop](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/teleop.py)
### Problem Description
Write a program to control the robot using the keyboard.

### Strategy and Solution
Creating the teleop program required both getting keyboard input and translating that keyboard input into robot movement.

### Getting Keyboard Input
I leveraged [existing skeleton code](https://comprobo20.github.io/assignments/warmup_project) to get non-blocking keyboard input using termios, tty, select, and sys.stdin. The node constantly gets the next keystroke, confirms that its value does not correspond to quitting the program, and then executes on the action associated with the input key.

### Translate Keyboard Input into Movement
When I initialized the node, I created a class attribute for each potential direction with the velocity message associated with the keystroke and the associated robot direction. The node's `run()` function gets the most recent keystroke and publishes the message associated with the keystroke. The table below shows each keystroke, message definition, and action.

| **Keystroke** 	| **Message**                                	| **Outcome**     	|
|---------------	|--------------------------------------------	|-----------------	|
| w             	| Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))  	| Drive Forwards  	|
| a             	| Twist(Vector3(0, 0, 0), Vector3(0, 0, -1)) 	| Turn Left       	|
| s             	| Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))  	| Turn Right      	|
| d             	| Twist(Vector3(-1, 0, 0), Vector3(0, 0, 0)) 	| Drive Backwards 	|
| no input      	| Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))  	| Stop            	|

### Design Decisions and Debugging
Here, the major design decision I had to make involved how to store the messages associated with the keys and execute on them. While I considered using more sophisticated data structures, like a map of key values and message values, I decided a set of conditional statements within the `run()` function was the appropriate level of program complexity given the simplicity of this specific program.

## [Drive in a Square](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/drive_square.py)
### Problem Description
The robot should autonomously travel in a 1 meter by 1 meter square.
### Strategy and Solution
As a first pass, I implemented this program using timing. In the [timing-based solution](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/drive_square.py), I measured how long it would take the robot to drive 1 meter and turn 90 degrees, and I hardcoded the velocity messages I needed to publish to drive in a square.

Afterwards, I developed a solution using odometry data. To do that, I created a simple state tracker using the variable `go_straight_state`, and I wrote method called `driveStraight()` and `driveRight()`.

### Design Decisions and Debugging
One initial decision I made was to implement this program using odometry data instead of timing data. While this was definitely more work, I felt that it was appropriate because I wanted my square to be more accurate than what
- For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

## [Follow a Wall](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/wall_follower.py)
### Problem Description
The robot should drive parallel to the closest wall.
### Strategy and Solution

### Design Decisions and Debugging
- For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

## [Follow a Person](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/person_follower.py)
### Problem Description
The robot should follow the closest person at a specified distance.
### Strategy and Solution

### Design Decisions and Debugging

- For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

## [Avoid Objects](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/avoid_obstacles.py)
### Problem Description
The robot should move forward while avoiding obstacles.
### Strategy and Solution

### Design Decisions and Debugging

- For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

## [Finite State Control](https://github.com/anushadatar/warmup_project/blob/master/warmup_project/scripts/finite_state_controller.py)
### Problem Description
The robot should combine and transition between multiple behaviors using a finite state controller. I chose to combine driving in a square and following a wall.
### Strategy and Solution

### Design Decisions and Debugging

### Overall Behavior

### State Combination and Transition

- For the finite state controller, what was the overall behavior. What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.

## Overall Code Structure
In general, each of my programs is an independent ROS node. Each node contains its own
- How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.

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

One mistake I made during this project was saving the majority of documentation of functionality to the very end of the project (especially for deliverables like recorded rosbags). I justified this by noting that my project would be most finished closer to the deadline, and I should document the most final version of the project. This proved problematic when I had issues with gazebo the night before the project was due, and also created a lot of work that was fairly easy to deprioritize in favor of spending time tuning the program before worrying about recording. Realisitically, however, recording a rosbag is a trivial task, and taking the time to do it while working is a great way to stay on top of documentation requirements and to hold a record of previous robot performance to compare improved programs to.
