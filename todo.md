Todo list:
- Create a solver, that creates a trajectory for the robot. That can be followed by a pid for position, angle, accelerations and speeds
- Have max velocities constraint
- Limit the torque in the arms
- Want to add some mass to the arms



About the planner:
Minimum Snap trajectory algorithm is a nice way to get the fastest velocity with lowest jerk and some constraints.
But its a bit theoretic and maybe not so nice when used in real life.


Could do RRT*, apply limits. With that global path


TODO next:
- Adhere to velocity constraints
  Which will create problems, since then it cannot follow so smooth anymore
- Plot the velocity and acceleration on a nice plot over time.

Okay so we want to have a certain angle goal as well.