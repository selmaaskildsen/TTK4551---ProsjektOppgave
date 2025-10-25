# TTK4551 – Project on Autonomous Vehicles

This repository contains the project work for the course **TTK4551 – Industrial Cybernetics** at NTNU.  
The focus of the project is **modelling, control and simulation** of an autonomous vehicle using the **bicycle model** and various **path-following algorithms**.

## 📂 Contents

- `Cubic spline generator manual.py` -This file generates the cubic splines manually, instead of using the **CubicSpline** utilized in `Cubic spline generator.py`. When generating a path with only increasing x-values went great. Then, when adding increasing AND decreasing values along the x-axis, there was an error message. The standard cubic spline algorithm assumes that the input data can be expressed as a single-valued function y = f(x), ,meaning each x-value corresponds to exactly one y-value. If the path contains sections where x decreases or repeats (for example, when the trajectory turns back on itself), this assumption is violated, and the interpolation will fail. To handle such cases, the data should be parameterized by a monotonically increasing variable, typically the cumulative distance along the path, and the spline should then be computed for both coordinates:

  x = x(s), y = y(s)

  This parametric spline approach allows for smooth 2D paths even   when x is non-monotonic, making it suitable for closed loops or complex trajectories.

- `Cubic spline generator.py` - Made a file that just focused on making the desired path using cubic spline interpolation. It uses the already incorporated **CubicSpline** in python. It works very well, on all types of waypoints put in manually by the user. **OBS**: Make sure you don't put in the same point after eachother. Then the code does not work. There must be some change in distance. When trying to interpolate a rectangle, the more points it has, the better it becomes. With only waypoints at the corners, the path look more like a circle than a rectangle, even though it touches at every waypoint. It also helps adding more waypoints closer to each other when there is a sharp turn. That way the path doesn't overshoot too much in the turns.
  
- `Simulering m rektangel.py` – First try to implement the bicycle model, stanley controller and RK4 to simulate a car following a path shaped as a rectangle. So far it struggles as it comes to the midpoint between the first two points. After a discussion with Peder he mentioned that Stanley controller might have a hard time with sharp turns, such as this 90 degree turn. This happens if I only have a waypoint at each corner, or if I make each side consist of 5 waypoints. Then, the model follows the path well, until it reaches the midpoint between a corner and the waypoint before.

- `Simplest cubic spline generator.py`- The simplest code I've made yet that generates a path using cubic spline interpolation. Note that for this code to work the x-coordinated have to be in increasing order.

- `Simulation test 1.py` - A file just to test different code before i get it to work.

- `Simulering m PID.py` - Using the same path with cubic spline as `Cubic spline generator.py`, but instead of using Stanley Controller I'm using PID-controller to compare.
  
- `Simulering rektangel m cubic spline.py` – Have tried to expand on the file 'Simulering m rektangel.py' adding cubic spline to make the turns smoother. The controller keeps the car on the desired path in the beginning, but begins to stray of course after some time.
  

