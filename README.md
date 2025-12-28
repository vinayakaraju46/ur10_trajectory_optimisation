# ur10_trajectory_optimisation
Goal is to optimise the waypoints (free-trajectories). Sort the waypoints in such a way that ur10 robot manipulator can reach all of them in shortest point. The algorithm used here is the travelling salesman problem with heuristic approach that seeks nearest neighbour after visiting each point and marks it as visited.

The code requires further optimisation. 
1. Dynamic programming approach that can reduce the calculation of joint distances everytime.
2. Other variations of TSP.
3. Analytics and comparition of every approach
