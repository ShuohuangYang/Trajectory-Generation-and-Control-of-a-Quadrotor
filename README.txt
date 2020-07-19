1. Graph Search
	A STAR is used to find the path, with the use of Maximum metric distance. The graph search is done by search 26 neighbors of the host. This is the same as the last lab.

2. se3_control
	The control algorithm is the same as last lab. However the gain are adjusted to fit the world_traj. 

3. world_traj
	The linear trajectory generation is used to find the trajectory from the path found by Graph Search. I used minimum snap trajectory generation and tried to form the 8m*8m matrix and add 8m constraints to the system. However I spend too much time on defining the matrix and clearify it for myself. There is no enough time to debug the system therefore I switch to linear trajectory generation by the inspiration from Yaojun Yu. The trajectory are form by defining a line from two consecutive points. The velocity and the acceleration are all set to zeros. I am still working on switching the speed if the path is all straight and long so it could travel faster. 

