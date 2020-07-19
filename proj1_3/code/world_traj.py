import numpy as np

from proj1_3.code.graph_search import graph_search

class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.5

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1,3)) # shape=(n_pts,3)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE

        # trim the path into sparse waypoint combination
        n, d = self.path.shape
        path = self.path
        removeList = []
        for i in range(n - 2):
            if path[i + 2][0] - path[i + 1][0] == path[i + 1][0] - path[i][0] and path[i + 2][1] - path[i + 1][1] == \
                    path[i + 1][1] - path[i][1] and path[i + 2][2] - path[i + 1][2] == path[i + 1][2] - path[i][2]:
                removeList.append(i+1)
        self.points = np.delete(path, removeList, axis=0)

        self.num_pt, d = self.points.shape
        self.maxVel = 2.57

        self.x_int_pt = self.points[0:self.num_pt - 1, 0]
        self.x_fin_pt = self.points[1:self.num_pt, 0]

        self.y_int_pt = self.points[0:self.num_pt - 1, 1]
        self.y_fin_pt = self.points[1:self.num_pt, 1]

        self.z_int_pt = self.points[0:self.num_pt - 1, 2]
        self.z_fin_pt = self.points[1:self.num_pt, 2]

        self.t_spend = []
        self.t_spend.append(0)


        # estimate time spent on each path
        for i in range(self.num_pt - 1):
            x_dist = np.abs(self.x_fin_pt[i] - self.x_int_pt[i])
            y_dist = np.abs(self.y_fin_pt[i] - self.y_int_pt[i])
            z_dist = np.abs(self.z_fin_pt[i] - self.z_int_pt[i])

            t_x = x_dist/self.maxVel
            t_y = y_dist/self.maxVel
            t_z = z_dist/self.maxVel

            t_max = np.max([t_x, t_y, t_z])
            self.t_spend.append(t_max + self.t_spend[-1])


    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x = np.zeros((3,))
        x_dot = np.zeros((3,))
        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        t_0 = self.t_spend[0:len(self.t_spend) - 1]
        t_f = self.t_spend[1:len(self.t_spend)]

        # determine which path the robot is currently at
        num_path = 0  # time interval
        for i in range(self.num_pt):
            if t > self.t_spend[i]:
                num_path = i
        if num_path >= self.num_pt - 1:  # the last point or exceed
            x = self.points[-1, :]
        else:
            t0 = t_0[num_path]
            tf = t_f[num_path]
            xi = self.x_int_pt[num_path]
            xf = self.x_fin_pt[num_path]
            yi = self.y_int_pt[num_path]
            yf = self.y_fin_pt[num_path]
            zi = self.z_int_pt[num_path]
            zf = self.z_fin_pt[num_path]

            # calculate ax + b for position
            A = np.array([[t0, 1],
                          [tf, 1]])
            B_x = np.array([xi, xf])
            B_y = np.array([yi, yf])
            B_z = np.array([zi, zf])

            a_x = np.dot(np.linalg.inv(A), np.transpose(B_x))
            a_y = np.dot(np.linalg.inv(A), np.transpose(B_y))
            a_z = np.dot(np.linalg.inv(A), np.transpose(B_z))
            # k = (t - t0) / (tf - t0)
            # my_x = (xf - xi) * k + xi
            # my_y = (yf - yi) * k + yi
            # my_z = (zf - zi) * k + zi
            # x = np.array([my_x, my_y, my_z])

            # k = (t - t0) / (tf-t0)
            # x = np.array([(1 - k) * xi + k * xf,
            #               (1 - k) * yi + k * yf,
            #               (1 - k) * zi + k * zf])



            my_x = t * a_x[0] + a_x[1]
            my_y = t * a_y[0] + a_y[1]
            my_z = t * a_z[0] + a_z[1]



            x = np.array([my_x, my_y, my_z])
        # x_dot = np.array([self.maxVel, self.maxVel, self.maxVel])
        # x_ddot = np.array([my_x_ddot, y_ddot, z_ddot])
        # x_dddot = np.array([my_x_dddot, y_dddot, z_dddot])
        # x_ddddot = np.array([my_x_ddddot, y_ddddot, z_ddddot])

        flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                       'yaw': yaw, 'yaw_dot': yaw_dot}
        return flat_output

