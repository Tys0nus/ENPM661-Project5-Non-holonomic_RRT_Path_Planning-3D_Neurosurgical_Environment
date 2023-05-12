import numpy as np
from rrt import RRT
import time

if __name__ == "__main__":
    start_time = time.time()
    print("-----------------------")
    print("RRT Path Planning Implementation through non-holonomic multi-actions in 3D Neurosurgical Environment") 
    print("-----------------------")

    np.random.seed(7)

    obstacles = [[0, 20, 0, 100, 30, 60],
                [0, 40, 20, 100, 60, 100]]

    bounds = np.array([0,90])

    start = np.array([100,0,10])
    goal = np.array([30,90,90])

    """Uncomment below code block to execute RRT : holonomic"""
    ## ---------------------------------------------------------------------------------------
    print("Executing Case: RRT : holonomic \n\n")
    from visualize_rrt import Map
    space = Map(obstacles, bounds)
    rrt = RRT(start = start, goal = goal,
        Map = space, max_iter = 5000,
        goal_sample_rate = 0.1)
    waypts = rrt.plan()
    space.plot_map(start, goal,path=waypts, t_obj = rrt)
    ## ---------------------------------------------------------------------------------------


    """Uncomment below code block to execute RRT star : holonomic"""
    ## ---------------------------------------------------------------------------------------
    # print("Executing Case: RRT star : holonomic \n\n")
    # from rrt_star import RRTstar
    # from visualize_rrt import Map
    # space = Map(obstacles, bounds)
    # rrtstar = RRTstar(start = start, goal = goal,
    #     Map = space, max_iter = 5000,
    #     goal_sample_rate = 0.1)
    # waypts, min_cost = rrtstar.plan()
    # space.plot_map(start, goal,path=waypts, t_obj = rrtstar)
    ## ---------------------------------------------------------------------------------------

    """Uncomment below code block to execute RRT : non-holonomic"""
    ## ---------------------------------------------------------------------------------------
    # print("Executing Case: RRT : non-holonomic \n\n")
    # from rrt_nh import RRTNH
    # from visualize_rrtnh import Map
    # space = Map(obstacles, bounds)
    # rrt_nh = RRTNH(start = start, goal = goal,
    #     Map = space, max_iter = 5000,
    #     goal_sample_rate = 0.1)
    # waypts = rrt_nh.plan()
    # space.plot_map(start, goal, t_obj = rrt_nh)
    ## ---------------------------------------------------------------------------------------

    end_time = time.time()
    runtime = end_time - start_time
    print("Runtime: {:.4f} seconds".format(runtime),"\n\n")