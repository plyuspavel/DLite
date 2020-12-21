from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM

OBSTACLE = 255
UNOCCUPIED = 0

if __name__ == '__main__':

    """
    set initial values for the map occupancy grid
    |----------> y, column
    |           (x=0,y=2)
    |
    V (x=2, y=0)
    x, row
    """
    x_dim = 100
    y_dim = 80
    start = (10, 30)
    goal = (40, 75)
    view_range = 5

    gui = Animation(title="D* Lite Path Planning",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    goal=goal,
                    viewing_range=view_range)

    new_map = gui.world
    old_map = new_map

    new_position = start
    last_position = start

    dstar = DStarLite(map=new_map,
                      s_start=start,
                      s_goal=goal)

    slam = SLAM(map=new_map,
                view_range=view_range)

    path, g, rhs = dstar.move_and_replan(robot_position=new_position)

    while not gui.done:
        gui.run_game(path=path)

        new_position = gui.current
        new_observation = gui.observation
        new_map = gui.world


        if new_observation is not None:
            old_map = new_map
            slam.set_ground_truth_map(gt_map=new_map)

        if new_position != last_position:
            last_position = new_position

            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
