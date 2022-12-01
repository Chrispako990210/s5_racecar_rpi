#!/usr/bin/env python

from geometry_msgs.msg import Pose

class Goal:
    def __init__(self, name: str, coord: Pose, wait_time: int):
        self.pose = coord
        self.name = name
        self.atGoal = False
        self.wait_time = wait_time
        if "ballon" in name:
            self.picture_name = f'{name}.png'
            self.path_name = f'path_{name}.bmp'
        else:
            self.picture_name = None
            self.path_name = None
        # TODO: add dynamic params for global planner! Les ballons ne naviguent pas de la meme facon que les end zones!
    
    def get_pose(self):
        return self.pose

    def __str__(self):
        return "[Coord: x = %f, y = %f] [Picture_name: %s] [Path_name: %s] " % (self.x, self.y, self.picture_name, self.path_name)

    def __repr__(self):
        return "[Coord: x = %f, y = %f] [Picture_name: %s] [Path_name: %s] " % (self.x, self.y, self.picture_name, self.path_name)