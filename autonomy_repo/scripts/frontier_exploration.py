#!/usr/bin/env python3

import numpy as np
import typing as T

import rclpy
from rclpy.node import Node
from asl_tb3_msgs.msg import TurtleBotState
from nav_msgs.msg import OccupancyGrid
from asl_tb3_lib.grids import StochOccupancyGrid2D
from std_msgs.msg import Bool
from scipy.signal import convolve2d

class FrontierExploration(Node):
    def __init__(self) -> None:
        super().__init__("frontier")
        
        self.state: T.Optional[TurtleBotState] = None
        self.occupancy: T.Optional[StochOccupancyGrid2D] = None
        
        self.state_sub = self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.nav_success_pub = self.create_subscription(Bool, "/nav_success", self.plan_frontier_state, 10)
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)
        
        self.frontier_index = 0
        self.frontier_states = np.empty((0, 2))
        self.initial_goal_published = False
        
    def plan_frontier_state(self, msg: Bool) -> None:
        if msg.data:
            self.frontier_index = 0
            self.frontier_states = self.explore()
            self.publish_goal_state()
        else:
            self.frontier_index += 1
            self.publish_goal_state()
               
        
    def publish_goal_state(self) -> None:
        if self.frontier_index >= len(self.frontier_states):
            self.get_logger().info("Exploration complete: no more viable frontiers")
            return
        
        goal_xy = self.frontier_states[self.frontier_index]
        goal = TurtleBotState()
        goal.x = float(goal_xy[0])
        goal.y = float(goal_xy[1])

        dx = goal.x - float(self.state.x)
        dy = goal.y - float(self.state.y)
        goal.theta = float(np.arctan2(dy, dx))
        self.cmd_nav_pub.publish(goal) 
        
    def explore(self) -> np.ndarray:
        window_size = 13   
        num_cells = window_size**2

        unknown_mask = (self.occupancy.probs < 0).astype(float)
        occupied_mask = (self.occupancy.probs >= self.occupancy.thresh).astype(float)
        free_mask = ((self.occupancy.probs >= 0) & (self.occupancy.probs < self.occupancy.thresh)).astype(float)

        kernel = np.ones((window_size, window_size))

        unknown_count = convolve2d(unknown_mask, kernel, mode='same', boundary='fill', fillvalue=0)
        occupied_count = convolve2d(occupied_mask, kernel, mode='same', boundary='fill', fillvalue=0)
        free_count = convolve2d(free_mask, kernel, mode='same', boundary='fill', fillvalue=0)

        frontier_mask = (unknown_count / num_cells >= 0.2) & \
                        (occupied_count == 0) & \
                        (free_count / num_cells >= 0.3)

        frontier_idx = np.argwhere(frontier_mask)
        frontier_states = np.array([self.occupancy.grid2state(idx[::-1]) for idx in frontier_idx])
        
        if self.state is not None and frontier_states.size > 0:
            dx = frontier_states[:, 0] - self.state.x
            dy = frontier_states[:, 1] - self.state.y
            distances = np.sqrt(dx**2 + dy**2)
            sorted_indices = np.argsort(distances)
            frontier_states = frontier_states[sorted_indices]
            
        return frontier_states
        
    def state_callback(self, msg: TurtleBotState) -> None:
        self.state = msg
        
        if not self.initial_goal_published and self.occupancy is not None:
            self.frontier_index = 0
            self.frontier_states = self.explore()
            self.publish_goal_state()
            self.initial_goal_published = True
            
        
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )
        
        if not self.initial_goal_published and self.state is not None:
            self.frontier_index = 0
            self.frontier_states = self.explore()
            self.publish_goal_state()
            self.initial_goal_published = True
    
if __name__ == "__main__": 
    rclpy.init()
    node = FrontierExploration()
    rclpy.spin(node)
    rclpy.shutdown()