#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import Pose
from turtlesim_msgs.srv import Spawn, Kill
from std_msgs.msg import Int32
import math
import random

class TurtleChase(Node):
    def __init__(self):
        super().__init__('turtle_chase')
        
        #Store enemy positions
        self.enemy_positions = {}
        self.player_pose = None
        self.score = 0

        #Services
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill service not available, waiting...')
         
        # Publisher and Subscriber
        self.score_publisher = self.create_publisher(Int32, 'score', 10)
        self.player_subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.player_callback,
            10
        )
        #Timer for collision checking
        self.timer = self.create_timer(0.1, self.check_collisions)

        # Spawn initial enemies
        self.spawn_inital_enemies()

        self.get_logger().info('Turtle chase node has been started.')
    def player_callback(self, msg):
        self.player_pose = msg
    def enemy_callback(self, msg, enemy_name):
        self.enemy_positions[enemy_name] = msg
    def spawn_inital_enemies(self):
        for i in range(1, 4): 
            enemy_name = f'enemy{i}'
            self.spawn_enemy(enemy_name)
    def spawn_enemy(self, enemy_name):
        #Random position and orientation
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0, 2 * math.pi)

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = enemy_name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda future: self.spawn_callback(future, enemy_name))
    def spawn_callback(self, future, name , x , y):
        try: 
            response = future.result()
            if response.name:
                self.create_subscription(
                    Pose,
                    f'{name}/pose',
                    lambda msg, n=name: self.enemy_callback(msg, n),
                    10
                )
                self.get_logger().info(f'Spawned {response.name} at ({x:.1f}, {y:.1f})')
        except Exception as e:
            self.get_logger().error(f'Spawn service call failed: {e}')
            