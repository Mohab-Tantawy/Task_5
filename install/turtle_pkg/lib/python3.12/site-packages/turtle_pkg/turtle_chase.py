#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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
            '/turtle1/pose',
            self.player_callback,
            10
        )
        #Timer for collision checking
        self.timer = self.create_timer(0.1, self.check_collisions)

        # Spawn initial enemies
        self.spawn_initial_enemies()

        self.get_logger().info('Turtle chase node has been started.')
    def player_callback(self, msg: Pose):
        self.player_pose = msg

    def enemy_callback(self, msg: Pose, enemy_name: str):
        self.enemy_positions[enemy_name] = msg

    def spawn_initial_enemies(self):
        #Spawn 3 enemies
        for i in range(1, 4): 
            enemy_name = f'enemy{i}'
            self.spawn_enemy(enemy_name)

    def spawn_enemy(self, name: str):
        #Random position and orientation
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0,2 * math.pi)

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda future: self.spawn_callback(future, name, x, y))
    
    def spawn_callback(self, future, name , x , y):
        try: 
            response = future.result()
            if response.name:
                self.create_subscription(
                    Pose,
                    f'/{name}/pose',
                    lambda msg, n=name: self.enemy_callback(msg, n),
                    10
                )
                self.get_logger().info(f'Spawned {response.name} at ({x:.1f}, {y:.1f})')
        except Exception as e:
            self.get_logger().error(f'Spawn service call failed: {e}')
   
    def kill_enemy(self,name: str ):
        #Kill enemy turtle
        request = Kill.Request()
        request.name = name

        future = self.kill_client.call_async(request)
        future.add_done_callback(lambda future: self.kill_callback(future, name))
    
    def kill_callback(self, future, name):
        try:
            future.result()
            if name in self.enemy_positions:
                del self.enemy_positions[name]
            self.get_logger().info(f'Killed {name}')
            
            self.spawn_enemy(name)
        except Exception as e:
            self.get_logger().error(f'Kill service call failed: {e}')
    
    def find_distance(self, pose1: Pose, pose2: Pose) -> float:
        return math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)
    
    def check_collisions(self):
        if self.player_pose is None:
            return
        
        for name,enemy_pose in list(self.enemy_positions.items()):
            distance = self.find_distance(self.player_pose, enemy_pose)
            if distance < 0.5:
                self.get_logger().info(f'{name} was hit!!')

                self.score += 1
                self.publish_score()

                self.kill_enemy(name)

                self.kill_enemy(name)
                
    def publish_score(self):
        msg = Int32()
        msg.data = self.score
        self.score_publisher.publish(msg)
        self.get_logger().info(f'Score: {self.score}')
        
def main(args=None):
    rclpy.init(args=args)
    turtle_chase = TurtleChase()
    rclpy.spin(turtle_chase)
    turtle_chase.destroy_node()
    rclpy.shutdown()