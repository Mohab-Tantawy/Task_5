#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim_msgs.srv import Spawn, Kill  # Changed from turtlesim_msgs to turtlesim
from turtlesim_msgs.msg import Pose  # Changed from turtlesim_msgs to turtlesim
from std_msgs.msg import Int32
import math
import random

class TurtleChase(Node):
    def __init__(self):
        super().__init__('turtle_chase')
        
        # Dictionary to store enemy positions
        self.enemy_positions = {}
        
        # Player turtle pose
        self.player_pose = None
        
        # Score
        self.score = 0
        
        # Service clients
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        
        # Wait for services to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting...')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('kill service not available, waiting...')
        
        # Score publisher
        self.score_publisher = self.create_publisher(Int32, 'score', 10)
        
        # Player pose subscriber
        self.player_sub = self.create_subscription(
            Pose, 
            '/turtle1/pose', 
            self.player_callback, 
            10
        )
        
        # Timer for collision detection
        self.collision_timer = self.create_timer(0.1, self.check_collisions)
        
        # Spawn initial enemies
        self.spawn_initial_enemies()
        
        self.get_logger().info('Turtle Chase game started!')

    def player_callback(self, msg: Pose):
        """Callback for receiving /turtle1/pose"""
        self.player_pose = msg

    def enemy_callback(self, msg: Pose, enemy_name: str):
        """Callback for receiving enemy poses"""
        self.enemy_positions[enemy_name] = msg

    def spawn_initial_enemies(self):
        """Spawn 3 initial enemy turtles"""
        for i in range(1, 4):
            enemy_name = f'enemy{i}'
            self.spawn_enemy(enemy_name)

    def spawn_enemy(self, name: str):
        """Spawn an enemy turtle at random position"""
        # Generate random position (x between 1-10, y between 1-10)
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 2 * math.pi)
        
        # Create spawn request
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        # Call spawn service
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda future: self.spawn_callback(future, name, x, y))

    def spawn_callback(self, future, name, x, y):
        """Callback for spawn service"""
        try:
            response = future.result()
            if response.name:
                # Create subscription for this enemy's pose
                self.create_subscription(
                    Pose,
                    f'/{name}/pose',
                    lambda msg, n=name: self.enemy_callback(msg, n),
                    10
                )
                self.get_logger().info(f'Spawned {name} at ({x:.2f}, {y:.2f})')
        except Exception as e:
            self.get_logger().error(f'Failed to spawn {name}: {e}')

    def kill_enemy(self, name: str):
        """Kill an enemy turtle"""
        request = Kill.Request()
        request.name = name
        
        future = self.kill_client.call_async(request)
        future.add_done_callback(lambda future: self.kill_callback(future, name))

    def kill_callback(self, future, name):
        """Callback for kill service"""
        try:
            future.result()
            # Remove from enemy positions dictionary
            if name in self.enemy_positions:
                del self.enemy_positions[name]
            self.get_logger().info(f'Killed {name}')
            
            # Respawn the enemy after successful kill
            self.spawn_enemy(name)
        except Exception as e:
            self.get_logger().error(f'Failed to kill {name}: {e}')

    def find_distance(self, pose1: Pose, pose2: Pose) -> float:
        """Calculate distance between two poses"""
        return math.sqrt((pose2.x - pose1.x) ** 2 + (pose2.y - pose1.y) ** 2)

    def check_collisions(self):
        """Timer callback to check for collisions"""
        # If player pose isn't available, exit the function
        if self.player_pose is None:
            return
            
        # Make a copy of items to avoid modification during iteration
        for name, enemy_pose in list(self.enemy_positions.items()):
            # Find distance between player and enemy
            distance = self.find_distance(self.player_pose, enemy_pose)
            
            # If distance < 0.05, count as hit
            if distance < 0.5:
                self.get_logger().info(f'{name} was hit!')
                
                # Update score and publish it
                self.score += 1
                self.publish_score()
                
                # Kill the enemy (respawn happens in kill_callback)
                self.kill_enemy(name)

    def publish_score(self):
        """Publish the current score"""
        score_msg = Int32()
        score_msg.data = self.score
        self.score_publisher.publish(score_msg)
        self.get_logger().info(f'Score: {self.score}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleChase()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()