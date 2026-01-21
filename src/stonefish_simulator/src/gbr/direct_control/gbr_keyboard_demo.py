#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from pynput import keyboard
import threading
from packages.gbr_direct_interface import GBRDirectInterface

# Control state
active_keys = set()
lock = threading.Lock()
emergency_stop = False
latest_thrust_values = [0.0] * 8

thrust_map = {
    'w': [20.0, 20.0, -20.0, -20.0, 0.0, 0.0, 0.0, 0.0],    # Forward (psoitive Y)
    's': [-20.0, -20.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0],    # Backward (negative Y)
    'a': [20.0, -20.0, 20.0, -20.0, 0.0, 0.0, 0.0, 0.0],    # Left turn (negative X)
    'd': [-20.0, 20.0, -20.0, 20.0, 0.0, 0.0, 0.0, 0.0],    # Right turn (positive X)
    'q': [0.0, 0.0, 0.0, 0.0, -20.0, -20.0, -20.0, -20.0],  # Down (positive Z)
    'e': [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, 20.0, 20.0],      # Up   (negative Z)
    'Key.up': [0.0, 0.0, 0.0, 0.0, -20.0, -20.0, 20.0, 20.0],      # Negative pitch (points down)
    'Key.down': [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, -20.0, -20.0],    # Positive pitch (points up)
    'Key.left': [0.0, 0.0, 0.0, 0.0, 20.0, -20.0, 20.0, -20.0],    # Positive Roll (rolls left)
    'Key.right': [0.0, 0.0, 0.0, 0.0, -20.0, 20.0, -20.0, 20.0],   # Negative Roll (rolls right)
    'Key.page_up': [20.0, -20.0, -20.0, 20.0, 0.0, 0.0, 0.0, 0.0],     # Negative yaw (turns left)
    'Key.page_down': [-20.0, 20.0, 20.0, -20.0, 0.0, 0.0, 0.0, 0.0],   # Positive yaw (turns right)
}

def on_press(key):
    global active_keys, emergency_stop
    try:
        key_str = key.char if hasattr(key, 'char') and key.char else str(key)
        with lock:
            active_keys.add(key_str)
            if key_str == 'Key.space':
                emergency_stop = True
    except AttributeError:
        pass

def on_release(key):
    global active_keys
    try:
        key_str = key.char if hasattr(key, 'char') and key.char else str(key)
        with lock:
            active_keys.discard(key_str)
    except AttributeError:
        pass

class KeyboardController(Node):
    def __init__(self):
        super().__init__('gbr_keyboard_controller')
        self.rov = GBRDirectInterface(self)
        self.active_keys = set()
        self.emergency_stop = False
        self.thrust_values = [0.0] * 8
        
        # Start keyboard listener i egen thread
        self.listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release
        )
        self.listener.start()
        
        # Create timer for control loop
        self.timer = self.create_timer(0.05, self.control_callback)  # 20Hz
        
    def control_callback(self):
        global active_keys, emergency_stop
        
        with lock:
            current_keys = active_keys.copy()
            current_estop = emergency_stop
            
        # Display status
        print(f"\033[2J\033[H")  # Clear screen
        print(f"Active Keys: {current_keys}")
        print(f"Emergency Stop: {'ACTIVE' if current_estop else 'inactive'}")
        
        if current_estop == 2:
            self.rov.stop()
            self.get_logger().error("EMERGENCY STOP ACTIVATED!")
        else:
            # Combine thrust values
            thrust_values = [0.0] * 8
            for key in current_keys:
                if key in thrust_map:
                    thrust_values = [sum(pair) for pair in zip(
                        thrust_values, 
                        thrust_map[key]
                    )]
            
            # Apply thrust
            self.rov.set_thrusters(thrust_values)
            self.thrust_values = thrust_values
            
        self.rov.print_state()
        
    def destroy_node(self):
        self.listener.stop()
        self.rov.stop()
        super().destroy_node()

def main():
    rclpy.init()
    
    try:
        node = KeyboardController()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
       print("\nShutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()