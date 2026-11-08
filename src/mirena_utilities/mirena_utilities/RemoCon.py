####################################################################################################
# 
# Remote controller for MANUALLY moving the car around, saving snapshots of the pointcloud and camera, etc...
# This was based off the RemoCon node in mirena_examples
# 
# Author: Alberto Colmenero Fernandez
#
####################################################################################################
import tkinter as tk
import threading
from typing import Literal 

import rclpy 
from rclpy.node import Node 

from mirena_common.msg import CarControl

class RemoCon(Node): 
    """Remote controller for MANUALLY moving the car around, saving snapshots of the pointcloud and camera, etc..."""
     
    # SNAPSHOT_PATH = pathlib.Path("./remocon_snapshots")
    # ID_COUNTER_PATH = SNAPSHOT_PATH.joinpath(".id_counter")

    def __init__(self, cleanup_flag: threading.Event = None): 
        # Ros2-related attributes
        super().__init__("RemoCon") 
        self._car_control_publisher = self.create_publisher(CarControl, 'car/control', 10) 
        self._publish_timer = self.create_timer(0.1,  self.timer_callback) 

        self._emission_lock = threading.Lock()

        # Keyspress register
        self._tracked_keys = {
            "up": False,
            "down": False,
            "left": False,
            "right": False,
            "space": False
        }

        # Initialize the emitter state 
        self.reset_emitter_state()
    
        self.cleanup_flag = cleanup_flag

        # Print console Message 
        self.get_logger().info(
                               "RemoCon Running!\n" \
                               "Use WASD and SPACE to move the car around\n"
                               "[CarControl]\n"
                               ">> WS controls g (gas)\n"
                               ">> AD controls d (steer angle)\n"
                               ">> SPACE gradually reduces 'a' and 'd' to 0\n"
                               )

    def timer_callback(self): 
        # If the cleanup flag is set, delete this node
        if self.cleanup_flag is not None and self.cleanup_flag.is_set():
            rclpy.shutdown()
            return
        if self._emission_lock.acquire(timeout=1):
            # Update the values we are going to send based on the keys pressed
            self.update_local_control() 
            # Make a message
            msg = CarControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.gas = float(self._control_curr_gas)
            msg.steer_angle = float(self._control_curr_steer)
            
            # Send the message trough the publisher
            self._car_control_publisher.publish(msg)

            self._emission_lock.release()
        else: return
    
    def press_key(self, key):
        if key in self._tracked_keys:
            self._tracked_keys[key] = True
            return 

    def release_key(self, key):
        if key in self._tracked_keys:
            self._tracked_keys[key] = False

    def get_tracked_key_states(self) -> dict[str, bool]:  
        return self._tracked_keys

    def get_car_control_values(self) -> tuple[float, float]:
        "Return a tuple of the longitudinal, brake and steer values (in that order)"
        return (self._control_curr_gas, self._control_curr_steer)
    
    def reset_emitter_state(self) -> None:
        self._control_curr_gas = 0
        self._control_curr_steer = 0
    
    def switch_mode(self, to_mode: str) -> None:
        if self._emission_lock.acquire():
            self._emitter_mode = to_mode
            self.reset_emitter_state()

            self._emission_lock.release()

    def update_local_control(self):
        """Method that reads the keyboard keys and updates the control attributes accordingly"""
        if self._tracked_keys["up"]:
            self._control_curr_gas = min(self._control_curr_gas +0.4, 1) # Up to 4g (which is kinda insane but ok...???)
        elif self._tracked_keys["down"]:
            self._control_curr_gas = max(self._control_curr_gas -0.4, -1) # Up to 4g (which is kinda insane but ok...???)

        if self._tracked_keys["left"]: # Unos 30 grados de maniobra de giro
            self._control_curr_steer = min(self._control_curr_steer + 0.10, 0.5)

        elif self._tracked_keys["right"]:
            self._control_curr_steer = max(self._control_curr_steer- 0.10, -0.5)

        if self._tracked_keys["space"]:
            # Reduce both by 25% and snap if less than 0.5 and 0.01 respectively
            self._control_curr_gas = self._control_curr_gas*0.75
            if abs(self._control_curr_gas) <= 0.5:
                self._control_curr_gas = 0

            self._control_curr_steer = self._control_curr_steer*0.75
            if abs(self._control_curr_steer) <= 1:
                self._control_curr_steer = 0


            

class KeyboardGUI(tk.Tk):

    UPDATE_PERIOD = 100 # 100 MS

    def __init__(self, remo_con_instance: RemoCon, cleanup_flag: threading.Event):
        super().__init__()
        
        self.cleanup_flag = cleanup_flag
        self.remo_con = remo_con_instance

        # Set up the window
        self.title("RemoCon")
        self.protocol("WM_DELETE_WINDOW", self.close_gui)

        self.geometry("400x460")
        self.configure(bg="#1a2d47")  # Dark blue background
        
        # CarInput
        self.controls_title = tk.Label(self, text="Controls: ", font=("Helvetica", 18), fg="white", bg="#2e3d5e", width=10, padx=5)
        self.controls_title.pack(pady=10)

        # Labels to show the current state of the arrow key
        self.arrow_display_frame = tk.Frame(self, bg="#1a2d47")
        self.arrow_display_frame.pack(pady=20)

        self.squares = {
            "up": tk.Frame(self.arrow_display_frame, width=50, height=50, bg="#2e3d5e"),
            "left": tk.Frame(self.arrow_display_frame, width=50, height=50, bg="#2e3d5e"),
            "down": tk.Frame(self.arrow_display_frame, width=50, height=50, bg="#2e3d5e"),
            "right": tk.Frame(self.arrow_display_frame, width=50, height=50, bg="#2e3d5e"),
            "space": tk.Frame(self.arrow_display_frame, width=160, height=50, bg="#2e3d5e")
        }
        
        self.squares["up"].grid(row=0, column=1, padx=5, pady=5)
        self.squares["left"].grid(row=1, column=0, padx=5, pady=5)
        self.squares["down"].grid(row=1, column=1, padx=5, pady=5)
        self.squares["right"].grid(row=1, column=2, padx=5, pady=5)
        self.squares["space"].grid(row=2, column=0, columnspan=3, padx=5, pady=5)

        #  CarControl Values Displays "CarControl" 
        self.car_control_val_displays = {
            "gas": tk.Label(self, text="a: 0", font=("Helvetica", 12), fg="white", bg="#2e3d5e", width=10, anchor="w", padx=5),
            "steer": tk.Label(self, text="d: 0.00", font=("Helvetica", 12), fg="white", bg="#2e3d5e", width=10, anchor="w", padx=5),
        }
    

        # Show values
        for lbl in self.car_control_val_displays.values():
            lbl.pack(pady=2)
                # Instructions
        self.instructions = tk.Label(self, text="RemoCon Running!\n" \
                               "Use WASD and SPACE to move the car around\n"
                               "[CarControl]\n"
                               ">> WS controls g (gas)\n"
                               ">> AD controls d (steer angle)\n"
                               ">> SPACE gradually reduces 'a' and 'd' to 0\n",anchor="center",justify=tk.LEFT, font=("Helvetica", 11), fg="white", bg="#2e3d5e", pady=5)
        self.instructions.pack()


        # Bind arrow keys to update current state
        self.bind("<w>", lambda event: self.press_key("up"))
        self.bind("<s>", lambda event: self.press_key("down"))
        self.bind("<a>", lambda event: self.press_key("left"))
        self.bind("<d>", lambda event: self.press_key("right"))
        self.bind("<space>", lambda event: self.press_key("space"))
        self.bind("<KeyRelease-w>", lambda event: self.release_key("up"))
        self.bind("<KeyRelease-s>", lambda event: self.release_key("down"))
        self.bind("<KeyRelease-a>", lambda event: self.release_key("left"))
        self.bind("<KeyRelease-d>", lambda event: self.release_key("right"))
        self.bind("<KeyRelease-space>", lambda event: self.release_key("space"))
        


        # Initialize the timed update system
        self.after(self.UPDATE_PERIOD, self.do_timed_update)

    def do_timed_update(self):
        # If we need to cleanup, call the cleanup method
        if self.cleanup_flag.is_set():
            self.close_gui()  
        # Do updates
        self.update_num_labels()

        # Schedule new update
        self.after(self.UPDATE_PERIOD, self.do_timed_update)


    def press_key(self, key):
        self.remo_con.press_key(key)
        self.update_arrow_display()

    def release_key(self, key):
        self.remo_con.release_key(key)
        self.update_arrow_display()
            
    def update_arrow_display(self):
        for direction, is_pressed in self.remo_con.get_tracked_key_states().items():
            color = "#4a90e2" if is_pressed else "#2e3d5e"  # Light blue when pressed, dark blue when not
            if direction in self.squares:
                self.squares[direction].configure(bg=color)

    def update_num_labels(self):
        accel, steer = self.remo_con.get_car_control_values()
        self.car_control_val_displays["gas"].config(text=f"a: {accel}")
        self.car_control_val_displays["steer"].config(text=f"d: {steer}")

    def close_gui(self):
        self.cleanup_flag.set() 
        self.quit()


def ros_thread_exec(node: Node, cleanup_flag: threading.Event):
    try:
        # Start the node 
        rclpy.spin(node)
    finally:
        if not cleanup_flag.is_set():
            cleanup_flag.set()


def keyboardGUI_thread_exec(node: RemoCon, cleanup_flag: threading.Event):
    try: 
        # Initialize the GUI and run it
        gui = KeyboardGUI(node, cleanup_flag)
        while not cleanup_flag.is_set():
            gui.update_idletasks()
            gui.update() 
    finally:
        # gui.destroy()
        if not cleanup_flag.is_set():
            cleanup_flag.set()

def main(args=None):
    """ The main function is the entry point to the node, and defines the behabiour of it whenever it is launched"""
    
    # We initialize a claenup event to kill both threads as soon as one finishes up
    cleanup_flag = threading.Event()

    # ros2 initialization and node instanciation
    rclpy.init(args=args)
    remo_con = RemoCon(cleanup_flag)
    
    # We initialize the threads and start them
    ros_thread = threading.Thread(target=ros_thread_exec, args=(remo_con, cleanup_flag))
    ros_thread.start()
    gui_thread = threading.Thread(target=keyboardGUI_thread_exec, args=(remo_con, cleanup_flag))
    gui_thread.start()

if __name__ == '__main__':
    main()
