class Blimp:
    def __init__(self, blimp_id):
        # Blimp ID
        self.id = blimp_id

        # Blimp Name (will be set by node handler)
        self.name = None

        # Blimp Type
        self.type = 0 # 0: Catching, 1: Attacking

        # Auto 
        self.auto = False

        # Killed
        self.killed = False

        # Motor Commands
        self.motor_commands = [0.0, -0.0, 0.0, -0.0]

        # Grabbing
        self.grabbing = False

        # Shooting
        self.shooting = False

        # Target Goal Color
        self.goal_color = 0 # 0: Orange, 1: Yellow

        # Target Color
        self.target_color = 0 # 0: Blue, 1: Red

        # State Machine
        self.state_machine = 0
        """
        0: searching
        1: approach
        2: catching
        3: caught
        4: goalSearch
        5: approachGoal
        6: scoringStart
        7: shooting
        8: scored
        """

        # Controlled status
        self.selected = False  # Initially not selected

        # Blimp Livestream Current Frame
        self.frame = None

        # Bounding Box Values
        self.bounding_box = None

        # Barometer Value
        self.barometer = 99668.2 # Competition Default Value

        # Calibrate Barometer
        self.calibrate_barometer = False

        # Height
        self.height = None

        # Z Velocity
        self.z_velocity = None

        # Last Online
        self.last_online = 0

        # Last Message
        self.log = None

        # Show Image
        self.show_image = False

        self.frontend_update_auto = False

    def to_dict(self):
        return {
            "blimp_id": self.id,
            "blimp_name": self.name,
            "blimp_type": self.type,
            "auto": self.auto,
            "killed": self.killed,
            "motorCommands": self.motor_commands,
            "grabbing": self.grabbing,
            "shooting": self.shooting,
            "goal_color": self.goal_color,
            "target_color": self.target_color,
            "state_machine": self.state_machine,
            "selected": self.selected,
            "barometer": self.barometer,
            "calibrateBarometer": self.calibrate_barometer,
            "height": self.height,
            "z_velocity": self.z_velocity,
            "log": self.log
        }

    def update_dict(self, data_dict):
        if "blimp_id" in data_dict:
            self.id = data_dict["blimp_id"]
        if "blimp_name" in data_dict:
            self.name = data_dict["blimp_name"]
        if "blimp_type" in data_dict:
            self.type = data_dict["blimp_type"]
        if "auto" in data_dict:
            self.auto = data_dict["auto"]
        if "killed" in data_dict:
            self.killed = data_dict["killed"]
        if "motorCommands" in data_dict:
            self.motor_commands = data_dict["motorCommands"]
        if "grabbing" in data_dict:
            self.grabbing = data_dict["grabbing"]
        if "shooting" in data_dict:
            self.shooting = data_dict["shooting"]
        if "goal_color" in data_dict:
            self.goal_color = data_dict["goal_color"]
        if "target_color" in data_dict:
            self.target_color = data_dict["target_color"]
        if "state_machine" in data_dict:
            self.state_machine = data_dict["state_machine"]
        if "selected" in data_dict:
            self.selected = data_dict["selected"]   
        if "barometer" in data_dict:
            self.barometer = data_dict["barometer"]
        if "calibrateBarometer" in data_dict:
            self.calibrateBarometer = data_dict["calibrateBarometer"]
        if "height" in data_dict:
            self.height = data_dict["height"]
        if "z_velocity" in data_dict:
            self.z_velocity = data_dict["z_velocity"]
        if "log" in data_dict:
            self.log = data_dict["log"]

