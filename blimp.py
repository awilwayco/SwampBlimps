class Blimp:
    def __init__(self, blimp_id):
        # Blimp ID
        self.blimp_id = blimp_id

        # Blimp Name (will be set by node handler)
        self.blimp_name = None

        # Blimp Type
        self.blimp_type = 0 # 0: Catching, 1: Attacking

        # Auto 
        self.auto = False

        # Killed
        self.killed = False

        # Motor Commands
        self.motorCommands = [0.0, -0.0, 0.0, -0.0]

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

        # Connected status
        self.connected = False  # Initially not connected

        # Blimp Livestream Current Frame
        self.frame = None

        # Bounding Box Values
        self.bounding_box = None

        # Barometer Value
        self.barometer = 100700.0

        # Calibrate Barometer
        self.calibrateBarometer = False

        # Height
        self.height = None

        # Z Velocity
        self.z_velocity = None

        # Last Online
        self.last_online = 0

        # Last Message
        self.log = None

    def to_dict(self):
        return {
            "blimp_id": self.blimp_id,
            "blimp_name": self.blimp_name,
            "blimp_type": self.blimp_type,
            "auto": self.auto,
            "killed": self.killed,
            "motorCommands": self.motorCommands,
            "grabbing": self.grabbing,
            "shooting": self.shooting,
            "goal_color": self.goal_color,
            "target_color": self.target_color,
            "state_machine": self.state_machine,
            "connected": self.connected,
            "barometer": self.barometer,
            "calibrateBarometer": self.calibrateBarometer,
            "height": self.height,
            "z_velocity": self.z_velocity,
            "log": self.log
        }

    def update_dict(self, data_dict):
        if "blimp_id" in data_dict:
            self.blimp_id = data_dict["blimp_id"]
        if "blimp_name" in data_dict:
            self.blimp_name = data_dict["blimp_name"]
        if "blimp_type" in data_dict:
            self.blimp_type = data_dict["blimp_type"]
        if "auto" in data_dict:
            self.auto = data_dict["auto"]
        if "killed" in data_dict:
            self.killed = data_dict["killed"]
        if "motorCommands" in data_dict:
            self.motorCommands = data_dict["motorCommands"]
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
        if "connected" in data_dict:
            self.connected = data_dict["connected"]   
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
