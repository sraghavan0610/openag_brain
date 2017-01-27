SENSORS = "sensors"
"""
Used for any module that reads from a sensor or any module output that
represents a sensor reading
"""

ACTUATORS = "actuators"
"""
Used for any module that commands an actuator or any module input that
represents an actuator command
"""

CONTROL = "control"
""" Used for any modules that run control loops """

CALIBRATION = "calibration"
"""
Used for any module inputs or outputs that are required for the process of
calibrating a sensors
"""

PERSISTENCE = "persistence"
"""
Used for any modules that read data from ROS topics in order to write it to the
database
"""

default_categories = frozenset([SENSORS, ACTUATORS, PERSISTENCE, CONTROL])
""" The set of categories that should be enabled by default """

all_categories = frozenset([
    SENSORS, ACTUATORS, CALIBRATION, PERSISTENCE, CONTROL
])
""" The set of all valid categories """
