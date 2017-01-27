SENSORS = "sensors"
"""
All data streams coming from sensors or informing about sensors should be
located under this namespace.
"""

ACTUATORS = "actuators"
"""
All data streams used to directly control an actuator or inform about an
actuator should be located under this namespace.
"""

CALIBRATION = "calibration"
"""
All data streams used for calibrating the sensors in the system should be
located under this namespace.
"""

ENVIRONMENTS = "environments"
"""
This namespace provides an alternate organization of the sensor and actuator
information. It should be further broken down into individual namespaces by
environment ID, and all of the sensor and actuators topics should be copied to
the appropriate namespace based on what environment they are in. For example,
the namespace `/environments/environment_1` should contain data streams from
all sensors and actuators in environment_1. All control and persistence modules
operate under this namespace.
"""
