from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Image

class EnvVar:
    def __init__(self, name, msg_type=None, units=None):
        self.name = name
        self.units = units
    def __str__(self):
        return self.name

AIR_TEMPERATURE = EnvVar("air_temperature", Float64, "degrees C")
""" Temperature of the air in degrees Celcius """

AIR_HUMIDITY = EnvVar("air_humidity", Float64, "percent relative")
"""
A measure of the concentration of water in the air relative to the maximum
concentration at teh current temperature
"""

AIR_CARBON_DIOXIDE = EnvVar("air_carbon_dioxide", Int32, "ppm")
""" The amound of Carbon Dioxide in the air """

WATER_TEMPERATURE = EnvVar("water_temperature", Float64, "degrees C")
""" Temperature of the water in degrees Celcius """

WATER_POTENTIAL_HYDROGEN = EnvVar("water_potential_hydrogen", Float64, "pH")
""" Potential hydrogen of the water """

WATER_ELECTRICAL_CONDUCTIVITY = EnvVar(
    "water_electrical_conductivity", Float64, "uS/cm"
)
""" Electrical conductivity of the water """

WATER_OXIDATION_REDUCTION_POTENTIAL = EnvVar(
    "water_oxidation_reduction_potential", Float64, "mV"
)
""" Oxidation-reduction potential of the water """

WATER_DISSOLVED_OXYGEN = EnvVar("water_dissolved_oxygen", Float64, "mg/L")
""" A measure of the amount of oxygen in the water """

LIGHT_ILLUMINANCE = EnvVar("light_illuminance", Float64, "lux")
""" The intensity of light falling at the plants """

RECIPE_START = EnvVar("recipe_start")
""" Represents the start of a recipe """

RECIPE_END = EnvVar("recipe_end",)
""" Represents the end of a recipe """

MARKER = EnvVar("marker")
""" Marks some user-defined event """

AERIAL_IMAGE = EnvVar("aerial_image", Image, "png")
""" Image from above the tray looking down on the plants"""

FRONTAL_IMAGE = EnvVar("frontal_image", Image, "png")
""" Image from in front of the tray looking towards the plants """

ALL_VARIABLES = frozenset([
    AIR_TEMPERATURE, AIR_HUMIDITY, AIR_CARBON_DIOXIDE, WATER_TEMPERATURE,
    WATER_POTENTIAL_HYDROGEN, WATER_ELECTRICAL_CONDUCTIVITY,
    WATER_OXIDATION_REDUCTION_POTENTIAL, WATER_DISSOLVED_OXYGEN,
    LIGHT_ILLUMINANCE, RECIPE_START, RECIPE_END, MARKER, AERIAL_IMAGE,
    FRONTAL_IMAGE
])
""" The set of all valid variables """

SENSOR_VARIABLES = frozenset([
    AIR_TEMPERATURE, AIR_HUMIDITY, AIR_CARBON_DIOXIDE, WATER_TEMPERATURE,
    WATER_POTENTIAL_HYDROGEN, WATER_ELECTRICAL_CONDUCTIVITY,
    WATER_OXIDATION_REDUCTION_POTENTIAL, WATER_DISSOLVED_OXYGEN,
    LIGHT_ILLUMINANCE
])
""" The set of all valid sensor variables """

CAMERA_VARIABLES = frozenset([
    AERIAL_IMAGE, FRONTAL_IMAGE
])
""" The set of all valid camera variables """
