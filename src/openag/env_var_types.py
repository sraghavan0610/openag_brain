class EnvVar:
    def __init__(self, name, units=None):
        self.name = name
        self.units = units
    def __str__(self):
        return self.name

AIR_TEMPERATURE = EnvVar("air_temperature", "degrees C")
""" Temperature of the air in degrees Celcius """

AIR_HUMIDITY = EnvVar("air_humidity", "percent relative")
"""
A measure of the concentration of water in the air relative to the maximum
concentration at teh current temperature
"""

AIR_CARBON_DIOXIDE = EnvVar("air_carbon_dioxide", "ppm")
""" The amound of Carbon Dioxide in the air """

WATER_TEMPERATURE = EnvVar("water_temperature", "degrees C")
""" Temperature of the water in degrees Celcius """

WATER_POTENTIAL_HYDROGEN = EnvVar("water_potential_hydrogen", "pH")
""" Potential hydrogen of the water """

WATER_ELECTRICAL_CONDUCTIVITY = EnvVar(
    "water_electrical_conductivity", "uS/cm"
)
""" Electrical conductivity of the water """

WATER_OXIDATION_REDUCTION_POTENTIAL = EnvVar(
    "water_oxidation_reduction_potential", "mV"
)
""" Oxidation-reduction potential of the water """

WATER_DISSOLVED_OXYGEN = EnvVar("water_dissolved_oxygen", "mg/L")
""" A measure of the amount of oxygen in the water """

RECIPE_START = EnvVar("recipe_start")
""" Represents the start of a recipe """

RECIPE_END = EnvVar("recipe_end",)
""" Represents the end of a recipe """

MARKER = EnvVar("marker")
""" Marks some user-defined event """

AERIAL_IMAGE = EnvVar("aerial_image", "png")
""" Image from above the tray looking down on the plants"""

FRONTAL_IMAGE = EnvVar("frontal_image", "png")
""" Image from in front of the tray looking towards the plants """

LIGHT_ILLUMINANCE = EnvVar("light_illuminance", "lux")
""" The intensity of light falling at the plants """
