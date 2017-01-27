RECIPE = "recipes"
""" The database for holding :py:class:`~openag.models.Recipe` objects """

SOFTWARE_MODULE_TYPE = "software_module_type"
"""
The database for holding :py:class:`~openag.modles.SoftwareModuleType` objects
"""

SOFTWARE_MODULE = "software_module"
"""
The database for holding :py:class:`~openag.models.SoftwareModule` objects
"""

FIRMWARE_MODULE_TYPE = "firmware_module_type"
"""
The database for holding :py:class:`~openag.models.FirmwareModuleType`
objects
"""

FIRMWARE_MODULE = "firmware_module"
"""
The database for holding :py:class:`~openag.models.FirmwareModule` objects
"""

ENVIRONMENT = "environment"
""" The database for holding :py:class:`~openag.models.Environment` objects """

ENVIRONMENTAL_DATA_POINT = "environmental_data_point"
""" 
The database for holding :py:class:`~openag.models.EnvironmentalDataPoint`
objects
"""

all_dbs = frozenset([
    RECIPE, SOFTWARE_MODULE_TYPE, SOFTWARE_MODULE, FIRMWARE_MODULE_TYPE,
    FIRMWARE_MODULE, ENVIRONMENT, ENVIRONMENTAL_DATA_POINT
])
""" The set of all database names """

global_dbs = frozenset([
    RECIPE, SOFTWARE_MODULE_TYPE, FIRMWARE_MODULE_TYPE
])
""" The names of the databases that can be synced from a global source """

per_farm_dbs = all_dbs - global_dbs
""" The names of the databases that have only farm-specific information """
