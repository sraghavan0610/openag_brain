class DbName:
    RECIPE = 'recipes'
    SOFTWARE_MODULE_TYPE = 'software_module_type'
    SOFTWARE_MODULE = 'software_module'
    FIRMWARE_MODULE_TYPE = 'firmware_module_type'
    FIRMWARE_MODULE = 'firmware_module'
    ENVIRONMENT = 'environment'
    ENVIRONMENTAL_DATA_POINT = 'environmental_data_point'

global_dbs = [
    DbName.RECIPE,
    DbName.SOFTWARE_MODULE_TYPE,
    DbName.FIRMWARE_MODULE_TYPE,
]

per_farm_dbs = [
    DbName.SOFTWARE_MODULE,
    DbName.FIRMWARE_MODULE,
    DbName.ENVIRONMENT,
    DbName.ENVIRONMENTAL_DATA_POINT,
]

all_dbs = global_dbs + per_farm_dbs
