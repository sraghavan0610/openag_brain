import json
import subprocess
from os import path
from shutil import rmtree
from tempfile import mkdtemp
from openag.utils import make_dir_name_from_url
from openag.models import FirmwareModuleType
from openag.db_names import FIRMWARE_MODULE_TYPE

def update_record(obj, temp_folder):
    if not "repository" in obj:
        return obj
    repo = obj["repository"]
    if repo["type"] != "git":
        return obj
    url = repo["url"]
    branch = repo.get("branch", "master")
    dir_name = make_dir_name_from_url(url)
    subprocess.call(
        ["git", "clone", "-b", branch, url, dir_name], cwd=temp_folder
    )
    config_path = path.join(temp_folder, path.join(dir_name, "module.json"))
    with open(config_path) as f:
        info = json.load(f)
    new_obj = dict(obj)
    new_obj.update(info)
    return new_obj

def update_module_types(server):
    db = server[FIRMWARE_MODULE_TYPE]
    temp_folder = mkdtemp()
    for _id in db:
        if _id.startswith("_"):
            continue
        obj = db[_id]
        new_obj = update_record(FirmwareModuleType(obj), temp_folder)
        new_obj["_rev"] = obj["_rev"]
        if new_obj != obj:
            print "Updating object"
            db[_id] = new_obj
    rmtree(temp_folder)
