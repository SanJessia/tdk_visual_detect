from roboflow import Roboflow
rf = Roboflow(api_key="Ykaw3fGRd6YJyPkRH7w9")
project = rf.workspace("playaround-4brpu").project("tdk_goose")
version = project.version(3)
dataset = version.download("yolov11")