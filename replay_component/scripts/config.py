import json

config_file_path = '/home/jesmij/ros_ws/src/replay_component/config/config.json'

with open(config_file_path) as config_file:
    config = json.load(config_file)

class Config:
    BENCH_DATA_DIRECTORY = config.get("BENCH_DATA_DIRECTORY")

    SQLALCHEMY_TRACK_MODIFICATIONS = False

    SERVICES = [
        {'path': '.server.routes', 'blueprint': 'server'}
    ]
    
    BUNDLE_FILE_LOC = config.get("BUNDLE_BENCH_FILE_LOCATION")
    TCP_IP = config.get("TCP_IP")
    PORT = config.get("PORT")


