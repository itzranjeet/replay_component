# Reprocessing Sensor Validation: Replay Component

This code provides a framework to easily add different datasets, in order to run Sensor Validation using ROS and C++/Python API. 

## Getting Started

To get started you need to:

1. Clone the repository.
2. Install dependencies and Prerequisites
3. Compile the package(catkin_make).
5. Run Executable

### Prerequisites

1. Ubuntu 18.04 bionic
2. Python Dependencies
3. ROS Melodic
 

### Installing

**Python Dependencies**
```
sudo apt install python3-dev python3-pip
pip3 install -U --user six numpy wheel mock
pip3 install -U --user keras_applications==1.0.6 --no-deps
pip3 install -U --user keras_preprocessing==1.0.5 --no-deps
pip3 install -r requirements.txt
```

### To-Do List

**Go to config folder**
1. Open config.json
- update "BENCH_DATA_DIRECTORY": "Bench input path"
- update  "BUNDLE_BENCH_FILE_LOCATION" : "Bench output path"

2. Open input.csv
- update input images paths(Line #1)
- update unzip paths(Line #2)

**Go to scripts**
1. Open config.py
-  update config.json file path (visit config folder)

**Place the input.zip in data folder**


### Execution

**Compilation**
```
cd workspace
catkin_make --only-pkg-with-deps replay_component
```

**Roslaunch**
```
roslaunch repaly_component start.launch
```

**Python Local Agent and UI**
```
python3 app.py
python3 UI.py
```
