import logging
import json
import os
import sys
import paramiko
from flask import Flask, Response, request, send_file
import requests
from scp import SCPClient
from flask import Response
from flask_api import status
from config import Config
import os.path
from os import path
import shutil
from flask import jsonify
from helper_bench import getHeader
import socket

app = Flask(__name__)

folder_path_in =  Config.BENCH_DATA_DIRECTORY
folder_path_out = os.path.join(Config.BUNDLE_FILE_LOC,"Reprocessed_Output")


def createSSHClient(server, port, user, password):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(server, port, user, password)
    return client


def __checkProcessingStatus():
    #
    # TODO : Connect Bench and return if reprocessing started
    #        different state of bench are
    #           - UNKOWN
    #           - error
    #           - data_dispatched
    #           - completed_with_success
    #           - completed_with_error
    #           - data_dispatching_in_progress
    #

    with open('bench_bundle.json') as f:
        data = json.load(f)
#    print("DATA : ",data)
    seq_list = data["sequence"]
    final_res = []
    for rec in range(len(seq_list)):
        final_res.append(seq_list[rec]["status"])

    checker = 0
    counter = 0
    for checker in range(len(final_res)):
        if final_res[checker] == 'processing_completed':
            counter = counter + 1

    if counter == len(final_res):
        return "processing_completed"
    else:
        return "processing_in_progress"


@app.route('/check-json-status', methods=['GET', 'POST'])
def checkJsonFile():
    clean_data = request.get_json()
    bench_output_location = clean_data['bench_output_location']
    bundle_file_path = os.path.join(
        bench_output_location, "Reprocessed_Output", "configuration.json")
    if path.exists(bundle_file_path):
        return Response(json.dumps({"status": "reprocessing_started"}), status=200, mimetype='application/json')
    else:
        return Response(json.dumps({"status": "reprocessing_not_started"}), status=200, mimetype='application/json')
#
# api :  checkBenchDataStatus(): will maintain json file to know if processing is completed or not
#


@app.route('/check-bench-data-status', methods=['GET', 'POST'])
def checkBenchDataStatus():
    checker = 0
    counter = 0
    data_from_req = request.data
    clean_data = request.get_json()
    bench_dump_location = clean_data['bench_dump_location']
    bench_output_location = clean_data['bench_output_location']
    # dummyProcessor(bench_output_location, bench_dump_location)
    bundle_file_path = os.path.join(
        bench_output_location, "Reprocessed_Output", "configuration.json")
    with open(bundle_file_path) as f:
        data = json.load(f)

    seq_list = data["video_list"]
    final_res = []

    for rec in range(len(seq_list)):
        final_res.append(seq_list[rec]["status"])

    for checker in range(len(final_res)):
        if final_res[checker] == 'completed_with_success' or final_res[checker] == 'completed_with_error':
            counter = counter + 1

    if counter == len(final_res):
        return Response(json.dumps({"status": "reprocessing_completed"}), status=200, mimetype='application/json')
    else:
        return Response(json.dumps({"status": "reprocessing_in_progress"}), status=204, mimetype='application/json')


# dummy function to create output folder as we dont have processing module as of now.
def dummyProcessor(output_location, dump_location):
    target_dir_name = "Reprocessed_Output"
    if path.isdir(os.path.join(output_location, target_dir_name)):
        os.mknod(os.path.join(output_location,
                              target_dir_name, "reprocessed_output2.json"))
        shutil.copy(os.path.join(dump_location, "bench_bundle.json"),
                    os.path.join(output_location, target_dir_name))
        return
    else:
        os.mkdir(os.path.join(output_location, target_dir_name))
        os.mknod(os.path.join(output_location,
                              target_dir_name, "reprocessed_output2.json"))
        shutil.copy(os.path.join(dump_location, "bench_bundle.json"),
                    os.path.join(output_location, target_dir_name))
        return


@app.route('/make-bundle-file', methods=['GET', 'POST'])
def makeBundleFile():
    final_data = request.get_json()
    bundle_file_path = os.path.join(
        final_data["bench_dump_location"], "configuration.json")
    json_object = {
        "video_list": final_data["video_list"],
        "parameters": final_data["parameters"],
        "bench_dump_location": final_data["bench_dump_location"],
        "bench_output_location": final_data["bench_output_location"],
        "sensor": final_data["sensor"]
    }
    data_to_write = json.dumps(json_object, indent=4)
    with open(bundle_file_path, "w") as outfile:
        outfile.write(data_to_write)
    return Response(status=200)


@app.route('/ping', methods=['GET', 'POST'])
def ping():
    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    return status

#
# usage
#   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/replay-ping-test
#


@app.route("/replay-ping-test", methods=['GET', 'POST'])
def replay_ping_test():
    logging.info("inside replay-ping-test")
    data = json.loads(request.data.decode('utf-8'))
    logging.info(data)
    result = json.dumps(
        {"status": "available", "method": "home", "data": data})
    status = Response(result, status=200, mimetype='application/json')
    return status


@app.route('/clean-bench-input', methods=['GET', 'POST'])
def cleanBench():
    data = request.get_json()
    if 'bench_output_location' in data:
        logging.info('output')
    if 'bench_dump_location' in data:
        logging.info('dump')
    # if data.get('bench_dump_location',None) != None :
    if 'bench_dump_location' in data:
        logging.info("bench dump location cleaning process :  " +
                     data['bench_dump_location'])
        for item in os.listdir(data['bench_dump_location']):
            item_path = os.path.join(data['bench_dump_location'], item)
            logging.info("removing " + item_path)
            if os.path.isfile(item_path):
                os.remove(item_path)
            elif os.path.isdir(item_path):
                shutil.rmtree(item_path)
            else:
                print("cannot remove " + item_path)
            logging.info("removed " + item_path)

    else:
        # if data.get('bench_output_location',None) != None :
        # if 'bench_output_location' in data:
        logging.info("bench output location cleaning......", +
                     data['bench_output_location'])
        for item in os.listdir(data['bench_output_location']):
            item_path = os.path.join(data['bench_output_location'], item)
            logging.info("removing " + item_path)
            if os.path.isfile(item_path):
                os.remove(item_path)
            elif os.path.isdir(item_path):
                shutil.rmtree(item_path)
            else:
                logging.info("cannot remove " + item_path)
            logging.info("removed " + item_path)

    result = json.dumps({"status": "bench-data-cleaned-successfully"})

    return result


@app.route('/clean-bench-output', methods=['GET', 'POST'])
def cleanBenchOutput():
    data = request.get_json()
    for items in os.listdir(data['bench_output_location']):
        logging.info(items)
        item_path = os.path.join(data['bench_output_location'], items)
        logging.info(item_path)
        if os.path.isdir(item_path):
            logging.info("removing : " + item_path)
            shutil.rmtree(item_path)
    return Response(json.dumps({"status": "bench-data-cleaned-successfully"}))


@app.route('/check-bench-status', methods=['GET', 'POST'])
def checkBenchStatus():
    #    print('im in check Bench Status')
    is_dir = False
    is_file = False
    is_processing = False

    data_from_req = request.data
    final_data = json.loads(data_from_req.decode('utf-8'))
    bench_dump_location = final_data['bench_dump_location']
    bench_output_location = final_data['bench_output_location']

    if path.isdir(bench_dump_location):
        bench_dump_location_is_dir = True

    if path.exists(bench_dump_location):
        bench_dump_location_is_file = True

    if path.isdir(bench_output_location):
        bench_output_location_is_dir = True

    if path.exists(bench_output_location):
        bench_output_location_is_file = True

    is_processing = __checkProcessingStatus()
    if is_processing == 'processing_completed':

        result = json.dumps({"status": "available", "bench_dump_location_is_dir": bench_dump_location_is_dir, "bench_dump_location_is_file": bench_dump_location_is_file,
                             "bench_output_location_is_dir": bench_output_location_is_dir, "bench_output_location_is_file": bench_output_location_is_file, "is_processing": is_processing})

        return Response(json.dumps(result), status=200, mimetype='application/json')
    else:
        return Response(status=201)


@app.route('/bench/', methods=['GET', 'POST'])
def server():
    print(Config.BENCH_DATA_DIRECTORY)
    if request.method == 'POST':
        ssh = createSSHClient('localhost', 22, 'rajivt', 'password')
        scp = SCPClient(ssh.get_transport())
        data = request.get_json()
        data = json.loads(data)
        logging.info(data)
        logging.info(data)
        for x in data:
            scp.get(x['paths'], Config.BENCH_DATA_DIRECTORY, recursive=True)
        data = json.dumps(data)
        scp.close()
        resp = Response(data, status=200, mimetype='application/json')
        return resp


@app.route('/check-status/', methods=['GET', 'POST'])
def checkStatusOfProcessing():
    # it will tell server if processing has completed or not
    result = os.path.isdir('/home/shubham/Desktop/result/Done/')
    if result:
        return Response(status=200)
    else:
        return Response(status=109)


@app.route('/post-processing/', methods=['GET', 'POST'])
def sendAllTheFiles():
    # assumimg processing module has done its work and generated output files
    # on the bench .
    basepath = '/home/shubham/Desktop/result/'
    list_of_files = []
    for entry in os.listdir(basepath):
        print("entries : ", entry)
        # if entry is a folder
        if os.path.isdir(basepath+entry):
            print("Directory")
            new_path = basepath+entry
            for items in os.listdir(new_path):
                if os.path.isfile(os.path.join(new_path, items)):
                    list_of_files.append(items)
        if os.path.isfile(os.path.join(basepath, entry)):
            list_of_files.append(entry)

    print("list of files sending to server : ", list_of_files)
    final_result = {'files': list_of_files}
    final_result = json.dumps(final_result)
    return Response(final_result, status=200, mimetype='application/json')


#
# usage
#   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/ping-replay-test
#
@app.route("/ping-replay-test", methods=['GET', 'POST'])
def ping_replay_test():

    TCP_IP = Config.TCP_IP
    TCP_PORT = int(Config.PORT)
    BUFFER_SIZE = 1024
    data = "Hello, World!"
    MESSAGE = bytes(data, 'utf-8')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    s.close()

    logging.info("received data:", data)

    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    return status


#   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/start_bench
@app.route("/start_bench", methods=['GET', 'POST'])
def start_bench():
    if not os.path.exists(os.path.join(Config.BUNDLE_FILE_LOC,"Reprocessed_Output")):
        os.mkdir(os.path.join(Config.BUNDLE_FILE_LOC,"Reprocessed_Output"))
    print("IN START BENCH")
    TCP_IP = "127.0.0.1"
    TCP_PORT = 5005
    BUFFER_SIZE = 1024
    data = "start_bench"
    MESSAGE = bytes(data, 'utf-8')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    s.close()

    logging.info("received data:", data)

    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    return status


#   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/status
@app.route("/status", methods=['GET', 'POST'])
def status():

    TCP_IP = Config.TCP_IP
    TCP_PORT = int(Config.PORT)
    BUFFER_SIZE = 1024
    data = "checking_status"
    MESSAGE = bytes(data, 'utf-8')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    s.close()

    logging.info("received data:", data)

    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    return status

#   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/status_processing


@app.route("/status_processing", methods=['GET', 'POST'])
def status_processing():

    TCP_IP = Config.TCP_IP
    TCP_PORT = int(Config.PORT)
    BUFFER_SIZE = 1024
    data = "The_Data_is_Processing"
    MESSAGE = bytes(data, 'utf-8')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    s.close()

    logging.info("received data:", data)

    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    return status

#   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/status_idle


@app.route("/status_idle", methods=['GET', 'POST'])
def status_idle():

    TCP_IP = Config.TCP_IP
    TCP_PORT = int(Config.PORT)
    BUFFER_SIZE = 1024
    data = "The_System_is_Idle"
    MESSAGE = bytes(data, 'utf-8')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    s.close()

    logging.info("received data:", data)

    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    return status

#   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/stop_bench


@app.route("/stop_bench", methods=['GET', 'POST'])
def stop_bench():

    print("IN STOP BENCH")
    TCP_IP = Config.TCP_IP
    TCP_PORT = int(Config.PORT)
    BUFFER_SIZE = 1024
    data = "stop_bench"
    MESSAGE = bytes(data, 'utf-8')
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    s.close()
    logging.info("received data:", data)
    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    return status

    #   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/data_over


@app.route("/data_over", methods=['GET', 'POST'])
def data_over():

    TCP_IP = Config.TCP_IP
    TCP_PORT = int(Config.PORT)
    BUFFER_SIZE = 1024
    data = "The_Data_is_Over"
    MESSAGE = bytes(data, 'utf-8')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    s.close()

    print("received data:", data)
    print("initializing pull.....")
    create_output_json(folder_path_in, folder_path_out,"completed_with_success")
    headers = getHeader()
    config_file_path = os.path.join(folder_path_out,"configuration.json")
    with open(config_file_path) as config_file:
        config = json.load(config_file)

    data_for_testStatus = {"bench_id": config.get("parameters").get("bench_id"), "test_point_id":      config.get("parameters").get("test_point_id")}

    url_bench_pull = "http://10.30.7.112:3000/pull-reprocessed-output-from-bench"
  #  response = requests.post(
    #url_bench_pull, headers=headers, data=json.dumps(data_for_testStatus))
    

    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    
    return status

  #   curl -H "Content-Type: application/json" -X GET -d '{"data":1234567}' localhost:8002/data_interrupted


@app.route("/data_interrupted", methods=['GET', 'POST'])
def data_interrupted():

    TCP_IP = Config.TCP_IP
    TCP_PORT = int(Config.PORT)
    BUFFER_SIZE = 1024
    data = "Data_is_Interrupted"
    MESSAGE = bytes(data, 'utf-8')

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    s.close()

    logging.info("received data:", data)
    create_output_json(folder_path_in, folder_path_out, "completed_with_error")
    result = json.dumps({"status": "available"})
    status = Response(result, status=200, mimetype='application/json')
    return status


def create_output_json(folder_path_in, folder_path_out, status_display):
    if not os.path.exists(os.path.join(Config.BUNDLE_FILE_LOC,"Reprocessed_Output")):
        os.mkdir(os.path.join(Config.BUNDLE_FILE_LOC,"Reprocessed_Output"))
    if os.path.exists(os.path.join(folder_path_out, 'configuration.json')):
        with open(os.path.join(folder_path_out, 'configuration.json')) as file:
            json_data = json.load(file)
            file.close()
    else:
        with open(os.path.join(folder_path_in, "configuration.json")) as file:
            json_data = json.load(file)
            file.close()

    with open(os.path.join(folder_path_out,"configuration.json"), 'w') as file2:
        for index, video in enumerate(json_data['video_list']):
            json_data['video_list'][index]['status'] = status_display
        file2.write(json.dumps(json_data, indent=4))


if __name__ == "__main__":
    logging.basicConfig(filename="app.log",
                        format='%(asctime)s - %(message)s', level=logging.INFO)
    app.run(host='0.0.0.0', port=8002, debug=True)
