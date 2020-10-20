import requests
loop = 1
while loop == 1:
    option = input(
        "Press 1: replay-ping-test\nPress 2: ping-replay-test\nPress 3: start_bench\nPress 4: status\nPress 5: stop_bench\n ")
    flag = int(option)

    if flag == 1:
        headers = {
            'Content-type': 'application/json',
        }
        data = '{"data":1234567}'
        response = requests.post(
            'http://localhost:8002/replay-ping-test', headers=headers, data=data)

    elif flag == 2:
        headers = {
            'Content-type': 'application/json',
        }
        data = '{"data":1234567}'
        response = requests.post(
            'http://localhost:8002/ping-replay-test', headers=headers, data=data)

    elif flag == 3:
        headers = {
            'Content-type': 'application/json',
        }
        data = '{"data":1234567}'
        response = requests.post(
            'http://localhost:8002/start_bench', headers=headers, data=data)

    elif flag == 4:
        headers = {
            'Content-type': 'application/json',
        }
        data = '{"data":1234567}'
        response = requests.post(
            'http://localhost:8002/status', headers=headers, data=data)

    elif flag == 5:
        headers = {
            'Content-type': 'application/json',
        }
        data = '{"data":1234567}'
        response = requests.post(
            'http://localhost:8002/stop_bench', headers=headers, data=data)

    else:
        print("\nPlease Provide a Proper Input\n")
