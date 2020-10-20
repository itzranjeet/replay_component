import uuid

def getHeader(): 
        headers = {
            #'Authorization': 'Bearer {0}'.format(access_token),
            'Accept': 'application/json',
            'client-request-id': str(uuid.uuid4()),
            'return-client-request-id': 'true',
            'Content-Type': 'application/json'
           }
        return headers; 
