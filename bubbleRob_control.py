import sim as vrep # access all the VREP elements
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    exit("Could not connect")