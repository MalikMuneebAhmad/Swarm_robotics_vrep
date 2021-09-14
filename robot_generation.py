from robot_epuck import Robot
import sim as vrep
import sys
import pickle


clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')
robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num))
          for num in range(6)]
with open('robot_file', 'wb') as f:  # to generate your own file  (wb for wrting in bytes)
    pickle.dump(robots, f)  # storing variable into file

print('Done')