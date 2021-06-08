import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import matplotlib.pyplot as plt
import matplotlib as mpl  # used for image plotting

# --------Initialization of Variables-------#

t = 0
PI = math.pi  # pi=3.14..., constant
sensor_handles = np.array([], dtype='i')
sensor_values = np.array([0.0]*16)
sensors_position = np.zeros([16, 3])
sensors_position_vector = np.zeros([16, 2])
detectedpoint_vector = np.zeros([16, 3])
all_euler_angles = list()
angles_offset = h = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.pi, math.pi, math.pi, math.pi, math.pi, math.pi, math.pi, math.pi])

# --------------------Initial Code To Connect with V-REP--------------------#

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

#-------------------get the handles of Ultrasonic sensors and motors-----------------#

errorCode, object_handle = vrep.simxGetObjectHandle(clientID, 'Cuboid4', vrep.simx_opmode_blocking)
# Robot Handle and its position initialization function
errorCode, robot_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx' + str(),
                                                       vrep.simx_opmode_oneshot_wait)
# print('Robot Handler', robot_handle)
returnCode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_streaming)
robot_position.pop()
returnCode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_oneshot_wait)
print('Robot position', robot_position)
#robot_position.pop()


for x in range(1, 16 + 1): # Sensor Handles Access
    errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x),
                                                        vrep.simx_opmode_oneshot_wait) # Retrieving Sensor handles

    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)  # First call for Read ultrasonic Sensor

    returnCode, eulerAngles = vrep.simxGetObjectOrientation(clientID, sensor_handle, robot_handle,
                                                            vrep.simx_opmode_streaming) # First call for Read Euler Angles of Sensor

    returnCode, sensor_position = vrep.simxGetObjectPosition(clientID, sensor_handle, vrep.sim_handle_parent,
                                                             vrep.simx_opmode_oneshot_wait) # Retrieving Sensor position w.r.t Robot
    #print("sensor {} position is {} ".format(x, sensor_position))
    sensors_position[x - 1] = sensor_position
    sensor_handles = np.append(sensor_handles, sensor_handle)
sensors_position_vector = sensors_position + robot_position # Sensor position w.r.t world frame of reference

#print("sensors_position_vector", sensors_position_vector)
# Find motors handles
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0',
                                                        vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0',
                                                         vrep.simx_opmode_oneshot_wait)

for j in range(1, 16 + 1): # For calculating Euler angles for each sensor
    returnCode, eulerAngles = vrep.simxGetObjectOrientation(clientID, sensor_handles[j - 1], robot_handle,
                                                            vrep.simx_opmode_buffer)  # Euler Angle for each sensor is calculated
    all_euler_angles.append(eulerAngles)


def rotational_matrix(eulerangles):
    psi = eulerangles[0]
    theta = eulerangles[1]
    phi = eulerangles[2]
    r_x = np.array([[1, 0, 0], [0, math.cos(psi), -math.sin(psi)], [0, math.sin(psi), math.cos(psi)]])
    r_y = np.array([[math.cos(theta), 0, math.sin(theta)], [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]])
    r_z = np.array([[math.cos(phi), -math.sin(phi), 0], [math.sin(phi), math.cos(phi), 0], [0, 0, 1]])
    mult1 = np.dot(r_x, r_y)
    r_t = np.dot(mult1, r_z)
    return r_t


def angles_calcu(sensor_x, sensor_y, robot_x, robot_y):
    grad_x = sensor_x - robot_x
    grad_y = sensor_y - robot_y
    grad = grad_y / grad_x
    angles = np.arctan(grad) * 180/math.pi
    return angles

#---------------------Main Code for object Avoidance---------------------#


while t < 5:
    for s in range(1, 16 + 1):
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_handles[s - 1], vrep.simx_opmode_oneshot_wait)  # Measure distance using Ultrasonic Sensor

        if np.max(detectedPoint) > 1:  # To overcome infinite distance problem
            detectedPoint = [0.0] * 3
        dist = np.linalg.norm(detectedPoint)
        #print('Euler Angles of sensor {} is {}'.format(s, all_euler_angles[s-1]))
        #print('Return Code', returnCode)
        #print('Sensor number {} detect point is {} and distance calculated is {} '.format(s, detectedPoint, dist))
        rotational_mat = rotational_matrix(all_euler_angles[s-1])  # rotation matrix R is Calculated
        detectedpoint_vector[s - 1] = robot_position + np.dot(rotational_mat, detectedPoint) + sensors_position[s-1]  # Detected point w.r.t world frame of reference
        if dist < 0.001 or dist > 1:  # To overcome out of bound values for Sensors
            dist = float()
        sensor_values[s-1] = dist
    sensor_values = np.round(sensor_values, 2)
    sensor_angles = angles_calcu(detectedpoint_vector[:, 0], detectedpoint_vector[:, 1], robot_position[0], robot_position[1]) + angles_offset
    force_co_efficient = - (1 - sensor_values)
    fx = force_co_efficient * np.cos(sensor_angles)
    fy = force_co_efficient * np.sin(sensor_angles)
    print('force_x', fx)
    print('force_x', fy)
    print('Sensor Values', sensor_values)
    print('Sensors Angles', sensor_angles)
    if t < 9:
        #print('Turn Left')
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
        time.sleep(0.5)
    else:
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
    t += 1

for i in range(0, len(sensors_position)):
    #plt.plot([robot_position[0], sensors_position_vector[i][0]],[robot_position[1], sensors_position_vector[i][1]], 'ro-')
    #plt.plot([sensors_position_vector[i][0], detectedpoint_vector[i][0]], [sensors_position_vector[i][1], detectedpoint_vector[i][1]], 'bo-')
    plt.plot([robot_position[0], detectedpoint_vector[i][0]], [robot_position[1], detectedpoint_vector[i][1]], 'bo-')
plt.show()
#print(all_euler_angles)
