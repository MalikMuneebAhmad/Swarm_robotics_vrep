import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import matplotlib.pyplot as plt
import random
import matplotlib as mpl  # used for image plotting

# --------Initialization of Variables-------#

maxDetectionDist = 0.3
theta = 0.0
t_turn = 0.0
t = 0
vs = 1
PI = math.pi  # pi=3.14..., constant
sensor_handles = np.array([], dtype='i')
sensor_values = np.array([0.0] * 16)
exact_dist = np.array([0.0] * 16)
sensor_states = [False] * 16
sensors_position = np.zeros([16, 3])
sensors_position_vector = np.zeros([16, 2])
force_co_efficient= np.array([0.0] * 16)
detectedpoint_vector = np.zeros([16, 3])
all_euler_angles = list()
angles_offset = h = np.array(
    [0.0, 0.0, 0.0, 0.0, math.pi, math.pi, math.pi, math.pi, 0.0, 0.0, 0.0, 0.0, math.pi, math.pi, math.pi, math.pi])
plt.xlim([-10, 10])
plt.ylim([-10, 10])
# --------------------Initial Code To Connect with V-REP--------------------#

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    vrep.simxSynchronous(clientID, True)
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit()

# -------------------get the handles of Ultrasonic sensors and motors-----------------#

errorCode, object_handle = vrep.simxGetObjectHandle(clientID, 'Cuboid4', vrep.simx_opmode_blocking)
# Robot Handle and its position initialization function
errorCode, robot_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx' + str(2),
                                                   vrep.simx_opmode_oneshot_wait)
# print('Robot Handler', robot_handle)
returnCode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_streaming)
# robot_position.pop()
returnCode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_oneshot_wait)
print('Robot position', robot_position)
# robot_position.pop()


for x in range(1, 16 + 1):  # Sensor Handles Access
    errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x),
                                                        vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor handles

    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)  # First call for Read ultrasonic Sensor

    returnCode, sensor_position = vrep.simxGetObjectPosition(clientID, sensor_handle, vrep.sim_handle_parent,
                                                             vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor position w.r.t Robot
    # print("sensor {} position is {} ".format(x, sensor_position))
    sensors_position[x - 1] = sensor_position
    sensor_handles = np.append(sensor_handles, sensor_handle)
    #robot_angle = math.atan2(sensor_position[1], sensor_position[0])
    #robot_pos_length = math.sqrt(sensor_position[1]**2 + sensor_position[0]**2)

#sensors_position_vector = sensors_position + robot_position  # Sensor position w.r.t world frame of reference

# print("sensors_position_vector", sensors_position_vector)
# Find motors handles
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0',
                                                        vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0',
                                                         vrep.simx_opmode_oneshot_wait)


def angles_calcu(sensor_x, sensor_y, robot_x, robot_y):
    grad_x = (sensor_x - robot_x)
    grad_y = (sensor_y - robot_y)
    angles = np.arctan2(grad_y, grad_x)
    return angles


# ---------------------Main Code for object Avoidance---------------------#


while t < 120:
    for s in range(1, 16 + 1):
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_handles[s - 1], vrep.simx_opmode_oneshot_wait)  # Measure distance using Ultrasonic Sensor

        sensor_states[s-1] = detectionState
        if not sensor_states[s-1]:  # To overcome infinite distance problem
            detectedPoint = [0.0] * 3
        dist = np.linalg.norm(detectedPoint)
        if dist > maxDetectionDist:
            sensor_values[s - 1] = maxDetectionDist
        elif dist <= maxDetectionDist:
            sensor_values[s - 1] = dist
        exact_dist[s-1] = dist
        detectedpoint_vector[s - 1] = detectedPoint
    print('detectpoint_vector', detectedpoint_vector)
    sensor_values = np.round(sensor_values, 2)
    sensor_angles = angles_calcu(sensors_position[:, 0], sensors_position[:, 1], 0.0, 0.0)
    print('Sensor Angles', sensor_angles * 180/math.pi)
    force_co_efficient = -(maxDetectionDist - sensor_values)
    fx = force_co_efficient * np.cos(sensor_angles)
    fy = force_co_efficient * np.sin(sensor_angles)
    fres_x = np.sum(fx)
    fres_y = np.sum(fy)
    if fres_x == 0.0 or fres_y == 0.0:
        theta = 0.1
    else:
        theta = math.atan2(fres_y, fres_x)
    print('Theta', theta)
    l = math.sqrt((fres_y) ** 2 + (fres_x) ** 2)
    if l == 0:
        k1 = 1
    else:
        #k1 = max((0.1, math.pi / 2 - abs(theta)))
        k1 = max((0.1, abs(theta) - math.pi / 2))
    w = theta
    v = k1 * vs
    print('force_x', fres_x)
    print('force_y', fres_y)
    print('force_co-efficient', force_co_efficient)
    #print('force_res_y', fres_y)
    print('Sensor Values', sensor_values)
    #print('Sensors Angles', sensor_angles * 180 / math.pi)

    if theta != 0.0 and (0.7 >= exact_dist[3] > 0.01 or 0.7 >= exact_dist[4] > 0.01):
        print('Exact_dist', exact_dist)
        print('Turn Left')
        #t_turn = min(0.1, abs(theta)/vs)
        v_left = -v if theta < 0 else v
        v_right = -v_left
        if v == 0:
            t_turn = 0.1
        elif t_turn < 0.005:
            t_turn = random.random()
            print('t_turn random', t_turn)
        else:
            t_turn = (1.5 * abs(theta))/(2 * v)
        print('t_turn', t_turn)
        print('v_left', v_left)
        print('v_right', v_right)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, v_left, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, v_right, vrep.simx_opmode_streaming)
        time.sleep(t_turn)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.0, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.0, vrep.simx_opmode_streaming)
        time.sleep(1)
    else:
        print('Straight movement')
        errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.8, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.8, vrep.simx_opmode_streaming)
        #time.sleep(0.1)
    t += 1

'''for i in range(0, len(sensors_position)):
    # plt.plot([robot_position[0], sensors_position_vector[i][0]],[robot_position[1], sensors_position_vector[i][1]], 'ro-')
    # plt.plot([sensors_position_vector[i][0], detectedpoint_vector[i][0]], [sensors_position_vector[i][1], detectedpoint_vector[i][1]], 'bo-')
    #plt.plot([robot_position[0], sensors_position_vector[i][0]], [robot_position[1], sensors_position_vector[i][1]],
             'bo-')
#plt.plot([robot_position[0], fres_x], [robot_position[1], fres_y], 'ro-')
#plt.show()
# print(all_euler_angles)'''
