#ROS_PO 

import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

#Variables
start_delay = 0

joint_t = np.array([])

joint_pos_1 = np.array([])
joint_pos_2 = np.array([])
joint_pos_3 = np.array([])
joint_pos_4 = np.array([])
joint_pos_5 = np.array([])
joint_pos_6 = np.array([])

joint_vel_1 = np.array([])
joint_vel_2 = np.array([])
joint_vel_3 = np.array([])
joint_vel_4 = np.array([])
joint_vel_5 = np.array([])
joint_vel_6 = np.array([])


plan_t = np.array([])

plan_pos_1 = np.array([])
plan_pos_2 = np.array([])
plan_pos_3 = np.array([])
plan_pos_4 = np.array([])
plan_pos_5 = np.array([])
plan_pos_6 = np.array([])

plan_vel_1 = np.array([])
plan_vel_2 = np.array([])
plan_vel_3 = np.array([])
plan_vel_4 = np.array([])
plan_vel_5 = np.array([])
plan_vel_6 = np.array([])



for topic, msg, t in bag.read_messages():

    if topic == "edo/joint_states":

        joint_t = np.append(joint_t, float(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0e-9))

        joint_pos_1 = np.append(joint_pos_1, msg.position[0])
        if abs(joint_pos_1[len(joint_pos_1)-1] - joint_pos_1[len(joint_pos_1)-2]) > 0.015:
        	joint_pos_1[len(joint_pos_1)-1] = joint_pos_1[len(joint_pos_1)-2]

        joint_pos_2 = np.append(joint_pos_2, msg.position[1])
        if abs(joint_pos_2[len(joint_pos_2)-1] - joint_pos_2[len(joint_pos_2)-2]) > 0.015:
        	joint_pos_2[len(joint_pos_2)-1] = joint_pos_2[len(joint_pos_2)-2]

        joint_pos_3 = np.append(joint_pos_3, msg.position[2])
        if abs(joint_pos_3[len(joint_pos_3)-1] - joint_pos_3[len(joint_pos_3)-2]) > 0.015:
        	joint_pos_3[len(joint_pos_3)-1] = joint_pos_3[len(joint_pos_3)-2]

        joint_pos_4 = np.append(joint_pos_4, msg.position[3])
        if abs(joint_pos_4[len(joint_pos_4)-1] - joint_pos_4[len(joint_pos_4)-2]) > 0.015:
        	joint_pos_4[len(joint_pos_4)-1] = joint_pos_4[len(joint_pos_4)-2]

        joint_pos_5 = np.append(joint_pos_5, msg.position[4])
        if abs(joint_pos_5[len(joint_pos_5)-1] - joint_pos_5[len(joint_pos_5)-2]) > 0.015:
        	joint_pos_5[len(joint_pos_5)-1] = joint_pos_5[len(joint_pos_5)-2]

        joint_pos_6 = np.append(joint_pos_6, msg.position[5])
        if abs(joint_pos_6[len(joint_pos_6)-1] - joint_pos_6[len(joint_pos_6)-2]) > 0.015:
        	joint_pos_6[len(joint_pos_6)-1] = joint_pos_6[len(joint_pos_6)-2]


        if msg.velocity:

            joint_vel_1 = np.append(joint_vel_1, msg.velocity[0])
            if abs(joint_vel_1[len(joint_vel_1)-1] - joint_vel_1[len(joint_vel_1)-2]) > 0.01:
                joint_vel_1[len(joint_vel_1)-1] = joint_vel_1[len(joint_vel_1)-2]

            joint_vel_2 = np.append(joint_vel_2, msg.velocity[1])
            if abs(joint_vel_2[len(joint_vel_2)-1] - joint_vel_2[len(joint_vel_2)-2]) > 0.01:
                joint_vel_2[len(joint_vel_2)-1] = joint_vel_2[len(joint_vel_2)-2]

            joint_vel_3 = np.append(joint_vel_3, msg.velocity[2])
            if abs(joint_vel_3[len(joint_vel_3)-1] - joint_vel_3[len(joint_vel_3)-2]) > 0.01:
                joint_vel_3[len(joint_vel_3)-1] = joint_vel_3[len(joint_vel_3)-2]

            joint_vel_4 = np.append(joint_vel_4, msg.velocity[3])
            if abs(joint_vel_4[len(joint_vel_4)-1] - joint_vel_4[len(joint_vel_4)-2]) > 0.01:
                joint_vel_4[len(joint_vel_4)-1] = joint_vel_4[len(joint_vel_4)-2]

            joint_vel_5 = np.append(joint_vel_5, msg.velocity[4])
            if abs(joint_vel_5[len(joint_vel_5)-1] - joint_vel_5[len(joint_vel_5)-2]) > 0.01:
                joint_vel_5[len(joint_vel_5)-1] = joint_vel_5[len(joint_vel_5)-2]

            joint_vel_6 = np.append(joint_vel_6, msg.velocity[5])
            if abs(joint_vel_6[len(joint_vel_6)-1] - joint_vel_6[len(joint_vel_6)-2]) > 0.02:
                joint_vel_6[len(joint_vel_6)-1] = joint_vel_6[len(joint_vel_6)-2]
        else: 
            joint_vel_1 = np.append(joint_vel_1, joint_vel_1[len(joint_vel_1)-1])
            joint_vel_2 = np.append(joint_vel_2, joint_vel_2[len(joint_vel_2)-1])
            joint_vel_3 = np.append(joint_vel_3, joint_vel_3[len(joint_vel_3)-1])
            joint_vel_4 = np.append(joint_vel_4, joint_vel_4[len(joint_vel_4)-1])
            joint_vel_5 = np.append(joint_vel_5, joint_vel_5[len(joint_vel_5)-1])
            joint_vel_6 = np.append(joint_vel_6, joint_vel_6[len(joint_vel_6)-1])

    if topic == "joint_trajectory":

        plan_t = np.append(plan_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))

        plan_pos_1 = np.append(plan_pos_1, msg.positions[0])
        plan_pos_2 = np.append(plan_pos_2, msg.positions[1])
        plan_pos_3 = np.append(plan_pos_3, msg.positions[2])
        plan_pos_4 = np.append(plan_pos_4, msg.positions[3])
        plan_pos_5 = np.append(plan_pos_5, msg.positions[4])
        plan_pos_6 = np.append(plan_pos_6, msg.positions[5])

        plan_vel_1 = np.append(plan_vel_1, msg.velocities[0])
        plan_vel_2 = np.append(plan_vel_2, msg.velocities[1])
        plan_vel_3 = np.append(plan_vel_3, msg.velocities[2])
        plan_vel_4 = np.append(plan_vel_4, msg.velocities[3])
        plan_vel_5 = np.append(plan_vel_5, msg.velocities[4])
        plan_vel_6 = np.append(plan_vel_6, msg.velocities[5])

bag.close()

## PLOTS ##

#Plot posizioni
plt.figure(1)

plt.subplot(3,2,1)
act11, = plt.plot(joint_t, joint_pos_1, label='actual')
ref11, = plt.plot(plan_t-start_delay, plan_pos_1, label='reference')
plt.ylabel('Position [rad]')
plt.title('Joint 1')
plt.legend(handles=[ref11, act11])
plt.grid(True)

plt.subplot(3,2,2)
act12, = plt.plot(joint_t,joint_pos_2, label='actual')
ref12, = plt.plot(plan_t-start_delay, plan_pos_2, label='reference')
plt.ylabel('Position [rad]')
plt.title('Joint 2')
plt.legend(handles=[ref12, act12])
plt.grid(True)

plt.subplot(3,2,3)
act13, = plt.plot(joint_t,joint_pos_3, label='actual')
ref13, = plt.plot(plan_t-start_delay, plan_pos_3, label='reference')
plt.ylabel('Position [rad]')
plt.title('Joint 3')
plt.legend(handles=[ref13, act13])
plt.grid(True)

plt.subplot(3,2,4)
act14, = plt.plot(joint_t,joint_pos_4, label='actual')
ref14, = plt.plot(plan_t-start_delay, plan_pos_4, label='reference')
plt.ylabel('Position [rad]')
plt.title('Joint 4')
plt.legend(handles=[ref14, act14])
plt.grid(True)

plt.subplot(3,2,5)
act15, = plt.plot(joint_t,joint_pos_5, label='actual')
ref15, = plt.plot(plan_t-start_delay, plan_pos_5, label='reference')
plt.ylabel('Position [rad]')
plt.title('Joint 5')
plt.legend(handles=[ref15, act15])
plt.grid(True)

plt.subplot(3,2,6)
act16, = plt.plot(joint_t,joint_pos_6, label='actual')
ref16, = plt.plot(plan_t-start_delay, plan_pos_6, label='reference')
plt.ylabel('Position [rad]')
plt.title('Joint 6')
plt.legend(handles=[ref16, act16])
plt.grid(True)

#plt.savefig("controller_fig1.svg")


#Plot velocities
plt.figure(2)

plt.subplot(3,2,1)
act21, = plt.plot(joint_t,joint_vel_1, label='actual')
ref21, = plt.plot(plan_t-start_delay, plan_vel_1, label='reference')
plt.ylabel('Velocity [rad/s]')
plt.title('Joint 1')
plt.legend(handles=[ref21, act21])
plt.grid(True)

plt.subplot(3,2,2)
act22, = plt.plot(joint_t,joint_vel_2, label='actual')
ref22, = plt.plot(plan_t-start_delay, plan_vel_2, label='reference')
plt.ylabel('Velocity [rad/s]')
plt.title('Joint 2')
plt.legend(handles=[ref22, act22])
plt.grid(True)

plt.subplot(3,2,3)
act23, = plt.plot(joint_t,joint_vel_3, label='actual')
ref23, = plt.plot(plan_t-start_delay, plan_vel_3, label='reference')
plt.ylabel('Velocity [rad/s]')
plt.title('Joint 3')
plt.legend(handles=[ref23, act23])
plt.grid(True)

plt.subplot(3,2,4)
act24, = plt.plot(joint_t,joint_vel_4, label='actual')
ref24, = plt.plot(plan_t-start_delay, plan_vel_4, label='reference')
plt.ylabel('Velocity [rad/s]')
plt.title('Joint 4')
plt.legend(handles=[ref24, act24])
plt.grid(True)

plt.subplot(3,2,5)
act25, = plt.plot(joint_t,joint_vel_5, label='actual')
ref25, = plt.plot(plan_t-start_delay, plan_vel_5, label='reference')
plt.ylabel('Velocity [rad/s')
plt.title('Joint 5')
plt.legend(handles=[ref25, act25])
plt.grid(True)

plt.subplot(3,2,6)
act26, = plt.plot(joint_t,joint_vel_6, label='actual')
ref26, = plt.plot(plan_t-start_delay, plan_vel_6, label='reference')
plt.ylabel('Velocity [rad/s]')
plt.title('Joint 6')
plt.legend(handles=[ref26, act26])
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()