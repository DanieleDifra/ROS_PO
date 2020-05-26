import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

#Parameters
start_delay = 0.5

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

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

       

        # joint_vel_1 = np.append(joint_vel_1, msg.velocity[0])
        # joint_vel_2 = np.append(joint_vel_2, msg.velocity[1])
        # joint_vel_3 = np.append(joint_vel_3, msg.velocity[2])
        # joint_vel_4 = np.append(joint_vel_4, msg.velocity[3])
        # joint_vel_5 = np.append(joint_vel_5, msg.velocity[4])
        # joint_vel_6 = np.append(joint_vel_6, msg.velocity[5])

bag.close()

plt.figure(1)

plt.subplot(3,2,1)
act1 = plt.plot(joint_t,joint_pos_1)
#act, = plt.plot(jointstate_t-start_delay,jointstate_pos_arm,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
#plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(3,2,2)
act2 = plt.plot(joint_t,joint_pos_2)
#act, = plt.plot(jointstate_t-start_delay,jointstate_pos_arm,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
#plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(3,2,3)
act3 = plt.plot(joint_t,joint_pos_3)
#act, = plt.plot(jointstate_t-start_delay,jointstate_pos_arm,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
#plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(3,2,4)
act4 = plt.plot(joint_t,joint_pos_4)
#act, = plt.plot(jointstate_t-start_delay,jointstate_pos_arm,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
#plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(3,2,5)
act5 = plt.plot(joint_t,joint_pos_5)
#act, = plt.plot(jointstate_t-start_delay,jointstate_pos_arm,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
#plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(3,2,6)
act6 = plt.plot(joint_t,joint_pos_6)
#act, = plt.plot(jointstate_t-start_delay,jointstate_pos_arm,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
#plt.legend(handles=[ref, act])
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()