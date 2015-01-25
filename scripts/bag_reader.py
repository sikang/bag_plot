#!/usr/bin/python

import rosbag
import rospy
import yaml
import numpy as np

#from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

topic_type_dict = {}
msg_types = ['nav_msgs/Odometry', 'sensor_msgs/Imu', 'geometry_msgs/PoseStamped']
var_types = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'acc_x', 'acc_y', 'acc_z',
        'ang_vel_x', 'ang_vel_y', 'ang_vel_z']

def read_bag(bagfile):
    global inbag
    inbag = rosbag.Bag(bagfile, 'r')
    return read_topic_type()

def read_topic_type():
    info_dict = yaml.load(inbag._get_yaml_info())
    for x in info_dict['topics']:
         topic_type_dict[x['topic']] = x['type']
    return topic_type_dict

def read_msg(topics):
    data = {}
    if len(topics) > 0:
        for topic, msg, type in inbag.read_messages():
         if topics.count(topic):
            if topic_type_dict[topic] == 'nav_msgs/Odometry':
                data = update_odometry(data, topic, msg)
            elif topic_type_dict[topic] == 'sensor_msgs/Imu':
                data = update_imu(data, topic, msg)
            elif topic_type_dict[topic] == 'geometry_msgs/PoseStamped':
                data = update_pose(data, topic, msg)


    return data

def update_odometry(data, topic, msg):
       quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
       [r, p, y] = euler_from_quaternion(quat)
            
       if data.has_key(topic):
           data[topic]['x'] = np.append(data[topic]['x'], msg.pose.pose.position.x)
           data[topic]['y'] = np.append(data[topic]['y'], msg.pose.pose.position.y)
           data[topic]['z'] = np.append(data[topic]['z'], msg.pose.pose.position.z)
           data[topic]['vx'] = np.append(data[topic]['vx'], msg.twist.twist.linear.x)
           data[topic]['vy'] = np.append(data[topic]['vy'], msg.twist.twist.linear.y)
           data[topic]['vz'] = np.append(data[topic]['vz'], msg.twist.twist.linear.z)
           data[topic]['roll'] = np.append(data[topic]['roll'], r)
           data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
           data[topic]['ang_vel_x'] = np.append(data[topic]['ang_vel_x'], msg.twist.twist.angular.x)
           data[topic]['ang_vel_y'] = np.append(data[topic]['ang_vel_y'], msg.twist.twist.angular.y)
           data[topic]['ang_vel_z'] = np.append(data[topic]['ang_vel_z'], msg.twist.twist.angular.z)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['x'] = np.array([msg.pose.pose.position.x])
           data[topic]['y'] = np.array([msg.pose.pose.position.y])
           data[topic]['z'] = np.array([msg.pose.pose.position.z])
           data[topic]['vx'] = np.array([msg.twist.twist.linear.x])
           data[topic]['vy'] = np.array([msg.twist.twist.linear.y])
           data[topic]['vz'] = np.array([msg.twist.twist.linear.z])
           data[topic]['ang_vel_x'] = np.array([msg.twist.twist.angular.x])
           data[topic]['ang_vel_y'] = np.array([msg.twist.twist.angular.y])
           data[topic]['ang_vel_z'] = np.array([msg.twist.twist.angular.z])
           data[topic]['roll'] = np.array([r])
           data[topic]['pitch'] = np.array([p])
           data[topic]['yaw'] = np.array([y])
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
       return data 

def update_pose(data, topic, msg):
       quat = [msg.pose.orientation.x, msg.pose.orientation.y,
               msg.pose.orientation.z, msg.pose.orientation.w]
       [r, p, y] = euler_from_quaternion(quat)
            
       if data.has_key(topic):
           data[topic]['x'] = np.append(data[topic]['x'], msg.pose.position.x)
           data[topic]['y'] = np.append(data[topic]['y'], msg.pose.position.y)
           data[topic]['z'] = np.append(data[topic]['z'], msg.pose.position.z)
           data[topic]['roll'] = np.append(data[topic]['roll'], r)
           data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['x'] = np.array([msg.pose.position.x])
           data[topic]['y'] = np.array([msg.pose.position.y])
           data[topic]['z'] = np.array([msg.pose.position.z])
           data[topic]['roll'] = np.array([r])
           data[topic]['pitch'] = np.array([p])
           data[topic]['yaw'] = np.array([y])
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
       return data 



def update_imu(data, topic, msg):
       quat = [msg.orientation.x, msg.orientation.y,
               msg.orientation.z, msg.orientation.w]
       [r, p, y] = euler_from_quaternion(quat)
  
       if data.has_key(topic):
           data[topic]['acc_x'] = np.append(data[topic]['acc_x'], msg.linear_acceleration.x)
           data[topic]['acc_y'] = np.append(data[topic]['acc_y'], msg.linear_acceleration.y)
           data[topic]['acc_z'] = np.append(data[topic]['acc_z'], msg.linear_acceleration.z)
           data[topic]['ang_vel_x'] = np.append(data[topic]['ang_vel_x'], msg.angular_velocity.x)
           data[topic]['ang_vel_y'] = np.append(data[topic]['ang_vel_y'], msg.angular_velocity.y)
           data[topic]['ang_vel_z'] = np.append(data[topic]['ang_vel_z'], msg.angular_velocity.z)
           data[topic]['roll'] = np.append(data[topic]['roll'], r)
           data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['acc_x'] = np.array([msg.linear_acceleration.x])
           data[topic]['acc_y'] = np.array([msg.linear_acceleration.y])
           data[topic]['acc_z'] = np.array([msg.linear_acceleration.z])
           data[topic]['ang_vel_x'] = np.array([msg.angular_velocity.x])
           data[topic]['ang_vel_y'] = np.array([msg.angular_velocity.y])
           data[topic]['ang_vel_z'] = np.array([msg.angular_velocity.z])
           data[topic]['roll'] = np.array([r])
           data[topic]['pitch'] = np.array([p])
           data[topic]['yaw'] = np.array([y])
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
       return data 

if __name__ == "__main__":
    read_topic_type()
