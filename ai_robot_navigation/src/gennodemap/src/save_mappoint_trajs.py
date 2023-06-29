#!/usr/bin/env python3

import os
import sys, time
import cv2
import numpy as np
import pylab as plt
import rospy
import tf

from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid


quittraj = False
quitmap = False

def trajcb(data):
    #print(data)
    global quittraj
    if(not quittraj):
        outname = '../Input/trajectory.txt'
        rospy.loginfo('start to save trajectory...')
        f = open(outname,'w')
        for one in data.markers:
            for ps in one.points:
                f.write(str(10000)+' '+str(ps.x)+' '+str(ps.y)+' '+str(ps.z)+' '+'0 0 0 1\n')
        rospy.loginfo('save trajectory done! save to file '+outname)
        quittraj = True
        f.close()

def mapcb(data):
    global quitmap
    if not quitmap:
        outname = '../Input/MapPointsPos.txt'
        rospy.loginfo('start to save map points...')
        mapimg = np.array(data.data)
        mapimg = mapimg + 1
        mapimg = np.where(mapimg > 60, 100, 0)
        #np.where(mapimg <=50, mapimg, 0)
        mapimg.shape = data.info.height, data.info.width
        q=[data.info.origin.orientation.x, data.info.origin.orientation.y, data.info.origin.orientation.z, data.info.origin.orientation.w]
        T=tf.transformations.quaternion_matrix(q)
        T[:3,3]=np.array([data.info.origin.position.x, data.info.origin.position.y, data.info.origin.position.z])
        mappointfile = open(outname,'w')
        flag_end = -2
        for x in range(data.info.width):
            for y in range(data.info.height):
                if(mapimg[y, x] > 50):#add point cloud
                    pimg = np.array([x*data.info.resolution, y*data.info.resolution, 0, 1])
                    pworld = np.dot(T, pimg)
                    mappointfile.write(str(pworld[0])+' '+str(pworld[1])+' '+str(pworld[2])+'\n')
        flag_end = 0
        rospy.loginfo('save map point done! save to file '+outname)
        mappointfile.close()
        quitmap = True

if __name__ == '__main__':
    rospy.init_node('trajlisten', anonymous=True)
    rospy.Subscriber("/trajectory_node_list", MarkerArray, trajcb)
    rospy.Subscriber("/map", OccupancyGrid, mapcb)

    while not rospy.is_shutdown():
        time.sleep(1)
        if(quitmap and quittraj):
            rospy.loginfo('exit')
            os._exit(0)