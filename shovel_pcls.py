#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import math
import random
import sys


def gen_pcl(num_points, scale, spin):
    points = []
    for i in range(num_points):
        #progress = i / num_points
        #val = progress * scale
        #sv = math.sin(val + spin)
        #cv = math.cos(val + spin)
        x = (random.random() - 0.5)
        y = (random.random() - 0.5)
        z = random.random()
        points.append([x, y, z, 8.0, 1.0])
    return point_cloud2.create_cloud(
        header=Header(
            stamp=rospy.Time.now(),
            frame_id="map",
        ),
        fields=[
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 16, PointField.FLOAT32, 1),
            PointField("ring", 20, PointField.FLOAT32, 1),
        ],
        points=points,
    )

    """
    row_step = num_points * 32
    return PointCloud2(
        header=Header(
            stamp=rospy.Time.now(),
            frame_id="map",
        ),
        height=1,
        width=num_points,
        fields=[
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 16, PointField.FLOAT32, 1),
            PointField("ring", 20, PointField.FLOAT32, 1),
        ],
        is_bigendian=False,
        point_step=32,
        row_step=row_step,
        data=[random.randint(0,255) for i in range(row_step)],
        is_dense=True,
    )
    """

spin = 0

def main():
    rospy.init_node('shove_pcls')

    width = int(sys.argv[1])
    period = float(sys.argv[2])

    pcl = gen_pcl(width, 0, 0)
    data_width = len(pcl.data)
    print('data width:', data_width, 'bytes')
    rate = data_width / period
    print('data rate :', rate, 'bytes/s')

    pcl_pub = rospy.Publisher('/bench/pcl', PointCloud2, queue_size=10)

    def animate(tev):
        global spin
        if tev.last_duration is not None:
            spin += tev.last_duration
        pcl = gen_pcl(width, 1.0, spin)
        pcl_pub.publish(pcl)

    rospy.Timer(rospy.Duration.from_sec(period), animate)

    rospy.spin()


if __name__ == '__main__':
    main()
