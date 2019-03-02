#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull
from hybrid_astar.msg import polygonArray as obs
from hybrid_astar.msg import polygon as poly


from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import PointStamped

viz = True
range_threshold = 100
rospy.init_node('converter', anonymous=True)
pub = rospy.Publisher("/obstacles", obs, queue_size=10)


def draw_polygon(img, polygon):
    polygon_to_draw = np.zeros((polygon.shape))
    polygon_to_draw[:, 0] = (300 - 10 * polygon[:, 1])
    polygon_to_draw[:, 1] = (300 - 10 * polygon[:, 0])
    polygon_to_draw = polygon_to_draw.reshape((-1, 1, 2)).astype(np.int32)
    cv2.polylines(img, [polygon_to_draw], True, (255, 0, 0))


def callback(data):
    print "Inside Callback"
    points = np.empty((np.sum(np.asarray(data.ranges) < range_threshold), 2), np.float64)

    if viz:
        img = 255 * np.ones((600, 600, 3), np.uint8)

    j = -1
    for i, r in enumerate(data.ranges):
        if r >= range_threshold:
            continue
        else:
            j += 1

        theta = data.angle_min + i * data.angle_increment
        points[j][0] = r * math.cos(theta)
        points[j][1] = r * math.sin(theta)

        if viz:
            cv2.circle(img, (int(300 - 10 * points[j][1]), int(300 - 10 * points[j][0])), 2, (128, 128, 128), -1)

    if points.size == 0:
        obstacles = obs()
        pub.publish(obstacles)
        return

    clustering = DBSCAN(eps=1, min_samples=1).fit(points)
    n_clusters = max(clustering.labels_) + 1

    obstacles = obs()

    for cluster_idx in range(n_clusters + 1):
        cluster_points = points[clustering.labels_ == cluster_idx, :]

        if cluster_points.shape[0] < 3:
            continue

        hull = ConvexHull(cluster_points)
        hull_points = cluster_points[hull.vertices, :]

        if viz:
            draw_polygon(img, hull_points)

        obsIndividual = poly()

        for x in range(hull_points.shape[0]):
            somePoint = PointStamped()
            somePoint.header.frame_id = "base_link"
            somePoint.point.x = hull_points[x, 0]
            somePoint.point.y = hull_points[x, 1]

            obsIndividual.polygon.append(somePoint)

        obstacles.obstacles.append(obsIndividual)

    pub.publish(obstacles)

    if viz:
        cv2.imshow("obstacle map", img)
        cv2.waitKey(10)


def converter():
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    converter()
