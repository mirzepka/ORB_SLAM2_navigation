#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import numpy
import math

class navigation:
    debug = True
    MARKER_SIZE = 0.167 #0.075
    scale = 1.0
    points = [[], [], []]
    points_raw = [[], [], []]
    marker = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    marker_raw = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    marker_center = [0.0, 0.0, 0.0]
    marker_end = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    marker_end_raw = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    marker_center_end = [0.0, 0.0, 0.0]
    vect = [0.0, 0.0, 0.0]
    rotX = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    rotY = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    rotZ = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    rotation_m = [rotX, rotY, rotZ]
    angles = [0, 0, 0]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1,1])
    ax.set_ylim([-1,1])
    ax.set_zlim([-1,1])
    l, = plt.plot(points[0], points[1], points[2])
    counter = 0
    isMarkerDetected = False
    isMarkerEndDetected = False
    isMarkerConfigured = False
    isMarkerEndConfigured = False

    def checkIfMarkerIsDetected(self):
        print "not finito"
        for corner in self.marker:
            if abs(corner[0]) < 0.00001 or abs(corner[1]) < 0.00001 or abs(corner[2]) < 0.00001:
                return
        self.isMarkerDetected = True
        corners = []
        for corner in self.marker:
            corners.append(numpy.array((corner[0], corner[1], corner[2])))

        dist = []
        for corner in range(3):
            dist.append(numpy.linalg.norm(corners[corner] - corners[corner+1]))
        dist.append(numpy.linalg.norm(corners[3] - corners[0]))
        dist.sort()
        # print "Dist min/max: "+str(dist[0])+str(dist[3])
        if (dist[3]-dist[0]) < dist[0]*0.1:
            print "FINITO"
            for corner in self.marker:
                print "({},{},{})".format(corner[0], corner[1], corner[2])
            print "X" \
                  ""
            self.scale = dist[1]/self.MARKER_SIZE
            print "Scale set to: " + str(self.scale)
            self.points = [[], [], []]
            for i in range(len(self.marker)):
                self.marker_raw[i][0] = self.marker[i][0]
                self.marker_raw[i][1] = self.marker[i][1]
                self.marker_raw[i][2] = self.marker[i][2]
            xx = []
            yy = []
            zz = []
            self.reinitialize()
            for corner in self.marker:
                for idx in range(3):
                    self.marker_center[idx] += corner[idx]
            for idx in range(3):
                self.marker_center[idx] /= 4

            for i in range(len(self.marker)):
                # self.marker[i][0] /= self.scale
                # self.marker[i][1] /= self.scale
                # self.marker[i][2] /= self.scale
                xx.append(self.marker[i][0])
                yy.append(self.marker[i][1])
                zz.append(self.marker[i][2])
            verts = [list(zip(xx, yy, zz))]
            self.ax.add_collection3d(Poly3DCollection(verts, facecolors='g'))
            if self.debug:
                xx = []
                yy = []
                zz = []
                for i in range(len(self.marker_raw)):
                    xx.append(self.marker_raw[i][0])
                    yy.append(self.marker_raw[i][1])
                    zz.append(self.marker_raw[i][2])
                verts = [list(zip(xx, yy, zz))]
                self.ax.add_collection3d(Poly3DCollection(verts, facecolors='w'))
            self.isMarkerConfigured = True

    def checkIfMarkerEndIsDetected(self):
        if not self.isMarkerConfigured:
            return
        for corner in self.marker_end:
            if abs(corner[0]) < 0.00001 or abs(corner[1]) < 0.00001 or abs(corner[2]) < 0.00001:
                return
        self.isMarkerEndDetected = True
        for corner in self.marker_end:
            print "({},{},{})".format(corner[0], corner[1], corner[2])
        print "X"

        print "FINITO END"
        corners = []
        for corner in self.marker_end:
            corners.append(numpy.array((corner[0], corner[1], corner[2])))

        for i in range(len(self.marker_end)):
            self.marker_end_raw[i][0] = self.marker_end[i][0]
            self.marker_end_raw[i][1] = self.marker_end[i][1]
            self.marker_end_raw[i][2] = self.marker_end[i][2]
        xx = []
        yy = []
        zz = []
        for i in range(len(self.marker_end)):
            self.marker_end[i][0] += self.vect[0]
            self.marker_end[i][1] += self.vect[1]
            self.marker_end[i][2] += self.vect[2]
        for i in range(len(self.marker_end)):
            self.marker_end[i][0] /= self.scale
            self.marker_end[i][1] /= self.scale
            self.marker_end[i][2] /= self.scale
            xx.append(self.marker_end[i][0])
            yy.append(self.marker_end[i][1])
            zz.append(self.marker_end[i][2])

        for corner in self.marker_end:
            for idx in range(3):
                self.marker_center_end[idx] += corner[idx]
        for idx in range(3):
            self.marker_center_end[idx] /= 4

        for idx in range(len(self.marker_end)):
            self.marker_end[idx] = self.rotate(self.marker_end[idx])

        verts = [list(zip(xx, yy, zz))]
        self.ax.add_collection3d(Poly3DCollection(verts, facecolors='r'))
        if self.debug:
            xx = []
            yy = []
            zz = []
            for i in range(len(self.marker_end_raw)):
                xx.append(self.marker_end_raw[i][0])
                yy.append(self.marker_end_raw[i][1])
                zz.append(self.marker_end_raw[i][2])
            verts = [list(zip(xx, yy, zz))]
            self.ax.add_collection3d(Poly3DCollection(verts, facecolors='b'))
        self.isMarkerEndConfigured = True
        self.calculateDistanceBetweenMarkers()

    def calculateDistanceBetweenMarkers(self):
        print "Distance makrers:"
        print self.marker
        print self.marker_end
        center_poit = numpy.array((self.marker_center[0], self.marker_center[1], self.marker_center[2]))
        center_poit_end = numpy.array((self.marker_center_end[0], self.marker_center_end[1], self.marker_center_end[2]))
        dist = numpy.linalg.norm(center_poit - center_poit_end)
        print "Distance is {} cm".format(dist*100)
        xx = [self.marker_center[0], self.marker_center_end[0]]
        yy = [self.marker_center[1], self.marker_center_end[1]]
        zz = [self.marker_center[2], self.marker_center_end[2]]
        print "Distancepoints:"
        print xx
        print yy
        print zz
        print "Kun"
        verts = [list(zip(xx, yy, zz))]
        self.ax.add_collection3d(Line3DCollection(verts, colors='k'))

    def reinitialize(self):
        target = [0.0, 0.0, 0.0]
        for p in range(len(target)):
            self.vect[p] = target[p] - self.marker[0][p]
        for i in range(len(self.marker)):
            self.marker[i][0] += self.vect[0]
            self.marker[i][1] += self.vect[1]
            self.marker[i][2] += self.vect[2]
        for i in range(len(self.marker)):
            self.marker[i][0] /= self.scale
            self.marker[i][1] /= self.scale
            self.marker[i][2] /= self.scale

        print "values :marker size , marker 1|0,1,2"
        print self.MARKER_SIZE
        self.angles = [math.atan(self.marker[1][1]/self.marker[1][0]),
                       math.atan(self.marker[1][2]/self.marker[1][0]),
                       -1*math.atan(self.marker[1][2]/self.marker[1][1])]
        print "ANGLEs are :"
        print self.angles


        self.rotX = [[1,                             0                       ,      0],
                     [0,                             math.cos(self.angles[0]),      -1 * math.sin(self.angles[0])],
                     [0,                             math.sin(self.angles[0]),      math.cos(self.angles[0])]]

        self.rotY = [[math.cos(self.angles[1])     , 0,                             math.sin(self.angles[1])],
                     [0                            , 1,                             0],
                     [-1 * math.sin(self.angles[1]), 0,                             math.cos(self.angles[1])]]

        self.rotZ = [[math.cos(self.angles[2]),      -1 * math.sin(self.angles[2]), 0],
                     [math.sin(self.angles[2]),      math.cos(self.angles[2])     , 0],
                     [0                       ,      0                            , 1]]
        self.rotation_m = [self.rotX, self.rotY, self.rotZ]
        for idx in range(len(self.marker)):
            self.marker[idx] = self.rotate(self.marker[idx])

    def rotate(self, point):
        return point        ### DISABLED
        for axe_rotation in self.rotation_m:
            nx = point[0] * axe_rotation[0][0] + \
                 point[1] * axe_rotation[0][1] + \
                 point[2] * axe_rotation[0][2]
            ny = point[0] * axe_rotation[1][0] + \
                 point[1] * axe_rotation[1][1] + \
                 point[2] * axe_rotation[1][2]
            nz = point[0] * axe_rotation[2][0] + \
                 point[1] * axe_rotation[2][1] + \
                 point[2] * axe_rotation[2][2]
            point = [nx, ny, nz]
        return point

    def callback(self, data):
        self.points_raw[0].append(data.position.x)
        self.points_raw[1].append(data.position.y)
        self.points_raw[2].append(data.position.z)
        # if len(self.points_raw[0]) > 8:   # if the same point as previous, then tracking is lost, do not store points
        #     if self.points_raw[0][-1] == self.points_raw[0][-7] and \
        #        self.points_raw[1][-1] == self.points_raw[1][-7] and \
        #        self.points_raw[2][-1] == self.points_raw[2][-7]:
        #         self.points = [[], [], []]
        #         self.points_raw = [[], [], []]
        #         return
        x = (data.position.x + self.vect[0]) / self.scale
        y = (data.position.y + self.vect[1]) / self.scale
        z = (data.position.z + self.vect[2]) / self.scale
        point = self.rotate([x,y,z])
        self.points[0].append(point[0])
        self.points[1].append(point[1])
        self.points[2].append(point[2])
        # rospy.loginfo('I heard %f %f %f', x, y, z)
        if self.isMarkerDetected and self.counter % 10 == 0:
            if self.debug == True:
                self.l.set_data([-2]+self.points[0] + self.points_raw[0], [-2]+self.points[1] + self.points_raw[1])
                self.l.set_3d_properties([-2]+self.points[2] + self.points_raw[2])
            else:
                self.l.set_data(self.points[0], self.points[1])
                self.l.set_3d_properties(self.points[2])
            self.fig.canvas.draw_idle()
            # plt.plot(points[1], points[0], '*')
            # plt.axis("equal")
            # plt.draw()
            # plt.pause(0.00000000001)
        self.counter += 1

    def marker1(self, data):
        if self.isMarkerDetected:
            return
        # x = data.position.x / self.scale
        # y = data.position.y / self.scale
        # z = data.position.z / self.scale
        self.marker[0][0] = data.position.x
        self.marker[0][1] = data.position.y
        self.marker[0][2] = data.position.z
        # rospy.loginfo('I heard position1 %f %f %f', x, y, z)
    def marker2(self, data):
        if self.isMarkerDetected:
            return
        # x = data.position.x / self.scale
        # y = data.position.y / self.scale
        # z = data.position.z / self.scale
        self.marker[1][0] = data.position.x
        self.marker[1][1] = data.position.y
        self.marker[1][2] = data.position.z
        # rospy.loginfo('I heard position2 %f %f %f', x, y, z)
    def marker3(self, data):
        if self.isMarkerDetected:
            return
        # x = data.position.x / self.scale
        # y = data.position.y / self.scale
        # z = data.position.z / self.scale
        self.marker[2][0] = data.position.x
        self.marker[2][1] = data.position.y
        self.marker[2][2] = data.position.z
        # rospy.loginfo('I heard position3 %f %f %f', x, y, z)
    def marker4(self, data):
        if self.isMarkerDetected:
            return
        # x = data.position.x / self.scale
        # y = data.position.y / self.scale
        # z = data.position.z / self.scale
        self.marker[3][0] = data.position.x
        self.marker[3][1] = data.position.y
        self.marker[3][2] = data.position.z
        # rospy.loginfo('I heard position4 %f %f %f', x, y, z)

        self.checkIfMarkerIsDetected()

    def marker_end1(self, data):
        if self.isMarkerEndDetected:
            return
        # x = data.position.x / self.scale
        # y = data.position.y / self.scale
        # z = data.position.z / self.scale
        self.marker_end[0][0] = data.position.x
        self.marker_end[0][1] = data.position.y
        self.marker_end[0][2] = data.position.z
        # rospy.loginfo('I heard position1 %f %f %f', x, y, z)
    def marker_end2(self, data):
        if self.isMarkerEndDetected:
            return
        # x = data.position.x / self.scale
        # y = data.position.y / self.scale
        # z = data.position.z / self.scale
        self.marker_end[1][0] = data.position.x
        self.marker_end[1][1] = data.position.y
        self.marker_end[1][2] = data.position.z
        # rospy.loginfo('I heard position2 %f %f %f', x, y, z)
    def marker_end3(self, data):
        if self.isMarkerEndDetected:
            return
        # x = data.position.x / self.scale
        # y = data.position.y / self.scale
        # z = data.position.z / self.scale
        self.marker_end[2][0] = data.position.x
        self.marker_end[2][1] = data.position.y
        self.marker_end[2][2] = data.position.z
        # rospy.loginfo('I heard position3 %f %f %f', x, y, z)
    def marker_end4(self, data):
        if self.isMarkerEndDetected:
            return
        # x = data.position.x / self.scale
        # y = data.position.y / self.scale
        # z = data.position.z / self.scale
        self.marker_end[3][0] = data.position.x
        self.marker_end[3][1] = data.position.y
        self.marker_end[3][2] = data.position.z
        # rospy.loginfo('I heard position4 %f %f %f', x, y, z)

        self.checkIfMarkerEndIsDetected()

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/slam/pos', Pose, self.callback)
        rospy.Subscriber('/slam/marker1', Pose, self.marker1)
        rospy.Subscriber('/slam/marker2', Pose, self.marker2)
        rospy.Subscriber('/slam/marker3', Pose, self.marker3)
        rospy.Subscriber('/slam/marker4', Pose, self.marker4)
        rospy.Subscriber('/slam/marker_end1', Pose, self.marker_end1)
        rospy.Subscriber('/slam/marker_end2', Pose, self.marker_end2)
        rospy.Subscriber('/slam/marker_end3', Pose, self.marker_end3)
        rospy.Subscriber('/slam/marker_end4', Pose, self.marker_end4)
        # spin() simply keeps python from exiting until this node is stopped
        # fig = plt.figure()
        # ax = plt.axes(projection='3d')
        # plt.ion()
        plt.show()
        rospy.spin()


if __name__ == '__main__':
    counter = 0
    navi = navigation()
    navi.listener()