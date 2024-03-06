import rospy
from visualization_msgs.msg import Marker

# rospy.init_node('rviz_marker')

class MarkerMaker():
    def __init__(self, marker_topic):
        self.marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=2)
    def pub_marker(self, xyz, frame_id, scale=.3, ns="0"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.ns = ns
        marker.id = 0
        # Set the scale of the marker
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = int(int(ns) % 3 == 0)
        marker.color.g = int(int(ns) % 3 == 1)
        marker.color.b = int(int(ns) % 3 == 2)
        marker.color.a = 1.0
        # Set the pose of the marker
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        self.marker_pub.publish(marker)


# while not rospy.is_shutdown():
#
#   marker_pub.publish(marker)
#   rospy.rostime.wallsleep(1.0)
if __name__ == '__main__':

    ARENA_EDGES = [[8.36, 3.75], [8.28, 6.26], [5.54, 5.88], [5.5177, 3.6958], [4.7759, 3.6698], [4.7012, 2.4829], [0.95, 2.08], [0.91, 0.66],\
                       [0.61, 0.72], [0.26, 9.41], [-2.37, 9.73], [-1.64, -1.19],
                       [0.58, -0.55], [4.30, -0.38], [5.23, 0.29], [7.00, 0.51],
                       [7.00, 2.60], [8.44, 2.78]]
    # ARENA_EDGES = [[1.167, -0.2321], [7.0443, -0.1863], [7.015, 2.5457], [8.2162, 2.6681], [8.2485, 6.0065],\
    #                     [5.6399, 5.8781], [5.5177, 3.6958], [4.7759, 3.6698], [4.7012, 2.4829], [0.9986, 2.0893]]

    marker = MarkerMaker('/snu/marker')
    while not rospy.is_shutdown():
        rospy.init_node('mm')
        marker.pub_marker([5.65, 5.89,1], 'map', ns='0')

