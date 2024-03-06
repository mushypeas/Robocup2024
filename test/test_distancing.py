import rospy
import tf
import cv2
import numpy as np

import ros_numpy

def createLineIterator(P1, P2, img):
    """
    Produces and array that consists of the coordinates and intensities of each pixel in a line between two points

    Parameters:
        -P1: a numpy array that consists of the coordinate of the first point (x,y)
        -P2: a numpy array that consists of the coordinate of the second point (x,y)
        -img: the image being processed

    Returns:
        -it: a numpy array that consists of the coordinates and intensities of each pixel in the radii (shape: [numPixels, 3], row = [x,y,intensity])     
    """
    #define local variables for readability
    imageH = img.shape[0]
    imageW = img.shape[1]
    P1X = P1[0]
    P1Y = P1[1]
    P2X = P2[0]
    P2Y = P2[1]

    #difference and absolute difference between points
    #used to calculate slope and relative location between points
    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    #predefine numpy array for output based on distance between points
    itbuffer = np.empty(shape=(np.maximum(dYa,dXa),3),dtype=np.float32)
    itbuffer.fill(np.nan)

    #Obtain coordinates along the line using a form of Bresenham's algorithm
    negY = P1Y > P2Y
    negX = P1X > P2X
    if P1X == P2X: #vertical line segment
       itbuffer[:,0] = P1X
       if negY:
           itbuffer[:,1] = np.arange(P1Y - 1,P1Y - dYa - 1,-1)
       else:
           itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)              
    elif P1Y == P2Y: #horizontal line segment
       itbuffer[:,1] = P1Y
       if negX:
           itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
       else:
           itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
    else: #diagonal line segment
       steepSlope = dYa > dXa
       if steepSlope:
           slope = dX.astype(np.float32)/dY.astype(np.float32)
           if negY:
               itbuffer[:,1] = np.arange(P1Y-1,P1Y-dYa-1,-1)
           else:
               itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
           itbuffer[:,0] = (slope*(itbuffer[:,1]-P1Y)).astype(int) + P1X
       else:
           slope = dY.astype(np.float32)/dX.astype(np.float32)
           if negX:
               itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
           else:
               itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
           itbuffer[:,1] = (slope*(itbuffer[:,0]-P1X)).astype(int) + P1Y

    #Remove points outside of image
    colX = itbuffer[:,0]
    colY = itbuffer[:,1]
    itbuffer = itbuffer[(colX >= 0) & (colY >=0) & (colX<imageW) & (colY<imageH)]

    #Get intensities from img ndarray
    itbuffer[:,2] = img[itbuffer[:,1].astype(np.uint),itbuffer[:,0].astype(np.uint)]
    return itbuffer



def distancing(depth, dist=0.6):

    # Transform from depth camera to base_link
    #depth = ros_numpy.numpify(depth).astype(np.uint16)
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            trans, rot = listener.lookupTransform('base_link',
                                                  'head_rgbd_sensor_depth_frame',
                                                  rospy.Time(0))
            rospy.loginfo(trans)
            rpy = tf.transformations.euler_from_quaternion(rot)
            rospy.loginfo(rpy)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(0.1)
            continue
    # Depth as CV image
    cv_depth = ((depth - depth.min()) / depth.max() * 255).astype(np.uint8)
    # Line Detection (Canny + Probabilistic Hough Transform)
    '''
    canny = cv2.Canny(cv_depth, 100, 150)
    lines = cv2.HoughLinesP(canny,
                            rho=1.0,
                            theta=np.pi / 180,
                            threshold=60,
                            minLineLength=200,
                            maxLineGap=30)
    '''
    # Line Detection by LSD
    '''
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_NONE)
    lines = lsd.detect(canny)[0]
    '''
    # Line Detection by FLD
    fld = cv2.ximgproc.createFastLineDetector(length_threshold=200,
                                              distance_threshold=30,
                                              canny_th1=100,
                                              canny_th2=150,
                                              do_merge=True)
    lines = fld.detect(cv_depth)
    # Skip it if lines not detected ...
    if lines is None:
        rospy.loginfo('1')
        return   
    if len(lines) == 0:
        rospy.loginfo('2')
        return
    cv_depth = cv2.cvtColor(cv_depth, cv2.COLOR_GRAY2BGR)
    print(lines)
    for line in lines:
        cv2.line(cv_depth,
                 (int(line[0][0]), int(line[0][1])),
                 (int(line[0][2]),int(line[0][3])),
                 (255, 0, 0), 2)
    # Remove line with large angles
    lines = np.round(np.array(lines).squeeze(1)).astype(int)
    angle_vals = np.arctan2(lines[:,1] - lines[:,3], lines[:, 0] - lines[:, 2])
    angle_vals = np.abs(np.abs(angle_vals) - np.pi/2)
    line_ok = np.isclose(angle_vals, np.pi/2, rtol=0., atol=np.pi/36)
    lines = np.delete(lines, np.logical_not(line_ok), axis=0)
    if len(lines) == 0:
        rospy.loginfo('3')
        return
    for line in lines:
        cv2.line(cv_depth, (line[0], line[1]), (line[2], line[3]), (0, 255, 0), 2)
    # Find the line at the most below
    y_vals = lines[:, 1] + lines[:, 3]
    if len(y_vals) == 0:
        rospy.loginfo('4')
        return
    below_idx = np.argmax(y_vals)
    table_edge = lines[below_idx]
    cv2.line(cv_depth,
             (table_edge[0], table_edge[1]),
             (table_edge[2], table_edge[3]),
             (0, 0, 255), 2)
    #cv2.imwrite('canny.png', canny)
    cv2.imwrite('hough.png', cv_depth)
    # Access depth info of the segemented table edge
    line_it = createLineIterator(table_edge[0:2], table_edge[2:4], depth)
    line_depth = np.median(np.array(line_it)[:, -1]) / 1000
    line_depth = line_depth * np.cos(np.pi/6)
    #correction = line_depth + trans[0] - dist
    correction = line_depth - dist
    rospy.loginfo(f'Estimated Distancing Correction: {correction} m')
    yaw = np.arctan2(table_edge[1] - table_edge[3], table_edge[0] - table_edge[2])
    rospy.loginfo(f'Yaw: {yaw}')
    #cv2.imshow('hough', cv_depth)
    #cv2.waitKey(10)
    return correction, -yaw

if __name__ == '__main__':

    import time
    from sensor_msgs.msg import Image
    from hsr_agent.agent import Agent
    rospy.init_node('distancing_test', disable_signals=True)
    '''
    sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, distancing)
    rospy.spin()
    '''
    agent = Agent()
    agent.pose.head_tilt(-30)
    st = time.time()
    corr, yaw = distancing(agent.depth_image)
    gap = time.time() - st
    rospy.loginfo(gap)
    agent.move_rel(corr, 0, yaw, wait=True)



