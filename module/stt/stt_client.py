import time
import rospy
from playsound import playsound
from std_msgs.msg import Float32, String

def stt_client(sec=5):

    pub_request = rospy.Publisher('/mic_request', Float32, queue_size=10)
    while not rospy.is_shutdown():
        playsound('./Tools/xtioncam_capture/ding_3x.mp3')
        rospy.loginfo('record start')
        st = time.time()
        topic = Float32(); topic.data = sec
        while time.time() - st <= 0.1:
            pub_request.publish(topic)
        result = rospy.wait_for_message('/stt_result', String)
        rospy.loginfo('record end')
        return result.data


if __name__ == '__main__':
    
    rospy.init_node('stt_client_test', anonymous=False, disable_signals=True)
    while not rospy.is_shutdown():
        print(stt_client())
