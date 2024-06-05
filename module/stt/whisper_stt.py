import rospy
import numpy as np
import io
from scipy.io.wavfile import write
from std_msgs.msg import Float32, Float32MultiArray
from .codebook_parser import parser
from playsound import playsound
import time
import torch
import whisper


# Request Publish
pub = rospy.Publisher('/mic_request', Float32, queue_size=10)

name_list = ['adel', 'angel', 'axel', 'charlie', 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone']
drink_list = ['red wine', 'juice pack', 'cola', 'tropical juice', 'milk', 'iced tea', 'orange juice']
yesno_list = ['yes', 'no']

def stt_client_hsr_mic(sec=5, mode=None):

    st = time.time()
    topic = Float32(); topic.data = sec
    while time.time() - st <= 0.1:
        pub.publish(topic)
    playsound('./Tools/xtioncam_capture/ding_3x.mp3')
    rospy.loginfo('record start')
    stream = rospy.wait_for_message('/mic_record', Float32MultiArray)
    rospy.loginfo('record end')
    content = np.array(stream.data, dtype=np.float32)
    model = whisper.load_model("base.en")
    result = model.transcribe(content)
    print(result["text"])

if __name__=="__main__":
    rospy.init_node('stt_server_mic', anonymous=False, disable_signals=True)

    while not rospy.is_shutdown():
        result = stt_client_hsr_mic()
        print(result)