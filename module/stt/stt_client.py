import rospy
import numpy as np
import sounddevice as sd
from robocup2023_stt.srv import SpeechToText
from .codebook_parser import parser
#from codebook_parser import parser
from playsound import playsound

def stt_client(sec=5, FS=16000, mode=None):

    '''
    Outer functions should call this client method only
    '''
    rospy.wait_for_service(f'/tidyboy_stt_cloud')
    client = rospy.ServiceProxy(f'/tidyboy_stt_cloud', SpeechToText)
    record = sd.rec(int(FS * sec), samplerate=FS, channels=1)
    playsound('../../Tools/xtioncam_capture/ding_3x.mp3')
    rospy.loginfo('record start')
    sd.wait()
    rospy.loginfo('record end')
    record = record * np.iinfo(np.int16).max
    record = record.astype(np.int16).flatten()
    stt = client(record)
    answer = parser(stt.result)
    
    return stt.result, answer


if __name__ == '__main__':
    
    rospy.init_node('stt_client_test', anonymous=False, disable_signals=True)
    while not rospy.is_shutdown():
        print(stt_client())
