import rospy
import numpy as np
import io
from scipy.io.wavfile import write
from google.cloud import speech
from google.api_core.retry import Retry
from google.api_core.exceptions import RetryError
from std_msgs.msg import Float32, Float32MultiArray
from .codebook_parser import parser
from playsound import playsound
from deepspeech import Model
import time

# This is actually a client to Google Cloud Server
client = speech.SpeechClient()
config = speech.RecognitionConfig(
    encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
    sample_rate_hertz=16000,
    language_code='en-US',
    model='latest_short'
)
# Request Publish
pub = rospy.Publisher('/mic_request', Float32, queue_size=10)
# Alternative Local Model
local_model = Model("/home/tidy/robocup2023/module/stt/weight/deepspeech-0.9.3-models.pbmm")
local_model.setBeamWidth(20)

def stt_client_hsr_mic(sec=5):

    st = time.time()
    topic = Float32(); topic.data = sec
    while time.time() - st <= 0.1:
        pub.publish(topic)
    playsound('./Tools/xtioncam_capture/ding_3x.mp3')
    rospy.loginfo('record start')
    stream = rospy.wait_for_message('/mic_record', Float32MultiArray)
    rospy.loginfo('record end')
    content = np.array(stream.data, dtype=np.int16)
    byte_io = io.BytesIO(bytes())
    write(byte_io, 16000, content)
    audio = speech.RecognitionAudio(content=byte_io.read())
    r = Retry(timeout=5.0)
    retry_count = 0
    while True:
        try:
            resp = client.recognize(config=config, audio=audio, retry=r)
            result = ''
            for res in resp.results:
                result += str(res.alternatives[0].transcript)
            answer = parser(result)
            return result, answer
        except RetryError:
            retry_count += 1
            print("timeout error (possibly internet issue)")
            if retry_count >= 2:
                result = local_model.stt(content)
                answer = parser(result)
                return result, answer
        except Exception:
            return '', ''

if __name__ == '__main__':

    rospy.init_node('stt_server_mic', anonymous=False, disable_signals=True)

    while not rospy.is_shutdown():
        # result = stt_client_mic()
        result = stt_client_hsr_mic()
        print(result)
