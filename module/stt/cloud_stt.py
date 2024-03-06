import rospy
import numpy as np
import io
from scipy.io.wavfile import write
from robocup2023_stt.srv import SpeechToText
from google.cloud import speech
from google.api_core.retry import Retry
from google.api_core.exceptions import RetryError
from deepspeech import Model, version

# This is actually a client to Google Cloud Server
client = speech.SpeechClient()
config = speech.RecognitionConfig(
    encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
    sample_rate_hertz=16000,
    language_code='en-US',
    model='latest_short'
)
local_model = Model("/home/tidy/robocup2024/module/stt/weight/deepspeech-0.9.3-models.pbmm")
local_model.setBeamWidth(20)
print(local_model.sampleRate())

def _stt_handler(srv):

    # Assume buffer msg is already scaled int16 array
    content = np.array(srv.buffer, dtype=np.int16)
    byte_io = io.BytesIO(bytes())
    write(byte_io, 16000, content)
    audio = speech.RecognitionAudio(content=byte_io.read())
    r = Retry(timeout=5.0)
    retry_count = 0
    while True:
        try:
            resp = client.recognize(config=config, audio=audio, retry=r)
            trans = ''
            for result in resp.results:
                trans += str(result.alternatives[0].transcript)
            print(f'Google: {trans}')
            local_result = local_model.stt(content)
            print(f'Deepspeech: {local_result}')
            return trans
        except RetryError:
            print("timeout error occured")
            retry_count += 1
            if retry_count >=1:
                return local_model.stt(content)

def _stt_server():

    rospy.Service('tidyboy_stt_cloud', SpeechToText, _stt_handler)
    rospy.loginfo('STT Server ON')
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('stt_server', anonymous=False, disable_signals=True)
    _stt_server()


