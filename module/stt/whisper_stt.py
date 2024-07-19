## whisper_stt.py

import whisper
import rospy, pydub, pyaudio
import numpy as np
from std_msgs.msg import Float32MultiArray, String
# from .codebook_parser import parser


class WhisperSTT:
    def __init__(self):
        self.model = whisper.load_model("small.en", device="cuda")
        self.portaudio = pyaudio.PyAudio()
        self.pub_result = rospy.Publisher('/stt_result', String, queue_size=10)

        self.filename = 'recording'
        self.chunk = 1024


    def listen(self):
        stream = rospy.wait_for_message('/mic_record', Float32MultiArray)
        content = np.array(stream.data, dtype=np.float32)   
        data = np.int16(content).tobytes()
        audio = pydub.AudioSegment(data=data, frame_rate=44100, sample_width=2, channels=1)
        audio.export(f"{self.filename}.mp3", format="mp3", bitrate="320k")
        result = self.model.transcribe(audio=f"{self.filename}.mp3", verbose=True)
        print(f"\nSTT Result: {result['text']}\n")
        return result

if __name__=="__main__":
    rospy.init_node('stt_server_mic', anonymous=False, disable_signals=True)
    stt = WhisperSTT()

    while not rospy.is_shutdown():
        rospy.loginfo('Ready to listen...')
        result = stt.listen()
        stt.pub_result.publish(result["text"])
        
        
