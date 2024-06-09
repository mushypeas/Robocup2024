import rospy
import numpy as np
import io
from std_msgs.msg import Float32, Float32MultiArray
import pydub
# from .codebook_parser import parser
from playsound import playsound
import time
import torch
import whisper

# Request Publish
pub = rospy.Publisher('/mic_request', Float32, queue_size=10)

name_list = ['adel', 'angel', 'axel', 'charlie', 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone']
drink_list = ['red wine', 'juice pack', 'cola', 'tropical juice', 'milk', 'iced tea', 'orange juice']
yesno_list = ['yes', 'no']

def record():
    import pyaudio
    import wave

    filename = 'myfile.wav'

    # Set chunk size of 1024 samples per data frame
    chunk = 1024  

    # Open the sound file 
    wf = wave.open(filename, 'rb')

    # Create an interface to PortAudio
    p = pyaudio.PyAudio()

    # Open a .Stream object to write the WAV file to
    # 'output = True' indicates that the sound will be played rather than recorded
    stream = p.open(format = p.get_format_from_width(wf.getsampwidth()),
                    channels = wf.getnchannels(),
                    rate = wf.getframerate(),
                    output = True)

    # Read data in chunks
    data = wf.readframes(chunk)

    # Play the sound by writing the audio data to the stream
    while data != '':
        stream.write(data)
        data = wf.readframes(chunk)

    # Close and terminate the stream
    stream.close()
    p.terminate()
def stt_client_hsr_mic(i, sec=5):
    st = time.time()
    topic = Float32(); topic.data = sec
    while time.time() - st <= 0.1:
        pub.publish(topic)
    playsound('./Tools/xtioncam_capture/ding_3x.mp3')
    rospy.loginfo('record start')
    stream = rospy.wait_for_message('/mic_record', Float32MultiArray)
    content = np.array(stream.data, dtype=np.float32)
    rospy.loginfo('record end')
    channels = 2 if (content.ndim == 2 and content.shape[1] == 2) else 1
    y = np.int16(content)
    audio = pydub.AudioSegment(y.tobytes(), frame_rate=44100, sample_width=2, channels=channels)
    audio.export(f"audio{i}.mp3", format="mp3", bitrate="320k")
    model = whisper.load_model("base.en")
    result = model.transcribe(audio=f"audio{i}.mp3", verbose=True)
    print(result["text"])

if __name__=="__main__":
    rospy.init_node('stt_server_mic', anonymous=False, disable_signals=True)

    i = 0
    while not rospy.is_shutdown():
        result = stt_client_hsr_mic(i)
        print(result)
        i += 1