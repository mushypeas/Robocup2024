import rospy
import numpy as np
import pyaudio
import time

from std_msgs.msg import Int8, Float32, Float32MultiArray


def record(p, sec, FS=16000):

    chunk = 1024
    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=FS,
                    input=True,
                    output=True,
                    frames_per_buffer=chunk)

    frames = []
    for _ in range(FS // chunk * int(sec)):
        data = stream.read(chunk)
        frames = frames + np.frombuffer(data, dtype=np.int16).tolist()
    stream.stop_stream()
    stream.close()
    return frames

def mic_server():

    pub = rospy.Publisher('/mic_record', Float32MultiArray, queue_size=10)
    p = pyaudio.PyAudio()
    try:
        while not rospy.is_shutdown():
            sec = rospy.wait_for_message('/mic_request', Float32).data
            print('record start')
            stream = record(p, sec)
            topic = Float32MultiArray()
            topic.data = stream
            st = time.time()
            while time.time() - st <= 0.1:
                pub.publish(topic)
            print('published')
    except KeyboardInterrupt:
        p.terminate()
        return

if __name__ == '__main__':

    rospy.init_node('hsr_mic', anonymous=False, disable_signals=True)
    mic_server()

