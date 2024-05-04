import rospy
import numpy as np
import io
from scipy.io.wavfile import write
from std_msgs.msg import Float32, Float32MultiArray
from .codebook_parser import parser
from playsound import playsound
import time
# import traceback
### huggingface.co/facebook/s2t-small-librispeech-asr
import torch
from transformers import Speech2TextProcessor, Speech2TextForConditionalGeneration


# Request Publish
pub = rospy.Publisher('/mic_request', Float32, queue_size=10)
model = Speech2TextForConditionalGeneration.from_pretrained("facebook/s2t-small-librispeech-asr")
processor = Speech2TextProcessor.from_pretrained("facebook/s2t-small-librispeech-asr")

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
    input_features = processor(content, sampling_rate=16_000, return_tensors="pt").input_features
    generated_ids = model.generate(input_features=input_features)
    transcription = processor.batch_decode(generated_ids)
    result = transcription[0].split('</s>')[1].strip()
    print('cloud_stt_hsr_mic result: ', result)
    answer=result
    if mode:
        if mode=='name':
            parse_list = name_list
        elif mode=='drink':
            parse_list = drink_list
        elif mode=='yesno':
            parse_list = yesno_list
        answer = parser(result, parse_list)
        print('cloud_stt_hsr_mic answer: ', answer)
    return answer, None


if __name__ == '__main__':

    rospy.init_node('stt_server_mic', anonymous=False, disable_signals=True)

    while not rospy.is_shutdown():
        # result = stt_client_mic()
        result = stt_client_hsr_mic()
        print(result)
