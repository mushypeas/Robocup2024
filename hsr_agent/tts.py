import rospy
from tmc_msgs.msg import Voice

class TTS():

    def __init__(self):
        self._pub = rospy.Publisher('/talk_request', Voice, queue_size=0)
        self._language = Voice.kEnglish

    def say(self, text):
        """Speak a given text

        Args:
            text (str): A text to be converted to voice sound (UTF-8)
        Returns:
            None
        """
        msg = Voice()
        msg.interrupting = False
        msg.queueing = False
        msg.language = self._language
        msg.sentence = text
        self._pub.publish(msg)
        print("finish")

if __name__ == '__main__':
    rospy.init_node('tidyboy_tts')
    tts = TTS()
    # rospy.sleep(1)
    print('tts init')
    tts.say('hello')
    # rospy.sleep(1)

