from std_msgs.msg import Float32
from playsound import playsound
import pyaudio
import whisper
import wave
from .codebook_parser import parser

name_list = ['adel', 'angel', 'axel', 'charlie', 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone']
drink_list = ['red wine', 'juice pack', 'cola', 'tropical juice', 'milk', 'iced tea', 'orange juice']
yesno_list = ['yes', 'no']

def record(sec=5, filename="temp_recording.wav"):
    # Parameters for audio recording
    FORMAT = pyaudio.paInt16  # Audio format (16-bit PCM)
    CHANNELS = 1  # Number of channels
    RATE = 16000  # Sample rate, Whisper models often use 16kHz
    CHUNK = 1024  # Chunk size

    # Initialize pyaudio
    audio = pyaudio.PyAudio()

    # Open stream
    stream = audio.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

    print("Recording...")

    frames = []

    # Record audio
    for _ in range(int(RATE / CHUNK * sec)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("Finished recording.")

    # Stop and close the stream
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # Save to a temporary WAV file
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))

    return filename


def whisper_stt(sec=5, mode=None):
    topic = Float32(); topic.data = sec
    playsound('./Tools/xtioncam_capture/ding_3x.mp3')
    print('Record start')
    
    wave_file = record(sec)
    
    print('Record end')
    
    model = whisper.load_model("base.en")
    result = model.transcribe(audio=wave_file, verbose=True)

    print('whisper_stt result: ', result)

    answer=result["text"]

    if mode:
        if mode=='name':
            parse_list = name_list
        elif mode=='drink':
            parse_list = drink_list
        elif mode=='yesno':
            parse_list = yesno_list
        answer = parser(result["text"], parse_list)
        print('whiper_stt parser answer: ', answer)
    
    return answer, None


if __name__=="__main__":
    while True:
        mode=input('mode: ')
        if mode=='':
            mode = None
        result = whisper_stt(3, mode)
        print(result)