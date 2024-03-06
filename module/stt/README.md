# stt module

### Tasks which requires this module

- restaurant

### Setup Manual

1. google-cloud authentication
[google-cloud authentication process](https://www.notion.so/google-cloud-authentication-process-659ee8d8751f4e7182220bf66f9c1642) 
2. Install following python libraries
    - NumPy
    - SciPy
    - [google-cloud-speech](https://pypi.org/project/google-cloud-speech/)
    - [pyphonetics](https://pypi.org/project/pyphonetics/)
    - yaml
    - [sounddevice](https://pypi.org/project/sounddevice/)
3. Clone below repository in the catkin workspace and build it.
https://github.com/qpwodlsqp/robocup2023_stt

### How to run

```bash
foo@bar:~/robocup2023/module/stt$> hsrb_mode

foo@bar:~/robocup2023/module/stt$> source ~/robocup2023/venv/bin/activate

foo@bar:~/robocup2023/module/stt$> python cloud_stt.py
```

### Service

- `/tidybboy_stt_cloud`: SpeechToText. The custom service message defined for speech recognition. It sends an encoded speech information and receives a parsed string result.