# HSR Head Display

### Description
HSR 의 head display 에 이미지 혹은 텍스트를 띄우기 위한 
코드 입니다. 

먼저 스크립트가 되는지 시도해보시고 안되면 아래 절차를 따라주세요 


### 스크립트
```Shell
    expect shell_hsr_head_monitor
```
expect가 깔려있지 않으면 yum install expect로 다운로드 후 실행해주세요

로봇의 모니터가 검정색으로 변하면 성공입니다.



### stt+monitor 스크립트
```Shell
    expect shell_monitor_and_stt
```
stt와 monitor를 사용하기 위해서 robot side에서 실행이 필요한 파이썬 파일들을 실행합니다.



### 스크립트가 안될경우
Connect with HSR via SSH
```Shell
    ssh administrator@hsrb.local
```

### Setup
xhost 설정위해 hsr-hmi 에 접속해야함.

```Shell
   sudo su - hsr-hmi
   export DISPLAY=:0
   xhost + 
   ```
   ctrl + d and logout

administrator 에서
```Shell 
   export DISPLAY=:0
   ```

### Running the demo 

```Shell
   cd hsr_head_display/
   # HSR의 administrator 내부에서 돌림
   python3 hsr_head_monitor.py
   # backpack 에서 돌릴때
   1. Text 를 띄우려면
   /hsr_head_msg 로 string 타입 topic 을 publish 해준다.
   2. 로봇 내에 저장된 이미지를 띄우려면
   /head_img_idx 로 file idx(ex. 1: plate ,2: bowl, 3: cutlery)을 int32 타입으로 publish 해준다.
   3. backpack의 이미지를 띄워주고 싶을때 
   /hsr_head_img 로 Image rostopic 을 publish 해준다. 
   ```

### Issue
HSR의 head display 가 error 메세지 나와도 동작함을 확인했습니다.

하지만, 혹시 HSR 의 head display 가 stereo camera 안된다는 error 메세지가 나오는데 끄고 싶을경우

1. /etc/opt/tmc/robot/docker.hsrb.user 파일의 USE_HEAD_STEREO_CAMERA=FALSE 가 되어있는지 확인
2. sudo systemctl restart docker.hsrb.roscore.service 를 실행한뒤 e-stop 한뒤 다시 e-stop 풀어주기


참고 https://docs.hsr.io/hsr_develop_manual_en/customize/sensor_on_off.html?highlight=sensor%20off
