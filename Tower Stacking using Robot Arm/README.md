## 이미지

<img width="381" alt="arm을 활용하여 탑쌓기1" src="https://github.com/user-attachments/assets/37fcd623-374a-4753-bbab-429071f1c77b" />

<img width="421" alt="arm을 활용하여 탑쌓기2" src="https://github.com/user-attachments/assets/df03d953-b39d-4dec-9e57-ed55461270cc" />


## **프로젝트 개요**

- 컵 10개를 이용한 4층 피라미드 탑 쌓기
- 물병을 집어 진자 운동을 통한 던지기 동작

## **주요 기능**

- **그리퍼 작동**: 물체를 잡고 놓는 기능
- **경로 이동**: 사전 정의된 위치로 로봇 암을 이동
- **던지기 동작**: 물병을 집어 던지는 특수 동작 수행
- **쓰레딩 활용**: 물병 던지기와 놓기를 동시에 수행하기 위한 비동기 처리
- **Z축 고정** :  물체를 집기 전에 Z축에 대해 정렬하여 보다 안정성있게 동작을 수행 가능

## 실행 흐름

1. 로봇 준비 위치로 이동
2. 1~3층 컵 처리 (총 10개)
    - 각 컵마다 감지, 집기, 배치 반복
    - 각 층에 맞는 적절한 높이와 방식으로 배치
3. 4층 특수 모드 실행 (1개 컵)
    - 특수 경로와 움직임으로 피라미드 최상단에 컵 배치
4. 물병 던지기 동작 실행
    - 물병 접근, 집기, 들어올리기
    - 진자 운동을 이용한 던지기 동작 수행
5. 마무리 및 원위치로 복귀

## 시연 영상

유튜브 업로드

https://youtu.be/mFHkpdyLhB4

## **코드 설명**

### 그리퍼 제어 함수

```python
def grip():
    print("Closing gripper...")
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(1)

def release():
    print("Opening gripper...")
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait(1)
```

### 물병 던지기 함수

```python
def throw_bottle(upright_pos):
    print("Preparing throwing motion")
    # [Wind-up] 던지기 전에 약간의 준비 동작
    windup_pos = [-6.201, 3.163, 77.054, -5.759, 90.0, -85.656]
    amovej(windup_pos, time=2.0)
    
    # [Throw] 빠른 가속으로 던지는 동작
    throw_target_pos = [-6.201, 3.164, 77.054, -5.759, -24.114, -85.657]
    release_thread_obj = threading.Thread(target=release_thread)
    release_thread_obj.start()
    amovej(throw_target_pos, time=2.0)
    
    release_thread_obj.join()
    wait(0.3)
```

### 힘 감지 로직

-Z방향으로 20N 힘만큼 가함

이때 7N이상의 힘이 감지되면 감지

```python
task_compliance_ctrl(stx=[500,500,500,100,100,100])
set_desired_force(fd=[0,0,-20,0,0,0],
                dir=[0,0,1,0,0,0],
                mod=DR_FC_MOD_REL)

# 접촉 감지 대기
start_time = time.time()
timeout = 5.0
contact_detected = False

while time.time() - start_time < timeout:
    if check_force_condition(DR_AXIS_Z, max=7):
        contact_detected = True
        break
    wait(0.1)
```
