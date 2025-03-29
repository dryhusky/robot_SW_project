## 이미지
<img width="757" alt="컨베이어 기반 매니퓰레이션 시스템1" src="https://github.com/user-attachments/assets/a699b358-dd7a-48fd-a31f-1806c8692f06" />

<img width="599" alt="컨베이어 기반 매니퓰레이션 시스템2" src="https://github.com/user-attachments/assets/4956c592-a2f4-4f37-93c7-48cb83521f66" />

<img width="724" alt="컨베이어 기반 매니퓰레이션 시스템3" src="https://github.com/user-attachments/assets/8f9491e5-16f5-47cf-8887-ad59f189d504" />


## **프로젝트 개요**

TurtleBot3 로봇과 컨베이어 벨트를 결합한 통합 자동화 시스템으로

컴퓨터 비전, 로봇 매니퓰레이션(로봇 제어), 컨베이어 제어, GUI를 결합하여 물체 감지, 분류, 운반 작업을 자동화하는 솔루션을 제공합니다.

## **주요 기능**

- **컨베이어 제어 시스템**
    - 거리 기반 컨베이어 이동 제어
    - 실시간 상태 모니터링 및 보고
    - 시리얼 통신을 통한 아두이노 제어
    - ROS2 토픽을 통한 상태 및 명령 인터페이스
- **객체 감지 및 인식**
    - YOLO 기반 객체 감지 및 분류
    - ArUco 마커 감지 및 위치 추적
    - 실시간 이미지 처리 및 시각화
- **로봇 제어 및 매니퓰레이션**
    - MoveIt 기반 로봇 팔 제어
    - 마커 기반 네비게이션
    - 다양한 작업(빨간색/파란색 물체 분류) 자동화
- **사용자 인터페이스**
    - PyQt5 기반 직관적 GUI
    - 작업 선택 및 실행 제어
    - 실시간 카메라 피드 및 상태 디스플레이
    - 컨베이어 거리 제어

## 시연 영상

유튜브 업로드

https://youtu.be/5Q_HrRy45Y0

gui에서 ‘job2’명령을 주게 되면, 

터틀봇은 red블록 1개와 blue블록 2개를 집게 됨.

 

https://youtu.be/CF1k3llyGnQ

## **코드 설명**

### 1. 컨베이어 제어 (`conveyer5.py`)

```python
class ConveyorController(Node):
    # ROS2 노드로 컨베이어 제어 담당
    # 시리얼 통신으로 아두이노와 연결
    # 'conveyor/status' 토픽 발행
    # 'conveyor/control' 토픽 구독하여 명령 처리
```

- QThread를 활용한 멀티스레딩 구현
- 시리얼 연결 자동 복구 메커니즘
- JSON 포맷 명령 처리 구조

### 2. GUI 인터페이스 (`gui10.py`)

```python
class WindowClass(QMainWindow, form_class):
    # PyQt5 기반 GUI 인터페이스
    # 작업 선택 및 실행 기능
    # 실시간 카메라 피드 표시
    # 로봇 및 컨베이어 상태 표시
```

- 멀티스레딩 ROS2 통신 구현
- 다양한 작업(job1, job2, job3) 선택 및 실행
- 컨베이어 거리 제어 인터페이스

### 3. 컴퓨터 비전 모듈

ArUco 마커 감지 (`aruco_detector.py`)

```python
class ArucoMarkerDetector(Node):
    # 카메라 이미지에서 ArUco 마커 감지
    # 마커 위치 및 자세 계산
    # 'detected_markers' 토픽으로 마커 정보 발행
    
	  def detect_markers(image, camera_matrix, dist_coeffs, marker_size):
	    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
	    parameters = cv2.aruco.DetectorParameters()
	    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
	    corners, ids, _ = detector.detectMarkers(image)
```

- 카메라 캘리브레이션 및 왜곡 보정
- 아루코 마커를 통해 3D 위치 추정 기능
- target_marker_id를 판단 기준으로 아루코 마커를 분류

YOLO 객체 감지 (`yolo_detector.py`)

```python
class YoloDetect(Node):
    # YOLOv8 기반 객체 감지
    # 객체 위치를 물리적 단위(mm)로 변환
    # 감지 결과 시각화 및 발행
```

- YOLO_v8 모델에 파인튜닝을 진행
- 카메라 캘리브레이션 매트릭스를 이용하여 카메라 이미지 왜곡 보정
- 픽셀 좌표를 물리적 거리로 변환(pix_2_mm = 실제 거리(mm) / 픽셀 수)

### 4. 로봇 제어 모듈

ArUco 기반 이동 (`aruco_move.py`)

```python
class ArucoMarkerListener(Node):
    # 감지된 마커 정보 기반 로봇 이동
    # 목표 마커까지의 거리에 따라 속도 제어
```

- TurtleBot 팔 제어
- MoveIt 프레임워크를 사용한 로봇 팔 움직임 계획 및 실행
