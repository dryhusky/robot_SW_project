## 이미지
![car1](https://github.com/user-attachments/assets/f9375306-a10f-4712-bce0-e0e52e4cbd55)


![car2](https://github.com/user-attachments/assets/fcc02cca-03fe-401f-b68b-15bc4804b409)

## **프로젝트 개요**

- YOLO 탐지를 통한 객체 인식 및 트래킹
- 인터페이스를 통한 amr에 명령전달
- SLAM을 통한 매핑
- 매핑으로 얻은 맵을 통한 amr의 자율주행

## **주요 기능**

- 사전 학습된 YOLO 모델을
    - 해당 모델을 학습할 때 사용한 사진들은 터틀봇의 웹캠의 시야 기준으로 촬영 및 라벨링
- LIDAR를 통한 장애물 탐지
- SLAM을 통한 매핑
- 카메라를 통한 모델학습과 인식
- Nav2를 통한 자율주행 (추적 주행)

## 시연 영상

https://youtube.com/shorts/aVuOOrhoYnU

https://youtu.be/4ltGimO7wxY


## **코드 설명**

- 카메라 관련 코드
    - timer_callback(self): 카메라의 피드를 받아서 publisher로 발행
        - cv2, CompressedImage 모듈 사용
        - cv2.VideoCapture - 카메라에서 피드를 받아오는 역할
        - cv2.imencode - 이미지를 CompressedImage 메시지로 발행할 때 맞는 포맷으로 변환
- AI 추론 관련 코드
    - 
- 주행 관련 코드
    - publish_initial_pose: 초기 위치 메세지 생성 및 퍼블리시
    - NavigateToPose: Nav2 내비게이션 목표 설정을 위한 Action 메시지
    - threading: 키보드 입력을 백그라운드에서 감지하도록 스레드 실행
    - euler_to_quaternion: Euler 각을 Quaternion으로 변환
    - feedback_callback: 로봇이 이동 중일 때 현재 위치 실시간으로 출력
    - follow_waypoints: 액션 서버에 여러 개의 목표 위치(웨이포인트)를 전송
 
