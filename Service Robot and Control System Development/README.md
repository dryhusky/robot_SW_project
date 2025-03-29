## 이미지

![서비스 로봇 및 관제 시스템1](https://github.com/user-attachments/assets/ad8978d6-6492-449f-98b8-944f05dca1ba)

6번 테이블 조리가 완료되어 로봇이 음식과 함께6번 테이블로 이동

![서비스 로봇 및 관제 시스템2](https://github.com/user-attachments/assets/a15cc748-3913-4d5e-8f48-19004f0f969c)

6번 테이블에 음식 배달 후 초기위치로 복귀

![서비스 로봇 및 관제 시스템3](https://github.com/user-attachments/assets/995810e9-5880-4401-9c61-0fdd81f58784)

rviz(3D시각화 툴)

<img width="1125" alt="restaurant_db" src="https://github.com/user-attachments/assets/01b9520a-c832-44ca-966f-266d8ba36055" />

restaurant_db 일부



## **프로젝트 개요**

- GUI(주방)에서 amr(서빙로봇)에게 테이블 번호와 메뉴 전송
- amr의 자율주행
- slam을 통한 매핑
- 주문 내역을 db에 저장
- 주방, 로봇, db간에 미들웨어 인터페이스 구현

## **주요 기능**

- 테이블→주방→로봇으로 토픽 발행
- way point를 이용한 자율주행
- slam을 통한 매핑
- 액션을 통해 로봇이 지정된 테이블로 간 후 초기위치로 복귀
- 주방과 DB와의 서비스 통신으로 실시간 재료 현황, 일일매출, 연간 매출 확인 가능

## **코드 설명**

- ActionClient:  ROS2 액션 클라이언트를 사용하여 amr의 목표 위치 이동 제어
- GoalStatus: 액션 상태(amr이 목표지점에 도달한 성공, 실패 여부 등 확인)
- PoseStamped: 위치를 포함하는 메세지
- self.table_waypoints: 각 테이블(웨이포인트)의 위치
- self.initial_position: 로봇이 기본적으로 대기하는 위치이자 목표 테이블 도착 후, 자동으로 이 위치로 복귀
- NavigateToPose: 로봇이 목표 위치를 설정하여 액션 서버에 전송하고 이동
