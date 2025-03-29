## 이미지

<img width="576" alt="ROS2비전 객체 인식 및 위치 추정 시스템 및 SLAM을 이용한 매핑1" src="https://github.com/user-attachments/assets/2fc5216e-74a6-499a-bbd7-0dfccf85c539" />

`node2_v5.py` : 사람을 인식하면 Rbiz상에 파란색과 검정색 막대 표시 (주황색 박스 참고)

<img width="524" alt="ROS2비전 객체 인식 및 위치 추정 시스템 및 SLAM을 이용한 매핑2" src="https://github.com/user-attachments/assets/9e44dc7b-d4f6-4cb4-a922-c8c9f8b635f0" />

`node2_v5.py` : 소화기를 인식하면 Rbiz상에 초록색과 검정색 막대로 표시 (주황색 박스 참고)

<img width="791" alt="ROS2비전 객체 인식 및 위치 추정 시스템 및 SLAM을 이용한 매핑3" src="https://github.com/user-attachments/assets/fc98805e-f61c-4417-b9c4-70ca78cdaabf" />

`node2_v5.py` : TF 변환 시 odom에서 oakd camera까지를 참고



![ROS2비전 객체 인식 및 위치 추정 시스템 및 SLAM을 이용한 매핑4_elplored_map](https://github.com/user-attachments/assets/9165c44c-b537-448b-82ce-861fa6e11179)

`rvmap4.py` : 만들어진 SLAM맵 


## **프로젝트 개요**

`node2_v5.py`

- 로봇의 카메라가 포착한 이미지에서 사전 정의된 객체를 인식하고, 그 객체의 정확한 위치를 Rbiz로 맵 좌표계에 표시

## **주요 기능**

### `node2_v5.py`

- 실시간 이미지 처리 및 SIFT를 사용하여 특징점 추출 및 매칭(객체 인식)
- PnP알고리즘을 통해 2D-3D 대응점을 이용한 객체의 정확한 위치 추정
- 인식된 객체의 맵 상 위치를 시각화하기 위한 마커 발행

## 시연 영상

유튜브 업로드

## **코드 설명**

### `node2_v5.py`

### 1. SIFTFeatureMatcher 클래스

이 클래스는 SIFT(Scale-Invariant Feature Transform) 알고리즘을 활용하여 이미지에서 특징점을 추출하고 매칭하는 역할을 담당

- **초기화**: SIFT 특징점 검출기와 FLANN 기반 매처를 설정
- **이미지 전처리**: 이미지 크기 조정, 히스토그램 평활화, 노이즈 제거 등을 수행
- **특징점 매칭**: Ratio Test와 RANSAC 필터링을 적용하여 정확한 매칭을 보장
- **유사도 판단**: 매칭된 특징점 수와 품질을 기반으로 두 이미지의 유사도를 평가

### 2. ImprovedDetectorNode 클래스

ROS2 노드로 구현된 메인 클래스로, 전체 시스템의 중심 역할

- **피클 파일 로드**: 미리 계산된 참조 객체('ext'와 'man')의 SIFT 특징점을 로드
- **이미지 구독**: 카메라로부터 압축된 이미지를 받아 처리
- **객체 매칭**: 입력 이미지와 참조 객체 간의 매칭을 수행
- **PnP 처리**: Perspective-n-Point 알고리즘을 사용하여 객체의 3D 위치를 추정
- **좌표계 변환**: TF2 프레임워크를 사용하여 다양한 좌표계 간 변환을 처리
- **마커 발행**: 인식된 객체의 위치를 시각화하기 위한 RViz2 마커를 발행

### 3. 좌표계 변환 체인

시스템은 다음과 같은 변환 체인을 구현됨.

- **Map → Base**: TF2를 통해 로봇의 맵 상 위치를 얻음
- **Base → Camera**: 로봇에 장착된 카메라의 상대적 위치를 나타냄
- **Camera → Object**: PnP 알고리즘을 통해 카메라에서 인식된 객체의 상대적 위치를 계산
- **Map → Object**: 위 세 변환을 결합하여 맵 좌표계에서 객체의 절대 위치를 구합
