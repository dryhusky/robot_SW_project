import cv2
import os
import time
import requests
import numpy as np
import serial

# 시리얼 포트 설정 (컨베이어 벨트 및 센서)
ser = serial.Serial("/dev/ttyACM0", 9600)

# Vision AI API 설정
VISION_API_URL = "https://suite-endpoint-api-apne2.superb-ai.com/endpoints/d5a043ed-fba6-4af4-858d-c681612fcafd/inference"
TEAM = "kdt2025_1-17"
ACCESS_KEY = "avBVWsAOoz3LS0N7dUTdm5b6fbK0F2dN1A9Jee7r"

# 크롭 영역 설정
CROP_X = 200
CROP_Y = 100
CROP_WIDTH = 300
CROP_HEIGHT = 300

# 클래스별 색상 정의
color_code = {
    "CHIPSET": (0, 0, 255),
    "BOOTSEL": (0, 255, 255),
    "OSCILLATOR": (0, 255, 0),
    "HOLE": (255, 0, 0),
    "HEADER": (255, 0, 255),
    "USB": (255, 255, 0),
    "RASPBERRY PICO": (128, 0, 1128),
}

# 필요한 모든 클래스 목록
required_classes = {
    "CHIPSET": 1,
    "BOOTSEL": 1,
    "OSCILLATOR": 1,
    "HOLE": 4,
    "HEADER": 1,
    "USB": 1,
    "RASPBERRY PICO": 1,
}

def get_unique_filename(directory, base_name, extension):
    """중복을 피하기 위해 고유한 파일 이름 생성"""
    file_path = os.path.join(directory, f"{base_name}{extension}")
    counter = 1

    # 파일 이름 중복 확인 및 새로운 이름 생성
    while os.path.exists(file_path):
        file_path = os.path.join(directory, f"{base_name} ({counter}){extension}")
        counter += 1

    return file_path

def inference_request(img):
    """Vision AI API로 이미지 처리 요청"""
    _, img_encoded = cv2.imencode(".jpg", img)
    img_bytes = img_encoded.tobytes()
    response = requests.post(
        url=VISION_API_URL,
        auth=(TEAM, ACCESS_KEY),
        headers={"Content-Type": "image/jpeg"},
        data=img_bytes,
    )
    return response.json() if response.status_code == 200 else None

def process_and_save_cropped_image(img):
    """이미지를 크롭하여 Vision AI API로 처리하고 결과를 저장"""
    # 이미지 크롭
    cropped_img = img[CROP_Y:CROP_Y + CROP_HEIGHT, CROP_X:CROP_X + CROP_WIDTH]

    # 저장 디렉토리 설정
    save_dir = "./processed_images"
    os.makedirs(save_dir, exist_ok=True)

    # Vision AI 객체 인식
    response = inference_request(cropped_img)
    detected_classes = {cls: 0 for cls in required_classes.keys()}

    # 물체가 감지되었을 때, 처리 후 로그 출력
    if response and "objects" in response:
        for obj in response["objects"]:
            class_name = obj["class"]
            score = obj["score"]
            box = obj["box"]
            start_point = (box[0], box[1])
            end_point = (box[2], box[3])

            # 클래스별 색상 적용
            color = color_code.get(class_name, (255, 255, 255))
            cv2.rectangle(cropped_img, start_point, end_point, color, 2)
            text = f"{class_name} ({score:.2f})"
            cv2.putText(
                cropped_img, text, (box[0], box[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
            )

            # 감지된 클래스 수 업데이트
            if class_name in detected_classes:
                detected_classes[class_name] += 1

    print("\n----------------------------------------")

    # 누락된 클래스 강조
    missing_classes = [
        cls for cls, count in required_classes.items() if detected_classes[cls] < count
    ]

    # 파일명에 누락된 클래스 포함
    missing_classes_str = "_".join(missing_classes) if missing_classes else "정상"

    # 로그 출력 및 결과 메시지
    if missing_classes:
        print("Missing Objects:")
        for cls in missing_classes:
            print(f" {cls}")
        print("Result: 불량입니다.")  # 불량 표시
    else:
        print("Result: 정상입니다.")  # 정상 표시

    print("\n----------------------------------------")

    # 중복 방지 파일 이름 생성
    save_path = get_unique_filename(save_dir, missing_classes_str, ".jpg")
    cv2.imwrite(save_path, cropped_img)

    # 결과 이미지 표시
    cv2.imshow("Cropped and Processed Image", cropped_img)
    cv2.waitKey(1000)  # 0.5초 동안 표시
    cv2.destroyAllWindows()

# 컨베이어 시스템 동작
file_count = 0
while True:
    data = ser.read()  # 센서 데이터 읽기
    if data == b"0":  # 적외선 센서 감지
        cam = cv2.VideoCapture(0)
        if not cam.isOpened():
            print("카메라 오류")
            continue

        ret, img = cam.read()
        cam.release()
        if not ret:
            continue

        print("##########")    
        # 크롭 및 객체 인식 및 저장
        process_and_save_cropped_image(img)

        print("##########")
        # 컨베이어 대기 2초
        time.sleep(1.5)

        # 컨베이어 재시작
        ser.write(b"1")
        file_count += 1
    else:
        time.sleep(0.1)