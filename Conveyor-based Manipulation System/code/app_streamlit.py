import streamlit as st
from PIL import Image
import os
import time
import shutil
from datetime import datetime

def is_file_ready(file_path, retries=3, retry_delay=0.5):
    """파일이 완전히 저장되었는지 확인, 여러번 시도"""
    for i in range(retries):
        try:
            initial_size = os.path.getsize(file_path)
            time.sleep(retry_delay)  # 대기
            if initial_size == os.path.getsize(file_path) and initial_size > 0:
                return True
        except (FileNotFoundError, PermissionError):
            time.sleep(retry_delay)  # 파일을 찾을 수 없거나 접근할 수 없는 경우 잠시 대기
    return False

def app():
    # 어플 상태 관리
    if 'running' not in st.session_state:
        st.session_state.running = True
    if 'last_check_time' not in st.session_state:
        st.session_state.last_check_time = time.time()
    if 'processed_files' not in st.session_state:
        st.session_state.processed_files = set()
    if 'displaying_file' not in st.session_state:
        st.session_state.displaying_file = None
    if 'error_count' not in st.session_state:
        st.session_state.error_count = 0
    
    # 어플 이미지 로드 및 표시
    app_logo = "/home/kdt/vision-ai-inference-practice/pk.png"  # 어플 이미지 파일 경로
    if os.path.exists(app_logo):
        st.image(app_logo, use_container_width=False, width=200)
    else:
        st.error("어플 로고 이미지를 찾을 수 없습니다.")

    # 웹사이트 제목 및 설명
    st.title("양품/불량품 검사 웹사이트")
    
    # 중지 버튼 추가
    if st.button("검사 중지" if st.session_state.running else "검사 시작"):
        st.session_state.running = not st.session_state.running
        st.experimental_rerun()

    # 로컬 디렉토리 설정
    watch_directory = "/home/kdt/vision-ai-inference-practice/5.conveyor-system/processed_images"  # 감시할 디렉토리
    processed_directory = "/home/kdt/vision-ai-inference-practice/5.conveyor-system/processed_images_end"  # 처리 완료 파일 디렉토리

    # 디렉토리 존재 여부 확인 및 생성
    for directory in [watch_directory, processed_directory]:
        if not os.path.exists(directory):
            try:
                os.makedirs(directory)
                st.success(f"디렉토리 생성 완료: {directory}")
            except Exception as e:
                st.error(f"디렉토리 생성 실패: {directory}, 오류: {str(e)}")
                return

    # 동적 업데이트를 위한 플레이스홀더 생성
    status_placeholder = st.empty()
    image_placeholder = st.empty()
    error_placeholder = st.empty()

    # 상태 표시
    status_text = "검사 대기 중..." if st.session_state.running else "검사 중지됨"
    status_placeholder.info(status_text)

    # 오류 카운트가 너무 많으면 리셋
    if st.session_state.error_count > 10:
        st.session_state.error_count = 0
        st.session_state.processed_files = set()
        error_placeholder.warning("오류가 많이 발생하여 상태를 초기화했습니다. 문제가 지속되면 관리자에게 문의하세요.")
        time.sleep(2)
        error_placeholder.empty()

    # 검사 시작
    if st.session_state.running:
        try:
            # 현재 디렉토리 상태 확인
            try:
                current_files = set(os.listdir(watch_directory))
            except Exception as e:
                st.error(f"디렉토리 읽기 실패: {str(e)}")
                st.session_state.error_count += 1
                return
            
            # 새로운 파일 찾기
            new_files = current_files - st.session_state.processed_files
            
            # jpg, jpeg, png 확장자만 필터링
            new_image_files = [file for file in new_files if file.lower().endswith(('.jpg', '.jpeg', '.png'))]

            if new_image_files:
                for file_name in sorted(new_image_files):
                    file_path = os.path.join(watch_directory, file_name)

                    # 파일이 완전히 저장되었는지 확인
                    if is_file_ready(file_path):
                        try:
                            # 이전에 표시 중이던 파일 정리
                            if st.session_state.displaying_file and os.path.exists(st.session_state.displaying_file):
                                try:
                                    # 이전 파일이 이미 처리 디렉토리로 이동되었는지 확인
                                    if os.path.dirname(st.session_state.displaying_file) == watch_directory:
                                        target_path = os.path.join(processed_directory, os.path.basename(st.session_state.displaying_file))
                                        # 대상 경로에 같은 이름의 파일이 있는지 확인
                                        if os.path.exists(target_path):
                                            # 동일 이름 파일이 있으면 이름 변경
                                            basename, ext = os.path.splitext(os.path.basename(st.session_state.displaying_file))
                                            target_path = os.path.join(processed_directory, f"{basename}_{int(time.time())}{ext}")
                                        
                                        shutil.move(st.session_state.displaying_file, target_path)
                                except Exception as e:
                                    error_placeholder.warning(f"이전 파일 처리 중 오류: {str(e)}")
                                    st.session_state.error_count += 1

                            # 새 파일을 열어서 화면에 표시
                            try:
                                image = Image.open(file_path)
                                
                                # 파일 이름에서 확장자 제거 (전처리)
                                file_name_without_ext = os.path.splitext(file_name)[0]
                                
                                # 현재 시간 정보 추가
                                current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                caption = f"{file_name_without_ext} - 검사 시간: {current_time}"
                                
                                image_placeholder.image(image, caption=caption, use_container_width=True)
                                
                                # 현재 파일 경로 저장
                                st.session_state.displaying_file = file_path
                                
                                # 처리된 파일 리스트에 추가
                                st.session_state.processed_files.add(file_name)
                                
                                # 상태 업데이트
                                status_placeholder.success(f"새 이미지 검사 중: {file_name_without_ext}")
                                
                                # 오류 카운트 리셋 (정상 동작 중)
                                st.session_state.error_count = 0
                                
                                # 바로 다음 파일로 넘어가지 않고 잠시 대기
                                # 사용자가 현재 이미지를 확인할 시간 제공
                                time.sleep(0.5)  # 화면 표시 시간 조정
                                break  # 한 번에 하나의 이미지만 처리
                                
                            except Exception as e:
                                error_placeholder.error(f"이미지 표시 중 오류: {file_name}, 오류: {str(e)}")
                                st.session_state.error_count += 1
                                # 오류 발생시에도 처리된 것으로 표시하여 다음 파일 확인
                                st.session_state.processed_files.add(file_name)
                                
                        except Exception as e:
                            error_placeholder.error(f"파일을 처리할 수 없습니다: {file_name}, 에러: {str(e)}")
                            st.session_state.error_count += 1
                            st.session_state.processed_files.add(file_name)  # 오류 발생 시에도 처리된 것으로 표시
            
            # 주기적으로 처리된 파일 목록 리셋 (너무 많아지면 메모리 문제 발생 가능)
            current_time = time.time()
            if current_time - st.session_state.last_check_time > 300:  # 5분마다 리셋
                # 이미 processed_directory로 이동된 파일은 제외하고 리셋
                try:
                    watch_files = set(os.listdir(watch_directory))
                    st.session_state.processed_files = st.session_state.processed_files.intersection(watch_files)
                    st.session_state.last_check_time = current_time
                except Exception as e:
                    error_placeholder.error(f"파일 목록 정리 중 오류: {str(e)}")
                    st.session_state.error_count += 1
                
        except Exception as e:
            error_placeholder.error(f"파일 감시 중 오류 발생: {str(e)}")
            st.session_state.error_count += 1

if __name__ == "__main__":
    app()