
#!/usr/bin/env python3
import rclpy
import DR_init
import time, math
import numpy as np


# Robot configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 700, 700

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

# 측정한 3개 꼭지점 (각각 [x, y, z, rx, ry, rz])
P1 = [366.724, 158.545, 108.596, 39.169, -179.997, 38.687]  # Cup1 (왼쪽 밑)
P2 =[366.502, -15.007, 108.483, 3.544, -179.979, 3.076]  # Cup3 (오른쪽 밑)
P3 = [525.536, 71.982, 108.366, 7.552, -179.971, 7.062]      # Cup6 (위쪽 꼭짓점)

# (cup_life_pos는 예시용)
cup_life_pos = [468.725, -220.685, 102.738, 176.625, -179.132, 175.601]

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cup_pyramid_stack", namespace=ROBOT_ID)
    DR_init.__dsr__node = node


    # Z축 정렬 기능을 위해 필요한 import
    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_current_posx,
            set_tool,
            set_tcp,
            movej,
            movel,
            set_robot_mode,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            release_compliance_ctrl,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            mwait,
            wait,
            trans,
            parallel_axis,  # Z축 정렬에 필요한 함수 추가
            amovej,
            amovel
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing modules: {e}")
        return
    
    # 기본 설정
    set_tool("Tool Weight")
    set_tcp("GripperDA_v1")
    set_robot_mode(1)
    JReady = [0, 0, 90, 0, 90, 0]


    # 컵 픽업 위치 (예시)
    pick_pos = posx([424.191, -216.679, 256.107, 159.448, 179.97, 160.182])

   

    # 1층 배치 계산: 3개 꼭지점을 이용해 삼각형 형태로 6개 배치
    def midpoint(a, b):
        return [(a[i] + b[i]) / 2.0 for i in range(6)]
    def average_points(points):
        n = len(points)
        return [sum(p[i] for p in points) / n for i in range(6)]
    
    cup1 = P1[:]
    cup3 = P2[:]
    cup6 = P3[:]
    cup2 = midpoint(P1, P2)
    cup4 = midpoint(P1, P3)
    cup5 = midpoint(P2, P3)
    floor1 = [cup1, cup2, cup3, cup6, cup4, cup5 ]

    print("\n--- 1층 Cup Positions ---")
    for i, pos in enumerate(floor1):
        print(f"Cup {i+1}: {pos}")

    # 2층 계산 (3개 컵)
    cup_z_offset = 80.0
    cup7 = average_points([cup4, cup5, cup6])
    cup7[2] += cup_z_offset
    cup8 = average_points([cup1, cup2, cup4])
    cup8[2] += cup_z_offset
    cup9 = average_points([cup2, cup3, cup5])
    cup9[2] += cup_z_offset
    floor2 = [cup8, cup7, cup9]

    # 중앙 좌표 (3층, 4층 공통)
    cx = (P1[0] + P2[0] + P3[0]) / 3.0
    cy = (P1[1] + P2[1] + P3[1]) / 3.0

    # 3층 계산 (1개 컵, z + 160mm)
    floor3 = [[cx, cy, (P1[2]+P2[2]+P3[2])/3.0 + 2*cup_z_offset] + P1[3:]]
    # 4층 계산 (1개 컵, z + 240mm) → 별도 처리 (특수 모드)
    floor4 = [[cx, cy, (P1[2]+P2[2]+P3[2])/3.0 + 4*cup_z_offset] + P1[3:]]

    # 일반 처리할 컵: 1층+2층+3층 (총 10개)
    final_positions_normal = floor1 + floor2 + floor3

    print("\n--- 최종 배치 좌표 ---")
    current_index = 1
    print("1층:")
    for pos in floor1:
        print(f"  Cup {current_index}: {pos}")
        current_index += 1
    print("2층:")
    for pos in floor2:
        print(f"  Cup {current_index}: {pos}")
        current_index += 1
    print("3층:")
    for pos in floor3:
        print(f"  Cup {current_index}: {pos}")
        current_index += 1
    print("4층 (특수 처리):")
    print(f"  Cup {current_index}: {floor4[0]}")

    # --- 픽업 및 배치 함수 ---
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
        
    def release_short():
        print("Opening gripper quickly...")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def simple_z_axis_alignment():
        """
        로봇 툴의 Z축을 간단히 아래 방향으로 정렬합니다.
        이 함수는 간편하게 호출할 수 있는 래퍼입니다.
        """
        print("Z축을 아래 방향으로 정렬합니다...")
        vect = [0, 0, -1]
        parallel_axis(vect, DR_AXIS_Z, DR_BASE)
        mwait()
        print("Z축 정렬 완료")
        # 정렬 후 현재 위치 출력
        current_pos = get_current_posx()[0]
        print(f"정렬 후 현재 위치: {current_pos}")
        return current_pos

    def detect_and_pick(pick_position):
        print(f"Moving to pick position: {pick_position}")
        movel(posx(list(pick_position)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        mwait()
        print("Enabling force control for pickup...")
        task_compliance_ctrl(stx=[500,500,500,100,100,100])
        set_desired_force(fd=[0,0,-40,0,0,0],
                          dir=[0,0,1,0,0,0],
                          mod=DR_FC_MOD_REL)
        start_time = time.time()
        timeout = 10.0
        contact_detected = False
        detected_pos = None
        while time.time() - start_time < timeout:
            if check_force_condition(DR_AXIS_Z, max=3):
                contact_detected = True
                detected_pos = list(get_current_posx()[0])
                break
            wait(0.1)
        release_compliance_ctrl()
        if contact_detected and detected_pos is not None:
            print("Force detected during pickup!")
            temp_pos = [detected_pos[0],
                        detected_pos[1],
                        detected_pos[2] + 10,
                        detected_pos[3],
                        detected_pos[4],
                        detected_pos[5]]
            movel(posx(temp_pos), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            mwait()
            release()
            wait(1)
            new_pick_pos = [detected_pos[0],
                            detected_pos[1],
                            # detected_pos[2] - 16,
                            detected_pos[2] - 17,
                            detected_pos[3],
                            detected_pos[4],
                            detected_pos[5]]
            movel(posx(new_pick_pos), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            mwait(1)

            simple_z_axis_alignment()
            grip()
            mwait(1)
            lifted_pos = [detected_pos[0],
                          detected_pos[1],
                          detected_pos[2] + 150,
                          detected_pos[3],
                          detected_pos[4],
                          detected_pos[5]]
            movel(posx(lifted_pos), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            mwait()
            movingm = trans(lifted_pos, [0,200,0,0,0,0])
            movel((movingm), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            mwait()
            return detected_pos
        else:
            print("No force detected during pickup.")
            return None
        


    def place_cup(dest_position, layer=1):
        print(f"Placing cup at: {dest_position} (Layer {layer})")
        
        # Set approach height based on layer
        z_offset = -15 if layer == 1 else 10
        
        approach_pos = [dest_position[0],
                        dest_position[1],
                        dest_position[2] + z_offset,
                        dest_position[3],
                        dest_position[4],
                        dest_position[5]]
        
        movel(posx(approach_pos), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        mwait()
        wait(1)
        
        print("Enabling force control for placement...")
        task_compliance_ctrl(stx=[500,500,500,100,100,100])
        set_desired_force(fd=[0,0,-20,0,0,0],
                        dir=[0,0,1,0,0,0],
                        mod=DR_FC_MOD_REL)
        
        start_time = time.time()
        timeout = 5.0
        contact_detected = False
        
        while time.time() - start_time < timeout:
            if check_force_condition(DR_AXIS_Z, max=7):
                contact_detected = True
                break
            wait(0.1)
        
        release_compliance_ctrl()
        
        if contact_detected:
            print("Force detected during placement! Releasing cup.")
            release()
            wait(1)
        else:
            print("No force detected during placement.")
        
        # Modify lift behavior based on layer
        if layer == 3:
            # For 3rd layer: First move up by z+50
            intermediate_pos = [dest_position[0],
                            dest_position[1],
                            dest_position[2] + 50,
                            dest_position[3],
                            dest_position[4],
                            dest_position[5]]
            movel(posx(intermediate_pos), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            mwait()
            
            # Then move up to standard height
            lifted_pos = [dest_position[0],
                        dest_position[1] ,
                        dest_position[2] + 180,
                        dest_position[3],
                        dest_position[4],
                        dest_position[5]]
        else:
            # For other layers: Standard lift
            lifted_pos = [dest_position[0],
                        dest_position[1],
                        dest_position[2] + 180,
                        dest_position[3],
                        dest_position[4],
                        dest_position[5]]
        
        movel(posx(lifted_pos), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        mwait()
        
        # Move back using trans (standard for all layers)
        backhome = trans(lifted_pos, [0, -200, 0, 0, 0, 0])
        movel(backhome, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        mwait()
        
    
    #4층을 위한거
    def detect_cup(pick_position):
        print(f"Moving to pick position for detection: {pick_position}")
        movel(posx(list(pick_position)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        mwait()
        grip()
        print("Enabling force control for detection...")
        task_compliance_ctrl(stx=[500,500,500,100,100,100])
        set_desired_force(fd=[0,0,-40,0,0,0],
                          dir=[0,0,1,0,0,0],
                          mod=DR_FC_MOD_REL)
        start_time = time.time()
        timeout = 5.0
        detected_pos = None
        while time.time() - start_time < timeout:
            if check_force_condition(DR_AXIS_Z, max=5):
                detected_pos = list(get_current_posx()[0])
                break
            wait(0.1)
        release_compliance_ctrl()
        if detected_pos is not None:
            print("Force detected during detection!")
            temp_pos = [detected_pos[0],
                        detected_pos[1],
                        detected_pos[2] + 190,
                        detected_pos[3],
                        detected_pos[4],
                        detected_pos[5]]
            movel(posx(temp_pos), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            mwait()
            release()
            wait(1)
            return detected_pos
        else:
            print("No force detected during detection.")
            return None

    # Special Mode 함수: 4층 특수 동작 수행
    def special_mode(current_pick, pick_pos, detected_pos):
        """
        1층~3층까지 쌓은 후, special_mode 함수가 호출됩니다.
        pick_pos에서 force를 주어 detect가 되면,
        modified_arm_pos로 이동 후 grip, 
        trans()를 이용해 z축으로 70mm 올리고,
        via_point1, via_point2로 순차 이동,
        다시 force를 적용하여 detect 후 release,
        via_point1로 이동한 후 최종적으로 pick_pos로 복귀하는 동작을 수행합니다.
        """
        print("Entering Special Mode for 4층 stacking...")
        # 1. pick_pos에서 force 감지 (detect_cup 사용)
        last_detected_pos = detect_cup(detected_pos)
        if last_detected_pos is None:
            print("Special Mode: No force detected at pick_pos.")
            return current_pick
        # 2. modified_arm_pos로 movej 후 grip 수행

        # 컵 잡기 전
        modified_arm_pos = [-75.231, 46.013, 97.28, 64.242, 105.246, -55.793]
        movej(modified_arm_pos, vel=VELOCITY/3, acc=ACC/3)
        mwait(1)
        wait(1)
        
        # 컵 위치에서 잡는 곳
        modified_arm_pos2 = [-67.503, 45.151, 103.312, 69.665, 99.618, -57.864]
        movej(modified_arm_pos2, vel=VELOCITY/3, acc=ACC/3)
        mwait(1)
        wait(1)
        grip()

        # 3. trans()를 이용해 z축 70mm 올리기
        via_point_05 = [-67.359, 15.551, 87.862, 82.343, 110.765, -12.822]
        movej(via_point_05, vel=VELOCITY/3, acc=ACC/3)
        mwait(1)
        wait(1)
        # 놓을 위치로 접근(경유지) - 돌리면서 앞으로 살짝
        via_point_06 = [-36.169, 25.905, 90.745, 70.836, 121.754, -209.685]
        movej(via_point_06, vel=VELOCITY/3, acc=ACC/3)
        mwait(1)
        wait(1)

        # 4. 최종 놓는 위치
        via_point1 = [-33.002, 23.588, 94.288, 72.732, 117.79, -209.815]
        movej(via_point1, vel=VELOCITY/3, acc=ACC/3)
        mwait(1)
        wait(1)
       
        
        # 6. 다시 force 적용하여 detect 후 release
        print("Applying final force for release...")
        task_compliance_ctrl(stx=[500,500,500,100,100,100])
        set_desired_force(fd=[0,0,-20,0,0,0],
                          dir=[0,0,1,0,0,0],
                          mod=DR_FC_MOD_REL)
        start_time = time.time()
        timeout = 5.0
        final_force = False
        while time.time() - start_time < timeout:
            if check_force_condition(DR_AXIS_Z, max=6):
                final_force = True
                break
            wait(0.1)
        release_compliance_ctrl()

         # 5. via_point2로 이동 (느린 속도) via5랑 똑같음
        
       
        if final_force:
            print("Final force detected, releasing cup.")
            release()
            wait(1)
        else:
            print("No final force detected.")

        left_pos = get_current_posx()[0]
        final_move=trans(left_pos,[0,-30,0,0,0,0])
        movel(final_move, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        mwait()
        wait(1)
        # 7. 다시 via_point1로 이동 (느린 속도)
        via_point2 = [-67.359, 15.551, 87.862, 82.343, 110.765, -12.822]
        movej(via_point2, vel=VELOCITY/3, acc=ACC/3)
        mwait(1)
        wait(1)
        # 8. 최종적으로 pick_pos로 movel 복귀
        movel(pick_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        mwait()
        print("Special Mode complete.")
        return pick_pos
        
    # 진자 던지기 관련 코드 - paste.txt에서 추가한 부분
    def pendulum_throw():
        # 진자 운동을 위한 joint 값
        start = [-6.41, 7.45, 79.075, 0.842, 120.678, -0.826]    # 진자 시작 위치 (뒤로 당긴 상태)
        middle = [-6.41, 7.454, 79.08, 0.842, 90.675, -0.826]    # 진자 중간 위치 (수직 상태)
        result = [-6.41, 7.453, 79.08, 0.842, -50.003, -0.826]   # 진자 끝 위치 (앞으로 내민 상태)
        
        print("Starting pendulum throw sequence")
        grip()
        # 1. 진자 시작 위치로 이동 (천천히)
        print("Moving to start position")
        amovej(start, vel=200, acc=200)
        mwait()
        wait(0.5)  # 잠시 안정화
        
        # 2. 진자 중간 위치로 이동 (천천히)
        print("Moving to middle position")
        amovej(middle, vel=400, acc=400)
        
        # 3. 진자 끝 위치로 빠르게 이동 (던지기 동작)
        print("Executing pendulum throw motion")
        amovej(result, time=2.092455)  # 빠른 속도로 던지기 동작
        
        print("Releasing bottle during pendulum motion")
        
        # 5. 던지기 동작 완료 대기
        print("Waiting for throw motion to complete")
        
        print("Pendulum throw complete")

    # -----------------------------
    # 메인 실행 흐름
    # -----------------------------
    print("\nMoving to ready position...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    mwait()
    print("Moving to initial pick position...", pick_pos)
    movel(posx(list(pick_pos)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
    mwait()
    # last_detected_pos = [0,0,0,0,0,0]
    current_pick = pick_pos

    detected_force = [0,0,0,0,0,0] 
    # 1층+2층+3층의 10개 컵을 일반 처리
    
    ####
    # 1층+2층+3층의 10개 컵을 일반 처리
    for idx, dest_pos in enumerate(final_positions_normal):
        print(f"\n----- Cup #{idx+1} of {len(final_positions_normal)} -----")
        detected_force = detect_and_pick(current_pick) # 
        if detected_force is not None:
            # Determine which layer this cup belongs to
            layer = 1
            if idx >= len(floor1):
                layer = 2
            if idx >= len(floor1) + len(floor2):
                layer = 3
                
            print(f"Placing cup on layer {layer}")
            place_cup(dest_pos, layer)
            
            current_pick = list(detected_force)
            print("Returning to force-detected pick position:", current_pick)
            movel(posx(current_pick), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            mwait()
        else:
            print("Pickup failed, ending cycle.")
            break


    # Special Mode 실행 (4층)
    print("\n----- Special Mode for 4층 -----")
    new_pick = special_mode(current_pick, pick_pos, detected_force)
    if new_pick is not None:
        current_pick = new_pick

    print("\nAll cups have been stacked in pyramid arrangement!")
   
    print("Moving to initial pick position...", pick_pos)
    movel(posx(list(pick_pos)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
    mwait()
    
    # 피라미드 적재 후 진자 던지기 실행
    print("\n\n----- Starting Pendulum Throw Motion -----")
    
    # 물병을 잡을 위치
    pre_grasp_pos = posx([290.431, -231.978, 264.636, 160.804, 178.538, 159.13])  # 물병 접근 전 준비 위치
    grasp_pos = posx([272.905, -229.464, 183.379, 58.66, -177.342, 59.453])    # 물병 잡기 위치
    upright_pos = posx([391.354, -132.982, 270.573, 166.625, 178.553, 164.932])   # 물병 들어올리는 위치
    
    # 1. 물병 잡기 전 동작
    print("Moving to pre-grasp position")
    movel(pre_grasp_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    mwait()
    wait(1)
    
    # 물병 잡을 위치
    print("Moving to grasp position")
    movel(grasp_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    grip()
    mwait()
    wait(1)

    # 물병 잡고 들어올리기
    print("Lifting bottle to upright position")
    movel(upright_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    mwait()
    wait(1)
    
    # 진자 던지기 동작 호출
    print("Executing pendulum throw motion")
    pendulum_throw()
    
    print("Pendulum throw operation complete")
    # 진자 던지기 마무리 - 중간 위치로 돌아가고 물병 놓기
    middle = [-6.41, 7.454, 79.08, 0.842, 90.675, -0.826]
    amovej(middle, time=2.2455)
    wait(0.01)
    release_short()
    mwait()
    wait(1)
    
    # 마지막으로 준비 자세로 돌아가기
    print("\nAll operations complete, returning to ready position")
    movej(JReady, vel=VELOCITY, acc=ACC)
    mwait()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()