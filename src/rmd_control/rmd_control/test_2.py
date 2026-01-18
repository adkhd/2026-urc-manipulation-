import sys
import time
import math

# [사용자 라이브러리 경로]
from can_interface import bringup_can_interface
from rmd_driver_4_3 import RMDMotorDriver as rmd43
from rmd_driver_4_01 import RMDMotorDriver as rmd401

# --- 전역 설정 ---
MOTOR_IDS = [1, 2, 3, 4, 5]
SPEED_LIMIT = 60  # dps (테스트용 안전 속도)

def get_fine_angle(mid, raw_angle):
    """
    모터의 Raw Angle(엔코더값)을 관절의 Fine Angle(실제 각도)로 변환
    """
    if mid == 2:
        return raw_angle + 90.0
    elif mid == 3:
        return -raw_angle + 90.0  # (direction -1 고려)
    else:
        return raw_angle

def get_raw_angle_from_fine(mid, fine_angle):
    """
    목표 Fine Angle(실제 각도)을 모터에 보낼 Raw Angle로 변환
    """
    if mid == 2:
        return fine_angle - 90.0
    elif mid == 3:
        return 90.0 - fine_angle
    else:
        return fine_angle

def print_current_status(motors):
    """현재 모든 관절의 각도를 출력"""
    angles = []
    for i, motor in enumerate(motors):
        motor.read_multi_turn_angle()
        time.sleep(0.005) # 통신 딜레이
        mid = MOTOR_IDS[i]
        fine = get_fine_angle(mid, motor.state['multi_angle'])
        angles.append(f"{fine:.2f}")
    print(f"[현재 각도] {angles}")

def move_to_ready(motors):
    """
    [초기 자세 이동]
    순차적으로 J1 -> J5 이동하여 충돌 방지
    목표: [0, 110, 140, 0, 0]
    """
    target_ready = [0, 110, 140, 0, 0]
    print("\n>>> Ready Pose 이동 시작 [0, 110, 140, 0, 0]...")

    for i, target in enumerate(target_ready):
        motor = motors[i]
        mid = MOTOR_IDS[i]
        
        # 목표 Raw 각도 계산
        raw_target = get_raw_angle_from_fine(mid, target)
        
        print(f"모터 {mid} 이동 중... (목표: {target}도)")
        motor.pos_control(raw_target, SPEED_LIMIT)
        
        # 도달 대기 루프
        start_time = time.time()
        while time.time() - start_time < 5.0: # 5초 타임아웃
            motor.read_multi_turn_angle()
            time.sleep(0.05)
            
            curr_fine = get_fine_angle(mid, motor.state['multi_angle'])
            if abs(target - curr_fine) < 1.0: # 오차 1도 이내 성공
                break
        else:
            print(f"경고: 모터 {mid} 시간 내 도달 실패!")
            
    print(">>> Ready Pose 완료.\n")

def move_all_joints(motors, target_angles):
    """
    [동시 제어]
    5개의 목표 각도 리스트를 받아 모든 모터를 동시에 움직임
    """
    print(f">>> 이동 명령 전송: {target_angles}")
    for i, target in enumerate(target_angles):
        mid = MOTOR_IDS[i]
        raw_target = get_raw_angle_from_fine(mid, target)
        motors[i].pos_control(raw_target, SPEED_LIMIT)

def main():
    # 1. CAN 설정
    DEVICE_PORT = "/dev/canable0" 
    if not bringup_can_interface(tty=DEVICE_PORT):
        print("CAN 설정 실패. 종료합니다.")
        sys.exit(1)

    # 2. 모터 객체 생성 및 연결
    print("모터 연결 중...")
    try:
        m1 = rmd401(channel='can0', motor_id=1)
        m2 = rmd43(channel='can0', motor_id=2)
        m3 = rmd43(channel='can0', motor_id=3)
        m4 = rmd401(channel='can0', motor_id=4)
        m5 = rmd43(channel='can0', motor_id=5)
        motors = [m1, m2, m3, m4, m5]

        # 통신 확인
        for m in motors:
            m.read_motor_status_1()
            time.sleep(0.1)
            if m.state['error_code'] != 0:
                print(f"모터 에러 발생! ID: {m.motor_id}")
                sys.exit(1)
        print("모터 연결 성공.")

        # 3. 오프셋 설정 (Motor 4)
        # 중요: 현재 값을 읽고 오프셋을 계산해야 함
        m4.read_multi_turn_angle()
        time.sleep(0.1)
        current_m4_raw = m4.state['multi_angle']
        m4.angle_offset = -130.65 + current_m4_raw
        print(f"Motor 4 오프셋 설정 완료: {m4.angle_offset:.2f}")

        # 4. 초기 상태 출력
        print_current_status(motors)

        # 5. Ready Pose 이동
        input("Enter를 누르면 Ready Pose로 이동합니다...")
        move_to_ready(motors)
        print_current_status(motors)

        # 6. 사용자 입력 제어 루프
        while True:
            try:
                user_input = input("\n이동할 관절 각도 5개를 입력하세요 (예: 0 110 140 30 0) 또는 'q' 종료: ")
                if user_input.lower() == 'q':
                    break
                
                # 입력 파싱
                targets = list(map(float, user_input.split()))
                if len(targets) != 5:
                    print("오류: 5개의 각도를 공백으로 구분해서 입력해주세요.")
                    continue
                
                # 이동 실행
                move_all_joints(motors, targets)
                
                # 이동 확인용 잠시 대기 후 상태 출력
                time.sleep(2.0)
                print_current_status(motors)

            except ValueError:
                print("오류: 숫자를 입력해주세요.")

    except KeyboardInterrupt:
        print("\n사용자 중단.")
    
    except Exception as e:
        print(f"\n오류 발생: {e}")

    finally:
        print("\n모터 정지 및 종료 중...")
        for m in motors:
            m.stop_motor()

if __name__ == "__main__":
    main()