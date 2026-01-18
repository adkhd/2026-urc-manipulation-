from can_interface import bringup_can_interface
from rmd_driver_4_3 import RMDMotorDriver as rmd43
from rmd_driver_4_01 import RMDMotorDriver as rmd401
from kinematics_angle import calculate_kinematics as fk, calculate_ik as ik # 필요시 사용

import time
import sys
def cal_fine_motor_angles(motor_angle): 
        #5개의 fine motor angle 리스트 반환
        fine_motor_angle = []
        motor_directions = [1, 1, -1, 1, 1]
        for mid in [1, 2, 3, 4, 5]:
                # Active Reply 덕분에 driver.state['angle']이 최신값임
                if mid in [2, 3]:
                    angle = motor_angle[mid-1] * motor_directions[mid-1] + 90 # J1모터 10도 = 관절 100도 = 10 + 90 #J3 모터 20도 = 관절각 70도 = -20+90
                else:
                    angle = motor_angle[mid-1] * motor_directions[mid-1] 
                fine_motor_angle.append(angle)
        return fine_motor_angle
if __name__ == "__main__":
    
    DEVICE_PORT = "/dev/canable0" 
    
    if not bringup_can_interface(tty=DEVICE_PORT):
        print("하드웨어 설정 실패로 프로그램을 종료합니다.")
        sys.exit(1)

    motor1 = rmd401(channel='can0', motor_id=1)
    motor2 = rmd43(channel='can0', motor_id=2)
    motor3 = rmd43(channel='can0', motor_id=3)
    motor4 = rmd401(channel='can0', motor_id=4)
    motor5 = rmd43(channel='can0', motor_id=5)
    time.sleep(5)
    try:
        
        # motor4.read_multi_turn_angle()
        # motor4.angle_offset = -130.65+motor4.state['multi_angle']
        # time.sleep(0.5)
        # motor_angles = [] 

        # for motor in [motor2, motor3, motor4, motor5]: 
        #     motor.read_multi_turn_angle()
        #     motor.write_acceleration(0x01, 20000)
        #     motor.write_acceleration(0x00, 20000)
        #     motor_angles.append(motor.state['multi_angle'])
        # for motor in [motor1]: 
        #     motor.read_multi_turn_angle()
        #     motor.write_acceleration(0x01, 15000)
        #     motor.write_acceleration(0x00, 15000)
        #     motor_angles.append(motor.state['multi_angle'])
        # for motor in [motor1, motor4]: 
        #     motor.read_multi_turn_angle()
            
        #     motor_angles.append(motor.state['multi_angle'])
        # #motor1.vel_control(-100)
        # #motor2.pos_control(50,20)
        # #motor3.pos_control(-90,20)
        # #print(cal_fine_motor_angles(motor_angles))
        # for motor in [motor1, motor2, motor3, motor4, motor5]: 
        #     motor.read_acceleration(0x01)
        #     motor.read_acceleration(0x00)
        
        #motor1.pos_control(0,100)
        motor2.debug = 0
        
        motor1.vel_control(180)
        time.sleep(16)
        for motor in [motor1, motor2, motor3, motor4, motor5]: 
             motor.read_multi_turn_angle()
    except KeyboardInterrupt:
        print("\n사용자 중단.")
    finally:
        for motor in [motor1, motor2, motor3, motor4, motor5]: 
            print(motor.state['multi_angle'])
            motor.stop_motor()

