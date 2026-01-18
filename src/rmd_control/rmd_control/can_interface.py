import subprocess
import time

import subprocess
import time
import os

def bringup_can_interface(tty="/dev/canable0", ifname="can0", bitrate="s8"):
    """
    CANable(slcand) 장치를 can0 인터페이스로 활성화
    tty: 장치 경로 (예: /dev/ttyACM0, /dev/canable0)
    bitrate: 's8' = 1Mbps (RMD 모터 기본값)
    """
    print(f"[System] {tty} 장치를 {ifname} 인터페이스(1Mbps)로 설정합니다...")

    def run_cmd(cmd):
        try:
            # 명령어 실행 및 에러 무시 (이미 닫혀있는 경우 등)
            subprocess.run(cmd, shell=True, check=True, stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            pass

    # 1. 기존 인터페이스 내리기 및 slcand 프로세스 정리
    run_cmd(f"sudo ip link set {ifname} down")
    run_cmd(f"sudo pkill -f 'slcand.*{tty}'")
    time.sleep(0.5)

    # 2. slcand 실행 (Serial -> CAN 변환)
    # -o: open, -c: close, -s8: 1Mbps, -S: UART 속도 1Mbps
    cmd_slcand = f"sudo slcand -o -c -s8 -S 1000000 {tty} {ifname}"
    
    try:
        subprocess.run(cmd_slcand, shell=True, check=True)
        time.sleep(0.5) # 장치 생성 대기

        # 3. 인터페이스 UP
        subprocess.run(f"sudo ip link set {ifname} up", shell=True, check=True)
        subprocess.run(f"sudo ip link set {ifname} txqueuelen 1000", shell=True, check=True)
        print(f"[System] {ifname} 활성화 완료.")
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"[Error] 인터페이스 설정 실패: {e}")
        print("팁: 장치 경로가 맞는지, USB가 연결되었는지 확인하세요.")
        return False
# 이 파일을 직접 실행했을 때만 동작하는 테스트 코드
if __name__ == "__main__":
    bringup_can_interface()