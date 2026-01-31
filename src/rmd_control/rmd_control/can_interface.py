import subprocess
import time

def bringup_can_interface(ifname="can0", bitrate=1000000):
    print(f"[System] {ifname} 장치를 {bitrate}bps 속도로 설정합니다.")

    # 명령어를 실행하는 내부 함수
    def run_cmd(cmd):
        try:
            # 에러가 나도 무시하고 진행 (이미 꺼져있는 경우 등 대비)
            subprocess.run(cmd, shell=True, check=True, stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            pass

    # 1. 설정 변경을 위해 인터페이스 끄기 (Down)
    # 켜져 있는 상태에서는 속도 변경이 안 되므로 먼저 끕니다.
    run_cmd(f"sudo ip link set {ifname} down")
    time.sleep(0.5)

    try:
        # 2. 비트레이트 설정 (Candlelight 전용 명령어)
        # slcand 대신 ip link set 명령어를 사용합니다.
        cmd_set_bitrate = f"sudo ip link set {ifname} type can bitrate {bitrate}"
        subprocess.run(cmd_set_bitrate, shell=True, check=True)

        # 3. 인터페이스 켜기 (Up)
        subprocess.run(f"sudo ip link set {ifname} up", shell=True, check=True)
        
        # 4. 데이터 전송 큐 길이 늘리기 (패킷 누락 방지)
        subprocess.run(f"sudo ip link set {ifname} txqueuelen 1000", shell=True, check=True)
        
        print(f"[System] {ifname} 활성화 완료 (Candlelight 모드).")
        return True

    except subprocess.CalledProcessError as e:
        print(f"[Error] 인터페이스 설정 실패: {e}")
        print("팁: USB가 연결되었는지 확인하세요.")
        return False

if __name__ == "__main__":
    # RMD 모터 기본 속도인 1Mbps(1000000)로 설정하여 실행
    bringup_can_interface()