#can 인터페이스를 다루는 파일

import subprocess, time, os

def bringup_slcan(tty="/dev/canable0", ifname="can0", serial_baud=1000000, can_bps="s6"):
    """
    serial_baud: 동글의 시리얼 속도 (예: 1000000)
    can_bps: slcand의 CAN 속도 프리셋(s0=10k ... s8=1M). 필요시 slcand 매뉴얼 참조.
    """
    def sh(cmd):
        subprocess.run(cmd, shell=True, check=True)

    # 이미 떠 있으면 내리기
    sh(f"sudo ip link set {ifname} down || true")
    # slcand 죽이기(동일 ifname로 떠있을 수 있으니)
    subprocess.run(f"sudo pkill -f 'slcand.* {tty} {ifname}'", shell=True)

    # slcand 실행 (-o: 오픈, -c: 클로즈 명령 전달, -s*: CAN 속도, -S: 시리얼 속도)
    sh(f"sudo slcand -o -{can_bps} -S{serial_baud} -c {tty} {ifname}")
    time.sleep(0.2)  # 인터페이스 생성 대기

    # 인터페이스 업
    sh(f"sudo ip link set {ifname} up")

# 예: CANable(슬캔 펌웨어), UART 1Mbps, CAN 1Mbps
bringup_slcan("/dev/canable0", "can0", serial_baud=1000000, can_bps="s6")
