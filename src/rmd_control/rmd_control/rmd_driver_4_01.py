import can
import struct
import threading
import time
import sys
from collections import deque

import rmd_control.can_interface as can_interface

class RMDMotorDriver:
    PID_SCALE = {
        "curr_kp": 3.0 / 256,
        "curr_ki": 0.1 / 256,
        "spd_kp": 0.1 / 256,
        "spd_ki": 0.01 / 256,
        "pos_kp": 0.1 / 256,
        "pos_ki": 0.01 / 256
    }
    
    ACCEL_INDEX_MAP = {
        0x00: "Position Accel",
        0x01: "Position Decel",
        0x02: "Speed Accel",
        0x03: "Speed Decel"
    }

    RUN_MODE_MAP = {
        0x01: "Current Loop",
        0x02: "Speed Loop",
        0x03: "Position Loop"
    }

    def __init__(self, channel='can0', motor_id=1, debug=False):
        self.channel = channel
        self.id = motor_id
        self.angle_offset = 0
        self.req_id = 0x140 + motor_id
        self.res_id = 0x240 + motor_id
        self.debug = debug
        
        self.state_lock = threading.Lock()
        
        self.stats = {
            "total_sent": 0,
            "total_received": 0,
            "last_received_time": 0,
            "receive_errors": 0
        }
        
        self.msg_buffer = deque(maxlen=100)
        
        self.state = {
            "temp": 0,
            "mos_temp": 0,
            "brake_released": 0,
            "voltage": 0.0,
            "current": 0.0,
            "speed": 0,
            "angle": 0,
            "multi_angle": 0.0,
            "single_angle": 0.0,
            "phase_currents": [0.0, 0.0, 0.0],
            "run_mode": "Unknown",
            "power": 0.0,
            "runtime_ms": 0,
            "sw_date": 0,
            "model_name": "",
            "enc_pos": 0,
            "enc_raw": 0,
            "enc_offset": 0,
            "enc_multi_pos": 0,
            "enc_multi_raw": 0,
            "enc_multi_offset": 0,
            "pid": {
                "curr_kp": 0, "curr_ki": 0,
                "spd_kp": 0, "spd_ki": 0,
                "pos_kp": 0, "pos_ki": 0
            },
            "error_code": 0,
            "error_string": "Normal",
            "last_update": 0,
            "update_count": 0
        }
        
        self.running = True
        
        try:
            self.bus = can.interface.Bus(channel=self.channel, bustype='socketcan')
        except OSError:
            print(f"[Driver Error] {self.channel} 인터페이스를 찾을 수 없습니다.")
            sys.exit(1)

        # [변경점] 수동 스레드 대신 Notifier 사용
        # 버퍼링과 수신 처리를 라이브러리에 위임하여 누락 방지
        self.notifier = can.Notifier(self.bus, [self._on_message_received])
        
        time.sleep(0.1)
        print(f"[Driver] 모터 ID {self.id} 초기화 완료 (Notifier 모드)")

    def _on_message_received(self, msg):
        """
        [변경점] can.Notifier에 의해 호출되는 콜백 함수
        메시지가 도착하자마자 즉시 실행됩니다.
        """
        if not self.running:
            return

        # ID 필터링
        if msg.arbitration_id != self.res_id:
            return

        try:
            # 통계 업데이트
            with self.state_lock:
                self.stats["total_received"] += 1
                self.stats["last_received_time"] = time.time()
            
            # 메시지 버퍼 저장
            self.msg_buffer.append({
                'time': time.time(),
                'cmd': msg.data[0],
                'data': bytes(msg.data)
            })
            
            # 디코딩 (내부에서 state_lock 사용함)
            self._decode_packet(msg.data)
            
            if self.debug:
                print(f"[RX] ID:{self.id} CMD:0x{msg.data[0]:02X}")
                
        except Exception as e:
            print(f"[Error] Motor {self.id} 처리 중 에러: {e}")
            with self.state_lock:
                self.stats["receive_errors"] += 1

    def _decode_packet(self, data):
        """프로토콜 응답 파싱"""
        cmd = data[0]
        
        with self.state_lock:
            self.state["last_update"] = time.time()
            self.state["update_count"] += 1
            
            if cmd in [0x30, 0x31, 0x32]:
                self.state["pid"]["curr_kp"] = data[2]
                self.state["pid"]["curr_ki"] = data[3]
                self.state["pid"]["spd_kp"]  = data[4]
                self.state["pid"]["spd_ki"]  = data[5]
                self.state["pid"]["pos_kp"]  = data[6]
                self.state["pid"]["pos_ki"]  = data[7]

            elif cmd in [0x9C, 0xA1, 0xA2, 0xA4]:
                self.state["temp"] = data[1]
                iq_raw = struct.unpack('<h', data[2:4])[0]
                self.state["current"] = iq_raw * 0.01
                self.state["speed"] = struct.unpack('<h', data[4:6])[0]
                self.state["angle"] = struct.unpack('<h', data[6:8])[0] - self.angle_offset

            elif cmd == 0x9A:
                self.state["temp"] = data[1]
                self.state["mos_temp"] = data[2]
                self.state["brake_released"] = data[3]
                vol_raw = struct.unpack('<H', data[4:6])[0]
                self.state["voltage"] = vol_raw * 0.1
                self.state["error_code"] = struct.unpack('<H', data[6:8])[0]
                self.state["error_string"] = self._decode_error_string(self.state["error_code"])

            elif cmd == 0x9D:
                self.state["temp"] = data[1]
                iA = struct.unpack('<h', data[2:4])[0] * 0.01
                iB = struct.unpack('<h', data[4:6])[0] * 0.01
                iC = struct.unpack('<h', data[6:8])[0] * 0.01
                self.state["phase_currents"] = [iA, iB, iC]

            elif cmd == 0x92:
                raw_angle = struct.unpack('<i', data[4:8])[0]
                self.state["multi_angle"] = raw_angle * 0.01 - self.angle_offset

            elif cmd == 0x94:
                raw_angle = struct.unpack('<H', data[6:8])[0]
                self.state["single_angle"] = raw_angle * 0.01
            
            elif cmd == 0x90:
                self.state["enc_pos"] = struct.unpack('<H', data[2:4])[0]
                self.state["enc_raw"] = struct.unpack('<H', data[4:6])[0]
                self.state["enc_offset"] = struct.unpack('<H', data[6:8])[0]

            elif cmd == 0x70:
                mode = data[7]
                self.state["run_mode"] = self.RUN_MODE_MAP.get(mode, f"Unknown({mode})")

            elif cmd == 0x71:
                power_raw = struct.unpack('<H', data[6:8])[0]
                self.state["power"] = power_raw * 0.1

            elif cmd == 0xB1:
                self.state["runtime_ms"] = struct.unpack('<I', data[4:8])[0]

            elif cmd == 0xB2:
                self.state["sw_date"] = struct.unpack('<I', data[4:8])[0]

            elif cmd == 0xB5:
                try:
                    model = data[1:8].decode('ascii').strip()
                    self.state["model_name"] = model
                except: 
                    pass

            elif cmd == 0x60:
                self.state["enc_multi_pos"] = struct.unpack('<i', data[4:8])[0]

            elif cmd == 0x61:
                self.state["enc_multi_raw"] = struct.unpack('<i', data[4:8])[0]

            elif cmd == 0x62:
                self.state["enc_multi_offset"] = struct.unpack('<i', data[4:8])[0]

            elif cmd == 0x64:
                new_offset = struct.unpack('<i', data[4:8])[0]
                self.state["enc_multi_offset"] = new_offset

    def _decode_error_string(self, code):
        if code == 0: return "Normal"
        errors = []
        if code & 0x0002: errors.append("Stall")
        if code & 0x0004: errors.append("Low Voltage")
        if code & 0x0008: errors.append("Over Voltage")
        if code & 0x0010: errors.append("Over Current")
        if code & 0x0040: errors.append("Power Overrun")
        if code & 0x0080: errors.append("Calib Param Error")
        if code & 0x0100: errors.append("Speeding")
        if code & 0x1000: errors.append("Motor OverTemp")
        if code & 0x2000: errors.append("Enc Calib Error")
        return ", ".join(errors)
        
    def _send(self, data):
        msg = can.Message(arbitration_id=self.req_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            with self.state_lock:
                self.stats["total_sent"] += 1
            
            if self.debug:
                print(f"[TX] ID:{self.id} CMD:0x{data[0]:02X}")
            
            # 버스 부하를 줄이기 위한 최소한의 지연
            time.sleep(0.001)
            return True
        except can.CanError as e:
            print(f"[Warning] Motor {self.id}: CAN 전송 실패: {e}")
            return False

    # --- 기존 메서드 유지 (t_control, vel_control 등) ---
    def t_control(self, curr):
        val = int(curr * 100)
        val = max(-100, min(100, val))
        data = bytearray([0xA1, 0, 0, 0]) + struct.pack('<h', val) + bytearray([0, 0])
        return self._send(data)
    
    def vel_control(self, speed_dps):
        val = int(speed_dps * 100)
        data = bytearray([0xA2, 0, 0, 0]) + struct.pack('<i', val)
        return self._send(data)

    def pos_control(self, angle_deg, max_speed_dps):
        angle_val = int((angle_deg + self.angle_offset) * 100)
        speed_val = int(max_speed_dps)
        data = bytearray([0xA4, 0]) + struct.pack('<H', speed_val) + struct.pack('<i', angle_val)
        return self._send(data)

    def stop_motor(self):
        return self._send([0x81, 0, 0, 0, 0, 0, 0, 0])

    def shutdown_motor(self):
        return self._send([0x80, 0, 0, 0, 0, 0, 0, 0])

    def read_pid(self):
        return self._send([0x30, 0, 0, 0, 0, 0, 0, 0])
    
    def read_multi_turn_angle(self):
        return self._send([0x92, 0, 0, 0, 0, 0, 0, 0])
    
    def read_motor_status_2(self):
        return self._send([0x9C, 0, 0, 0, 0, 0, 0, 0])

    def read_motor_status_1(self):
        return self._send([0x9A, 0, 0, 0, 0, 0, 0, 0])

    def get_state(self):
        with self.state_lock:
            return self.state.copy()

    def get_current(self):
        with self.state_lock:
            return self.state["current"]

    def get_speed(self):
        with self.state_lock:
            return self.state["speed"]

    def get_angle(self):
        with self.state_lock:
            return self.state["angle"]

    def get_multi_angle(self):
        with self.state_lock:
            return self.state["multi_angle"]

    def get_stats(self):
        with self.state_lock:
            return self.stats.copy()

    def is_data_fresh(self, max_age_sec=0.5):
        with self.state_lock:
            age = time.time() - self.state["last_update"]
            return age < max_age_sec

    def set_active_reply(self, cmd_byte, enable, interval_ms):
        enable_byte = 0x01 if enable else 0x00
        interval_val = int(interval_ms // 10)
        data = bytearray([0xB6, cmd_byte, enable_byte, 0, 0, 0, 0, 0])
        struct.pack_into('<H', data, 3, interval_val)
        return self._send(data)

    def perform_homing_task(self, homing_speed_dps, current_limit, timeout_sec=15.0):
        start_time = time.time()
        stall_count = 0
        stall_threshold = 15
        last_angle = None
        is_homed = False
        
        print(f"[Homing] 시작 - ID:{self.id}, 속도:{homing_speed_dps}dps, 전류제한:{current_limit}A")

        self.set_active_reply(0x9C, True, 50)
        time.sleep(0.1)
        
        self.vel_control(homing_speed_dps)
        time.sleep(0.3)

        while (time.time() - start_time) < timeout_sec:
            curr_iq = self.get_current()
            curr_spd = self.get_speed()
            curr_angle = self.get_multi_angle()
            
            if not self.is_data_fresh(0.2):
                print("[Warning] 데이터 수신 지연됨")
                self.read_motor_status_2()
                self.read_multi_turn_angle()
                time.sleep(0.05)
                continue

            if last_angle is not None:
                diff = abs(curr_angle - last_angle)
                if diff <= 0.02:
                    stall_count += 1
                else:
                    stall_count = 0
            
            is_over_current = (abs(curr_iq) > current_limit)
            is_speed_dropped = (abs(curr_spd) < 5)

            if self.debug:
                print(f"Iq:{curr_iq:.2f}A Spd:{curr_spd}dps Ang:{curr_angle:.2f}° Stall:{stall_count}")

            if ((stall_count >= stall_threshold) or is_over_current) and is_speed_dropped:
                print(f"[Homing] 벽 도달 감지!")
                is_homed = True
                break
            
            last_angle = curr_angle
            time.sleep(0.05)

        self.set_active_reply(0x9C, False, 0)
        self.stop_motor()
        time.sleep(0.5)

        if is_homed:
            self.shutdown_motor()
            time.sleep(1)
            final_angle = self.get_multi_angle()
            self.angle_offset = final_angle
            self.stop_motor()
            print(f"[Homing] 성공! 오프셋: {self.angle_offset:.2f}")
            return True
        
        print(f"[Homing] 실패: 타임아웃")
        return False

    def print_stats(self):
        stats = self.get_stats()
        state = self.get_state()
        print(f"\n=== 모터 {self.id} 통계 ===")
        print(f"전송: {stats['total_sent']} / 수신: {stats['total_received']}")
        print(f"업데이트 횟수: {state['update_count']}")
        print(f"마지막 수신: {time.time() - stats['last_received_time']:.3f}초 전")
        print(f"수신 에러: {stats['receive_errors']}")
        print(f"버퍼 크기: {len(self.msg_buffer)}")

    def close(self):
        self.running = False
        self.stop_motor()
        # Notifier 종료
        if hasattr(self, 'notifier'):
            self.notifier.stop()
        print(f"[Driver] 모터 {self.id} 드라이버 종료")

if __name__ == "__main__":
    # 실행 코드는 동일하게 유지
    DEVICE_PORT = "/dev/canable0"
    
    if not can_interface.bringup_can_interface(tty=DEVICE_PORT):
        print("하드웨어 설정 실패")
        sys.exit(1)

    motor1 = RMDMotorDriver(channel='can0', motor_id=1, debug=False)
    
    try:
        print("\n=== 비동기 테스트 ===")
        print("Active Reply 활성화...")
        motor1.set_active_reply(0x9C, True, 100)
        
        for i in range(50):
            current = motor1.get_current()
            speed = motor1.get_speed()
            angle = motor1.get_multi_angle()
            print(f"[{i}] Curr:{current:.2f}A, Spd:{speed}dps, Ang:{angle:.2f}°")
            time.sleep(0.1)
        
        motor1.print_stats()
        motor1.set_active_reply(0x9C, False, 0)
        
    except KeyboardInterrupt:
        print("\n사용자 중단")
    finally:
        motor1.close()