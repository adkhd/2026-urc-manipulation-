import can
import struct
import threading
import time
import sys
from collections import deque  # 버퍼 기능을 위해 추가

# 위에서 만든 설정 파일 import
import rmd_control.can_interface as can_interface

class RMDMotorDriver:
    # PID 인덱스 매핑
    PID_MAP = {
        0x01: "Current Loop Kp", 0x02: "Current Loop Ki",
        0x04: "Speed Loop Kp",   0x05: "Speed Loop Ki",
        0x07: "Position Loop Kp",0x08: "Position Loop Ki", 0x09: "Position Loop Kd"
    }
    
    # 가속도 인덱스 매핑
    ACCEL_MAP = {
        0x00: "Pos Plan Accel", 0x01: "Pos Plan Decel",
        0x02: "Speed Plan Accel", 0x03: "Speed Plan Decel"
    }
    
    # 시스템 운영 모드 매핑 (0x70)
    RUN_MODE_MAP = {
        0x01: "Current Loop",
        0x02: "Speed Loop",
        0x03: "Position Loop"
    }

    def __init__(self, channel='can0', motor_id=1, debug=False):
        self.channel = channel
        self.req_id = 0x140 + motor_id
        self.res_id = 0x240 + motor_id
        self.id = motor_id
        self.debug = debug  # 디버그 모드 추가
        
        # 스레드 충돌 방지용 Lock
        self.state_lock = threading.Lock()
        
        # 통신 통계용 변수
        self.stats = {
            "total_sent": 0,
            "total_received": 0,
            "last_received_time": 0,
            "receive_errors": 0
        }
        
        # 최근 메시지 저장용 버퍼
        self.msg_buffer = deque(maxlen=100)

        # 모터 상태 변수
        self.state = {
            "temp": 0,          # 모터 온도
            "mos_temp": 0,      # MOS 온도
            "brake_released": 0,# 브레이크 상태
            "voltage": 0.0,     # 전압
            "current": 0.0,     # 토크 전류 (Iq)
            "speed": 0,         # 속도
            "angle": 0,         # 각도
            "multi_angle": 0.0,
            "phase_currents": [0.0, 0.0, 0.0], # 3상 전류
            "error_code": 0,
            "error_string": "Normal",
            "run_mode": "Unknown",
            "runtime_ms": 0,
            "sw_date": 0,
            "model_name": "",
            "can_id": 0
        }
        
        self.running = True
        
        try:
            self.bus = can.interface.Bus(channel=self.channel, bustype='socketcan')
        except OSError:
            print(f"[Driver Error] {self.channel} 인터페이스를 찾을 수 없습니다.")
            sys.exit(1)

        # 기존 수동 스레드 대신 Notifier 사용
        # 메시지가 오면 즉시 _on_message_received 호출
        self.notifier = can.Notifier(self.bus, [self._on_message_received])
        
        time.sleep(0.1)
        print(f"[Driver] 모터 ID {self.id} 초기화 완료 (Notifier Mode)")

    def _on_message_received(self, msg):
        """
        can.Notifier에 의해 호출되는 콜백 함수
        메시지가 도착하자마자 즉시 실행됩니다.
        """
        if not self.running:
            return

        # ID 필터링 (내 모터의 응답인지 확인)
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

        # 데이터 갱신 시 충돌 방지를 위해 Lock 사용
        with self.state_lock:
            # 0. 제어 응답
            if cmd in [0xA2, 0xA4]:
                self.state["temp"] = data[1]
                self.state["current"] = struct.unpack('<h', data[2:4])[0] * 0.01
                self.state["speed"] = struct.unpack('<h', data[4:6])[0]
                self.state["angle"] = struct.unpack('<h', data[6:8])[0]
                
            # 1. Read Motor Status 1 (0x9A)
            elif cmd == 0x9A:
                self.state["temp"] = data[1]
                self.state["mos_temp"] = data[2]
                self.state["brake_released"] = data[3]

                vol_raw = struct.unpack('<H', data[4:6])[0]
                self.state["voltage"] = vol_raw * 0.1

                self.state["error_code"] = struct.unpack('<H', data[6:8])[0]
                self.state["error_string"] = self._decode_error_string(self.state["error_code"])
                
                if self.debug:
                    print(f"[Status 1] Temp:{self.state['temp']}C, MOS:{self.state['mos_temp']}C, "
                          f"Volt:{self.state['voltage']:.1f}V, Err:{self.state['error_string']}")

            # 2. Read Motor Status 2 (0x9C)
            elif cmd == 0x9C:
                self.state["temp"] = data[1]
                iq_raw = struct.unpack('<h', data[2:4])[0]
                self.state["current"] = iq_raw * 0.01
                self.state["speed"] = struct.unpack('<h', data[4:6])[0]
                self.state["angle"] = struct.unpack('<h', data[6:8])[0]
                
            # 3. Read Motor Status 3 (0x9D)
            elif cmd == 0x9D:
                self.state["temp"] = data[1]
                iA = struct.unpack('<h', data[2:4])[0] * 0.01
                iB = struct.unpack('<h', data[4:6])[0] * 0.01
                iC = struct.unpack('<h', data[6:8])[0] * 0.01
                self.state["phase_currents"] = [iA, iB, iC]
                if self.debug:
                    print(f"[Status 3] Phase A:{iA:.2f}A, B:{iB:.2f}A, C:{iC:.2f}A")

            # 4. System Operating Mode (0x70)
            elif cmd == 0x70:
                mode_code = data[7]
                self.state["run_mode"] = self.RUN_MODE_MAP.get(mode_code, f"Unknown({mode_code})")
                print(f"[System] Current Mode: {self.state['run_mode']}")

            # 5. System Runtime (0xB1)
            elif cmd == 0xB1:
                self.state["runtime_ms"] = struct.unpack('<I', data[4:8])[0]
                hours = self.state["runtime_ms"] / 3600000
                print(f"[System] Runtime: {self.state['runtime_ms']} ms ({hours:.2f} hours)")

            # 6. Software Version Date (0xB2)
            elif cmd == 0xB2:
                self.state["sw_date"] = struct.unpack('<I', data[4:8])[0]
                print(f"[System] SW Date: {self.state['sw_date']}")

            # 7. Motor Model (0xB5)
            elif cmd == 0xB5:
                try:
                    model_str = data[1:8].decode('ascii').strip()
                    self.state["model_name"] = model_str
                    print(f"[System] Model: {model_str}")
                except:
                    print(f"[System] Model Decode Error: {data[1:8]}")

            # 8. CAN ID (0x79)
            elif cmd == 0x79:
                if data[2] == 0x01: # Read flag reply
                    read_id = struct.unpack('<H', data[6:8])[0]
                    self.state["can_id"] = read_id
                    print(f"[System] Current CAN ID: {read_id} (0x{read_id:02X})")
                else:
                    print("[System] CAN ID Write Success (Check reply ID)")

            elif cmd == 0x20: print(f"[Function] Index 0x{data[1]:02X} Command executed.")
            elif cmd == 0x80: print("[Motor] Shutdown Confirmed.")
            elif cmd == 0x81: print("[Motor] Stop Confirmed.")
            elif cmd == 0x76: print("[System] Reset Commanded.")
            elif cmd == 0x77: print("[System] Brake Released.")
            elif cmd == 0x78: print("[System] Brake Locked.")
            elif cmd == 0xB3: print("[Comm] Timeout Protection Set.")
            elif cmd == 0xB4: print("[Comm] Baud Rate Set.")
            
            # 엔코더 관련 응답
            elif cmd == 0x60: # Multi-turn Position
                pos = struct.unpack('<i', data[4:8])[0]
                print(f"[Encoder] Position: {pos}")
            elif cmd == 0x61: # Original Position
                raw_pos = struct.unpack('<i', data[4:8])[0]
                print(f"[Encoder] Raw Position: {raw_pos}")
            elif cmd == 0x64: # Zero Set Result
                offset = struct.unpack('<i', data[4:8])[0]
                print(f"[Encoder] New Zero Offset Set: {offset} (Restart required)")
            elif cmd == 0x94: # Single Turn Angle
                angle = struct.unpack('<H', data[6:8])[0] * 0.01
                print(f"[Angle] Single Turn: {angle:.2f} deg")
            
            # 기타 PID, 엔코더 등
            elif cmd in [0x30, 0x31, 0x32, 0x42, 0x43, 0x90, 0x92]:
                self._decode_generic_replies(cmd, data)

    def _decode_generic_replies(self, cmd, data):
        # 이미 Lock 내부에서 호출되므로 Lock 불필요
        if cmd in [0x30, 0x31, 0x32]:
            pid_index = data[1]
            value = struct.unpack('<f', data[4:8])[0]
            mode = "READ" if cmd == 0x30 else "WRITE"
            print(f"[PID {mode}] {self.PID_MAP.get(pid_index, str(pid_index))}: {value:.4f}")
        elif cmd in [0x42, 0x43]:
            acc_index = data[1]
            value = struct.unpack('<i', data[4:8])[0]
            mode = "READ" if cmd == 0x42 else "WRITE"
            print(f"[ACCEL {mode}] {self.ACCEL_MAP.get(acc_index, str(acc_index))}: {value}")
        elif cmd == 0x90:
            enc_pos = struct.unpack('<H', data[2:4])[0]
            print(f"[Enc Single] Pos:{enc_pos}")
        elif cmd == 0x92:
            angle = struct.unpack('<i', data[4:8])[0] * 0.01
            self.state['multi_angle'] = angle

    def _decode_error_string(self, code):
        if code == 0: return "Normal"
        
        errors = []
        if code & 0x0002: errors.append("Motor Stall")
        if code & 0x0004: errors.append("Low Voltage")
        if code & 0x0008: errors.append("Over Voltage")
        if code & 0x0010: errors.append("Over Current")
        if code & 0x0040: errors.append("Power Overrun")
        if code & 0x0080: errors.append("Calib Param Error")
        if code & 0x0100: errors.append("Speeding")
        if code & 0x0800: errors.append("Comp OverTemp")
        if code & 0x1000: errors.append("Motor OverTemp")
        if code & 0x2000: errors.append("Enc Calib Error")
        if code & 0x4000: errors.append("Enc Data Error")
        
        return ", ".join(errors) if errors else f"Unknown(0x{code:04X})"

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

    # --- Methods ---
    def read_motor_status_1(self): return self._send([0x9A, 0, 0, 0, 0, 0, 0, 0])
    def read_motor_status_2(self): return self._send([0x9C, 0, 0, 0, 0, 0, 0, 0])
    def read_motor_status_3(self): return self._send([0x9D, 0, 0, 0, 0, 0, 0, 0])
    def shutdown_motor(self): return self._send([0x80, 0, 0, 0, 0, 0, 0, 0])
    def stop_motor(self): return self._send([0x81, 0, 0, 0, 0, 0, 0, 0])
    
    def read_system_mode(self): return self._send([0x70, 0, 0, 0, 0, 0, 0, 0])
    def system_reset(self): return self._send([0x76, 0, 0, 0, 0, 0, 0, 0])
    def release_brake(self): return self._send([0x77, 0, 0, 0, 0, 0, 0, 0])
    def lock_brake(self): return self._send([0x78, 0, 0, 0, 0, 0, 0, 0])
    def read_system_runtime(self): return self._send([0xB1, 0, 0, 0, 0, 0, 0, 0])
    def read_version_date(self): return self._send([0xB2, 0, 0, 0, 0, 0, 0, 0])
    def read_motor_model(self): return self._send([0xB5, 0, 0, 0, 0, 0, 0, 0])
    
    def set_timeout_protection(self, timeout_ms):
        data = bytearray([0xB3, 0, 0, 0]) + struct.pack('<I', int(timeout_ms))
        return self._send(data)
        
    def set_baudrate(self, can_baud_idx):
        return self._send([0xB4, 0, 0, 0, 0, 0, 0, can_baud_idx])
        
    def set_active_reply(self, cmd_byte, enable, interval_ms):
        interval_val = int(interval_ms // 10)
        data = bytearray([0xB6, cmd_byte, int(enable)]) + struct.pack('<H', interval_val) + bytearray([0, 0, 0])
        return self._send(data)
        
    def set_can_id(self, new_id):
        return self._send([0x79, 0, 0, 0, 0, 0, 0, new_id])
        
    def read_can_id(self):
        return self._send([0x79, 0, 1, 0, 0, 0, 0, 0])

    def set_function_control(self, index, value):
        data = bytearray([0x20, index, 0, 0]) + struct.pack('<i', int(value))
        return self._send(data)

    def read_multiturn_pos(self):
        return self._send([0x60, 0, 0, 0, 0, 0, 0, 0])

    def read_original_pos(self):
        return self._send([0x61, 0, 0, 0, 0, 0, 0, 0])

    def read_multiturn_encoder_offset(self):
        return self._send([0x62, 0, 0, 0, 0, 0, 0, 0])

    def write_encoder_offset(self, offset_value):
        val_int = int(offset_value)
        val_bytes = struct.pack('<i', val_int)
        header = struct.pack('<BBBB', 0x63, 0, 0, 0)
        return self._send(header + val_bytes)

    def set_current_pos_as_zero(self):
        return self._send([0x64, 0, 0, 0, 0, 0, 0, 0])

    def read_multi_turn_angle(self): 
        return self._send([0x92, 0, 0, 0, 0, 0, 0, 0])

    def read_single_turn_angle(self):
        return self._send([0x94, 0, 0, 0, 0, 0, 0, 0])

    def write_acceleration(self, index, value_dps_sq):
        val_int = int(value_dps_sq)
        val_bytes = struct.pack('<i', val_int)
        header = struct.pack('<BBBB', 0x43, index, 0, 0)
        return self._send(header + val_bytes)

    def vel_control(self, speed_dps, max_torque_current=0):
        speed_val = int(speed_dps * 100)
        max_torque = max(0, min(255, int(max_torque_current)))
        data = bytearray([0xA2, max_torque, 0x00, 0x00]) + struct.pack('<i', speed_val)
        return self._send(data)

    def pos_control(self, angle_deg, max_speed_dps):
        angle_val = int(angle_deg * 100)
        speed_limit_val = int(max_speed_dps)
        data = bytearray([0xA4, 0x00]) + struct.pack('<H', speed_limit_val) + struct.pack('<i', angle_val)
        return self._send(data)

    def read_pid(self, index): return self._send([0x30, index, 0, 0, 0, 0, 0, 0])
    def read_acceleration(self, index): return self._send([0x42, index, 0, 0, 0, 0, 0, 0])

    def close(self):
        self.running = False
        self.stop_motor()
        # Notifier 종료
        if hasattr(self, 'notifier'):
            self.notifier.stop()
        print("[Driver] 드라이버 종료.")

    def print_stats(self):
        with self.state_lock:
            print(f"Sent: {self.stats['total_sent']}, Recv: {self.stats['total_received']}, Err: {self.stats['receive_errors']}")


if __name__ == "__main__":
    DEVICE_PORT = "/dev/canable0" 
    
    if not can_interface.bringup_can_interface(tty=DEVICE_PORT):
        print("하드웨어 설정 실패로 프로그램을 종료합니다.")
        sys.exit(1)

    # debug=True로 설정하면 TX/RX 로그가 출력됨
    motor2 = RMDMotorDriver(channel='can0', motor_id=5, debug=False)
    
    try:
        motor2.read_motor_status_2()
        motor2.read_multi_turn_angle()
        time.sleep(0.5)
        
        # 테스트: 통계 출력
        motor2.print_stats()
        
        time.sleep(5)
    except KeyboardInterrupt:
        print("\n사용자 중단.")
    finally:
        motor2.close()