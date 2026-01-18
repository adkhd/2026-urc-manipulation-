import canopen
from canopen import objectdictionary as od
from canopen.sdo.exceptions import SdoAbortedError, SdoCommunicationError
from servo_can_bridge.od_build import build_od
from servo_can_bridge.can_iface import bringup_slcan
import time
from typing import List, Tuple, Optional
import struct

# ===========
# 예: 목표속도/제어워드 SDO
# 필요한 함수들:
#  - 초기설정(전류값/피드백/엔코더/PID/저장/CAN통신모드/리밋)
#  - 속도/위치 제어 함수
#  - PDO 매핑 함수
#  - 호밍/ NMT / EMCY 처리
# ==========

class CanopenConnectError(RuntimeError):
    pass


# ---------------------------
# CANopen 연결 (임포트 시 자동 실행 금지!)
# ---------------------------
def add_canopen_node(
    net: canopen.Network,
    node_id: int,
    heartbeat_ms: int | None = 500,
    boot_timeout: float = 2.0,
    to_operational: bool = False,
):


    node = canopen.RemoteNode(node_id, build_od())
    net.add_node(node)

    try:
        node.nmt.send_command(0x81)  # reset comm
    except Exception:
        pass

    if not node.nmt.wait_for_bootup(boot_timeout):
        try:
            node.nmt.send_command(0x82)  # reset node
        except Exception:
            pass
        if not node.nmt.wait_for_heartbeat(boot_timeout):
            raise CanopenConnectError(
                f"Node 0x{node_id:02X} boot-up(0x700+ID, 0x00) 미수신"
            )

    # PRE-OP 보장
    node.nmt.state = "PRE-OPERATIONAL"

    # Heartbeat Producer 설정
    if heartbeat_ms is not None:
        hb = int(heartbeat_ms)
        if not (0 <= hb <= 0xFFFF):
            raise ValueError("heartbeat_ms must be 0..65535 ms")
        node.sdo.download(0x1017, 0, hb.to_bytes(2, "little"))

        t0 = time.time()
        while time.time() - t0 < 2.0:
            if node.nmt.state in ("PRE-OPERATIONAL", "OPERATIONAL", "STOPPED"):
                break
            time.sleep(0.05)

    time.sleep(0.1)
    if to_operational:
        node.nmt.state = "OPERATIONAL"

    node.sdo.RESPONSE_TIMEOUT = 1.0
    node.rpdo.read()
    node.tpdo.read()
    return node


_DT_BITS = {
    od.BOOLEAN: 1,
    od.INTEGER8: 8,   od.UNSIGNED8: 8,
    od.INTEGER16:16,  od.UNSIGNED16:16,
    od.INTEGER32:32,  od.UNSIGNED32:32,
    od.REAL32:  32,
    od.INTEGER64:64,  od.UNSIGNED64:64,
}


def dtype_and_bits(node, index, sub=0):
    """주어진 OD index/sub 의 데이터타입과 비트폭을 조회."""
    try:
        entry = node.sdo[index]
    except KeyError:
        return (None, None)

    is_record_like = hasattr(entry, "keys") or hasattr(entry, "__iter__")
    if (sub in (None, 0)) and not is_record_like:
        var = entry
    else:
        try:
            var = entry[sub]
        except Exception:
            return (None, None)

    meta = getattr(var, "od", None) or getattr(var, "od_entry", None)
    if meta is None:
        return (None, None)

    dt = getattr(meta, "datatype", None) or getattr(meta, "data_type", None)
    bits = _DT_BITS.get(dt)
    return (dt, bits)


# ---------------------------
# CANopen 기본 파라미터 SDO
# ---------------------------
def set_canopen_basic(node):
    node.sdo["CommMode"].raw = 1  # 통신모드
    node.sdo["NodeId"].raw = node.id  # 노드 ID (원하면 고정값으로)
    node.sdo["CanBaud"].raw = 5  # Baud code
    node.sdo["AutoOperational"].raw = 0
    node.sdo["CanSyncCounter"].raw = 0
    node.sdo["SyncTime"].raw = 0
    save_params(node)


def set_pre_control(node):
    node.sdo["regControlMode"].raw = 3
     #전류 다 비슷해서 5A로 통일, maxvel은 기본값 사용
    if node.id == 1:
        node.sdo["ServoType"].raw = 0
        set_encoder(node, 19)  
        set_limits_safety(node, 5.0)
    elif node.id in (2, 3):
        node.sdo["ServoType"].raw = 2
        set_encoder(node, 2)
        set_limits_safety(node, 4.0)
    elif node.id in (4, 5):
        node.sdo["ServoType"].raw = 0
        set_encoder(node, 13)
        set_limits_safety(node, 5.0)
    node.nmt.state = "OPERATIONAL"


# ---------------------------
# NMT 상태 전환
# ---------------------------
def set_nmt_preop(node):
    # 원본에서는 OPERATIONAL 이었음 → 의도상 PRE-OPERATIONAL 이 맞아 보임
    node.nmt.state = "PRE-OPERATIONAL"


def set_nmt_operational(node):
    node.nmt.state = "OPERATIONAL"


# ---------------------------
# EMCY/HB 모니터 (필요 시 확장)
# ---------------------------
def emcy_heartbeat_monitor():
    print()


# ---------------------------
# RPDO/TPDO 매핑
# ---------------------------
def rpdo_mapping(node, node_id, pdo_number, idx_sidx, type=254, rtr=0):
    set_nmt_preop(node)
    rpdo = node.rpdo[pdo_number]
    rpdo.enabled = False
    rpdo.clear()

    resolved, total_bits = [], 0
    for e in idx_sidx:
        idx = e[0]
        sidx = e[1] if len(e) >= 2 else 0
        _, bits = dtype_and_bits(node, idx, sidx)
        if bits is None:
            raise ValueError(f"'{idx}' 비트 길이를 알 수 없습니다. ('{idx}', bits) 형태로 넘겨주세요.")
        if bits % 8 != 0:
            raise ValueError(f"'{idx}' 길이 {bits}는 8의 배수여야 합니다.")
        total_bits += bits
        resolved.append((idx, sidx, bits))

    if total_bits > 64:
        raise ValueError(f"RPDO{pdo_number} 매핑 총 {total_bits} bits → 64bits 초과!")

    for idx, sidx, bits in resolved:
        rpdo.add_variable(idx, sidx, length=bits)

    rpdo.cob_id = (0x200 + 0x100 * (pdo_number - 1) + int(node.id))
    rpdo.inhibit_time = 0
    rpdo.event_timer = 0
    rpdo.trans_type = int(type)
    rpdo.rtr_allowed = int(rtr)
    rpdo.enabled = True
    rpdo.save()

    set_nmt_operational(node)


def tpdo_mapping(node, node_id, pdo_number, idx_sidx, type=255, event_timer=10, rtr=0, inhibit_time=0):
    set_nmt_preop(node)
    tpdo = node.tpdo[pdo_number]
    tpdo.enabled = False
    tpdo.clear()

    resolved, total_bits = [], 0
    for e in idx_sidx:
        if len(e) == 3:
            idx, sidx, bits = e
        else:
            idx, sidx = (e[0], e[1] if len(e) > 1 else 0)
            _, bits = dtype_and_bits(node, idx, sidx)
        if bits is None:
            raise ValueError(f"0x{idx:04X}:{sidx} 비트 길이를 알 수 없습니다. (index,sub,bits)로 넘겨주세요.")
        if bits % 8 != 0:
            raise ValueError(f"0x{idx:04X}:{sidx} 길이 {bits}는 8의 배수여야 합니다.")
        total_bits += bits
        resolved.append((idx, sidx, bits))

    if total_bits > 64:
        raise ValueError(f"TPDO{pdo_number} 매핑 총 {total_bits} bits → 64bits 초과!")

    for idx, sidx, bits in resolved:
        tpdo.add_variable(idx, sidx, bits)

    tpdo.cob_id = (0x180 + 0x100 * (pdo_number - 1) + int(node_id))
    tpdo.trans_type = int(type)
    try:
        tpdo.inhibit_time = int(inhibit_time)
    except Exception:
        pass
    try:
        tpdo.event_timer = int(event_timer)
    except Exception:
        pass
    tpdo.rtr_allowed = int(rtr)

    tpdo.enabled = True
    tpdo.save()
    set_nmt_operational(node)
    return tpdo


def on_tpdo1(pdo):
    snapshot = {var.name: var.raw for var in pdo}


# ---------------------------
# 모터 한계/안전 설정
# ---------------------------
def set_limits_safety(node, current=5.0, maxvel=30000):
    node.sdo['RatedCurrent'].raw = current * 100  # 0.01A
    node.sdo['MAXCurrent'].raw   = current * 100
    node.sdo['BrakeCurrent'].raw = current / 3
    node.sdo['MaxVel'].raw       = maxvel
    save_params(node)


# ---------------------------
# 엔코더/피드백 설정
# ---------------------------
def set_encoder(node, ppr):
    node.sdo['FeedbackType'].raw = 2
    node.sdo['FeedbackSpec'].raw =  ppr
    # node.sdo['MaxFeedbackHz'].raw = 500
    save_params(node)


def reset_encoder(node):
    node.sdo["ResetEncoder"].raw = 0
def get_state(node):
    ls_level = node.sdo["Read_MotorLsState"].raw
    motor_pulse = node.sdo["Read_MotorPos"].raw
    motor_rpm = node.sdo["Read_MotorVel"].raw
    return ls_level, motor_pulse, motor_rpm
# ---------------------------
# PID 설정
# ---------------------------
def set_sPID(node, sKp, sKi, sKd):
    node.sdo['SpeedKp'].raw = sKp
    node.sdo['SpeedKi'].raw = sKi
    node.sdo['SpeedKd'].raw = sKd
    save_params(node)


def set_psPID(node, psKp, psKi, psKd):
    node.sdo['PosSpeedKp'].raw = psKp
    node.sdo['PosSpeedKi'].raw = psKi
    node.sdo['PosSpeedKd'].raw = psKd
    save_params(node)


def set_pPID(node, pKp, pKi, pKd):
    node.sdo['PosKp'].raw = pKp
    node.sdo['PosKi'].raw = pKi
    node.sdo['PosKd'].raw = pKd
    save_params(node)


def set_PID(node):
    if node.id == 1:  #base
        set_sPID(node, 0.45, 0.01, 0.3)
        set_pPID(node, 6.0, 0.0005, 0.1)
        set_psPID(node, 1.8, 0, 0.2)
    elif node.id == 2: #lin1
        set_sPID(node, 0.18, 0.01, 0)
        set_pPID(node, 6.0, 0.0009, 0.1)
        set_psPID(node, 1.0, 0.0005, 0.2)
    elif node.id == 3: #lin2
        set_sPID(node, 0.18, 0.01, 0)
        set_pPID(node, 6.0, 0.0005, 1.4)
        set_psPID(node, 0.8, 0.0005, 0.2)
    elif node.id == 4: #wrist dc1, 2
        set_sPID(node, 0.17, 0.002, 0.05)
        set_pPID(node, 6.3, 0.0004, 0.06)
        set_psPID(node, 1.4, 0.0002, 0.025)
    elif node.id == 5: #wrist dc1, 2
        set_sPID(node, 0.17, 0.002, 0.05)
        set_pPID(node, 8.5, 0.0004, 0.06)
        set_psPID(node, 1.9, 0.0002, 0.025)


def get_PID(node):
    print(node.sdo['SpeedKp'].raw)
    print(node.sdo['SpeedKi'].raw)
    print(node.sdo['SpeedKd'].raw)
    print(node.sdo['PosSpeedKp'].raw)
    print(node.sdo['PosSpeedKi'].raw)
    print(node.sdo['PosSpeedKd'].raw)
    print(node.sdo['PosKp'].raw)
    print(node.sdo['PosKi'].raw)
    print(node.sdo['PosKi'].raw)


# ---------------------------
# 파라미터 저장
# ---------------------------
def save_params(node, mode="od_all"):
    if mode == "od_all":
        save_reg(node)
        save_od(node)
    elif mode == "od":
        save_od(node)
    elif mode == "reg":
        save_reg(node)


def save_reg(node):
    node.sdo["Save1"].raw = 1
    node.sdo["Save2"].raw = 1
    node.sdo["Save3"].raw = 1
    node.sdo["Save4"].raw = 1
    node.sdo["Save5"].raw = 1
    node.sdo["Save6"].raw = 1
    node.sdo["Save7"].raw = 1
    node.sdo["Save8"].raw = 1


def save_od(node):
    node.sdo[0x1010][1].raw = 0x65766173  # 'ev as' little-endian


# ---------------------------
# 제어 (PDO)
# ---------------------------
def pos_control(node, pos, vel, acc, dcc):
    node.rpdo[1]["ControlMode"].raw = 3          # 위치 제어
    node.rpdo[1]["ControlVal"].raw = vel     # 위치 제어 속도(RPM) → 장치 스케일
    node.rpdo[1]["PosControlType"].raw = 0       # 0: 절대
    node.rpdo[1]["ClAcc"].raw = acc          # rpm/s
    node.rpdo[2]["ClDcc"].raw = dcc
    node.rpdo[2]["PosControlVal"].raw = pos
    node.rpdo[1].transmit()
    node.rpdo[2].transmit()


def speed_control(node, vel, acc, dcc):
    if abs(vel) <= 10:
        node.rpdo[1]["ControlMode"].raw = 0x10          # 속도 제어
    else:
        node.rpdo[1]["ControlMode"].raw = 1
    node.rpdo[1]["ControlVal"].raw = vel     # rpm → 장치 스케일
    node.rpdo[1]["ClAcc"].raw = acc 
    node.rpdo[2]["ClDcc"].raw = dcc 
    node.rpdo[1].transmit()
    node.rpdo[2].transmit()


def stop_motor(node):
    node.rpdo[1]["ControlMode"].raw = 0x10
    node.rpdo[1]["ControlVal"].raw = 0    # rpm → 장치 스케일
    node.rpdo[1].transmit()


def Homming_ifneeded(node):
    node.sdo["ControlMode"].raw = 3
    node.sdo["ControlVal"].raw = 1
    node.sdo["PosControlType"].raw = 0
    node.sdo["ServoType"].raw = 2
    node.sdo["PosControlVal"].raw = 1000


def mapping(node, ms):
    rpdo_mapping(node, node.id, 1, [("ControlMode",), ("ControlVal",), ("PosControlType",), ("ClAcc",)])
    rpdo_mapping(node, node.id, 2, [("ClDcc",), ("PosControlVal",)])
    tpdo_mapping(node, node.id, 1, [("Read_MotorPos", 0), ("Read_MotorVel", 0)], 255, ms)
    tpdo_mapping(node, node.id, 2, [("Read_MoErrState", 0)], 255, ms)


def set_linear1(node):
    set_limits_safety(node, 4.0)
    set_encoder(node, 2, 1)

#l1 = 1rpm = 6.35mm
# (주의) 임포트 시 자동 동작 금지
# 사용 예:
#net, node = connect_canopen(2, "can0")
#set_pre_control(node)
#set_PID(node)
#mapping(node)
#pos_control(node, 33*2*4, 10, 10)
#speed_control(node, vel=-50, acc=200)
