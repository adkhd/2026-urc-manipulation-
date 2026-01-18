# od_build.py (심플 버전)
from canopen import objectdictionary as od
from servo_can_bridge.constant import *            # 네가 올린 상수들
from servo_can_bridge.constant import od_from_reg  # 0x4000 + addr 규칙

# ── 아주 단순한 헬퍼 2개 ─────────────────────────────────────────────
def add_reg(O: od.ObjectDictionary, name: str, reg_addr: int, *, dtype=None, sub=0, access="rw"):
    """레지스터 주소 → 프록시 OD(od_from_reg)로 등록"""
    if reg_addr is None:
        return None
    v = od.Variable(name, od_from_reg(reg_addr), sub)
    v.data_type = dtype if dtype is not None else od.UNSIGNED32   # 모르면 U32로 시작
    v.access_type = access
    O.add_object(v)
    return v

def add_od(O: od.ObjectDictionary, name: str, od_index: int, *, dtype=None, sub=0, access="rw"):
    """표준/벤더 OD 인덱스를 그대로 등록"""
    if od_index is None:
        return None
    v = od.Variable(name, od_index, sub)
    v.data_type = dtype if dtype is not None else od.UNSIGNED32
    v.access_type = access
    O.add_object(v)
    return v

# (옵션) Record가 필요할 때만: 멤버 튜플 리스트로 간단 등록
def add_record(O: od.ObjectDictionary, name: str, od_index: int, members):
    """
    members = [(sub, field_name, dtype, access), ...]
    dtype/access는 None 가능(기본값 U32/rw)
    """
    rec = od.Record(name, od_index)
    O.add_object(rec)
    for (si, fname, dt, acc) in members:
        m = od.Variable(fname, od_index, si)
        m.subindex = si
        m.data_type = dt if dt is not None else od.UNSIGNED32
        m.access_type = acc if acc is not None else "rw"
        rec.add_member(m)
    return rec

# ── OD 빌더 ───────────────────────────────────────────────────────────
def build_od() -> od.ObjectDictionary:
    O = od.ObjectDictionary()

    # 통신/노드/하트비트 CAN 통신 파라미터
    add_reg(O, "CommMode",   REG_COMM_MODE,   dtype=od.UNSIGNED16)
    add_reg(O, "NodeId",     REG_NODE_ID,     dtype=od.UNSIGNED8)
    add_reg(O, "CanBaud",    REG_CAN_BAUD,    dtype=od.UNSIGNED16)  # 장치 코드값이면 추후 수정
    add_reg(O, "AutoOperational", REG_CANOPEN_AUTO_OP, dtype=od.UNSIGNED8)
    add_reg(O, "HeartbeatMs",REG_CAN_HB,      dtype=od.UNSIGNED16)
    add_od(O, "CanNodeId",      CAN_ID,           dtype=od.UNSIGNED8, access="rw")
    add_od(O, "CanBitrate",     CAN_BAUD,         dtype=od.UNSIGNED8, access="rw")
    add_od(O, "CanSyncCounter", CAN_SYNC_COUNTER, dtype=od.UNSIGNED16, access="rw")
    add_od(O, "SyncTime",       SYNC_TIME,        dtype=od.UNSIGNED16, access="ro")  # 인덱스 확인 필요

    # 저장 레지스터들
    for nm, ra in [
        ("Save1", REG_SAVE_1), ("Save2", REG_SAVE_2), ("Save3", REG_SAVE_3), ("Save4", REG_SAVE_4),
        ("Save5", REG_SAVE_5), ("Save6", REG_SAVE_6), ("Save7", REG_SAVE_7), ("Save8", REG_SAVE_8),
    ]:
        add_reg(O, nm, ra, dtype=od.UNSIGNED16)
        
    add_record(O, "StoreParameters", SAVE_INDEX, [
        (0, "NumSubs",  od.UNSIGNED8,  "ro"),
        (1, "Save1",    od.UNSIGNED32, "rw"),
        (2, "Save2",    od.UNSIGNED32, "rw"),
    ])
    add_record(O, "RestoreParameters", LOAD_INDEX, [
        (0, "NumSubs", od.UNSIGNED8,  "ro"),
        (1, "Load1",   od.UNSIGNED32, "rw"),
        (2, "Load2",   od.UNSIGNED32, "rw"),
    ])

    # 전류설정
    add_reg(O, "RatedCurrent", REG_IRATED,    dtype=od.UNSIGNED16)
    add_reg(O, "MAXCurrent",  REG_IWORK,     dtype=od.UNSIGNED16)
    add_reg(O, "BrakeCurrent", REG_IBRAKE,    dtype=od.UNSIGNED16)
    add_reg(O, "regControlMode",     REG_MODE_COMM, dtype=od.UNSIGNED16)   # 하위 바이트 사용

    # 엔코더/속도 관련
    # 피드백 유형/주기/최대속도
    add_reg(O, "FeedbackType",    REG_FB_TYPE,     dtype=od.UNSIGNED16)
    add_reg(O, "FeedbackSpec",    REG_FB_SPEC,     dtype=od.UNSIGNED16)
    add_reg(O, "MaxFeedbackHz", MAX_FEEDBACK_HZ_U, dtype=od.UNSIGNED32)
    #add_reg(O, "regMaxFeedbackHz_L", MAX_FEEDBACK_HZ_D, dtype=od.UNSIGNED16)
    add_reg(O, "READ_EncoderPulse", REG_ENCODER_PULSE_U,  dtype=od.UNSIGNED16)
    #add_reg(O, "regEncoderPulse_L16", REG_ENCODER_PULSE_D,  dtype=od.UNSIGNED16)

    # 가감속
    #add_reg(O, "ClAcc",  REG_CL_ACC_U, dtype=od.UNSIGNED32)
    add_reg(O, "ClAcc16",  REG_CL_ACC_D, dtype=od.UNSIGNED16)
    #add_reg(O, "ClDcc",  REG_CL_DCC_U, dtype=od.UNSIGNED32)
    #add_reg(O, "ClDcc16",  REG_CL_DCC_D, dtype=od.UNSIGNED16)
    add_od(O, "ClAcc",  CL_ACC, dtype=od.UNSIGNED16)
    add_od(O, "ClDcc",  CL_DCC, dtype=od.UNSIGNED16)
    #최대속도
    #add_reg(O, "MaxVel", REG_MAX_VEL_U, dtype=od.UNSIGNED32)
    #add_reg(O, "regMaxVel_L16", REG_MAX_VEL_D, dtype=od.UNSIGNED16)

    #위치제어 완료 상태
    add_reg(O, "PosComState", REG_POS_COM_STATE, dtype=od.UNSIGNED8)

    # 리미트 스위치(리미트스위치의 작동모드)
    add_reg(O, "LsEnable", REG_LS_ENABLE, dtype=od.UNSIGNED16)   #0:off 1:on
    add_reg(O, "Ls1Mode",  REG_LS1_MODE,  dtype=od.UNSIGNED16)   #0:LOW, 1:HIGH, 2:log edge 3: high edge
    add_reg(O, "Ls2Mode",  REG_LS2_MODE,  dtype=od.UNSIGNED16)



    # 속도 제어 PID (float이라면 나중에 raw bytes/스케일 확정)
    add_reg(O, "SpeedKp", REG_SPEED_KP_U, dtype=od.REAL32)
    #add_reg(O, "regSpeedKp_L16", REG_SPEED_KP_D, dtype=od.UNSIGNED16)
    add_reg(O, "SpeedKi", REG_SPEED_KI_U, dtype=od.REAL32)
    #add_reg(O, "regSpeedKi_L16", REG_SPEED_KI_D, dtype=od.UNSIGNED16)
    add_reg(O, "SpeedKd", REG_SPEED_KD_U, dtype=od.REAL32)
    #add_reg(O, "regSpeedKd_L16", REG_SPEED_KD_D, dtype=od.UNSIGNED16)

    add_reg(O, "PosSpeedPidEna", 0x00b6, dtype=od.UNSIGNED16)
    # 위치제어 속도 PID
    add_reg(O, "PosSpeedKp", REG_POS_SPEED_KP_U, dtype=od.REAL32)
    #add_reg(O, "regPosSpeedKp_L16", REG_POS_SPEED_KP_D, dtype=od.UNSIGNED16)
    add_reg(O, "PosSpeedKi", REG_POS_SPEED_KI_U, dtype=od.REAL32)
    #add_reg(O, "regPosSpeedKi_L16", REG_POS_SPEED_KI_D, dtype=od.UNSIGNED16)
    add_reg(O, "PosSpeedKd", REG_POS_SPEED_KD_U, dtype=od.REAL32)
    #add_reg(O, "regPosSpeedKd_L16", REG_POS_SPEED_KD_D, dtype=od.UNSIGNED16)

    # 위치제어 위치 PID
    add_reg(O, "PosKp", REG_POS_KP_U, dtype=od.REAL32)
    #add_reg(O, "regPosKp_L16", REG_POS_KP_D, dtype=od.UNSIGNED16)
    add_reg(O, "PosKi", REG_POS_KI_U, dtype=od.REAL32)
    #add_reg(O, "regPosKi_L16", REG_POS_KI_D, dtype=od.UNSIGNED16)
    add_reg(O, "PosKd", REG_POS_KD_U, dtype=od.REAL32)
    #add_reg(O, "regPosKd_L16", REG_POS_KD_D, dtype=od.UNSIGNED16)

    #add_reg(O, "PosSpeedCfg", REG_POS_SPEED, dtype=od.INTEGER32)   # 추후 확인
    #add_reg(O, "PosEnaErr",  REG_POS_ENA_ERR, dtype=od.UNSIGNED16)

    # ── 표준/벤더 OD는 인덱스 그대로 ───────────────────────────
    add_od(O, "ErrorRegister",     ERROR_INDEX,    dtype=od.UNSIGNED8,  access="ro")
    add_record(O, "ErrorLog",      ERROR_LOG_INDEX, [
        (0, "NumErrors", od.UNSIGNED8,  "rw"),
        (1, "Err1",      od.UNSIGNED32, "ro"),
        (2, "Err2",      od.UNSIGNED32, "ro"),
        (3, "Err3",      od.UNSIGNED32, "ro"),
        (4, "Err4",      od.UNSIGNED32, "ro"),
        (5, "Err5",      od.UNSIGNED32, "ro"),
        (6, "Err6",      od.UNSIGNED32, "ro"),
        (7, "Err7",      od.UNSIGNED32, "ro"),
        (8, "Err8",      od.UNSIGNED32, "ro"),
    ])

    add_od(O, "HeartbeatTimeMs", HB_HZ_INDEX, dtype=od.UNSIGNED16, access="rw")

    # Motor control/status (벤더)
    add_reg(O, "ServoType", REG_SERVO_TYPE, dtype=od.UNSIGNED16)
    add_od(O, "ControlMode",     CONTROL_MODE,     dtype=od.UNSIGNED8,  access="rw")
    add_od(O, "ControlVal",      CONTROL_VAL,      dtype=od.INTEGER32,  access="rw")
    add_od(O, "PosControlType",  POS_CONTROL_TYPE, dtype=od.UNSIGNED8,  access="rw")
    add_od(O, "PosControlVal",   POS_CONTROL_VAL,  dtype=od.INTEGER32,  access="rw")
    add_od(O, "MaxVel",     MAX_VELOCITY,     dtype=od.UNSIGNED32, access="rw")
    add_od(O, "ResetEncoder",    RESET_ENCODER,    dtype=od.INTEGER32,  access="wo")

    add_od(O, "Read_MotorCurr",   MOTOR_CA,         dtype=od.UNSIGNED16, access="ro")
    add_od(O, "Read_MotorPos",     MOTOR_POS,        dtype=od.INTEGER32,  access="ro")
    add_od(O, "Read_MotorVel",     MOTOR_VEL,        dtype=od.INTEGER32, access="ro")
    add_od(O, "Read_MotorLsState",      MOTOR_LS_STATE,   dtype=od.UNSIGNED8,  access="ro")
    add_od(O, "Read_MoErrState",   MOTOR_ERROR_STATE,dtype=od.UNSIGNED8,  access="ro")

    # PDO 통신/매핑 (레코드)
    for name, idx in [
        ("RPDO1_Comm", RPDO1_COM_INDEX), ("RPDO2_Comm", RPDO2_COM_INDEX),
        ("RPDO3_Comm", RPDO3_COM_INDEX), ("RPDO4_Comm", RPDO4_COM_INDEX),
    ]:
        add_record(O, name, idx, [
            (0, "NumSubs", od.UNSIGNED8,  "ro"),
            (1, "COB_ID",  od.UNSIGNED32, "rw"),
            (2, "TransType", od.UNSIGNED8, "rw"),
            (3,"Inhibit_100us",    od.UNSIGNED16, "ro"),  
            (5,"EventTimer_ms",    od.UNSIGNED16, "ro"),  
        ])

    for name, idx in [
        ("TPDO1_Comm", TPDO1_COM_INDEX), ("TPDO2_Comm", TPDO2_COM_INDEX),
        ("TPDO3_Comm", TPDO3_COM_INDEX), ("TPDO4_Comm", TPDO4_COM_INDEX),
    ]:
        add_record(O, name, idx, [
            (0, "NumSubs", od.UNSIGNED8,  "ro"),
            (1, "COB_ID",  od.UNSIGNED32, "rw"),
            (2, "TransType", od.UNSIGNED8, "rw"),
            (3, "Inhibit_100us", od.UNSIGNED16, "rw"),
            (4, "SyncStart", od.UNSIGNED8, "rw"),
            (5, "Event_ms", od.UNSIGNED16, "rw"),
        ])

    for name, idx in [
        ("RPDO1_Map", RPDO1_MAP_INDEX), ("RPDO2_Map", RPDO2_MAP_INDEX),
        ("RPDO3_Map", RPDO3_MAP_INDEX), ("RPDO4_Map", RPDO4_MAP_INDEX),
        ("TPDO1_Map", TPDO1_MAP_INDEX), ("TPDO2_Map", TPDO2_MAP_INDEX),
        ("TPDO3_Map", TPDO3_MAP_INDEX), ("TPDO4_Map", TPDO4_MAP_INDEX),
    ]:
        add_record(O, name, idx, [
            (0, "NumEntries", od.UNSIGNED8, "rw"),
            (1, "Entry1", od.UNSIGNED32, "rw"),
            (2, "Entry2", od.UNSIGNED32, "rw"),
            (3, "Entry3", od.UNSIGNED32, "rw"),
            (4, "Entry4", od.UNSIGNED32, "rw"),
            (5, "Entry5", od.UNSIGNED32, "rw"),
            (6, "Entry6", od.UNSIGNED32, "rw"),
            (7, "Entry7", od.UNSIGNED32, "rw"),
            (8, "Entry8", od.UNSIGNED32, "rw"),
        ])

        # 목표/제어(OD에 있는 것 사용)
    add_reg(O, "regTargetSpeed",    REG_TARGET_SPEED, dtype=od.INTEGER32)   # 모르면 16/32 검증
    add_reg(O, "regTargetPosition", REG_TARGET_POS,   dtype=od.INTEGER32)
    add_reg(O, "regControlWord",    REG_CTRL,         dtype=od.UNSIGNED16)

    return O

