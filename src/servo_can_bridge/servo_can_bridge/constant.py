# =========================
# 메뉴얼 레지스터 주소 (네가 가진 표 그대로 채워)
# =========================
# 통신/노드/속도
REG_COMM_MODE   = 0x0120   # 1 = CANopen 통신모드
REG_NODE_ID     = 0x0121   # Node-ID
REG_CAN_BAUD    = 0x0122   # CAN bitrate (예: 500kbps 코드값)
REG_CANOPEN_AUTO_OP = 0x0128
REG_CAN_HB = 0x0129

#저장 레지스터
REG_SAVE_1 = 0x0170
REG_SAVE_2 = 0x0180
REG_SAVE_3 = 0x01A0
REG_SAVE_4 = 0x0190
REG_SAVE_5 = 0x01D0
REG_SAVE_6 = 0x01E0
REG_SAVE_7 = 0x01E8
REG_SAVE_8 = 0x01F0

#엔코더/모터속도 관련

REG_ENCODER_SPEC = 0x0069
REG_ENCODER_PULSE_U = 0x002c
REG_ENCODER_PULSE_D = 0x002d

#??
REG_POS_COM_STATE = 0x0032 #0 미완, 1 완료, 2

#아래에 있는 건 canopen제어에서는 사용x
# === 네가 반드시 메뉴얼에서 찾아서 아래 값을 채워 넣어야 하는 항목 ===
REG_TARGET_SPEED = 0x00A0  # (예시) 목표속도 레지스터 (2 or 4바이트? 메뉴얼 표 단위/폭 확인)
REG_TARGET_POS   = 0x00A2  # (예시) 목표위치 레지스터
REG_CTRL         = 0x0040  # (예시) 제어비트(START/STOP/DIR 등) 레지스터
# ↑↑↑ 꼭 네 표대로 주소/비트/단위를 맞춰 넣어라 ↑↑↑

#가속도
REG_CL_ACC_U = 0x0060
REG_CL_ACC_D = 0x0061
REG_CL_DCC_U = 0x0062
REG_CL_DCC_D = 0x0063

#엔코더 피드백 주기(기본값사용)
MAX_FEEDBACK_HZ_U = 0x0064
MAX_FEEDBACK_HZ_D = 0x0065

#피드백 유형, 엔코더 펄스
REG_FB_TYPE = 0x0068
REG_FB_SPEC = 0x0069

#최대속도
REG_MAX_VEL_U = 0x006c
REG_MAX_VEL_D = 0x006d

#리미트스위치
REG_LS_ENABLE = 0x0082
REG_LS1_MODE = 0x0083
REG_LS2_MODE = 0x0084

# 전류/모드
REG_IRATED      = 0x0086   # 정격전류 (0.01A 단위)
REG_IWORK       = 0x0087   # 최대정격전류 (0.01A 단위)
REG_IBRAKE      = 0x0088   # 제동전류 (0.01A 단위)
REG_MODE_COMM   = 0x0080   # 통신 제어 모드: "하위 바이트"에 모드 코드 (메뉴얼 표)

#속도제어 PID(FLOAT)
REG_SPEED_KP_U = 0x0098
REG_SPEED_KP_D = 0x0099
REG_SPEED_KI_U = 0x009a
REG_SPEED_KI_D = 0x009b
REG_SPEED_KD_U = 0x009c
REG_SPEED_KD_D = 0x009d

#위치제어 속도 PID
REG_POS_SPEED_KP_U = 0x00b0
REG_POS_SPEED_KP_D = 0x00b1
REG_POS_SPEED_KI_U = 0x00b2
REG_POS_SPEED_KI_D = 0x00b3
REG_POS_SPEED_KD_U = 0x00b4
REG_POS_SPEED_KD_D = 0x00b5

#위치제어 위치 PID
REG_POS_KP_U = 0x00b8
REG_POS_KP_D = 0x00b9
REG_POS_KI_U = 0x00ba
REG_POS_KI_D = 0x00bb
REG_POS_KD_U = 0x00bc
REG_POS_KD_D = 0x00bd

REG_SERVO_TYPE = 0x00c1 #0:속도 2:비고정위치제어
REG_POS_SPEED = 0x00c7 #위치제어 속도?
REG_POS_ENA_ERR = 0x00ca #허용에러
# =======================
# OD ADDRESS
# ===================
ERROR_INDEX = 0x1001 #RO
ERROR_LOG_INDEX = 0x1003 #sub:0~8
SAVE_INDEX = 0x1010 #sub0:(u8, RO) sub1:(u32,RW) sub2:(u32,RW)
LOAD_INDEX = 0x1011 #0x1010동일
HB_HZ_INDEX = 0x1017 #u16 RW

#MOTOR CONTROL OD
CONTROL_MODE = 0x2000 #sub0(U8,RW/E) 1:속도 2:토크 3:위치 0x10: 정상정지 0x11: 비상정지 0x12: 자유정지
CONTROL_VAL = 0x2001 #sub0(S16 or S32, RW) RPM or 위치제어속도
POS_CONTROL_TYPE = 0x2002 #(U8, RW), 0:절대 1:상대(현재위치기준)
POS_CONTROL_VAL = 0x2003 #(S32, RW) 
CL_ACC = 0x2006
CL_DCC = 0x2007
MAX_VELOCITY = 0x2008 #(U16, RW)
RESET_ENCODER = 0x200f #(S32, WO) 0= 카운트값 초기화

#MOTOR STATUS OD
MOTOR_CA = 0x2100 #(U16, RO)
MOTOR_POS = 0x2105 #(S32, RO)
MOTOR_VEL = 0x210a #(U32, RO)
MOTOR_LS_STATE = 0x2114 #(U8, RO) 00 or 01 or 10 or 11 : (SQ2)(SQ1)  
MOTOR_ERROR_STATE = 0x2112 #(U8, RO)

#sub1(u32, RW): bit31:0/1 PDO활성/비활성, bit29:0/1 11비트/29비트 #sub2: U8, RW, T-TYPE
RPDO1_COM_INDEX = 0x1400 
RPDO2_COM_INDEX = 0x1401 
RPDO3_COM_INDEX = 0x1402
RPDO4_COM_INDEX = 0x1403

#sub0(U8,RW) : 매핑개수, sub1~8(u32,RW): 각 매핑 엔트리 Index(16) | Sub(8) | Size(8)
RPDO1_MAP_INDEX = 0x1600 
RPDO2_MAP_INDEX = 0x1601
RPDO3_MAP_INDEX = 0x1602
RPDO4_MAP_INDEX = 0x1603

#sub0 (U8, RO): 서브인덱스 수(보통 0x05).sub1 – COB-ID (U32, RW): 이 TPDO가 송신할 CAN ID.sub2 – 전송 타입(U8, RW)
#sub3 – Inhibit time(U16, 100μs 단위): 최소 전송 간격(찌꺼기 전송 방지). sub5 – Event time(U16, ms): 전송 타입이 0xFF일 때 주기 송신 간격.
TPDO2_COM_INDEX = 0x1801
TPDO1_COM_INDEX = 0x1800 
TPDO3_COM_INDEX = 0x1802
TPDO4_COM_INDEX = 0x1803

#sub0: 맵핑 개수(0~8).
#sub1..sub8: 형식은 RPDO와 동일(인덱스/서브/바이트수).
TPDO1_MAP_INDEX = 0x1A00
TPDO2_MAP_INDEX = 0x1A01
TPDO3_MAP_INDEX = 0x1A02
TPDO4_MAP_INDEX = 0x1A03

#CAN COMM PARAM
CAN_ID = 0x2201
CAN_BAUD = 0x2202 
CAN_SYNC_COUNTER = 0x2203 #(U16, RW)
SYNC_TIME = 0x2204 #(U16, RO)

# =========================
# 0x4000 프록시 OD 매핑 함수 (장비가 지원하면 A경로로 사용)
# =========================

def od_from_reg(addr: int) -> int:
    # 기본형: 레지스터 0x0000~0x0FFF → 0x4000 + addr
    # (장비에 따라 0x7000대 → addr - 0x2000 규칙이 있을 수 있음. 필요 시 분기 추가)
    if addr >=0x7000:
        return addr - 0x2000
    else:    
        return 0x4000 + addr


def COBID_RPDO1(node_id): return 0x200 + node_id   # Host→Drive
def COBID_TPDO1(node_id): return 0x180 + node_id   # Drive→Host

def COBID_SDO_TX(node_id): return 0x600 + node_id
def COBID_SDO_RX(node_id): return 0x580 + node_id