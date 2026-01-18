import math

def calculate_kinematics(th):
    """
    th1, th2: 입력 각도 (단위: 도, Degree)
    th = th2 - th1 (deg)
    반환값: J2_th (단위: 도, Degree)
    """
    
    # 삼각함수 계산을 위해 th를 라디안으로 변환
    th_rad = math.radians(th)
    
    # 2. l3 계산
    # 수식: l3 = sqrt((380-50costh)^2+(50sinth)^2)
    term1 = (380 - 50 * math.cos(th_rad)) ** 2
    term2 = (50 * math.sin(th_rad)) ** 2
    l3 = math.sqrt(term1 + term2)
    
    # 3. th_o 계산
    # 수식: th_o = arccos((-(380)^2+50^2+l3^2)/(100*l3))
    # 주의: acos 입력값은 -1과 1 사이여야 하므로, 부동소수점 오차를 대비해 범위를 제한(clamp)하는 것이 안전함
    numerator_o = -(380**2) + (50**2) + (l3**2)
    denominator_o = 100 * l3
    
    if denominator_o == 0:
        print("오류: l3가 0이 되어 나눗셈을 할 수 없습니다.")
        return None
        
    cos_val_o = numerator_o / denominator_o
    
    # acos 범위 보호 (-1.0 ~ 1.0)
    cos_val_o = max(min(cos_val_o, 1.0), -1.0)
    
    th_o_rad = math.acos(cos_val_o)
    th_o_deg = math.degrees(th_o_rad) # 결과를 도(Degree)로 변환
    
    # 4. th_1p 계산
    # 수식: th_1p = 180 - th - th_o
    # 식에 180이 포함되어 있으므로 모든 단위를 도(Degree)로 통일하여 계산
    th_1p = (180 - abs(th) - th_o_deg) * (1 if th >= 0 else -1)
    
    # 5. th_2p 계산
    # 수식: th_2p = arccos((-(370)^2+51.277^2+l3^2)/(2*51.277*l3))
    const_val = 51.277
    numerator_2p = -(370**2) + (const_val**2) + (l3**2)
    denominator_2p = 2 * const_val * l3
    
    cos_val_2p = numerator_2p / denominator_2p
    
    # acos 범위 보호 (-1.0 ~ 1.0)
    cos_val_2p = max(min(cos_val_2p, 1.0), -1.0)
    
    th_2p_rad = math.acos(cos_val_2p) * (1 if th >= 0 else -1)
    th_2p_deg = math.degrees(th_2p_rad) # 결과를 도(Degree)로 변환
    
    # 6. 최종 결과 J2_th 계산
    # 수식: J2_th = -(th_1p + th_2p)
    J2_th = -(th_1p + th_2p_deg)
    J2_th = J2_th -12.81 #실제 관절 각도
    return math.radians(J2_th)

def calculate_ik(J2): #J2 각도를 받아서 사이각 내보내기
    J2_kinematic_rad = J2 + math.radians(12.81) # 관절각 = 기구학 각도 + 12.81deg
    term1 = (380+51.277*math.cos(math.pi+J2_kinematic_rad))**2
    term2 = (51.277*math.sin(math.pi+J2_kinematic_rad))**2
    l = math.sqrt(term1 + term2)

    cos1_p = max(min((50**2+l**2-(370)**2)/(2*50*l),1.0), -1)
    cos2_p = max(min((380**2+l**2-(51.277)**2)/(2*l*380),1.0), -1)
    th = math.acos(cos1_p)+math.acos(cos2_p)

    return th

# --- 사용 예시 ---
if __name__ == "__main__":
    # 각도 제한 J1, J2(-22~-137), th(사잇각)()
    print((calculate_kinematics(195-186)))
    print((calculate_kinematics(220.41-83.42)))
    print(calculate_ik(-2.5))


