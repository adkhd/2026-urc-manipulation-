# a = 50
# b = 410
# c = 318.944
# x0 = 
# cob = 13.6 deg
import math
#mm
A = 50.0
B = 410.0
C = 318.944
COB_DEG = 13.6
COB_RAD = math.radians(COB_DEG)
X0 = 323
X_LIM= C+A-X0

L1_X_MAX=45
L1_X_MIN=0
THETA2_MAX=1.330
THETA2_MIN=-0.0281
THETA2_D_MAX = 0.4

L2_X0 = 323
L2_A = 277.714
L2_B = 327.878
L2_TH1 = math.radians(13.54)
L2_TH2 = math.radians(13.19)


def x2th_L1(x):
    theta_p = math.acos(((X0+x)**2-A**2-C**2)/(-2*A*C))
    theta = math.pi-COB_RAD-theta_p
    return theta

def thd2xd_L1(theta_d, x):
    x_d = -theta_d*(2*A*C*math.sin(math.pi-COB_RAD-x2th_L1(x)))/2.0/(X0+x)
    return x_d

def x2th_L2(x):
    theta_p = math.acos(((L2_X0+x)**2-L2_A**2-L2_B**2)/(-2*L2_A*L2_B))
    theta = -math.pi+L2_TH1+L2_TH2+theta_p
    return theta

def thd2xd_L2(theta_d, x):
    x_d = -theta_d*(2*L2_A*L2_B*math.sin(math.pi-L2_TH1-L2_TH2-x2th_L2(x)))/2.0/(L2_X0+x)
    return x_d
def th2x_L1(theta):
    # 순기구학 식: theta = pi - COB - theta_p
    # 역산: theta_p = pi - COB - theta
    theta_p = math.pi - COB_RAD - theta
    
    # 코사인 제 2법칙 변형:
    # (X0 + x)^2 = A^2 + C^2 - 2AC * cos(theta_p)
    term = A**2 + C**2 - 2*A*C*math.cos(theta_p)
    
    # x에 대해 정리
    x = math.sqrt(term) - X0
    return x

# --------------------------------------------------------
# L2 (J3) 역기구학: 각도(rad) -> 길이(mm)
# --------------------------------------------------------
def th2x_L2(theta):
    # 순기구학 식: theta = -pi + TH1 + TH2 + theta_p
    # 역산: theta_p = theta + pi - TH1 - TH2
    theta_p = theta + math.pi - L2_TH1 - L2_TH2
    
    # 코사인 제 2법칙 변형:
    # (X0 + x)^2 = A^2 + B^2 - 2AB * cos(theta_p)
    term = L2_A**2 + L2_B**2 - 2*L2_A*L2_B*math.cos(theta_p)
    
    # x에 대해 정리
    x = math.sqrt(term) - L2_X0
    return x
# # --- 이 코드를 테스트하기 위한 메인 실행 부분 ---
if __name__ == "__main__":
    
    # 1. 테스트용 상수값 정의 (단위: mm, rad, rad/s)
    # (이 값들은 실제 기구에 맞게 수정해야 합니다)
    
    # 함수 호출
    print(x2th_L1(0))
    print(x2th_L1(44))
    print(x2th_L2(0))
    print(x2th_L2(150))
    print(th2x_L1(1.1))
    print(th2x_L2(-0.8878))
    # print(thd2xd_L2(0.03, 40))
    # print(int(thd2xd_L2(0.05, 0) * 33.*60./12.7))
    # print(int(thd2xd_L1(0.09, 0) * 33.*60./6.35))