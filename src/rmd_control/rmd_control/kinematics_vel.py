import sympy as sp
import numpy as np
class KinematicsSolver:
    def __init__(self):
        th = sp.Symbol('th')
        # 수식 정의
        l3 = sp.sqrt((380 - 50 * sp.cos(th))**2 + (50 * sp.sin(th))**2)
        th1 = sp.pi - th - sp.acos((-380**2 + 50**2 + l3**2) / (2 * 50 * l3))
        th2 = sp.acos((-370**2 + 51.277**2 + l3**2) / (2 * 51.277 * l3))
        dJ_dth = -(sp.diff(th1, th) + sp.diff(th2, th))

        # Lambdify (고속 연산용)
        safe_modules = [{'arccos': lambda x: np.arccos(np.clip(x, -1.0, 1.0))}, 'numpy']
        self.get_jacobian = sp.lambdify(th, dJ_dth, modules=safe_modules)

    def calculate_safe_velocity(self, current_th_rad, input_vel):
        epsilon = 1e-5 
        if abs(current_th_rad) < epsilon:
            current_th_rad = epsilon if current_th_rad >= 0 else -epsilon

        try:
            J = self.get_jacobian(current_th_rad)
        except Exception: return 0.0
        
        if np.isnan(J) or np.isinf(J) or abs(J) < 1e-9: return 0.0

        damping = 1e-2
        result = (J * input_vel) / (J**2 + damping**2)
        return float(result)
th = sp.Symbol('th')
input_val = sp.Symbol('input_val')

# 수식 정의
l3 = sp.sqrt((380 - 50 * sp.cos(th))**2 + (50 * sp.sin(th))**2)
th1 = sp.pi - th - sp.acos((-380**2 + 50**2 + l3**2) / (2 * 50 * l3))
th2 = sp.acos((-370**2 + 51.277**2 + l3**2) / (2 * 51.277 * l3))

dJ_dth = -(sp.diff(th1, th) + sp.diff(th2, th))

# lambdify 생성 (기존 설정 유지)
safe_modules = [{'arccos': lambda x: np.arccos(np.clip(x, -1.0, 1.0))}, 'numpy']
get_jacobian = sp.lambdify(th, dJ_dth, modules=safe_modules)
class KinematicsSolver:
    def __init__(self):
        th = sp.Symbol('th')
        # 수식 정의
        l3 = sp.sqrt((380 - 50 * sp.cos(th))**2 + (50 * sp.sin(th))**2)
        th1 = sp.pi - th - sp.acos((-380**2 + 50**2 + l3**2) / (2 * 50 * l3))
        th2 = sp.acos((-370**2 + 51.277**2 + l3**2) / (2 * 51.277 * l3))
        dJ_dth = -(sp.diff(th1, th) + sp.diff(th2, th))

        # Lambdify (고속 연산용)
        safe_modules = [{'arccos': lambda x: np.arccos(np.clip(x, -1.0, 1.0))}, 'numpy']
        self.get_jacobian = sp.lambdify(th, dJ_dth, modules=safe_modules)

    def calculate_safe_velocity(self, current_th_rad, input_vel):
        epsilon = 1e-5 
        if abs(current_th_rad) < epsilon:
            current_th_rad = epsilon if current_th_rad >= 0 else -epsilon

        try:
            J = self.get_jacobian(current_th_rad)
        except Exception: return 0.0
        
        if np.isnan(J) or np.isinf(J) or abs(J) < 1e-9: return 0.0

        damping = 1e-2
        result = (J * input_vel) / (J**2 + damping**2)
        return float(result)
        
def calculate_safe_velocity(current_th, current_input):
    # [수정 1] 입력값 회피 (Perturbation)
    # th가 0이면 미분식의 분모가 0이 되어 폭발하므로, 아주 미세한 값을 더해 회피합니다.
    epsilon_th = 1e-5 
    if abs(current_th) < epsilon_th:
        current_th = epsilon_th if current_th >= 0 else -epsilon_th

    # 1. 야코비안 계산 (이제 NaN이 뜨지 않음)
    try:
        J = get_jacobian(current_th)
    except FloatingPointError:
        return 0.0
    
    # 혹시라도 NaN이 나오면 0으로 처리 (안전장치)
    if np.isnan(J) or np.isinf(J):
        return 0.0

    # 2. 특이점 처리 (Damped Least Squares)
    damping = 1e-2  # 0 근처에서는 damping을 조금 더 키우는 것이 안정적일 수 있음
    
    if abs(J) < 1e-9:
        return 0.0

    result = (J * current_input) / (J**2 + damping**2)
    return result

# 테스트
current_th = 0.3
current_input = 0.1

