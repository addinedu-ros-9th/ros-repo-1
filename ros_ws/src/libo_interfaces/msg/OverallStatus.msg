# 로봇의 전반적인 상태를 나타내는 메시지

builtin_interfaces/Time timestamp # 메시지가 생성된 시간
string robot_id             # 예: "libo_a", "libo_b"
string robot_state          # 예: "INIT", "CHARGING", "ESCORT", "DELIVERY" 
bool is_available         # 현재 로봇이 다른 작업을 할당받을 수 있는지 여부 (True/False)
uint8 battery               # 배터리 잔량 (0-100%)
float64 book_weight         # 현재 로봇이 들고 있는 책의 무게 (kg)
float64 position_x          # 로봇의 현재 위치 (x 좌표)
float64 position_y          # 로봇의 현재 위치 (y 좌표)
float64 position_yaw        # 로봇의 현재 방향 (각도, 0-360) 