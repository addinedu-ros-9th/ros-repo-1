# Task 상태를 나타내는 메시지

string robot_id           # 작업을 수행하는 로봇 ID (예: libo_a, libo_b, libo_c)
string task_type          # 작업 유형 (escort, delivery, assist)
int32 task_stage          # 작업 단계 (1: 시작, 2: 진행중, 3: 완료)
string call_location      # 요청 위치 (예: a1, b2, c3 등 웨이포인트)
string goal_location      # 목표 위치 (예: d4, e5, f6 등 웨이포인트)
builtin_interfaces/Time start_time   # 작업 시작 시간
builtin_interfaces/Time end_time     # 작업 종료 시간 (진행중일 때는 0) 