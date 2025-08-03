# 사람이 탐지되었는지 여부
# true: 탐지됨, false: 탐지 안됨
bool is_detected

# 탐지된 사람의 고유 추적 ID
# 탐지되지 않았을 경우 -1
int32 track_id

# 탐지 신뢰도 (0.0 ~ 1.0)
# YOLO 모델이 이 탐지를 얼마나 확신하는지에 대한 점수
float32 confidence

# 계산된 거리 (미터 단위)
# 로봇으로부터 사람까지의 추정 거리
float32 distance

# 카메라 중앙 기준 수평 오프셋
# -1.0: 화면 가장 왼쪽, 0.0: 정중앙, +1.0: 화면 가장 오른쪽
float32 horizontal_offset