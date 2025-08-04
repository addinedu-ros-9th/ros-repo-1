# Nav2 커스텀 플래너 구현 개요

## 1. 필요한 파일들 (5일 개발)

### A. 플래너 클래스 (C++) - 2일
```cpp
// waypoint_planner.hpp/cpp
class WaypointPlanner : public nav2_core::GlobalPlanner
{
public:
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;
    
private:
    // 기존 A* 로직 포팅
    std::vector<std::string> findPathAstar(const std::string& start, const std::string& goal);
    std::map<std::string, WaypointData> waypoints_;
    std::set<std::string> blocked_waypoints_;
};
```

### B. 플러그인 설정 - 1일
```xml
<!-- nav2_params.yaml -->
planner_server:
  planner_plugins: ["GridBased"]
  GridBased:
    plugin: "libo_navigator::WaypointPlanner"
    waypoint_file: "waypoints.yaml"
```

### C. 동적 재계획 서비스 - 1일
```cpp
// waypoint_manager.cpp
class WaypointManager : public rclcpp::Node
{
    // 장애물 감지 시 blocked_waypoints 업데이트
    // 플래너에 재계획 요청
};
```

### D. 빌드 및 테스트 - 1일

## 2. 장점들

### ✅ 완전한 Nav2 통합
- Nav2의 모든 기능(costmap, controller, recovery) 사용 가능
- RViz에서 경로 시각화 자동
- Nav2 라이프사이클 관리 자동

### ✅ 성능 향상
- 서비스 호출 오버헤드 제거
- 직접적인 costmap 접근
- 실시간 재계획 가능

### ✅ 기존 코드 재사용
- A* 알고리즘 그대로 포팅
- 웨이포인트 구조 동일 사용
- 헤딩 계산 로직 유지

## 3. 구현 전략

### Phase 1 (Day 1-2): 기본 플래너
```cpp
// 기존 파이썬 로직을 C++로 포팅
// waypoints.yaml 로딩
// A* 경로 생성
```

### Phase 2 (Day 3): 동적 기능
```cpp
// 장애물 감지 콜백
// blocked_waypoints 관리
// 실시간 재계획
```

### Phase 3 (Day 4): 헤딩 및 최적화
```cpp
// 헤딩 계산 추가
// 성능 최적화
// 로깅 추가
```

### Phase 4 (Day 5): 통합 테스트
```bash
# Gazebo 테스트
# 실제 로봇 테스트
# 디버깅 및 튜닝
```

## 4. 현재 시스템 대비 개선점

### 🔄 기존: 서비스 기반
```
ROS Service Call → 경로 계산 → followWaypoints()
```

### ⚡ 신규: 내장 플래너
```
Nav2 Goal → 커스텀플래너 → Path 직접 생성
```

## 5. 개발 권장사항

### ✅ 추천하는 이유:
1. **개발기간 충분**: 5일이면 여유롭게 가능
2. **성능 향상**: 통신 오버헤드 제거
3. **Nav2 생태계**: 완전한 통합으로 확장성 증대
4. **기존 로직 재사용**: Python → C++ 포팅만 하면 됨

### ⚠️ 고려사항:
1. **C++ 개발**: Python보다 개발 복잡도 증가
2. **Nav2 학습**: 플러그인 인터페이스 이해 필요
3. **디버깅**: 플러그인 디버깅이 독립 노드보다 어려움

## 6. 결론
**5일 개발기간으로 충분히 가능하며, 현재 시스템보다 성능과 통합성 면에서 훨씬 좋습니다!**
