
| **Index** | **Scenario** | **Description**                          | **Sender**    | **Receiver**  | **Type**    | **Name**         | **Type name**         | **Example Message** |
| --------- | ------------ | ---------------------------------------- | ------------- | ------------- | ----------- | ---------------- | --------------------- | ------------------- |
|           | 최초 실행        | 프로그램이 최초로 실행된 뒤 연결 상태를 서로에게 신호를 보내며 알림   | Libo Service  | all           | Topic -rate | /main_heartbeat  | Heartbeat.msg         |                     |
|           |              |                                          | Libo Operator | all           | Topic -rate | /robot_heartbeat | Heartbeat.msg         |                     |
|           |              |                                          | Admin GUI     | all           | Topic -rate | /admin_heartbeat | Heartbeat.msg         |                     |
|           | 시스템 상태 공유    | Libo Service가 GUI로 전체 상황에 대한 상태를 전달하는 과정 | Libo Service  | Admin GUI     | Topic -rate | /task_status     | TaskStatus.msg        |                     |
|           | 로봇 상태 공유     | 로봇에서 GUI 까지 상태 정보를 전달하는 과정               | Libo Operator | all           | Topic -rate | /robot_status    | RobotStatus.msg       |                     |
|           | 로봇 주행 명령     | 이동 명령 요청                                 | Admin GUI     | Libo Service  | Topic -once | /navigate        | Navigate.msg          |                     |
|           |              | 목표지 도착 알림                                | Libo Service  | Admin GUI     | Topic -once | /robot_arrival   | NavigateComplete.msg  |                     |
|           |              | 이동 명령 전달                                 | Libo Service  | Libo Operator | Action      | /navigate        | NavigateToPose.action |                     |
|           | 도서검색         | 키오스크에서 도서 검색 요청                          | Kiosk UI      | Main Server   | Service     | /book_search     | BookSearch.srv        |                     |
|           | 에스코팅 요청      | 키오스크에서 로봇 에스코팅 요청                        | Kiosk UI      | Libo Service  | Service     | /escort_request  | EscortRequest.srv     |                     |
|           | 에스코팅상태       | 에스코팅 진행 상태 전달                            | Libo Service  | all           | Topic -rate | /escort_status   | EscortStatus.msg      |                     |
|           | 에스코팅 완료      | 에스코팅 완료 알림                               | Libo Service  | all           | Topic -once | /escort_complete | EscortComplete.msg    |                     |


# Heartbeat.msg

| **type**                | **name**  | **value**                    |
| ----------------------- | --------- | ---------------------------- |
| string                  | sender_id | robot_01, admin_01, kiosk_01 |
| builtin_interfaces/Time | timestamp | sec: 1721794567              |
# TaskStatus.msg

| **type**                | **name**    | **value**                    |
| ----------------------- | ----------- | ---------------------------- |
| string                  | robot_id    | robot_01, robot_02           |
| string                  | mode        | admin_mode, idle, navigating |
| string                  | waypoint_id | A1, A2, A3, B1, B2           |
| string                  | state       | executing, completed, failed |
| builtin_interfaces/Time | timestamp   | sec: 1721794567              |
# RobotStatus.msg

| **type**                | **name**      | **value**                               |
| ----------------------- | ------------- | --------------------------------------- |
| string                  | sender_id     | robot_01, robot_02                      |
| string                  | availability  | available, unavailalbe                  |
| string                  | mode          | admin_mode, idle, navigating, escorting |
| float32                 | battery_level | 0.0 ~ 100.0                             |
| builtin_interfaces/Time | timestamp     | sec: 1721794567                         |
# TaskStatus.msg

| **type**                | **name**    | **value**                         |
| ----------------------- | ----------- | --------------------------------- |
| string                  | robot_id    | robot_01, robot_02                |
| string                  | mode        | 모드 (admin_mode, idle, navigating) |
| string                  | waypoint_id | 웨이포인트 ID (A1, A2, A3, B1, B2)     |
| string                  | state       | 상태 (executing, completed, failed) |
| builtin_interfaces/Time | timestamp   |                                   |
# Navigate.srv

| **type** | **name**    | **value**          |
| -------- | ----------- | ------------------ |
| string   | robot_id    | robot_01, robot_02 |
| string   | waypoint_id | A1, A2, A3, B1, B2 |
# NavigateComplete.msg

| **type**                | **name**    | **value**          |
| ----------------------- | ----------- | ------------------ |
| string                  | robot_id    | robot_01, robot_02 |
| string                  | waypoint_id | 도착한 웨이포인트 ID       |
| builtin_interfaces/Time | timestamp   | 도착 시간              |
# **NavigateToPose.action**

| **type**  | **name**    | **value**                                 |
| --------- | ----------- | ----------------------------------------- |
| #goal     |             |                                           |
| string    | robot_id    | 로봇 ID                                     |
| string    | waypoint_id | 웨이포인트 ID (A1, A2, A3, B1, B2, D3, D5, E3) |
| float32   | target_x    | 목표 x 좌표                                   |
| float32   | target_y    | 목표 y 좌표                                   |
| float32   | target_yaw  | 목표 yaw 각도                                 |
| ---       |             |                                           |
| #result   |             |                                           |
| bool      | success     | 이동 성공 여부                                  |
| string    | message     | 결과 메시지                                    |
| float32   | final_x     | 최종 도착 x 좌표                                |
| float32   | final_y     | 최종 도착 y 좌표                                |
| float32   | final_yaw   | 최종 도착 yaw 각도                              |
| ---       |             |                                           |
| #feedback |             |                                           |
| float32   | current_x   | 현재 x 좌표                                   |
| float32   | current_y   | 현재 y 좌표                                   |
| float32   | current_yaw | 현재 yaw 각도                                 |
| float32   | progress    | 진행률 (0.0 ~ 1.0)                           |
| string    | status      | 상태: "moving", "arrived", "failed"         |
# **EscortRequest.srv** (새로 추가)

| **type**  | **name**      | **value**         |
| --------- | ------------- | ----------------- |
| #request  |               |                   |
| string    | robot_id      | 로봇 ID (robot_01)  |
| string    | book_title    | 도서 제목             |
| string    | book_location | 도서 위치 코드 (D3, D5) |
| ---       |               |                   |
| #response |               |                   |
| bool      | success       | 요청 수락 여부          |
| string    | message       | 응답 메시지            |
| string    | escort_id     | 에스코팅 세션 ID        |
# **EscortStatus.msg** (새로 추가)

| **type**                | **name**         | **value**                                                    |
| ----------------------- | ---------------- | ------------------------------------------------------------ |
| string                  | escort_id        | 에스코팅 세션 ID                                                   |
| string                  | robot_id         | 로봇 ID                                                        |
| string                  | status           | 상태: "moving_to_kiosk", "escorting", "completed", "returning" |
| string                  | current_location | 현재 위치                                                        |
| string                  | target_location  | 목표 위치                                                        |
| float32                 | progress         | 진행률 (0.0 ~ 1.0)                                              |
| builtin_interfaces/Time | timestamp        |                                                              |
# **EscortComplete.msg** (새로 추가)

| **type**                | **name**       | **value**  |
| ----------------------- | -------------- | ---------- |
| string                  | escort_id      | 에스코팅 세션 ID |
| string                  | robot_id       | 로봇 ID      |
| string                  | book_title     | 도서 제목      |
| string                  | final_location | 최종 도착 위치   |
| builtin_interfaces/Time | timestamp      |            |
