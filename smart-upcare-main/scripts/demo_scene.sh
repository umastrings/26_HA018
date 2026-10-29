#!/bin/bash
# 영상 촬영용: 장면을 정해진 시간표대로 자동 실행한다.
# 사용법:  ./demo_scene.sh patrol    (순회 시작 → 대기)
#          ./demo_scene.sh bedsore   (욕창 알림 → 해제)
#          ./demo_scene.sh fall      (낙상 대응 → 해제)
#          ./demo_scene.sh estop     (비상정지 → 해제)  ※ LED·음성 반응만, 모터 정지는 미구현
#          ./demo_scene.sh all       (위 4개를 한 영상으로 이어서)
# 노드 3개(LED, 음성, manager)가 먼저 켜져 있어야 한다.

source /opt/ros/humble/setup.bash

COUNTDOWN=${COUNTDOWN:-5}   # 시작 전 카운트다운(초): 촬영 버튼 누를 시간
HOLD=${HOLD:-15}            # 알림 상태 유지 시간(초)

pub_bool () {   # $1 = 토픽, $2 = true/false
  ros2 topic pub --once "$1" std_msgs/msg/Bool "{data: $2}" > /dev/null
}
pub_mode () {   # $1 = IDLE / PATROL
  ros2 topic pub --once /care/mode std_msgs/msg/String "{data: $1}" > /dev/null
}
countdown () {
  for i in $(seq "$COUNTDOWN" -1 1); do echo "  시작까지 $i초..."; sleep 1; done
}

scene_patrol () {
  echo "▶ [순회] 순회 시작";       pub_mode PATROL
  sleep 8
  echo "▶ [순회] 도크 복귀(대기)"; pub_mode IDLE
  sleep 3
}
scene_bedsore () {
  echo "▶ [욕창] 욕창 위험 발생";      pub_bool /care/bedsore true
  sleep "$HOLD"
  echo "▶ [욕창] 대시보드 확인(해제)"; pub_bool /care/bedsore false
  sleep 4
}
scene_fall () {
  echo "▶ [낙상] 낙상 감지"; pub_bool /care/fall true
  sleep "$HOLD"
  echo "▶ [낙상] 상황 해제"; pub_bool /care/reset true
  sleep 4
}
scene_estop () {
  echo "▶ [비상정지] 비상정지 작동"; pub_bool /care/estop_sw true
  sleep 8
  echo "▶ [비상정지] 해제";          pub_bool /care/estop_sw false
  sleep 4
}

case "$1" in
  patrol)  echo "[장면] 순회";            countdown; scene_patrol ;;
  bedsore) echo "[장면] 욕창 알림";       countdown; scene_bedsore ;;
  fall)    echo "[장면] 낙상 (카운트다운 끝나면 쓰러지기)"; countdown; scene_fall ;;
  estop)   echo "[장면] 비상정지";        countdown; scene_estop ;;
  all)
    echo "[전체] 순회 → 욕창 → 낙상 → 비상정지 (낙상 차례에 쓰러지기)"
    countdown
    scene_patrol
    scene_bedsore
    echo "  ※ 다음은 낙상: 3초 후 쓰러지기"; sleep 3
    scene_fall
    scene_estop
    ;;
  *)
    echo "사용법: $0 patrol | bedsore | fall | estop | all"
    echo "옵션:  HOLD=20 $0 fall   (알림 유지 시간 바꾸기)"
    exit 1
    ;;
esac
echo "끝"
