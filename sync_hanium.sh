#!/usr/bin/env bash
# sync_hanium.sh — 공유폴더(정본) → VM 작업폴더 동기화 + 빌드
#
#   정본: /media/sf_hanium/hanium_ws/src   (윈도우 C:\hanium\hanium_ws)
#   빌드: ~/hanium_ws                       (VM 네이티브 디스크)
#
#   공유폴더(vboxsf)에서 직접 colcon build 하면 심볼릭 링크를 지원하지 않아
#   깨지므로, 소스만 받아오고 빌드는 VM 디스크에서 한다.
#
# 설치 (최초 1회):
#   cp /media/sf_hanium/hanium_ws/sync_hanium.sh ~/
#   chmod +x ~/sync_hanium.sh
#
# 사용:
#   ~/sync_hanium.sh          # 동기화 + 변경 패키지만 빌드
#   ~/sync_hanium.sh --clean  # build/install 지우고 전체 재빌드
#   ~/sync_hanium.sh --dry    # 뭐가 바뀌는지 확인만 (실제 복사 안 함)

set -u
SRC=/media/sf_hanium/hanium_ws/src
WS=$HOME/hanium_ws
PKGS="smart_upcare smart_upcare_control"

if [ ! -d "$SRC" ]; then
  echo "!! 공유폴더를 못 찾음: $SRC"
  echo "   VirtualBox 공유폴더가 마운트됐는지 확인: ls /media/"
  return 1 2>/dev/null || exit 1
fi

RSYNC_OPT="-rt --delete --exclude=__pycache__ --exclude=*.pyc"

# ── --dry : 미리보기만 ──
if [ "${1:-}" = "--dry" ]; then
  echo ">> 변경 예정 목록 (실제 복사 안 함)"
  rsync $RSYNC_OPT --dry-run -i "$SRC"/ "$WS"/src/ | grep -v '^\.d' || echo "   (변경 없음)"
  return 0 2>/dev/null || exit 0
fi

# ── 실행 중인 Gazebo 정리 ──
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
sleep 1

echo ">> [1/3] 공유폴더 → VM 동기화"
rsync $RSYNC_OPT "$SRC"/ "$WS"/src/ || { echo "!! rsync 실패"; exit 1; }

cd "$WS" || exit 1

if [ "${1:-}" = "--clean" ]; then
  echo ">> [2/3] build/install/log 삭제 후 전체 재빌드"
  rm -rf build install log
else
  echo ">> [2/3] 증분 빌드"
fi

colcon build --packages-select $PKGS || { echo "!! 빌드 실패"; exit 1; }

echo ">> [3/3] 설치 확인"
echo "   실행파일:"
ls "$WS"/install/smart_upcare_control/lib/smart_upcare_control/ 2>/dev/null | sed 's/^/     /'
echo "   월드:"
ls "$WS"/install/smart_upcare/share/smart_upcare/worlds/*.world 2>/dev/null \
   | xargs -n1 basename | sed 's/^/     /'

cat <<'EOS'

>> 완료. 새 터미널마다 아래를 실행해야 반영된다:

     source ~/hanium_ws/install/setup.bash

>> 자주 쓰는 실행:

     # 헤드리스 (SLAM/튜닝용, 가장 빠름)
     ros2 launch smart_upcare_control gazebo_4bed.launch.py gui:=false
     ros2 service call /unpause_physics std_srvs/srv/Empty {}

     # 저부하 (스캔 스톨 생길 때)
     ros2 launch smart_upcare_control gazebo_4bed.launch.py \
         gui:=false lidar_rate:=5 imu_rate:=50

     # GUI (영상 녹화용) — 창은 절반 크기로 줄일 것
     ros2 launch smart_upcare_control gazebo_4bed.launch.py

EOS
