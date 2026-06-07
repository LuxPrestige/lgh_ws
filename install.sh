#!/bin/bash
# lux_ws 설치 스크립트
# 사용법: bash install.sh
# 전제 조건: ROS 2 Jazzy (또는 동등 배포판)가 시스템에 설치되어 있어야 합니다.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── 1. ROS 2 환경 확인 ────────────────────────────────────────────
if [ -z "$ROS_DISTRO" ]; then
    # 일반적인 경로에서 자동 소싱 시도
    for distro in jazzy humble iron rolling; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            # shellcheck source=/dev/null
            source "/opt/ros/$distro/setup.bash"
            echo "[OK] ROS 2 $distro 자동 소싱 완료"
            break
        fi
    done
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS 2가 설치되어 있지 않거나 소싱되지 않았습니다."
    echo "  설치 방법: https://docs.ros.org/en/jazzy/Installation.html"
    echo "  소싱 방법: source /opt/ros/<distro>/setup.bash"
    exit 1
fi
echo "[OK] ROS 2 $ROS_DISTRO 확인"

# ── 2. Python venv 생성 (ROS 2 시스템 패키지 상속) ───────────────
# --system-site-packages: rclpy 등 ROS 2 Python 패키지를 venv 내에서 사용 가능하게 함
if [ ! -d "venv" ]; then
    python3 -m venv venv --system-site-packages
    echo "[OK] venv 생성 완료 (--system-site-packages)"
else
    echo "[OK] 기존 venv 사용"
fi

# shellcheck source=/dev/null
source venv/bin/activate
echo "[OK] venv 활성화"

# ── 3. pip 의존성 설치 ────────────────────────────────────────────
pip install --upgrade pip --quiet
pip install -r requirements.txt
echo "[OK] pip 의존성 설치 완료"

# ── 4. ROS 2 시스템 의존성 설치 (rosdep) ─────────────────────────
if command -v rosdep &> /dev/null; then
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        sudo rosdep init 2>/dev/null || true
    fi
    rosdep update --quiet 2>/dev/null || true
    rosdep install --from-paths src --ignore-src -r -y
    echo "[OK] ROS 의존성 설치 완료"
else
    echo "[WARN] rosdep 없음 — ROS 의존성 수동 확인 필요"
fi

# ── 5. colcon 빌드 ────────────────────────────────────────────────
colcon build --symlink-install
echo "[OK] colcon 빌드 완료"

# ── 6. 캘리브레이션 파일 기본값 복사 ─────────────────────────────
for robot in spot gim; do
    src="src/lux/config/${robot}_servo_calib.yaml.example"
    dst="src/lux/config/${robot}_servo_calib.yaml"
    if [ -f "$src" ] && [ ! -f "$dst" ]; then
        cp "$src" "$dst"
        echo "[OK] ${dst} 기본값 복사 완료 (캘리브레이션 후 수정 필요)"
    fi
done

# ── 완료 메시지 ───────────────────────────────────────────────────
echo ""
echo "================================================================"
echo " 설치 완료!"
echo ""
echo " 새 터미널마다 아래 명령을 실행하세요:"
echo "   source /opt/ros/$ROS_DISTRO/setup.bash"
echo "   source $SCRIPT_DIR/venv/bin/activate"
echo "   source $SCRIPT_DIR/install/setup.bash"
echo ""
echo " ~/.bashrc에 추가하면 자동으로 적용됩니다:"
echo "   echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc"
echo "   echo 'source $SCRIPT_DIR/venv/bin/activate' >> ~/.bashrc"
echo "   echo 'source $SCRIPT_DIR/install/setup.bash' >> ~/.bashrc"
echo "================================================================"
