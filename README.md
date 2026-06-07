# lgh_ws — quattro팀 4족 보행 로봇 ROS 2 워크스페이스

2026년 캡스톤디자인II quattro팀 ROS 2 워크스페이스입니다. 

기구학, Bezier 보행 생성, 상태 머신, 모터 제어(CAN, PWM), 센서(IMU), 텔레오퍼레이션, RViz2 시각화

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `lux` | 핵심 Python 패키지 — 기구학, 보행 생성, 상태 머신, 모터 제어, 센서, 텔레오퍼레이션 |
| `lux_msgs` | 커스텀 ROS 2 메시지 정의 (CMake/rosidl) |
| `lux_description` | URDF/Xacro 로봇 설명 및 RViz 시각화 |

## 전제 조건

- **ROS 2** (Jazzy 권장. Humble / Iron / Rolling 도 자동 인식됩니다.)
  - 설치: <https://docs.ros.org/en/jazzy/Installation.html>
- **Python 3** 및 `python3-venv`
- (실제 로봇 구동 시) CAN 인터페이스(`can0` @ 500 kbps), BNO085 IMU

## 1. 저장소 클론

SSH (권장 — 푸시 권한 필요 시):

```bash
git clone git@github.com:LuxPrestige/lgh_ws.git
cd lgh_ws
```

HTTPS:

```bash
git clone https://github.com/LuxPrestige/lgh_ws.git
cd lgh_ws
```

## 2. 설치

워크스페이스 루트에서 설치 스크립트를 실행합니다:

```bash
bash install.sh
```

스크립트가 자동으로 수행하는 작업:

1. ROS 2 환경 확인 및 자동 소싱 (`/opt/ros/<distro>/setup.bash`)
2. Python 가상환경 생성 — `~/lgh_ws/venv/` (`--system-site-packages`로 `rclpy` 등 ROS 2 패키지 공유)
3. `requirements.txt`의 pip 의존성 설치 (venv 내부)
4. `rosdep`으로 ROS 시스템 의존성 설치
5. `colcon build --symlink-install`로 빌드
6. 캘리브레이션 기본값 파일 복사 (`*_servo_calib.yaml.example` → `*_servo_calib.yaml`)

> **참고:** 설치 스크립트는 서브셸에서 실행되므로, 종료 후 venv가 현재 셸에 자동
> 활성화되지는 않습니다. 아래 "환경 소싱"을 따라 직접 활성화해야 합니다.

## 3. 환경 소싱 (새 터미널마다)

```bash
source /opt/ros/jazzy/setup.bash      # 본인 ROS 2 배포판에 맞게 수정
source ~/lgh_ws/venv/bin/activate
source ~/lgh_ws/install/setup.bash
```

매번 입력하기 번거로우면 `~/.bashrc`에 추가:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/lgh_ws/venv/bin/activate' >> ~/.bashrc
echo 'source ~/lgh_ws/install/setup.bash' >> ~/.bashrc
```

## 4. 실행

```bash
# 전체 시스템 실행 (joy, teleop, 상태 머신, spot commander, MIT publisher, IMU)
ros2 launch lux lux.launch.py
```

개별 노드 실행:

```bash
ros2 run lux spot_real_interface_ros2   # spot_commander (기구학 + 보행)
ros2 run lux spot_sm_py                 # 상태 머신
ros2 run lux teleop_node_py             # 조이스틱 텔레오퍼레이션
ros2 run lux mit_publisher_ros2         # CAN 모터 드라이버
ros2 run lux bno085_node_py             # IMU 센서
ros2 run lux motor_calibrator_ros2      # 모터 캘리브레이션 도구
ros2 run lux lux_pygame_dashboard       # pygame 대시보드
```

## 빌드 & 테스트

```bash
colcon build                            # 전체 빌드
colcon build --packages-select lux      # 단일 패키지 빌드
source install/setup.bash               # 빌드 후 소싱

colcon test                             # 전체 테스트
colcon test-result --verbose            # 결과 확인

ament_flake8 src/lux/lux/               # 린팅
```

## Git 작업 워크플로

```bash
# 최신 변경 가져오기
git pull origin main

# 새 작업 브랜치 생성
git switch -c feature/<브랜치명>

# 변경 사항 커밋
git add -A
git commit -m "설명 메시지"

# 원격에 푸시
git push -u origin feature/<브랜치명>
```

기본 원격 저장소: `git@github.com:LuxPrestige/lgh_ws.git`

## 참고

- 기계별 캘리브레이션 파일(`*_servo_calib.yaml`)은 `.gitignore`에 의해 버전 관리에서
  제외됩니다. 설치 시 `*.example` 템플릿에서 자동 복사됩니다.
- 아키텍처 및 노드/토픽 상세는 `CLAUDE.md`를 참고하세요.
