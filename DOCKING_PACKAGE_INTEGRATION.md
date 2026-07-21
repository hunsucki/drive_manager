# 원격 도킹 패키지 연동 규격

이 문서는 `drive_manager` 컨테이너와 로봇의 라즈베리파이에서 실행되는
`docking` 패키지 사이의 현재 통신 방식, 도킹 패키지가 지켜야 할 실행 계약,
안전 제약과 시험 절차를 정의합니다.

대상 원격 실행 파일은 현재 다음과 같습니다.

```bash
ros2 run docking dock_turn_backup
```

## 핵심 요약

- 도킹 요청은 ROS 2 action/service가 아니라 **SSH 원격 프로세스 실행**으로 전달됩니다.
- `drive_manager`는 원격 프로그램의 stdout 메시지나 ROS 토픽을 성공 신호로 읽지 않습니다.
- 원격 프로세스가 **종료 코드 0으로 끝나는 것만** 도킹 성공으로 판단합니다.
- 종료 코드 0은 단순한 동작 완료가 아니라 실제 체결 및 가능하면 충전 확인까지 끝났다는 뜻이어야 합니다.
- 실패, 센서 오류, 시간 초과, 체결 불확실 상태는 반드시 0이 아닌 종료 코드로 끝나야 합니다.
- 원격 프로그램은 포그라운드에서 실행되어야 하며 자식 프로세스를 분리하거나 daemonize하면 안 됩니다.
- 도킹 중 Nav2 navigation은 PAUSE 상태이므로 원격 도킹 패키지가 속도 및 충돌 안전을 책임집니다.
- 도킹 패키지가 `/cmd_vel`을 사용한다면 도킹 중 유일한 활성 속도 발행자여야 합니다.
- `STOP`/`ESTOP` 또는 SSH 연결 종료 시 로봇을 즉시 정지시키는 로봇 측 watchdog이 필요합니다.

## Nav2 docking_server와 원격 docking 패키지의 차이

현재 Nav2 구성에는 `/docking_server`도 포함되어 있지만, START/HOME 미션의 마지막
도킹 단계는 Nav2의 `DockRobot` action을 호출하지 않습니다. 실제 미션 경로는
라즈베리파이의 별도 패키지를 SSH로 실행합니다.

| 구분 | 현재 미션에서 사용 여부 | 역할 |
| --- | --- | --- |
| Nav2 `/docking_server` | 사용하지 않음 | Nav2 bringup에 포함된 OpenNav docking server |
| 원격 `docking/dock_turn_backup` | 사용함 | 라즈베리파이에서 최종 체결 동작 수행 |

두 구현의 토픽이나 파라미터가 보인다고 해서 같은 도킹 경로라고 가정하면 안 됩니다.

## 전체 동작 흐름

```text
웹 앱
  │ /robot_command: START 또는 HOME
  ▼
command_manager
  │ /mission_command
  ▼
mission_driver
  │
  ├─ Nav2로 home_to_dock_pose까지 이동
  ├─ navigation lifecycle PAUSE 확인
  ├─ /mission_status: DOCKING
  │
  └─ SSH TCP/22
       │ source /opt/ros/jazzy/setup.bash
       │ source /home/user/colcon_ws/install/setup.bash
       │ exec ros2 run docking dock_turn_backup
       ▼
     라즈베리파이 docking 프로세스
       │
       ├─ 도킹 성공 → 로봇 정지 → exit 0
       └─ 실패/불확실 → 로봇 정지 → exit non-zero
```

종료 코드를 받은 뒤 `drive_manager`는 다음처럼 처리합니다.

```text
exit 0
  -> 도킹 성공
  -> localization/navigation RESET
  -> /robot_pose에 docked_pose 발행
  -> /robot_pose_status: DOCKED
  -> /robot_status: DOCKED_NAV2_INACTIVE
  -> 최종 SUCCEEDED START 또는 SUCCEEDED HOME

exit non-zero, SSH 실패 또는 timeout
  -> 도킹 실패
  -> /robot_status: FAILED START 또는 FAILED HOME
  -> docked_pose로 전환하지 않음
```

`DOCKED_NAV2_INACTIVE`는 원격 프로그램이 종료 코드 0을 반환한 뒤에만 정상적으로
발행됩니다. 다만 아래의 "현재 구현상 주의점"에 설명한 placeholder 성공 처리도
있으므로 운영 설정 검증이 필요합니다.

## 실제 SSH 명령

현재 설정은 [param/mission_config.yaml](param/mission_config.yaml)의 다음 값으로
구성됩니다.

```yaml
mission_driver:
  ros__parameters:
    docking_mode: "ssh"
    docking_ssh_user: "user"
    docking_ssh_host: "192.168.0.3"
    docking_ssh_port: 22
    docking_ssh_identity_file: "/root/.ssh/id_ed25519_drive_manager"
    docking_ssh_strict_host_key_checking: "accept-new"
    docking_remote_setup_files:
      - "/opt/ros/jazzy/setup.bash"
      - "/home/user/colcon_ws/install/setup.bash"
    docking_remote_command: "ros2 run docking dock_turn_backup"
    docking_timeout_sec: 120.0
    docking_stop_grace_sec: 3.0
```

이를 개념적으로 풀면 다음 명령과 같습니다.

```bash
ssh \
  -o BatchMode=yes \
  -o StrictHostKeyChecking=accept-new \
  -p 22 \
  -i /root/.ssh/id_ed25519_drive_manager \
  user@192.168.0.3 \
  "bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/user/colcon_ws/install/setup.bash && exec ros2 run docking dock_turn_backup'"
```

실제 코드는 각 setup 파일 경로를 shell escaping한 뒤 `&&`로 연결합니다. 앞 단계가
실패하면 뒤의 setup 또는 도킹 실행 파일은 시작되지 않습니다.

### SSH 관련 제약

- `BatchMode=yes`이므로 비밀번호나 key passphrase를 대화형으로 입력할 수 없습니다.
- 컨테이너의 공개키가 라즈베리파이 사용자의 `~/.ssh/authorized_keys`에 등록되어야 합니다.
- 개인키는 컨테이너 안의 설정 경로에 존재하고 SSH가 읽을 수 있는 권한이어야 합니다.
- 암호화된 개인키를 사용한다면 컨테이너에서 접근 가능한 ssh-agent가 별도로 필요합니다.
- `accept-new`는 최초 호스트 키는 등록하지만 기존 키가 바뀌면 접속을 거부합니다.
- 컨테이너의 `known_hosts`를 사용하는 계정과 실제 launch 실행 계정이 같아야 합니다.
- 컨테이너에서 라즈베리파이 TCP 22번 포트로 접근할 수 있어야 합니다.
- `docking_remote_command`는 운영자가 관리하는 신뢰된 정적 설정이어야 합니다. 웹 입력이나 사용자 문자열을 이어 붙이면 안 됩니다.

## SSH와 ROS 2 통신은 별개입니다

SSH는 라즈베리파이에서 프로그램을 시작하고 종료 코드를 돌려받는 제어 채널일 뿐,
ROS 2 토픽을 전달하거나 터널링하지 않습니다.

원격 docking 노드가 라즈베리파이 내부의 센서와 베이스 드라이버만 사용한다면 로봇
내부 ROS graph만 정상이어도 됩니다. 반대로 컨테이너의 토픽이나 다른 컴퓨터의
ROS 노드와 통신해야 한다면 다음 DDS 조건을 별도로 맞춰야 합니다.

- 양쪽 `ROS_DOMAIN_ID`
- RMW 구현과 DDS 설정
- multicast/discovery가 가능한 네트워크
- 방화벽 및 컨테이너 네트워크 모드
- 토픽 namespace와 remapping
- QoS 호환성

SSH 비대화형 shell은 사용자의 평소 interactive `.bashrc`와 환경이 다를 수 있습니다.
필요한 `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, DDS profile은 setup 파일이나 원격 실행
wrapper에서 명시적으로 설정하는 편이 안전합니다.

예를 들어 별도 wrapper를 쓰는 경우입니다.

```bash
#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/user/colcon_ws/install/setup.bash

export ROS_DOMAIN_ID=0
exec ros2 run docking dock_turn_backup
```

이 wrapper를 설치한 뒤 `docking_remote_command`가 wrapper를 실행하도록 설정할 수
있습니다. `ROS_DOMAIN_ID=0`은 예시이므로 실제 로봇 bringup과 같은 값으로 맞춰야 합니다.

## 원격 docking 실행 파일의 필수 계약

### 1. 반드시 포그라운드 실행

다음 형태는 허용하지 않습니다.

```bash
ros2 run docking dock_turn_backup &
nohup ros2 run docking dock_turn_backup ...
systemd-run --no-block ...
```

SSH가 감시하는 프로세스와 실제 모터 제어 프로세스가 분리되면 `drive_manager`는
성공, 실패, 중단을 판단할 수 없고 STOP 시 원격 프로세스가 남을 수 있습니다.

`exec ros2 run ...`으로 실행한 프로세스 자체가 최종 도킹이 끝날 때까지 살아 있어야
합니다. 내부에서 작업 스레드나 자식 프로세스를 만들더라도 부모는 완료를 기다리고,
종료 시 모두 정리해야 합니다.

### 2. 종료 코드가 성공 프로토콜

`drive_manager`가 해석하는 값은 다음 둘뿐입니다.

| 종료 결과 | 의미 |
| --- | --- |
| `0` | 물리적 도킹 성공이 확정됨 |
| `0 이외` | 도킹 실패, 중단, 센서 오류 또는 성공 불확실 |

다음 상태를 모두 만족한 뒤에만 0을 반환하는 것을 권장합니다.

1. 최종 위치와 방향 조건 만족
2. 모터 정지 명령 전송
3. 도킹 접촉 센서 또는 충전 상태 확인
4. 충전 신호가 짧은 순간값이 아니라 설정 시간 동안 안정적으로 유지됨
5. 모든 publisher/timer/모터 제어 정리 완료

단지 회전·후진 시퀀스를 끝냈다는 이유만으로 0을 반환하면 실제로 체결되지 않은
로봇을 서버와 웹이 `DOCKED`로 표시하게 됩니다.

권장 종료 코드 예시는 다음과 같습니다. 서버는 현재 숫자별 원인을 구분하지 않지만,
라즈베리파이 로그 분석에는 도움이 됩니다.

| 코드 | 권장 의미 |
| ---: | --- |
| 0 | 체결 및 충전 확인 성공 |
| 2 | 입력/파라미터 오류 |
| 3 | 필수 센서 또는 베이스 제어 연결 실패 |
| 4 | 도킹 탐색/정렬 실패 |
| 5 | 내부 시간 초과 |
| 6 | 접촉했지만 충전 확인 실패 |
| 130 | SIGINT에 의한 사용자 중단 |

### 3. 내부 timeout은 서버 timeout보다 짧게 설정

서버는 기본 120초 후 로컬 SSH 프로세스를 중단합니다. 원격 docking 패키지도 이보다
짧은 내부 제한 시간을 가져야 합니다. 예를 들어 전체 도킹 제한을 100초로 두고,
탐색·회전·후진·충전 확인 단계마다 더 짧은 제한을 두는 방식입니다.

내부 timeout이 필요한 이유는 SSH 연결이 끊어졌을 때 서버의 종료 동작만으로 원격
프로세스 종료가 완전히 보장되지 않기 때문입니다.

### 4. 모든 종료 경로에서 0 속도 보장

정상 성공, 일반 실패, 예외, timeout, SIGINT, SIGTERM, SIGHUP에서 모두 다음을
수행해야 합니다.

1. 새 움직임 명령 생성 중단
2. `/cmd_vel` 0을 여러 회 발행하거나 베이스의 정지 API 호출
3. 모터 드라이버 watchdog이 0 명령 또는 명령 단절을 확인할 때까지 짧게 대기
4. 그 뒤 ROS node와 프로세스 종료

프로세스가 비정상 종료되더라도 베이스가 일정 시간 이후 자동 정지하도록 로봇 측
속도 명령 watchdog도 반드시 활성화해야 합니다. 네트워크나 SSH 안전은 물리적
비상정지 장치를 대신할 수 없습니다.

### 5. 신호와 SSH 연결 종료 처리

`mission_driver`는 STOP/ESTOP/컨테이너 종료/120초 timeout 시 로컬 SSH 프로세스
그룹에 다음 순서로 신호를 보냅니다.

```text
SIGINT
-> docking_stop_grace_sec 동안 대기(기본 3초)
-> SIGTERM
-> 1초 대기
-> SIGKILL
```

이 신호는 우선 로컬 SSH 클라이언트에 전달됩니다. 원격 프로세스에 동일한 신호가
그대로 도착하는 것은 SSH 구현과 원격 shell 상태에 따라 보장되지 않습니다. 원격
docking 패키지는 최소한 다음을 모두 중단 조건으로 처리해야 합니다.

- SIGINT
- SIGTERM
- SIGHUP
- ROS context shutdown
- 제어 heartbeat 또는 명령 갱신 단절

원격 프로세스가 SSH 종료 후에도 살아남을 수 있는 구조라면 현재 방식만으로는 안전하지
않습니다. 그런 구현에는 별도의 원격 stop service/topic, heartbeat watchdog 또는
systemd service의 명시적 start/stop 연동이 추가로 필요합니다.

### 6. 중복 실행 방지와 재실행 가능성

네트워크 장애 후 원격 프로세스가 남아 있는 상태에서 HOME을 다시 누르면 두 docking
노드가 동시에 `/cmd_vel`을 발행할 수 있습니다. 원격 패키지는 다음 중 하나로 중복
실행을 거부해야 합니다.

- systemd unit의 단일 인스턴스 보장
- PID/lock 파일과 실제 PID 생존 확인
- ROS node/service 기반 active 상태 확인
- 베이스 제어권 lease

이전 실행이 실패한 뒤 다시 실행해도 안전한 초기 상태에서 시작하는 idempotent한
동작이어야 합니다. stale lock은 자동으로 판별하고 복구해야 합니다.

## `/cmd_vel` 소유권과 충돌 제약

Nav2 경로는 평상시에 다음과 같습니다.

```text
Nav2 controller
-> /cmd_vel_nav
-> velocity_smoother
-> collision_monitor
-> /cmd_vel
```

도킹 직전 `mission_driver`는 navigation lifecycle을 PAUSE합니다. 따라서 원격
docking 노드가 `/cmd_vel`을 직접 발행한다면 Nav2 smoother와 collision monitor를
통과하지 않으며, 장애물 안전도 원격 패키지의 책임입니다.

ROS 2에서 같은 토픽의 여러 publisher 사이에는 자동 속도 arbitration이 없습니다.
두 노드가 `/cmd_vel`을 동시에 발행하면 베이스가 도착 순서대로 서로 다른 명령을
받게 됩니다. 도킹 중에는 다음 조건을 지켜야 합니다.

- Nav2 navigation이 PAUSE되기 전에는 원격 docking을 시작하지 않음
- docking 실행 중 웹 safe/force 수동 조종을 시작하지 않음
- 별도의 teleop 또는 테스트 publisher를 실행하지 않음
- 실패 및 종료 뒤 `/cmd_vel` 0을 남김
- 필요하면 향후 twist_mux 같은 명시적 arbitration 계층 도입

특히 현재 `web_teleop`은 Nav2 goal은 취소하지만 실행 중인 SSH docking 프로세스까지
자동으로 중단하지 않습니다. 웹 앱은 `/robot_status`가 `DOCKING`일 때 수동 조종
버튼을 비활성화해야 합니다. 수동 개입이 꼭 필요하면 먼저 `STOP`을 보내고
`STOPPED`와 로봇 정지를 확인한 뒤 수동 조종을 시작해야 합니다.

## 도킹 패키지가 사용하는 ROS 인터페이스

원격 `docking` 패키지 소스는 이 저장소에 포함되어 있지 않으므로 실제 구독/발행
토픽은 이 문서에서 확정할 수 없습니다. 최소한 다음 항목을 도킹 패키지 README에
명시하고 실제 ROS graph에서 검증해야 합니다.

| 구분 | 확인할 내용 |
| --- | --- |
| 속도 출력 | `/cmd_vel`인지, 다른 토픽이면 어디에서 remap하는지 |
| 베이스 상태 | odometry, wheel state 또는 motor feedback 토픽 |
| 근접 센서 | LiDAR, ultrasonic, IR, AprilTag 등 실제 토픽과 QoS |
| 도킹 확인 | contact GPIO, 충전 전압/전류, battery state 등 성공 판정 입력 |
| TF | 필요한 frame과 transform 공급자 |
| 시간 | system time인지 ROS time인지, `use_sim_time` 값 |
| 안전 입력 | bumper, cliff, ESTOP, heartbeat/watchdog |

도킹 패키지가 컨테이너 쪽 토픽을 사용하지 않고 로봇 로컬 장치만 사용한다면 그 사실도
명시해야 합니다. 반대로 컨테이너 토픽에 의존한다면 DDS 단절을 즉시 실패 및 정지로
처리해야 합니다.

## 상태와 오류가 웹 앱에 전달되는 방식

원격 docking 패키지의 로그나 내부 상태는 현재 웹 앱으로 직접 전달되지 않습니다.
웹 앱은 `drive_manager`가 발행하는 `/robot_status`를 봅니다.

| 대표 상태 | 의미 |
| --- | --- |
| `NAVIGATING HOME_TO_DOCK` | Nav2로 도킹 전 위치에 접근 중 |
| `DOCKING` | navigation PAUSE 후 SSH 원격 프로세스 실행 중 |
| `DOCKED_NAV2_INACTIVE` | 원격 프로세스가 성공했고 Nav2 reset 단계 수행됨 |
| `SUCCEEDED START` | 전체 순회·복귀·도킹 성공 |
| `SUCCEEDED HOME` | HOME 복귀·도킹 성공 |
| `FAILED START` | START 흐름의 도킹 또는 이전 단계 실패 |
| `FAILED HOME` | HOME 흐름의 도킹 또는 이전 단계 실패 |
| `STOPPED` | STOP으로 현재 동작 중단 |
| `ESTOP_LATCHED` | ESTOP으로 중단, RESET 전 재시작 금지 |

현재는 원격 종료 코드의 세부 의미를 `/robot_status`에 포함하지 않습니다. 예를 들어
센서 실패와 충전 확인 실패가 모두 `FAILED HOME`으로 보입니다. 세부 원인이 웹 앱에
필요하다면 향후 아래 중 하나를 추가해야 합니다.

- SSH 종료 코드별 상태 매핑
- 원격 docking result ROS topic/service/action
- 구조화된 JSON 결과 파일 또는 표준 출력 한 줄 파싱

가장 견고한 장기 구조는 SSH로 프로세스를 매번 실행하는 대신 로봇 측 supervisor를
항상 실행하고, ROS 2 action 또는 service로 start/cancel/result를 주고받는 방식입니다.

## 현재 구현상 주의점

### 빈 도킹 명령이 성공처럼 처리됨

현재 `mission_driver`는 `docking_mode`가 `none`, `placeholder`, 빈 문자열이거나 SSH
사용자/호스트 설정이 비어 명령을 만들지 못하면 `DOCKING_NOT_IMPLEMENTED`를 발행한
뒤 도킹 단계를 성공으로 반환합니다. 알 수 없는 mode도 명령이 비어 같은 경로로 갈
수 있습니다.

따라서 운영 환경에서는 반드시 다음을 확인해야 합니다.

- `docking_mode: ssh`
- `docking_ssh_user`와 `docking_ssh_host`가 비어 있지 않음
- launch 로그에 `DOCKING_NOT_IMPLEMENTED`가 나타나지 않음
- 실제 체결 없이 `DOCKED_NAV2_INACTIVE`가 발생하지 않는지 시험

이 동작은 개발용 placeholder 성격이며, 운영 안정성을 높이려면 추후 빈 명령을
성공이 아니라 실패로 변경하는 것이 좋습니다.

### 성공 여부는 충전 상태를 독립적으로 확인하지 않음

서버는 원격 종료 코드 0만 확인합니다. 서버 자체는 contact sensor, 배터리 전류,
충전기 GPIO를 확인하지 않습니다. 실제 성공 판정은 원격 docking 패키지가 해야 합니다.

### 성공 후 Nav2 reset 실패가 도킹 실패로 바뀌지 않음

원격 도킹 성공 후 Nav2 reset이 실패하면 오류 로그는 남지만 물리적 도킹 성공 상태와
고정 `docked_pose`는 유지됩니다. 웹 앱은 필요하면 `/nav2_supervisor/status`와 서버
로그도 함께 감시해야 합니다.

### 원격 stdout/stderr는 프로토콜이 아님

원격 프로세스의 stdout/stderr는 SSH를 거쳐 launch 로그에 보일 수 있지만 현재
`mission_driver`는 내용을 파싱하지 않습니다. 성공을 알리는 문자열만 출력하고
프로세스를 계속 실행하면 도킹은 성공 처리되지 않고 120초 뒤 timeout됩니다.

## 라즈베리파이 도킹 패키지 권장 구조

```text
dock_turn_backup
  ├─ 시작 전 중복 인스턴스 확인
  ├─ 필수 센서/베이스 연결 확인
  ├─ 전체 timeout 시작
  ├─ 정렬 또는 회전
  ├─ 제한 속도 후진/접근
  ├─ 접촉 감지
  ├─ 모터 정지
  ├─ 충전 상태 안정 확인
  ├─ 모든 제어 정리
  └─ exit 0

어느 단계든 실패/신호/timeout
  ├─ 속도 생성 중단
  ├─ 모터 정지
  ├─ 자식 프로세스/스레드 정리
  └─ exit non-zero
```

개념적인 Python 종료 구조는 다음과 같습니다.

```python
def main():
    success = False
    exit_code = 1

    try:
        controller = DockingController()
        success = controller.run_with_timeout()
        exit_code = 0 if success and controller.charging_is_stable() else 6
    except KeyboardInterrupt:
        exit_code = 130
    except Exception:
        logger.exception("Unhandled docking failure")
        exit_code = 1
    finally:
        # ROS context가 닫히기 전에 정지 명령을 전달해야 합니다.
        stop_robot_and_wait_for_watchdog()
        destroy_all_resources()

    raise SystemExit(exit_code)
```

실제 구현에서는 signal handler 안에서 복잡한 ROS 호출을 직접 수행하기보다 stop flag를
설정하고 메인 제어 루프가 안전 정지 및 cleanup을 수행하도록 만드는 편이 안전합니다.

## 설치 및 사전 점검

라즈베리파이에서 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/colcon_ws/install/setup.bash
ros2 pkg executables docking
```

출력에 다음 항목이 있어야 합니다.

```text
docking dock_turn_backup
```

컨테이너에서 공개키 로그인을 확인합니다.

```bash
ssh \
  -o BatchMode=yes \
  -o StrictHostKeyChecking=accept-new \
  -p 22 \
  -i /root/.ssh/id_ed25519_drive_manager \
  user@192.168.0.3 true
```

컨테이너에서 원격 ROS 환경과 실행 파일을 확인합니다. 이 명령은 도킹 동작을 시작하지
않습니다.

```bash
ssh \
  -o BatchMode=yes \
  -i /root/.ssh/id_ed25519_drive_manager \
  user@192.168.0.3 \
  "bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/user/colcon_ws/install/setup.bash && ros2 pkg executables docking'"
```

도킹 패키지에 `--dry-run` 또는 ROS parameter 기반 dry-run 기능을 제공하는 것을
권장합니다. 실제 모터를 움직이지 않고 센서 연결, timeout, 종료 코드, 중복 실행
거부와 signal cleanup을 검증할 수 있어야 합니다.

## 통합 시험 체크리스트

실제 로봇 시험은 저속 설정, 물리 비상정지 준비, 작업자 직접 감시 상태에서 진행합니다.

### 정상 도킹

- HOME_TO_DOCK 도착 뒤 Nav2 navigation이 PAUSE되는지 확인
- `/robot_status`가 `DOCKING`으로 바뀌는지 확인
- 라즈베리파이에 docking 프로세스가 하나만 실행되는지 확인
- `/cmd_vel`에 허용 속도 이상의 값이 나오지 않는지 확인
- 실제 체결 및 충전 확인 뒤 원격 프로세스가 exit 0으로 종료되는지 확인
- `DOCKED_NAV2_INACTIVE`와 최종 `SUCCEEDED HOME/START`를 확인
- `/robot_pose_status`가 `DOCKED`인지 확인

### 실패 시험

- 필수 센서 하나를 사용할 수 없을 때 즉시 정지하고 non-zero로 종료
- 체결 실패 시 exit 0을 반환하지 않음
- 내부 timeout에서 정지 후 non-zero 종료
- SSH 접속 실패 시 로봇이 움직이지 않음
- setup 파일 누락 시 명령이 시작되지 않고 FAILED 상태가 됨

### 중단 시험

- DOCKING 중 STOP을 보내 0.2초 수준의 감지 주기 뒤 중단이 시작되는지 확인
- SIGINT/SIGTERM/SIGHUP 각각에서 로봇이 정지하는지 확인
- SSH 연결을 강제로 끊은 뒤 라즈베리파이에 docking 프로세스가 남지 않는지 확인
- 남는 경우 자체 heartbeat timeout으로 로봇이 정지하고 프로세스가 끝나는지 확인
- ESTOP 뒤 RESET 전에는 START/HOME이 거부되는지 확인

### 충돌 시험

- DOCKING 중 웹 수동 조종 UI가 비활성화되는지 확인
- 별도 `/cmd_vel` publisher가 동시에 실행되지 않는지 확인
- `ros2 topic info /cmd_vel --verbose`로 publisher를 점검
- 원격 프로세스 중복 실행 시 두 번째 실행이 안전하게 거부되는지 확인

## 운영 체크리스트

- [ ] SSH 공개키 로그인이 비대화형으로 성공한다.
- [ ] 두 remote setup 파일이 존재한다.
- [ ] `docking dock_turn_backup` 실행 파일이 설치되어 있다.
- [ ] docking 프로세스는 포그라운드에서 실행된다.
- [ ] exit 0은 실제 체결 및 충전 확인 후에만 반환한다.
- [ ] 모든 실패와 신호 처리 경로가 0 속도를 보장한다.
- [ ] 로봇 베이스에 독립적인 속도 명령 watchdog이 있다.
- [ ] docking 전체 내부 timeout은 120초보다 짧다.
- [ ] 중복 docking 인스턴스를 거부한다.
- [ ] 필요한 ROS_DOMAIN_ID/DDS 환경이 비대화형 shell에도 설정된다.
- [ ] DOCKING 중 웹 수동 조종과 다른 `/cmd_vel` publisher를 차단한다.
- [ ] 웹에서 DOCKING, 성공, 실패, STOP, ESTOP 상태를 구분한다.
- [ ] 실제 로봇에서 SSH 단절 및 강제 종료 시험을 완료했다.

## 관련 파일

- [drive_manager/mission_driver.py](drive_manager/mission_driver.py): SSH 명령 생성, timeout, 종료 코드 판정 및 중단 처리
- [param/mission_config.yaml](param/mission_config.yaml): SSH 및 원격 명령 설정
- [README.md](README.md): 전체 서버 실행, 웹 앱 연동 및 미션 흐름

