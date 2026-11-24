# 자율주행 캡스톤 디자인 (Second)

전후방 웹캠 2대와 2D RPLIDAR로 차선, 신호등, 장애물을 인식하고 Arduino Mega로 구동과 조향을 제어한다. 미션 1은 신호등과 장애물이 있는 트랙 주행, 미션 2는 수직 주차와 평행 주차다. 시간 측정 경기는 범위에서 제외한다.

기본 주행은 트랙 지도와 코스 실측 없이 차량의 카메라와 LiDAR를 사용한다. 카메라가 본 차선과 횡단보도까지의 거리를 차량 기준으로 계산하고, 연속 LiDAR 스캔을 맞춰 출발 이후의 상대 이동을 추정한다. **실차 미션 완주는 아직 검증하지 않았다.** 센서 장착값, 카메라 보정값과 조향 변환표는 실제 차량에서 준비한다.

## 먼저 볼 문서

- [센서 장착, 차량 보정 및 모터 없는 재생 검증](docs/HARDWARE.md)
- [차량 설정 예시](configs/vehicle.example.json)

## 설치와 모터 없는 확인

Python 3.10 이상을 사용한다. 아래 명령은 저장소를 복제한 뒤 가상 환경에서 실행한다.

```bash
git clone https://github.com/swliris/AutonomousDrivingCapstoneDesign.git
cd AutonomousDrivingCapstoneDesign
python3 -m venv .venv
.venv/bin/pip install -e '.[test]'
.venv/bin/python -m second --help
.venv/bin/python -m second devices
.venv/bin/python -m second doctor --mode mission1
.venv/bin/python -m second demo --output logs/demo
.venv/bin/python -m pytest -q
```

`doctor`의 기본 결과가 `ready: false`인 것은 정상이다. 예시값/보정값/장치 누락을 나열한다. `demo`는 합성 관측에 대한 규칙 재생과 기하학적 후진 경로 확인이며 실제 카메라 인식/차량 주행 시뮬레이터가 아니다. 기존 시스템 Python의 외부 pytest 플러그인이 충돌하면 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`를 사용한다.

## 실측 설정 만들기

```bash
cp configs/vehicle.example.json configs/local.vehicle.json
```

장치 경로를 먼저 입력한다. 센서 수집은 차량 보정 전에도 가능하고 **아두이노 포트를 열지 않는다.**

```bash
.venv/bin/python -m second --config configs/local.vehicle.json record --output data/run_001 --seconds 60
```

`data/run_001/frames.jsonl`에 각 센서의 개별 monotonic 시각과 파일 경로를 저장한다. 카메라 영상은 JPG, LiDAR는 NPZ의 `points`(차량 좌표 m), `angles`(센서 좌표 rad), `ranges`(m)다. 카메라와 LiDAR는 하드웨어 동기화되지 않으며, 시간 차이를 로그로 확인한다. 이미 존재하는 수집 폴더는 덮어쓰지 않는다.

차량 치수, 조향 변환표, 전후방 카메라 보정값(K/D/H), LiDAR 장착 위치와 방향을 입력한다. 측정을 마치면 차량 설정의 `calibrated`를 true로 바꾼다. [하드웨어 안내](docs/HARDWARE.md)

### 구동 PWM 설정

`configs/local.vehicle.json`의 `control`에서 네 상황의 출력을 각각 0부터 200까지의 정수로 지정한다. 아래 값은 실차 조정 전 예시다.

| 상황 | 설정 키 | 예시 PWM |
| --- | --- | ---: |
| 직선 주행 | `straight_pwm` | 160 |
| 커브 주행 | `curve_pwm` | 140 |
| 차선 변경 | `lane_change_pwm` | 135 |
| 주차 | `parking_pwm` | 120 |

차선 추종 중 앞바퀴 목표 조향각의 절댓값이 `curve_steer_deg`(예시 10도) 이상이면 커브 PWM을 선택한다. 차선 변경 중에는 전용 출력을 쓰고, 주차 경로에서는 전진과 후진에 따라 PWM의 부호를 바꾼다.

신호 정차 지점까지 30cm 이내에서는 출력을 커브 PWM 이하로 낮춘다. 정지 명령을 보낼 거리는 LiDAR 추정 속도와 `deceleration_mps2`, `latency_budget_s`로 계산하며, 정차 여부도 LiDAR 속도로 확인한다. 주행 로그의 `command.pwm`은 요청 출력, 관측값 `speed`는 m/s 단위의 LiDAR 추정 속도다.

기존 차량 설정으로 실차를 운행하려면 위 네 PWM과 `curve_steer_deg`를 `control`에 추가한다. 이전 형식의 설정과 관측 로그로 `replay`를 실행하면 예시 PWM을 사용해 명령을 다시 계산한다.

## 센서로 미션 지점 구별하기

- 도로에서는 보이는 차선 경계를 짝지어 중심 경로를 구한다. 출발은 2차선이며, 반시계 주행에서 1차선은 왼쪽이다. 이후에는 이전 프레임의 차선과 점선의 위치로 두 경로를 추적한다. 회피 경로는 카메라가 본 도로 폭과 LiDAR가 확인한 빈 공간 안에서 선택한다.
- 정차 위치는 횡단보도 줄무늬의 가까운 끝에서 차체 맨 앞까지의 거리로 계산한다. 정차구역 깊이는 도면의 1.5 m를 사용한다. 랩타이머는 차로 양쪽에 반복 관측된 한 쌍의 기둥을 기준으로 차량 앞단의 통과를 검사한다. 현장 장치가 이 형태로 스캔되는지 녹화로 확인해야 한다.
- 주차는 IN에서 OUT을 향해 출발한다. 전후방 영상의 주차선을 이어 구획을 찾고, 양옆 기준 차량의 LiDAR 반사점으로 비어 있는 목표 칸을 구별한다. 도면의 구획 배열과 표준 치수는 인식에 사용한다. 후진 주차 후에는 뒤쪽 영상과 타이어 위치로 지정 칸 정차를 확인한다. OUT은 평행 주차열이 끝난 뒤 통로를 가로지르는 선으로 구별한다.

## 아두이노

`firmware/second_control/second_control.ino`를 Mega 2560에 업로드한다. **기존 AD-Project 펌웨어와 새 v2 펌웨어는 통신 형식/baud가 다르다.** 업로드는 별도 실차 작업이며 여기서 수행하지 않았다. 배선과 핀은 바꾸지 않는다. Mega 2560 대상으로 빌드 검증했다.

v2는 115200 baud에서 구동 PWM과 조향을 한 명령으로 전달하며 CRC16, 가변저항 피드백과 250ms 통신 watchdog을 사용한다. PC도 센서, 위치, 명령의 유효시간을 확인한다. 정지 명령을 받으면 구동 PWM을 즉시 0으로 내리고, 회전 방향을 바로 뒤집을 때는 일시 정지한다. PWM 0의 물리적 정지거리와 저속 동작은 실측해야 한다. 기존 3-byte 호환 인코더도 포함했지만 watchdog 없는 기존 펌웨어로는 `live`를 허용하지 않는다.

## 보정 후 미션 실행

다음 명령은 실제 모터 포트를 여는 명령이다. 지금은 예시값 때문에 실행할 수 없다. 시작할 준비를 마친 뒤 **출발 신호음 종료 후 G**, 종료/비상 정지는 **Space/Q/Esc**다. 대회 중에는 규정이 허용하는 시작·종료 조작만 한다.

```bash
# 미션 1: M3 2차선 출발 → 초록불 → 감지 기반 회피 → 빨간불 정지 → 초록불 → 한 바퀴 완료
.venv/bin/python -m second --config configs/local.vehicle.json live --mode mission1 --log logs/mission1_run01

# 미션 2: 예시는 수직 목표 2 / 평행 목표 1. 추첨 결과로 선택.
# 수직→평행→OUT는 이 프로세스 안에서 연속 수행한다.
.venv/bin/python -m second --config configs/local.vehicle.json live --mode parking --perpendicular 2 --parallel 1 --log logs/parking_run01
```

미션 1 실패 후 별도 신호등 시도는 규정상 장애물 미션 소요시간 3분 미만일 때 심판 안내에 따라 진행한다. 이때만 `--mode signal_only`로 60초 모드를 사용한다. 프로그램이 재시도 자격을 승인하는 것은 아니다.

실행 후 `logs/.../observations.jsonl`을 같은 설정과 모드로 재생할 수 있다. 재생은 시리얼을 열지 않는다. 이것은 인식 결과 이후의 판단·제어 재생이다. 원본 JPG/NPZ 수집과 구분한다.

```bash
.venv/bin/python -m second --config configs/local.vehicle.json replay --mode mission1 --input logs/mission1_run01/observations.jsonl --output logs/replayed_commands.jsonl
```

## 원본 센서 녹화 재생

`replay-raw`는 `record`가 만든 `frames.jsonl`, 카메라 JPG, LiDAR NPZ를 읽고 실차와 같은 `Observer.observe`로 관측을 계산한다. 같은 시각에 들어온 센서는 함께 반영하며, 각 시각에는 그때까지 도착한 가장 최근 샘플만 사용한다. 출력 JSONL에는 센서별 시각, 차선 경로, 신호색, 추정 위치와 방향, 장애물, 주차 판정이 들어간다.

```bash
.venv/bin/python -m second --config configs/local.vehicle.json replay-raw \
  --recording data/run_001 --mode mission1 --output logs/run_001_observations.jsonl
```

기본은 화면 없이 가능한 속도로 처리한다. `--view`를 붙이면 녹화 시각 간격에 맞춰 앞뒤 영상과 차량 좌표계의 LiDAR 점을 표시하며, Q 또는 Esc로 닫을 수 있다. 주차 녹화는 `--mode parking`으로 재생한다. 출력 파일은 새 경로를 사용한다.

차량 설정을 생략하면 녹화 당시의 `sensor_config.json`을 읽는다. 보정 전에 녹화했다면 같은 장착 상태와 영상 해상도에서 측정한 카메라 보정값과 LiDAR 장착값을 제공한다. LiDAR 장착 위치나 각도 부호를 보정한 경우에는 저장된 거리와 각도로 차량 좌표를 다시 계산한다. 녹화 시점에 거리 필터로 제거한 반사점은 복원할 수 없다.

`record`는 Arduino 피드백과 출발 키를 수집하지 않는다. 원본 재생에서는 관측만 기록하며, 주차 시 타이어 방향에는 실차 관측기의 피드백 누락 기본값인 조향각 0이 쓰인다. 관측 결과의 시각은 센서 갱신 시각이므로 당시 제어 루프의 반복 횟수와 다를 수 있다.

### 카메라 이미지만 있는 경우

`0_steer:0_left_speed:0_right_speed:0.png`처럼 번호로 시작하는 JPG 또는 PNG는 `replay-images`로 확인한다. 번호순으로 실차의 `Vision.mask`를 적용해 원본과 도색 후보 마스크를 보여주고, 프레임별 후보 픽셀 수를 JSONL에 쓴다. 이 값에는 배경의 흰 물체도 포함될 수 있다. 이 명령은 픽셀 단위 마스크를 확인하는 용도다. `--fps`는 타임스탬프가 없는 이미지의 화면 표시 속도다.

```bash
.venv/bin/python -m second replay-images --input data/camera_images \
  --output logs/camera_preview.jsonl --view --fps 10
```

### ICP 위치와 처리 시간 비교

기본 주행은 최근 LiDAR 스캔을 기준으로 상대 이동을 계산한다. 기존 실측 지도로 운행하거나 과거 로그를 재생할 때는 전역 옵션 `--course configs/local.course.json`을 지정한다. 아래 `compare-icp`는 기존 정적 지도에서 전수 검색과 KDTree를 비교하는 명령이다.

```bash
.venv/bin/python -m second compare-icp --synthetic --repeats 3
.venv/bin/python -m second --config configs/local.vehicle.json --course configs/local.course.json compare-icp \
  --recording data/run_001 --mode mission1 --start M3 --repeats 3
```

출력에는 스캔 채택 수와 판정 불일치 수, 두 방식이 모두 채택한 시각의 최대 위치 및 방향 차이, 스캔당 처리 시간 중앙값과 95백분위가 들어간다. 시간은 지도 선별과 트리 생성을 포함한 `Localizer.update` 전체를 측정하며 파일 읽기와 카메라 처리는 제외한다. 합성 입력은 정답 궤적에 대한 위치와 방향 RMSE도 제공한다.

## 학습

학습 없이 먼저 OpenCV 흰색 차선/주차선 검출로 보정과 주행 파이프라인을 시험할 수 있다. 신호등은 학습된 검출기 또는 실제 트랙에서 검증한 고정 ROI가 필요하다. 고정 ROI가 시야 변화에 대응하지 못하면 검출기를 학습한다.

```bash
.venv/bin/pip install -e '.[learn,signal]'
.venv/bin/python tools/prepare_dataset.py --recording data/run_001 --output data/seg --split train
.venv/bin/python tools/prepare_dataset.py --recording data/run_002 --output data/seg --split val
# 생성된 images를 사람이 라벨링해 masks/*.png에 저장한 뒤:
.venv/bin/python tools/train_segmentation.py --manifest data/seg/manifest.jsonl --output models/seg --pretrained
.venv/bin/python tools/export_segmentation.py --weights models/seg/best.pt --output models/seg/lane.onnx
.venv/bin/python tools/train_signal.py --data data/signal/data.yaml --project models/signal
```

모델 다운로드와 학습은 아직 실행하지 않았다. 신호 검출 클래스 이름은 `red`, `green`, `yellow`, `traffic_light` 중에서 사용한다. 자동 다운로드가 가능한 학습 도구는 네트워크가 필요하다. Ultralytics는 AGPL/Enterprise 라이선스를 제공하므로 배포 조건도 확인한다.
