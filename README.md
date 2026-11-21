# 자율주행 캡스톤 디자인 (Second)

전후방 웹캠 2대와 2D RPLIDAR로 차선, 신호등, 장애물을 인식하고 Arduino Mega로 구동과 조향을 제어한다. 미션 1은 신호등과 장애물이 있는 트랙 주행, 미션 2는 수직 주차와 평행 주차다. 시간 측정 경기는 범위에서 제외한다.

센서 녹화와 재생, LiDAR 지도 기반 위치 추정, 장애물 회피, 전후진 경로 생성과 추종, 주차 후 5초 정지 판정을 지원한다. **실차 미션 완주는 아직 검증하지 않았다.** 저장소에는 예시 설정이 들어 있다. 실차 주행에는 차량 실측값과 트랙 지도가 필요하며, 학습 모델을 사용하려면 가중치도 별도로 준비해야 한다. 예시 설정으로 `live`를 실행하면 보정 누락을 알리고 모터 포트를 열기 전에 종료한다.

## 먼저 볼 문서

- [센서 장착, 카메라 보정, 트랙 지도 작성 및 구동 핀 안내](docs/HARDWARE.md)
- [차량 설정 예시](configs/vehicle.example.json)와 [코스 설정 양식](configs/course.template.json)

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
cp configs/course.template.json configs/local.course.json
```

장치 경로를 먼저 입력한다. 센서 수집은 차량 보정 전에도 가능하고 **아두이노 포트를 열지 않는다.**

```bash
.venv/bin/python -m second --config configs/local.vehicle.json record --output data/run_001 --seconds 60
```

`data/run_001/frames.jsonl`에 각 센서의 개별 monotonic 시각과 파일 경로를 저장한다. 카메라 영상은 JPG, LiDAR는 NPZ의 `points`(차량 좌표 m), `angles`(센서 좌표 rad), `ranges`(m)다. 카메라와 LiDAR는 하드웨어 동기화되지 않으며, 시간 차이를 로그로 확인한다. 이미 존재하는 수집 폴더는 덮어쓰지 않는다.

차량 치수와 조향 및 속도 변환표, 전후방 카메라 보정값(K/D/H), LiDAR 장착 좌표와 지도를 입력한다. 실제 차선과 주차선, 목표 구획, 출발 위치까지 측량한 뒤 `calibrated`와 `surveyed`를 true로 바꾼다. 센서별 측정 방법과 명령어는 [하드웨어 안내](docs/HARDWARE.md)에 있다.

## 아두이노

`firmware/second_control/second_control.ino`를 Mega 2560에 업로드한다. **기존 AD-Project 펌웨어와 새 v2 펌웨어는 통신 형식/baud가 다르다.** 업로드는 별도 실차 작업이며 여기서 수행하지 않았다. 배선과 핀은 바꾸지 않는다. Mega 2560 대상으로 빌드 검증했다.

v2는 115200 baud, CRC16, 속도/조향 원자적 명령, 가변저항 피드백, 250ms 통신 watchdog을 사용한다. PC도 센서/위치/명령 유효시간을 확인한다. 정지 명령은 구동 PWM을 즉시 0으로 하고, 정·역전 전환은 일시 정지한다. PWM 0의 물리적 정지거리와 저속 동작은 실측해야 한다. 기존 3-byte 호환 인코더도 포함했지만 watchdog 없는 기존 펌웨어로는 `live`를 허용하지 않는다.

## 보정 후 미션 실행

다음 명령은 실제 모터 포트를 여는 명령이다. 지금은 예시값 때문에 실행할 수 없다. 시작할 준비를 마친 뒤 **출발 신호음 종료 후 G**, 종료/비상 정지는 **Space/Q/Esc**다. 대회 중에는 규정이 허용하는 시작·종료 조작만 한다.

```bash
# 미션 1: M3 2차선 출발 → 초록불 → 감지 기반 회피 → 빨간불 정지 → 초록불 → 한 바퀴 완료
.venv/bin/python -m second --config configs/local.vehicle.json --course configs/local.course.json live --mode mission1 --start M3 --log logs/mission1_run01

# 미션 2: 예시는 출발 3 / 수직 목표 2 / 평행 목표 1. 추첨 결과로 선택.
# 수직→평행→OUT는 이 프로세스 안에서 연속 수행한다.
.venv/bin/python -m second --config configs/local.vehicle.json --course configs/local.course.json live --mode parking --start 3 --perpendicular 2 --parallel 1 --log logs/parking_run01
```

미션 1 실패 후 별도 신호등 시도는 규정상 장애물 미션 소요시간 3분 미만일 때 심판 안내에 따라 진행한다. 이때만 `--mode signal_only`로 60초 모드를 사용한다. 프로그램이 재시도 자격을 승인하는 것은 아니다.

실행 후 `logs/.../observations.jsonl`을 같은 설정과 모드로 재생할 수 있다. 재생은 시리얼을 열지 않는다. 이것은 인식 결과 이후의 판단·제어 재생이다. 원본 JPG/NPZ 수집과 구분한다.

```bash
.venv/bin/python -m second --config configs/local.vehicle.json --course configs/local.course.json replay --mode mission1 --input logs/mission1_run01/observations.jsonl --output logs/replayed_commands.jsonl
```

## 원본 센서 녹화 재생

`replay-raw`는 `record`가 만든 `frames.jsonl`, 카메라 JPG, LiDAR NPZ를 읽고 실차와 같은 `Observer.observe`로 관측을 계산한다. 같은 시각에 들어온 센서는 함께 반영하며, 각 시각에는 그때까지 도착한 가장 최근 샘플만 사용한다. 출력 JSONL에는 센서별 시각, 차선 경로, 신호색, 추정 위치와 방향, 장애물, 주차 판정이 들어간다.

```bash
.venv/bin/python -m second --config configs/local.vehicle.json --course configs/local.course.json replay-raw \
  --recording data/run_001 --mode mission1 --start M3 --output logs/run_001_observations.jsonl
```

기본은 화면 없이 가능한 속도로 처리한다. `--view`를 붙이면 녹화 시각 간격에 맞춰 앞뒤 영상과 차량 좌표계의 LiDAR 점을 표시하며, Q 또는 Esc로 닫을 수 있다. 주차는 `--mode parking --start 1`처럼 실제 출발 위치를 지정한다. 출력 파일은 새 경로를 사용한다.

차량 설정을 생략하면 녹화 당시의 `sensor_config.json`을 읽는다. 보정 전에 녹화했다면 같은 장착 상태와 영상 해상도에서 측정한 카메라 보정값, LiDAR 지도, 출발 좌표를 제공한다. LiDAR 장착 위치나 각도 부호를 보정한 경우에는 저장된 거리와 각도로 차량 좌표를 다시 계산한다. 녹화 시점에 거리 필터로 제거한 반사점은 복원할 수 없다.

`record`는 Arduino 피드백과 출발 키를 수집하지 않는다. 원본 재생에서는 관측만 기록하며, 주차 시 타이어 방향에는 실차 관측기의 피드백 누락 기본값인 조향각 0이 쓰인다. 관측 결과의 시각은 센서 갱신 시각이므로 당시 제어 루프의 반복 횟수와 다를 수 있다.

### 카메라 이미지만 있는 경우

`0_steer:0_left_speed:0_right_speed:0.png`처럼 번호로 시작하는 JPG 또는 PNG는 `replay-images`로 확인한다. 번호순으로 실차의 `Vision.mask`를 적용해 원본과 도색 후보 마스크를 보여주고, 프레임별 후보 픽셀 수를 JSONL에 쓴다. 이 값에는 배경의 흰 물체도 포함될 수 있다. 보정값과 지도가 없는 영상에서 차선 번호나 차량 위치를 추정하지 않는다. `--fps`는 타임스탬프가 없는 이미지의 화면 표시 속도다.

```bash
.venv/bin/python -m second replay-images --input data/camera_images \
  --output logs/camera_preview.jsonl --view --fps 10
```

### ICP 위치와 처리 시간 비교

실차와 원본 재생은 SciPy KDTree를 사용한다. 각 스캔에서 기존과 같은 범위의 지도를 골라 트리를 한 번 만들고 ICP 반복 중에 재사용한다. `compare-icp`는 같은 시작 위치와 입력 순서에서 기존 전수 검색과 KDTree를 각각 실행한다.

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
