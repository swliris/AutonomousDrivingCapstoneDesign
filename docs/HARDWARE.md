# 공용 지지대에 설치하는 전후방 카메라와 2D LiDAR

전방 카메라는 노트북 위쪽 가로 지지대 중앙에 고정하고, 후방 카메라는 같은 지지대의 반대쪽 면에 달아 뒤를 보게 한다. 2D LiDAR의 1차 설치 후보는 두 카메라 바로 위다. 정확한 높이는 실제 기준 차량을 스캔하고 정한다.

## 장착 위치와 시야

```text
측면에서 본 배치

                     [2D LiDAR]       바닥과 평행한 스캔면
                          │
차량 뒤쪽 ← [후방 카메라] │ [전방 카메라] → 차량 앞쪽 (+x)
                    [공용 지지대]
                 [노트북과 나무 프레임]
```

| 센서 | 최종 고정 전에 확인할 장면 |
|---|---|
| 전방 카메라 | 가까운 양쪽 차선과 정차 위치의 신호등을 한 화면에 담는다. 앞쪽 약 0.3~3 m를 시야 확인 범위로 삼고, 특히 뒤 차축 기준 전방 0.8 m 부근의 차선과 정차 후 빨강, 초록 렌즈가 보이는지 확인한다. |
| 후방 카메라 | 뒤쪽 노면을 향해 약간 아래로 기울인다. 수직 주차와 평행 주차의 최종 위치에 차를 놓아 목표 주차선 두 변이 보이는 각도를 고른다. 실제 주행 각도로 연 노트북 화면, 지지대, 나무판이 가까운 주차선을 가리는지도 확인한다. |
| LiDAR | 수평계를 앞뒤와 좌우 두 방향으로 대어 수평을 맞춘다. 기준 차량을 앞, 옆, 뒤에 차례로 두고 차체가 스캔에 잡히는지 살핀다. 빔이 지붕 위로 지나가면 장착점을 낮춘다. 노트북, 카메라, 케이블에 가리는 방향이 있으면 지지대에서 위치를 옮겨 다시 확인한다. 신호등 다리와 랩타이머도 같은 높이에서 확인한다. |

센서용 지그는 제공 프레임에 고정하고 차체에 구멍을 뚫지 않는다. 센서와 나무 프레임을 포함한 전체 치수는 **길이 1.10 m, 폭 0.60 m, 높이 0.75 m 이내**다. 수직 주차 판정에는 센서의 전방 돌출부도 포함된다. 기존 모터와 Arduino 배선을 사용하고, SMPS는 전압을 측정할 수 있게 지정된 파란 영역에 두고 12.0 V를 유지한다. 제공 부품과 차량 개조에 관한 항목은 평가 규정집 §3.1.1과 §3.2에 있다.

## 1. 차량 기준점 표시와 장치 확인

준비물은 줄자, 테이프, 평평한 체커보드, 수평계, 세워 둘 판이나 상자다. 차량을 평평한 바닥에 세우고 좌우 뒷바퀴의 차축 중심을 바닥에 표시한다. 두 표시의 중점이 **뒤 차축 중앙 `(0, 0)`**이다. 이 점을 지나 차량 정면으로 향하는 중심선을 긋는다. 차량 앞쪽이 `+x`, 운전자 기준 왼쪽이 `+y`이며 위에서 보아 반시계 회전이 양의 `yaw`다. 전후방 영상과 LiDAR에 모두 이 기준을 쓴다.

차량용 Linux 노트북에서 [설치 절차](../README.md#설치와-모터-없는-확인)를 마친 뒤, 저장소 최상위 폴더에서 아래 명령을 실행한다. 이후 같은 터미널에서 `sec`를 사용하면 차량 보정 설정을 불러온다.

```bash
source .venv/bin/activate
cp -n configs/vehicle.example.json configs/local.vehicle.json
mkdir -p calibration/local/board/front calibration/local/board/rear
sec() {
  python -m second --config configs/local.vehicle.json "$@"
}
sec devices
```

`configs/local.vehicle.json`에 `cameras.front.device`, `cameras.rear.device`, `lidar.port`를 입력한다. 웹캠은 `/dev/v4l/by-id/` 경로를 우선 쓰고 동일 모델을 구별할 수 없으면 `/dev/v4l/by-path/`와 고정 USB 포트를 사용한다. 실제 지원되는 해상도와 FPS를 정한다. 예시 설정은 MJPEG 1280×720, 20 fps다.

```bash
sec record --output data/mount_check_01 --seconds 8
```

녹화 중 전방 렌즈만 잠시 가리고 `data/mount_check_01/front/`에 그 장면이 저장됐는지 확인한다. 종료 시 `front`, `rear`, `lidar` 개수가 모두 양수여야 한다. `record`는 세 센서만 열며, 재촬영할 때는 새 출력 폴더를 지정한다.

## 2. 카메라별 렌즈 보정

체커보드 여러 칸의 총길이를 재고 칸 수로 나누어 한 칸의 길이(m)를 구한다. `--columns`와 `--rows`에는 가로와 세로의 **내부 교차점 개수**를 넣는다. 아래 명령의 수치는 가로 10칸, 세로 7칸이고 한 칸이 실제로 25 mm인 보드의 예다.

```bash
sec record --output data/board_01 --seconds 90
```

차와 카메라를 고정한 채 각 렌즈 앞에서 보드를 움직인다. 보드 전체가 들어오는 범위에서 중앙과 네 모서리, 거리와 기울기를 바꿔 가며 자세마다 잠시 멈춘다. 각 카메라에서 자세가 다른 선명한 JPG 15~25장을 골라 `calibration/local/board/front/`와 `rear/`에 각각 복사한다. 보정기는 카메라 한 대당 교차점 검출에 성공한 사진을 최소 12장 요구한다.

```bash
for cam in front rear; do
  sec calibrate-camera --images "calibration/local/board/$cam/*.jpg" \
    --columns 9 --rows 6 --square-m 0.025 \
    --output "calibration/local/${cam}_lens.json"
done
```

각 결과의 `width`, `height`가 차량 설정의 해상도와 같은지 확인한다. **다음 단계 전에** `front_lens.json`의 `intrinsics`와 `distortion`을 차량 설정의 `cameras.front`에, `rear_lens.json`의 두 값은 `cameras.rear`에 복사한다.

## 3. 카메라별 바닥 좌표 보정

1. 차를 세운 채 앞뒤 바닥에 테이프로 십자를 붙인다. 카메라당 가까운 곳, 중간, 먼 곳에 걸쳐 좌우로 분산된 점 8~12개를 권장한다. 보정에 쓰지 않을 검증점 두 개도 따로 표시한다.
2. 각 십자 중심의 `(x, y)`를 뒤 차축 중앙 기준으로 재서 미터로 적는다. 예를 들어 앞 1.20 m, 왼쪽 0.40 m는 `(1.20, 0.40)`, 뒤 0.80 m, 왼쪽 0.40 m는 `(-0.80, 0.40)`이다. **후방 화면에서도 차량 왼쪽은 `+y`**다. 원본 영상의 좌우 방향을 유지한다.
3. 십자가 모두 보일 때 녹화하고 카메라별로 왜곡을 제거한 PNG를 만든다. 아래 첫 프레임이 흐리면 같은 녹화의 선명한 JPG를 선택한다.

```bash
sec record --output data/ground_01 --seconds 5
for cam in front rear; do
  sec undistort --camera "$cam" --image "data/ground_01/$cam/0000000.jpg" \
    --output "calibration/local/${cam}_ground.png"
  cp -n calibration/ground_points.example.json "calibration/local/${cam}_points.json"
done
```

각 PNG에서 십자 중심의 픽셀 `(u, v)`를 읽는다. 원본 크기 기준 왼쪽 위가 `(0, 0)`이다. 해당 `*_points.json`의 예시값을 전부 교체해 `undistorted_image_px`에는 픽셀, `rear_axle_xy_m`에는 같은 순서의 실측 좌표를 입력한다. 검증점 두 개는 따로 보관한다.

```bash
for cam in front rear; do
  sec calibrate-ground --points "calibration/local/${cam}_points.json" \
    --output "calibration/local/${cam}_ground_h.json"
done
```

각 결과의 `ground_homography`를 차량 설정의 해당 카메라에 복사한다. 보정기는 유효 대응점 6개 이상을 요구하며 입력점 오차의 90백분위가 5 cm를 넘으면 실패한다.

따로 둔 검증점은 아래 예시의 `front 640 510`을 카메라 이름과 PNG에서 읽은 픽셀 `u v`로 바꾸어 확인한다. 출력된 차량 좌표(m)를 줄자 실측값과 비교하고 후방 검증점에도 반복한다.

```bash
python - front 640 510 <<'PY'
import json
import sys
import numpy as np

camera = sys.argv[1]
u, v = map(float, sys.argv[2:])
with open("configs/local.vehicle.json") as stream:
    matrix = json.load(stream)["cameras"][camera]["ground_homography"]
x, y, scale = np.array(matrix) @ [u, v, 1]
print(f"x = {x / scale:.3f} m, y = {y / scale:.3f} m")
PY
```

## 4. LiDAR 장착 위치와 방향 보정

LiDAR 회전 중심을 수직으로 바닥에 표시한다. 뒤 차축 중앙에서 그 표시까지 앞뒤 거리와 좌우 거리를 따로 재서 차량 설정에 넣는다.

| 차량 설정 | 실측값 |
|---|---|
| `lidar.pose[0]`, `lidar.pose[1]` | 뒤 차축에서 회전 중심까지의 `x, y`(m). 지지대가 차 폭의 중앙에 있어도 차축보다 앞에 있으면 `x`는 양수다. |
| `lidar.pose[2]` | 차량 정면에서 센서 0도 방향까지의 회전각(rad). 정면과 일치하면 0, 왼쪽 90도면 약 1.571이다. |
| `lidar.angle_sign` | 센서가 보고한 각도에 곱할 회전 방향 부호. 예시값은 `-1`이며 실제 표적으로 확인한다. |
| `height_m` | 센서와 지지대를 포함한 차량 전체 높이(m). 별도로 바닥에서 레이저 스캔면까지의 높이를 재서 기준 차량 몸체와 대조한다. |

앞쪽과 왼쪽에 스캔 높이까지 닿는 판을 서로 다른 거리에 놓고, 뒤 차축 기준으로 레이저가 맞을 **판의 가까운 면**까지의 좌표를 기록한다. 다음 명령은 녹화 중간의 스캔을 현재 설정으로 변환해 지도 없이 확인할 수 있는 PNG로 저장한다.

```bash
sec record --output data/lidar_check_01 --seconds 5
python - <<'PY'
import cv2
from second.config import load_json
from second.raw_replay import scan_view
from second.recording import Recording

rec = Recording("data/lidar_check_01")
config = load_json("configs/local.vehicle.json")
scans = [path for _, sensor, path in rec.events if sensor == "lidar"]
points = rec.scan(scans[len(scans) // 2], config)["points"]
cv2.imwrite("calibration/local/lidar_check.png", scan_view(points, 2.0))
PY
```

PNG 중앙이 뒤 차축이고 위가 차량 정면, 왼쪽이 차량 왼쪽이다. 이 그림은 100픽셀이 1 m다. 앞판의 면이 `x=1.20 m`이면 중앙에서 위로 120픽셀, 왼쪽 판이 `y=0.80 m`이면 왼쪽으로 80픽셀 부근에 찍혀야 한다.

정면을 맞춘 뒤에도 좌우가 뒤집히면 `angle_sign`, 두 판이 함께 회전해 보이면 `pose[2]`, 같은 방향으로 밀려 보이면 `pose[0]`과 `pose[1]`을 확인한다. 설정값만 수정한 경우 위 Python 부분을 다시 실행하면 같은 녹화로 비교할 수 있다. 지지대를 옮겼거나 스캔 높이를 바꾼 경우에는 실제 기준 차량을 앞, 옆, 뒤에 놓고 새로 녹화한다.

## 5. 모터 없이 현장 인식 확인

차량의 최종 장착 상태에서 M3의 2차선에 진행 방향으로 차를 놓고 녹화를 시작한다. 처음에는 잠시 정차해 연속 LiDAR 스캔을 확보한다. 차를 손으로 밀면서 차선, 횡단보도, 랩타이머 양쪽 기둥이 보이는 구간을 통과한다. 트랙 바닥에 원점을 표시하거나 출발 좌표를 입력할 필요는 없다.

```bash
sec record --output data/verify_road --seconds 60
sec replay-raw --recording data/verify_road --mode mission1 --output logs/verify_road.jsonl --view
sec doctor --mode mission1
```

관측 로그의 `pose`는 첫 스캔에 대한 상대 위치와 방향이다. `localized`가 반복해서 거짓이면 LiDAR 높이에서 벽이나 고정 물체가 여러 방향으로 잡히는지 확인한다. 도로에서는 `lanes`에 현재 차선이 잡히는지, 횡단보도에 접근할 때 `stop_zone_distance`가 줄어드는지, 실제 랩타이머 통과 시점에 `gate`가 켜지는지 살핀다. 신호색은 실제 트랙 영상에서 검증한 ROI 또는 학습된 검출기를 사용한다.

주차는 규정의 출발 표시에서 차를 IN에 수직으로 놓고, 두 기준 차량을 실제 배치한 상태에서 별도로 녹화한다. 수직 주차열을 오른쪽, 평행 주차열을 왼쪽에 두고 진행한다.

```bash
sec record --output data/verify_parking --seconds 60
sec replay-raw --recording data/verify_parking --mode parking --output logs/verify_parking.jsonl --view
sec doctor --mode parking
```

`scene.parking`에 추첨된 수직 및 평행 주차칸과 OUT 선이 실제로 보인 순서에 맞게 들어오는지 확인한다. `parked_slot`은 해당 위치에서 후방 영상의 도색선까지 확인했을 때 기록된다. `doctor`는 차량 설정과 장치 경로를 점검한다. 실제 장면의 인식과 정지거리는 녹화 및 실차 시험으로 확인한다.

## 기존 구동 배선

| 기능 | 기존 핀 | 펌웨어 동작 |
|---|---|---|
| 왼쪽 뒤 구동 | 7, 6 | 전진 PWM은 6, 반대 입력은 7 |
| 오른쪽 뒤 구동 | 5, 4 | 전진 PWM은 5, 반대 입력은 4 |
| 조향 DC 모터 | 3, 2 | 기존 극성 유지 |
| 조향 가변저항 | A2 | ADC 값으로 조향각 피드백 |

뒤 구동 모터 두 개에는 같은 PWM을 보내며 전진과 후진 모두 출력 상한은 200이다. 주행 상황별 출력은 차량 설정의 `control`에서 조정한다. [구동 PWM 설정](../README.md#구동-pwm-설정)

조향 중앙 ADC 483과 조향 출력 제한은 기존 코드의 초기값이다. v2 펌웨어는 조향에 비례 제어와 불감대를 적용한다. 실제 차량에서 중앙값과 가변저항 오류 범위를 확인하고, 소프트웨어의 조향각(rad)과 펌웨어 명령 사이의 부호는 실측 변환표에 한 번만 반영한다.
