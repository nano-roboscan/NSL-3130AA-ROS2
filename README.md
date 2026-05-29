# NSL-3130AA ROS2

ROS2 Humble driver for the NanoSystems NSL-3130AA Time-of-Flight camera.

**환경**: Ubuntu 22.04 LTS / ROS2 Humble / OpenCV 4.5+

---

## 0. 클론 (최초 1회)

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone --recurse-submodules https://github.com/zzapzzap/NSL-3130AA-ROS2.git
```

이후 모든 명령은 **`~/colcon_ws`** 를 작업 디렉토리로 사용합니다.

---

## 1. 최초 준비 (1회만)

### 1-1. USB 드라이버 & udev 규칙

```bash
sudo ~/colcon_ws/src/NSL-3130AA-ROS2/NSL3130_driver/src/roboscan_nsl3130/nsl_lib/script/install_libusb_linux.sh
```

설치 후 USB 케이블을 뽑았다 다시 꽂으면 `lsusb`에 `1fc9:0099`가 표시됩니다.

---

### 1-2. 호스트 NIC 고정 IP 설정

> **주의: 192.168.0.x 는 사내망과 충돌하므로 호스트에 절대 부여하지 마세요.**

**Ubuntu 설정 화면 (Settings → Network → Wired → IPv4):**

<img width="620" alt="ip_setup" src="NSL3130_driver/asset/ip_setup.png" />

| 항목 | 값 |
|------|-----|
| IPv4 Method | Manual |
| Address | `192.168.2.100` |
| Netmask | `255.255.255.0` |
| Gateway | `192.168.2.1` |

> **여러 대 운용 시**: 호스트 PC마다 주소를 다르게 설정해 주세요.  
> 예) PC-A → `192.168.2.100`, PC-B → `192.168.2.101`, PC-C → `192.168.2.102` …

터미널에서 설정하실 경우 (`<NIC>`는 `ip link show`로 확인하세요):

```bash
sudo nmcli con add type ethernet ifname <NIC> con-name nsl-cam \
    ipv4.method manual \
    ipv4.addresses 192.168.2.100/24 \
    ipv4.gateway 192.168.2.1
sudo nmcli con up nsl-cam
```

---

### 1-3. 카메라 IP 변경 (공장 출하 첫 연결)

공장 기본 IP는 `192.168.0.220`이므로, **USB로 연결한 상태에서** 192.168.2.x로 변경해 주세요.  
이더넷 케이블 없이 USB만으로 진행하시면 됩니다.

```bash
# USB auto-detect (카메라 1대 연결 시)
ros2 run roboscan_nsl3130 change_camera_ip 192.168.2.220 255.255.255.0 192.168.2.1

# 카메라 여러 대를 동시에 USB 연결한 경우 시리얼 번호를 지정해 주세요
ros2 run roboscan_nsl3130 change_camera_ip 192.168.2.221 255.255.255.0 192.168.2.1 N00A5060D
```

변경 후 **카메라 전원을 껐다 켜주세요**. 이더넷 케이블을 연결한 뒤 `ping 192.168.2.220`으로 응답을 확인하세요.

> 이미 192.168.2.x로 설정된 카메라는 이 단계를 건너뛰셔도 됩니다.

---

## 2. 빌드

```bash
cd ~/colcon_ws
colcon build
source install/setup.bash
```

새 터미널을 열 때마다 `source ~/colcon_ws/install/setup.bash` 를 실행해 주세요.

---

## 3. 카메라 실행

```bash
cd ~/colcon_ws
ros2 launch roboscan_nsl3130 camera.launch.py
```

실행 시 다음이 자동으로 시작됩니다:
- **roboscan_publish_node** — 카메라 드라이버 (USB 시리얼 자동 감지)
- **rviz2** — 포인트클라우드 시각화
- **rqt_reconfigure_combo** — 파라미터 실시간 변경 (12초 후 자동 실행)

**퍼블리시 토픽:**

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | RGB 이미지 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 거리 이미지 |
| `/camera/point_cloud` | `sensor_msgs/PointCloud2` | XYZI 포인트클라우드 |
| `/camera/point_cloud_rgb` | `sensor_msgs/PointCloud2` | XYZRGB (캘리브레이션 완료 후 활성화) |

**frame_id**: USB 시리얼에서 자동으로 결정됩니다 → `{serial}_lidar_frame` (예: `N00A5060D_lidar_frame`)  
시리얼 감지에 실패한 경우 `lidar_frame`이 사용됩니다.

---

## 4. 캘리브레이션

캘리브레이션 파일이 없으면 SDK 내장 호모그래피로 동작합니다.  
파일이 있으면 캘리브레이션 결과 (K, D, R, t)를 바탕으로 정밀한 XYZRGB 포인트클라우드를 생성합니다.

**반드시 아래 순서로 진행해 주세요**: Intrinsic → Extrinsic

캘리브레이션 파일은 저장소 내부에 저장됩니다:

```
~/colcon_ws/src/NSL-3130AA-ROS2/calib_output/
  {camera_id}_intrinsic.yml
  {camera_id}_extrinsic.yml
```

---

### 4-1. Intrinsic 캘리브레이션

**필요 조건**: `camera.launch.py` 실행 중, 체커보드 준비

```bash
cd ~/colcon_ws
ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py
```

(옵션) 체커보드 크기를 지정하는 경우 (내부 코너 수 × 정사각형 한 변 길이(m)):

```bash
ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py board_size:=8x6 square_size:=0.04
```

**진행 방법:**

<img src="NSL3130_driver/asset/intrinsic_setup.png" alt="intrinsic_setup" />

1. cameracalibrator GUI가 열립니다
2. 체커보드를 카메라 FOV 전체에서 다양한 각도·거리·틸트로 움직여 주세요
3. 우측 바 `X / Y / Size / Skew` 가 모두 초록색이 되거나 샘플이 40개 이상이 되면 **[CALIBRATE]** 버튼이 활성화됩니다
4. **[CALIBRATE]** 클릭
5. **[SAVE]** 클릭 → 터미널에 아래와 같이 출력되면 정상입니다

<img src="NSL3130_driver/asset/intrinsic_output.png" alt="intrinsic_output" />

```
[intrinsic] Final RMS=2.37  K=1004.4,1005.4  D=[[ 0.054  0.032 -0.031  0.016]]
[intrinsic] Saved → .../calib_output/N00A5060D_intrinsic.yml
```

> **참고**: GUI의 CALIBRATE 결과(`D = [0.0, 0.0, 0.0, 0.0]`)는 cameracalibrator 내부 초기화 문제로 정상적으로 보이지 않습니다.  
> 실제 fisheye 캘리브레이션은 **[SAVE]** 시 터미널에서 별도로 수행되며, 위 출력의 D 값이 실제 저장되는 값입니다.

**생성 결과:**

```
~/colcon_ws/src/NSL-3130AA-ROS2/calib_output/
  {camera_id}_intrinsic.yml
```

```yaml
# 파일 내용 예시 (N00A5060D_intrinsic.yml)
camera_id: "N00A5060D"
image_width: 1920
image_height: 1080
distortion_model: "equidistant"    # Fisheye 모델
camera_matrix: !!opencv-matrix
  rows: 3  cols: 3  dt: d
  data: [ fx, 0, cx, 0, fy, cy, 0, 0, 1 ]
distortion_coefficients: !!opencv-matrix
  rows: 1  cols: 4  dt: d
  data: [ k1, k2, k3, k4 ]
```

**Rectify 결과 확인 (선택):**

캘리브레이션 후 보정 결과가 올바른지 시각적으로 확인하려면 debug 모드를 사용하세요.

```bash
ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py debug:=true
```

<img src="NSL3130_driver/asset/intrinsic_debug.png" alt="intrinsic_debug" />

Original(원본) | Rectified(보정) 화면이 나란히 표시됩니다.  
상단 `balance` 트랙바로 크롭 비율을 실시간으로 조정할 수 있습니다 (0 = 검은 테두리 제거, 100 = 전체 픽셀 유지).

> 보정 결과가 이상해 보인다면 샘플이 부족하거나 편향된 것입니다. 다시 캘리브레이션을 수행해 주세요.

---

### 4-2. Extrinsic 캘리브레이션

**필요 조건**: `camera.launch.py` 실행 중, Intrinsic 완료 (`{camera_id}_intrinsic.yml` 존재)

> `{camera_id}_intrinsic.yml` 파일이 없으면 실행이 즉시 중단됩니다.

```bash
cd ~/colcon_ws
ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py
```

**진행 방법:**

1. 체커보드를 카메라와 LiDAR FOV가 겹치는 위치에 고정해 주세요
2. **`s`**: 현재 프레임 캡처 → 2D·3D 코너 선택 창이 열립니다 (여러 위치에서 반복, 최소 5~10장 권장)
3. **`c`**: R|t 행렬 계산 및 저장
4. **`r`**: 수집 데이터 초기화
5. **`q`**: 종료

**생성 결과:**

```
~/colcon_ws/src/NSL-3130AA-ROS2/calib_output/
  {camera_id}_extrinsic.yml
```

```yaml
# 파일 내용 예시 (N00A5060D_extrinsic.yml)
# 변환 방향: x_cam = R · x_lidar + t  (LiDAR 좌표 → 카메라 좌표)
camera_id: "N00A5060D"
R: !!opencv-matrix               # 3×3 회전 행렬
  rows: 3  cols: 3  dt: d
  data: [ r00, r01, r02,
          r10, r11, r12,
          r20, r21, r22 ]
t: !!opencv-matrix               # 3×1 평행이동 벡터 (단위: m)
  rows: 3  cols: 1  dt: d
  data: [ tx, ty, tz ]
```

저장 후 **드라이버를 재시작**하시면 `/camera/point_cloud_rgb` 토픽이 캘리브레이션 결과 기반으로 활성화됩니다.

---

## 5. 파라미터 설정

`~/colcon_ws/src/NSL-3130AA-ROS2/NSL3130_driver/src/roboscan_nsl3130/lidar_params.yaml` 에서 초기값을 설정하실 수 있습니다.  
실행 중에는 rqt_reconfigure_combo 창에서 실시간으로 변경·저장하실 수 있습니다.

| 키 | 기본값 | 설명 |
|----|--------|------|
| `IP Addr` | `192.168.2.220` | 카메라 IP |
| `ImageType` | `RGB_DISTANCE` | 이미지 모드 |
| `LensType` | `LENS_SF` | 렌즈 종류 |
| `MaxDistance` | `12500` | 최대 거리 (mm) |
| `LidarAngle` | `0` | 포인트클라우드 회전 오프셋 (도) |

---

## Phase Wrapping Avoidance and Correction

<img width="1597" height="464" alt="Image" src="https://github.com/user-attachments/assets/c715cf77-1e8a-4921-a657-dbc7799741fc" />

## Average FPS

```
최대 20 fps
```
