# IMU Device Setup — Wheeltec N100

## Kernel driver device files

Kernel driver tạo 4 char device files:

```
/dev/wheeltec_imu0_raw     — raw frame: gyro + accel + mag (400Hz)
/dev/wheeltec_imu0_filter  — filter frame: euler + quaternion (200Hz)
/dev/wheeltec_imu0_state   — trạng thái kết nối
/dev/wheeltec_imu0_ctrl    — ghi lệnh điều khiển (reboot, v.v.)
```

Permission mặc định: `crw-rw---- root dialout` — chỉ root và group `dialout` được đọc/ghi.

## Yêu cầu permission

User chạy RobotApp **phải thuộc group `dialout`**.

### Thêm user vào group dialout

```bash
sudo usermod -aG dialout <username>
```

Sau đó logout và login lại để group có hiệu lực.

### Kiểm tra

```bash
groups <username>
# Output phải có: dialout
```

### Ví dụ

```bash
# User "robot" chạy RobotApp
sudo usermod -aG dialout robot

# User "minh" trong môi trường dev
sudo usermod -aG dialout minh
```

## udev rule (tùy chọn)

Nếu muốn đổi group hoặc permission khác mặc định, tạo file `/etc/udev/rules.d/99-wheeltec-imu.rules`:

```
SUBSYSTEM=="misc", KERNEL=="wheeltec_imu*", GROUP="dialout", MODE="0660"
```

Reload udev:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Docker / container

Nếu chạy RobotApp trong Docker, cần mount device files và thêm group:

```yaml
# docker-compose.yml
devices:
  - /dev/wheeltec_imu0_raw:/dev/wheeltec_imu0_raw
  - /dev/wheeltec_imu0_filter:/dev/wheeltec_imu0_filter
  - /dev/wheeltec_imu0_ctrl:/dev/wheeltec_imu0_ctrl
  - /dev/wheeltec_imu0_state:/dev/wheeltec_imu0_state
group_add:
  - dialout
```
