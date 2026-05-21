# Hướng dẫn cài WheelTec N100 IMU Driver

## Yêu cầu

- Linux kernel 6.12+
- Kernel headers đã cài: `sudo apt install linux-headers-$(uname -r)`
- `libusb-1.0` để dùng `cp210x-cfg`: `sudo apt install libusb-1.0-0-dev`

---

## Bước 1 — Provisioning USB descriptor cho CP2102

Board N100 xuất xưởng với VID/PID mặc định của Silicon Labs (`10c4:ea60`).
Kernel driver chỉ bind với `10c4:e100`, nên phải reflash trước.

### 1.1 Build cp210x-cfg

```bash
cd imukernel/cp210x-cfg
make
sudo cp cp210x-cfg /usr/local/bin/
```

### 1.2 Cắm board N100 vào USB, kiểm tra device hiện tại

```bash
sudo cp210x-cfg
# Output mẫu:
# ID 10c4:ea60 @ bus 003, dev 011: CP2102 USB to UART Bridge Controller
# Product ID: ea60
# Serial: 0001
```

### 1.3 Flash PID mới

```bash
sudo cp210x-cfg -P 0xE100
```

Sau lệnh này device sẽ tự disconnect/reconnect (bình thường).

### 1.4 Flash Name và Serial

```bash
sudo cp210x-cfg -m 10c4:e100 \
    -N "N100 IMU" \
    -S "WT-N100-$(date +%Y%m%d-%H%M%S)"
```

### 1.5 Xác nhận

```bash
lsusb | grep -i 10c4
# Bus 003 Device 023: ID 10c4:e100 Silicon Labs N100 IMU
```

> **Lưu ý:** Thông tin được ghi vào EEPROM của chip, lưu vĩnh viễn sau khi unplug.

---

## Bước 2 — Build kernel module

```bash
cd imukernel/wheeltec_imu
make
```

Output mong đợi:

```
make -C /lib/modules/6.xx.x-generic/build M=... modules
  CC [M]  .../wheeltec_imu.o
  CC [M]  .../crc_tables.o
  LD [M]  .../wheeltec_imu_mod.o
  MODPOST .../wheeltec_imu_mod.ko
```

---

## Bước 3 — Cài module vào kernel

```bash
sudo make install
```

Lệnh này chạy:
```
install -m 0644 wheeltec_imu_mod.ko /lib/modules/$(uname -r)/extra/
depmod -a
```

---

## Bước 4 — Load module

```bash
sudo modprobe -r wheeltec_imu_mod
sudo modprobe wheeltec_imu_mod
```

Kiểm tra load thành công:

```bash
lsmod | grep wheeltec
# wheeltec_imu_mod    16384  0

dmesg | tail -5
# wheeltec_imu0 bound (serial=WT-N100-..., baud=921600, max=64)
```

Device node xuất hiện:

```bash
ls /dev/wheeltec_imu*
# /dev/wheeltec_imu0_ctrl
# /dev/wheeltec_imu0_raw
#/dev/wheeltec_imu0-filter
# /dev/wheeltec_imu0_state
```

---

## Bước 5 — Đọc dữ liệu IMU

### Xem trạng thái driver

```bash
cat /dev/wheeltec_imu0_state
# connected=1
# frames_ok=1234
# frames_crc_err=0
# frames_dropped=0
# last_frame_counter=1234
# last_timestamp_ns=1746700000000000000
```

### Đọc data binary từ userspace (C)

```c
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

struct wheeltec_snapshot {
    uint64_t timestamp_ns;
    uint32_t frame_counter;
    uint8_t  frame_type;
    uint8_t  payload_len;
    uint8_t  reserved[2];
    uint8_t  payload[84];   /* max INSGPS_LEN */
} __attribute__((packed));

int main(void) {
    int fd = open("/dev/wheeltec_imu0_data", O_RDONLY);
    struct wheeltec_snapshot snap;
    read(fd, &snap, sizeof(snap));
    /* snap.payload chứa float IEEE754: accel, gyro, angle, ... */
    close(fd);
}
```

---

## Load tự động khi boot

```bash
echo "wheeltec_imu_mod" | sudo tee /etc/modules-load.d/wheeltec.conf
```

---

## Unload module

```bash
sudo modprobe -r wheeltec_imu_mod
```

---

## Reboot IMU (reset mà không unplug)

### Build

```bash
gcc -O2 -Wall -o imu_reboot imu_reboot.c
```

### Dùng

**Cách 1 — USB reset** (nhanh ~1s, driver tự re-probe):

```bash
sudo ./imu_reboot --usb-reset
```

**Cách 2 — Module cycle** (rmmod + modprobe, đầy đủ ~3s):

```bash
sudo ./imu_reboot --module-cycle
```

Đổi baud khi load lại:

```bash
sudo ./imu_reboot --module-cycle --baud 115200
```

Sau khi reboot thành công, chương trình in ra nội dung `/dev/wheeltec_imu0_state`.

---

## Cấp quyền đọc device tự động (udev rule)

Mặc định `/dev/wheeltec_imu0_*` chỉ root mới đọc được. Tạo udev rule để tự động cấp quyền mỗi lần cắm USB hoặc boot:

```bash
echo 'SUBSYSTEM=="misc", KERNEL=="wheeltec_imu*", MODE="0444"' \
    | sudo tee /etc/udev/rules.d/99-wheeltec-imu.rules

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=misc
```

Kiểm tra:

```bash
ls -l /dev/wheeltec_imu0_raw /dev/wheeltec_imu0_filter
# cr--r--r-- 1 root root ... /dev/wheeltec_imu0_raw
# cr--r--r-- 1 root root ... /dev/wheeltec_imu0_filter
```

---

## Troubleshooting

| Triệu chứng | Nguyên nhân | Cách fix |
|---|---|---|
| `make` lỗi `No rule to make target` | Thiếu kernel headers | `sudo apt install linux-headers-$(uname -r)` |
| `modprobe` lỗi `Module not found` | Chưa chạy `depmod` | `sudo depmod -a` rồi thử lại |
| `/dev/wheeltec_imu0_data` không xuất hiện | PID chưa đổi hoặc sai | Kiểm tra `lsusb`, chạy lại bước 1 |
| `dmesg` báo `no bulk-in endpoint` | Board lỗi hoặc sai interface | Thử unplug/replug |
| `frames_crc_err` tăng liên tục | Sai baud rate | Load lại với `sudo modprobe wheeltec_imu_mod default_baud=115200` |
