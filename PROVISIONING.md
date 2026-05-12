# Provisioning USB descriptor cho WheelTec N100 (CP2102)

## Tổng quan

Board N100 dùng chip **CP2102** (USB-to-UART của Silicon Labs).
Kernel driver `wheeltec_imu` chỉ bind với VID/PID cụ thể — phải flash trước khi dùng.

| Thông số | Giá trị mặc định (factory) | Giá trị cần flash |
|---|---|---|
| VID | `0x10c4` | `0x10c4` (giữ nguyên) |
| PID | `0xea60` | `0xe100` |
| Name | `CP2102 USB to UART Bridge Controller` | `N100 IMU` |
| Serial | `0001` | `WT-N100-YYYYMMDD-HHMMSS` |

---

## Cài công cụ cp210x-cfg

```bash
cd imukernel/cp210x-cfg
make
sudo cp cp210x-cfg /usr/local/bin/
```

---

## Đọc trạng thái hiện tại

```bash
# Nếu PID vẫn là ea60 (factory):
sudo cp210x-cfg

# Nếu PID đã đổi thành e100:
sudo cp210x-cfg -m 10c4:e100
```

Output mẫu:
```
ID 10c4:ea60 @ bus 003, dev 011: CP2102 USB to UART Bridge Controller
Model: CP2102
Vendor ID: 10c4
Product ID: ea60
Name: CP2102 USB to UART Bridge Controller
Serial: 0001
```

---

## Flash từng thông số

### Đổi PID

```bash
sudo cp210x-cfg -P 0xE100
```

> Sau lệnh này device tự disconnect/reconnect — bình thường.
> Dùng `-m 10c4:e100` cho các lệnh tiếp theo.

### Đổi Name (product name)

```bash
sudo cp210x-cfg -m 10c4:e100 -N "N100 IMU"
```

### Đổi Serial

```bash
sudo cp210x-cfg -m 10c4:e100 -S "WT-N100-$(date +%Y%m%d-%H%M%S)"
```

### Flash tất cả cùng lúc (khuyến nghị)

```bash
sudo cp210x-cfg -m 10c4:e100 \
    -N "N100 IMU" \
    -S "WT-N100-$(date +%Y%m%d-%H%M%S)"
```

> **Lưu ý:** `cp210x-cfg` không hỗ trợ đổi Manufacturer string.
> `lsusb` sẽ hiện `Silicon Labs N100 IMU` — đây là bình thường.

---

## Xác nhận sau khi flash

```bash
lsusb | grep -i 10c4
# Bus 003 Device 023: ID 10c4:e100 Silicon Labs N100 IMU

sudo cp210x-cfg -m 10c4:e100
# Vendor ID: 10c4
# Product ID: e100
# Name: N100 IMU
# Serial: WT-N100-20260508-103045
```

---

## Flash nhiều board cùng lúc

Khi có nhiều board cắm cùng lúc, dùng `-d bus:dev` để chỉ định từng device:

```bash
# Liệt kê tất cả CP210x:
lsusb | grep 10c4

# Flash board cụ thể theo bus:dev:
sudo cp210x-cfg -d 003:011 -P 0xE100
sudo cp210x-cfg -d 003:011 -m 10c4:e100 -N "N100 IMU" -S "WT-N100-$(date +%Y%m%d-%H%M%S)"

sudo cp210x-cfg -d 003:012 -P 0xE100
sudo cp210x-cfg -d 003:012 -m 10c4:e100 -N "N100 IMU" -S "WT-N100-$(date +%Y%m%d-%H%M%S)"
```

---

## Recovery — khôi phục về factory

Nếu flash sai và cần reset về mặc định:

```bash
sudo cp210x-cfg -m 10c4:e100 \
    -V 0x10c4 \
    -P 0xea60 \
    -N "CP2102 USB to UART Bridge Controller" \
    -S "0001"
```

---

## Tham số kỹ thuật cp210x-cfg

| Flag | Ý nghĩa | Ví dụ |
|---|---|---|
| `-V <vid>` | Ghi Vendor ID | `-V 0x10c4` |
| `-P <pid>` | Ghi Product ID | `-P 0xE100` |
| `-N <name>` | Ghi product name string | `-N "N100 IMU"` |
| `-S <serial>` | Ghi serial string | `-S "WT-N100-001"` |
| `-m <vid:pid>` | Chọn device theo VID:PID | `-m 10c4:e100` |
| `-d <bus:dev>` | Chọn device theo bus:dev | `-d 003:011` |
| `-l` | Liệt kê tất cả CP210x | `-l` |

> Thông tin được ghi vào **EEPROM** của chip — lưu vĩnh viễn sau khi unplug.
> Một số CP2102 đời cũ có EEPROM **OTP** (one-time programmable) — chỉ ghi được 1 lần.
> CP2102N (đời mới) hỗ trợ ghi lại không giới hạn.