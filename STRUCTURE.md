# Cấu trúc wheeltec_imu.c

## Tổng quan luồng khởi động

```
insmod / cắm USB
    → wheeltec_probe()
        → cp210x_setup()         UART init
        → register /dev files
        → schedule tare_work     gyro tare trước khi stream
            → #fconfig → #fimucal_gyro → #fsave → #fconfig → #freboot → y
            → chờ USB disconnect/reconnect (hoặc manual reinit)
            → submit bulk_in_urb  stream bắt đầu
            → schedule calib_work  thu thập bias 3s
                → tính mean bias (nrad/s)
                → gyro_calib_done = true
    → publish_raw() mỗi frame:
        → subtract gyro bias trước khi gửi userspace
```

---

## 1. Constants & Defines

| Nhóm | Nội dung |
|------|----------|
| Module param | `default_baud = 921600` |
| USB ID | VID=`0x10c4`, PID=`0xE100` (CP2102 / N100) |
| CP210x commands | `IFC_ENABLE`, `PURGE`, `SET_BAUDRATE`, `SET_MHS`, `SET_LINE_CTL` |
| Frame protocol | `WT_FRAME_HEAD=0xFC`, `WT_FRAME_END=0xFD` |
| Frame types | `TYPE_IMU=0x40` (56B), `TYPE_AHRS=0x41` (48B), `TYPE_INSGPS=0x42` (84B) |

---

## 2. ABI Structs — giao diện /dev

### `wt_raw_frame` (60 bytes) — `/dev/wheeltec_imuN_raw`

| Field | Type | Đơn vị | Nguồn |
|-------|------|---------|-------|
| `ktime_ns` | u64 | ns | `ktime_get_real_ns()` khi nhận frame |
| `seq` | u32 | — | frame counter tăng dần |
| `hz` | u32 | Hz | tính từ hw_timestamp EMA |
| `gyro_x/y/z` | float | rad/s | firmware (đã trừ bias) |
| `accel_x/y/z` | float | m/s² | firmware |
| `mag_x/y/z` | float | mG | firmware |
| `hw_timestamp` | s64 | µs | firmware |

### `wt_filter_frame` (60 bytes) — `/dev/wheeltec_imuN_filter`

| Field | Type | Đơn vị | Nguồn |
|-------|------|---------|-------|
| `ktime_ns` | u64 | ns | kernel time |
| `seq` | u32 | — | frame counter |
| `hz` | u32 | Hz | EMA |
| `roll/pitch/heading_speed` | float | rad/s | firmware |
| `roll/pitch/heading` | float | rad | firmware |
| `qw/qx/qy/qz` | float | — | firmware |
| `hw_timestamp` | s64 | µs | firmware |

---

## 3. Per-device state (`struct wheeltec_dev`)

```
wheeltec_dev
├── USB
│   ├── udev, intf
│   ├── bulk_in_urb, bulk_in_buf, bulk_in_size, bulk_in_addr
│   └── bulk_out_addr
│
├── Frame parser
│   ├── fbuf[FRAME_BUF_SIZE]   ring buffer raw bytes từ USB
│   └── fbuf_len
│
├── Raw snapshot (TYPE_IMU)
│   ├── raw_snap, raw_seq      seqcount-protected snapshot
│   ├── raw_wq                 wait queue cho blocking read
│   ├── has_raw                atomic flag
│   ├── raw_frames_ok          stats
│   ├── raw_fseq               frame counter
│   ├── raw_last_hw_ts         để tính period
│   └── raw_period_avg         EMA period → hz
│
├── Filter snapshot (TYPE_AHRS)
│   └── (tương tự Raw snapshot)
│
├── Stats
│   ├── frames_crc_err
│   └── frames_dropped
│
├── Misc devices
│   ├── mdev_raw, name_raw
│   ├── mdev_filter, name_filter
│   ├── mdev_state, name_state
│   └── mdev_ctrl, name_ctrl
│
├── URB error recovery
│   ├── urb_retry_work
│   └── urb_err_cnt            reset về 0 khi URB thành công
│
├── Firmware reboot
│   ├── reboot_work
│   ├── reboot_pending         cmpxchg guard (chỉ 1 reboot tại 1 thời điểm)
│   ├── reboot_status          0=idle 1=in-progress 2=success 3=warning
│   └── fconfig_pending        báo completion handler đang ở config mode
│
├── Firmware tare
│   ├── tare_work
│   ├── tare_pending           cmpxchg guard
│   ├── tare_status            0=idle 1=in-progress 2=success 3=warning
│   ├── tare_type              0=gyro 1=acce 2=level
│   ├── tare_wait_ms           thời gian chờ calibration (default 2500ms)
│   ├── tare_cmd_pending       log ASCII response trong tare sequence
│   └── startup_tare_done      tránh tare lại khi reconnect
│
└── Gyro bias calibration
    ├── calib_work
    ├── gyro_calib_collecting  đang trong giai đoạn thu thập
    ├── gyro_calib_done        bias đã sẵn sàng, đang áp dụng
    ├── calib_gyro_[x|y|z]_bits  atomic: softirq feed → calib_work đọc
    ├── calib_sample_ready     atomic flag: có sample mới
    ├── gyro_sum_[x|y|z]       s64 accumulator (nrad/s)
    ├── gyro_calib_count       số sample đã thu
    └── gyro_bias_[x|y|z]      s32 mean bias (nrad/s), áp dụng mỗi frame
```

---

## 4. Các functions theo nhóm

### CRC
| Function | Mô tả |
|----------|-------|
| `wheeltec_crc8()` | CRC8 over 4 header bytes |
| `wheeltec_crc16()` | CRC16 over payload |

### CP210x UART helpers
| Function | Mô tả |
|----------|-------|
| `cp210x_ctrl_out()` | USB vendor control transfer |
| `cp210x_set_baud()` | Set baudrate qua control transfer |
| `cp210x_setup()` | Full UART init: enable → baud → 8N1 → DTR/RTS → purge |

### Bulk OUT
| Function | Mô tả |
|----------|-------|
| `wt_bulk_write()` | Gửi ASCII command xuống IMU (synchronous) |

### Workqueues
| Function | Context | Mô tả |
|----------|---------|-------|
| `wt_reboot_work_fn()` | process | Sequence reboot firmware |
| `wt_tare_work_fn()` | process | Sequence tare firmware + trigger calib |
| `urb_retry_work_fn()` | process | Retry URB submit sau lỗi transient |
| `gyro_calib_work_fn()` | process | Thu thập 3s gyro, tính bias |

### Soft-float helpers (pure integer, không cần SSE)
| Function | Mô tả |
|----------|-------|
| `f32bits_to_nrads(u32 bits)` | Decode IEEE 754 → nrad/s (s32) |
| `f32bits_sub_nrads(u32 bits, s32 bias)` | Subtract bias, trả về IEEE 754 bits |

### Frame processing
| Function | Mô tả |
|----------|-------|
| `expected_payload_len()` | Lookup payload length theo type |
| `parse_buffer()` | Scan fbuf, kiểm tra CRC, dispatch |
| `publish_raw()` | Build `wt_raw_frame`, feed calib, apply bias, wake waitqueue |
| `publish_filter()` | Build `wt_filter_frame`, wake waitqueue |

### URB completion
| Function | Mô tả |
|----------|-------|
| `wheeltec_read_complete()` | Nhận USB data, log ASCII, gọi parse_buffer, resubmit URB |

---

## 5. /dev interface

### `/dev/wheeltec_imuN_raw` — read-only
- Blocking read: chờ frame mới, trả về `wt_raw_frame` (60 bytes)
- Non-blocking: trả về `-EAGAIN` nếu chưa có frame
- `poll()` supported

### `/dev/wheeltec_imuN_filter` — read-only
- Tương tự `_raw`, trả về `wt_filter_frame` (60 bytes)

### `/dev/wheeltec_imuN_state` — read-only
```
connected=1
raw_frames_ok=12345
filter_frames_ok=12345
frames_crc_err=0
frames_dropped=0
reboot_status=idle
tare_status=idle
gyro_calib=done
gyro_bias_x_urad=123
gyro_bias_y_urad=-45
gyro_bias_z_urad=78
```

### `/dev/wheeltec_imuN_ctrl` — write-only

| Lệnh | Mô tả |
|------|-------|
| `freboot` | Reboot firmware |
| `tare:gyro:2500` | Gyro tare, chờ 2500ms |
| `tare:accel:2500` | Accelerometer tare |
| `tare:level:2500` | Level tare |
| `recalib` | Reset và chạy lại gyro bias calibration |

---

## 6. USB driver lifecycle

### `wheeltec_probe()`
```
1. Tìm bulk IN/OUT endpoints
2. kzalloc + init wheeltec_dev (atomic, waitqueue, workqueue)
3. cp210x_setup() — init UART
4. Register /dev files (raw, filter, state, ctrl)
5. startup_tare_done == false?
       YES → schedule tare_work
              (tare_work tự submit URB + schedule calib_work khi xong)
       NO  → submit URB + schedule calib_work
              (reconnect sau tare reboot)
```

### `wheeltec_disconnect()`
```
1. disconnected = true, wake_up tất cả waitqueues
2. usb_kill_urb
3. cancel_work_sync: urb_retry, reboot, tare, calib
4. Unregister /dev, disable UART, free URB/buf
5. kfree(wd)
```

### `wheeltec_pre_reset()` / `wheeltec_post_reset()`
- Pre: kill URB, cancel works
- Post: cp210x_setup() lại, resubmit URB

---

## 7. Sequence tare đầy đủ (mirrors ImuControl.TareAsync)

```
kill URB → disable UART → purge → enable UART
    TX >> #fconfig       (300+800ms, arm URB để đọc response)
    TX >> #fimucal_gyro  (300ms + tare_wait_ms, default 2500ms)
    TX >> #fsave         (300+800ms)
    TX >> #fconfig       (300+800ms, lần 2 — đảm bảo config mode)
    purge
    TX >> #freboot       (300+1500ms)
    purge
    TX >> y              (300+800ms)
    disable UART

    chờ USB disconnect (tối đa 15s)
        → disconnect: probe() tự restore + schedule calib_work
        → không disconnect: manual cp210x_setup() → submit URB → verify stream → schedule calib_work
```

## 8. Gyro bias calibration

```
gyro_calib_work_fn (process context, 3s):
    loop:
        đợi calib_sample_ready (atomic, set bởi publish_raw)
        đọc calib_gyro_[xyz]_bits (IEEE 754 float bits)
        f32bits_to_nrads() → cộng vào gyro_sum_[xyz]
        gyro_calib_count++
    sau 3s:
        gyro_bias_[xyz] = gyro_sum_[xyz] / gyro_calib_count   (nrad/s)
        gyro_calib_done = true

publish_raw() mỗi frame (softirq):
    nếu gyro_calib_done:
        f32bits_sub_nrads(gyro_x_bits, gyro_bias_x) → snap.gyro_x đã trừ bias
        tương tự y, z
    gửi snap lên userspace
```
