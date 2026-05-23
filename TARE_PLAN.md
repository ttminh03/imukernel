# Kế hoạch sửa `wt_tare_work_fn` trong `wheeltec_imu.c`

---

## 1. Cơ chế Tare của WHEELTEC N100

### 3 loại tare

| Loại | Lệnh ASCII | Mục đích | Yêu cầu phần cứng |
|------|-----------|----------|-------------------|
| `gyro` | `#fimucal_gyro` | Tính lại zero bias tĩnh của gyroscope | IMU phải đứng yên |
| `acce` | `#fimucal_acce` | Hiệu chỉnh zero bias accelerometer về 1g | IMU nằm ngang + đứng yên |
| `level` | `#fimucal_level` | Chuyển hệ tọa độ sang mặt phẳng lắp đặt | IMU nằm ngang + đứng yên |

### Sequence chuẩn (từ `ImuControl.cs` — đã được xác nhận hoạt động)

`SendOn()` trong C# luôn thêm 300ms internal delay trước khi return.  
Delay ghi trong code là delay *sau* `SendOn`, tức tổng = 300ms + delay.

```
1. _port.Open()  →  DiscardInBuffer()

2. Send("#fconfig")     →  Delay(800ms)   = 300 + 800  = 1100ms tổng
3. Send("#fimucal_X")   →  Delay(2500ms)  = 300 + 2500 = 2800ms tổng
4. Send("#fsave")       →  Delay(800ms)   = 300 + 800  = 1100ms tổng
5. Send("#fconfig")     →  Delay(800ms)   = 300 + 800  = 1100ms tổng  ← lần 2!
   DiscardInBuffer()
6. Send("#freboot")     →  Delay(1500ms)  = 300 + 1500 = 1800ms tổng
   DiscardInBuffer()
7. Send("y")            →  Delay(800ms)   = 300 + 800  = 1100ms tổng

8. _port.Close()
9. Delay(5000ms)  →  chờ thiết bị reboot

10. OpenFreshPort (retry 20s)
11. CountFrameHeads (15s)  →  xác nhận stream đã hoạt động lại
```

### Tại sao cần `#fconfig` lần 2?

IMU N100 có 2 mode: **streaming mode** (gửi binary frame liên tục) và **config mode** (nhận ASCII command).  
Sau khi `#fsave` lưu xong, firmware có thể tự chuyển về streaming mode.  
`#fconfig` lần 2 đảm bảo thiết bị đang ở config mode trước khi gửi `#freboot`.

---

## 2. Vấn đề của `wt_tare_work_fn` hiện tại

### So sánh kernel hiện tại vs C# reference

| Bước | C# Reference | Kernel hiện tại | Vấn đề |
|------|-------------|-----------------|--------|
| Stop stream | `usb_kill_urb` + disable/purge/enable | disable/purge/enable (không kill URB) | URB vẫn đang chạy, gây race condition |
| `#fconfig` lần 1 | 300 + 800 = 1100ms | 800ms | Thiếu 300ms SendOn |
| Purge sau `#fconfig` | Có (`DiscardInBuffer`) | Có | OK |
| `#fimucal_X` | 300 + 2500 = 2800ms | 2500ms (tare_wait_ms) | Thiếu 300ms SendOn |
| `#fsave` | 300 + 800 = 1100ms | 800ms | Thiếu 300ms SendOn |
| **`#fconfig` lần 2** | **Có** | **Không có** | **Thiếu bước quan trọng** |
| Purge trước `#freboot` | Có | Không có | Thiếu |
| `#freboot` | 300 + 1500 = 1800ms | 300ms | Sai thời gian, chỉ 300ms |
| Purge sau `#freboot` | Có | Không có | Thiếu |
| `y` | 300 + 800 = 1100ms | 300ms | Sai thời gian |
| Đóng port | `_port.Close()` (disable UART) | Không có | Thiếu |
| Chờ reboot | 5s | Không có (báo SUCCESS ngay) | **Thiếu hoàn toàn** |
| Verify stream | CountFrameHeads 15s | Không có | **Thiếu hoàn toàn** |
| Fallback reinit | OpenFreshPort retry 20s | Không có | Thiếu |

---

## 3. Kế hoạch sửa

### 3.1 Stop stream (đầu function)

**Hiện tại:**
```c
cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_DISABLE);
msleep(100);
cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);
msleep(50);
mutex_lock(&wd->io_mutex);
wd->fbuf_len = 0;
mutex_unlock(&wd->io_mutex);
cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_ENABLE);
msleep(50);
```

**Sửa thành:**
```c
usb_kill_urb(wd->bulk_in_urb);           // Kill URB trước — giống reboot
cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_DISABLE);
msleep(100);
cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);
msleep(50);
wd->fbuf_len = 0;                        // Bỏ mutex vì URB đã bị kill
cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_ENABLE);
msleep(50);
```

**Lý do:** `wt_reboot_work_fn` dùng pattern này và hoạt động tốt. Phải kill URB trước khi disable UART để tránh race condition giữa completion callback và tare sequence.

---

### 3.2 `#fconfig` lần 1 — thêm timing + re-arm URB để log response

**Hiện tại:**
```c
wt_bulk_write(wd, "#fconfig\r\n", 10);
msleep(800);
cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);
```

**Sửa thành:**
```c
dev_info(&wd->intf->dev, "TX >> #fconfig\n");
atomic_set(&wd->fconfig_pending, 1);
rc = usb_submit_urb(wd->bulk_in_urb, GFP_KERNEL);   // arm để đọc response
if (rc && rc != -EPERM && !wd->disconnected)
    dev_warn(&wd->intf->dev, "tare: URB submit rc=%d\n", rc);
wt_bulk_write(wd, "#fconfig\r\n", 10);
msleep(300 + 800);                                   // SendOn(300) + Delay(800)
atomic_set(&wd->fconfig_pending, 0);
usb_kill_urb(wd->bulk_in_urb);
cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL);
wd->fbuf_len = 0;
```

**Lý do:** Giống pattern trong `wt_reboot_work_fn`. Re-arm URB để ISR đọc và log response ASCII từ thiết bị. `fconfig_pending=1` báo cho completion handler biết đang ở config mode.

---

### 3.3 `#fimucal_X` — thêm 300ms SendOn

**Hiện tại:**
```c
wt_bulk_write(wd, cal_cmd, strlen(cal_cmd));
for (i = 0; i < wd->tare_wait_ms / 100 && !wd->disconnected; i++)
    msleep(100);
```

**Sửa thành:**
```c
dev_info(&wd->intf->dev, "TX >> %s (wait %dms)\n", cal_cmd, wd->tare_wait_ms);
wt_bulk_write(wd, cal_cmd, cal_len);
msleep(300);                                          // SendOn internal 300ms
for (i = 0; i < wd->tare_wait_ms / 100 && !wd->disconnected; i++)
    msleep(100);
```

**Lý do:** C# `SendOn` luôn delay 300ms sau khi gửi command. Calibration cần thời gian thiết bị xử lý xong *trước* khi đếm `tare_wait_ms`. Dùng `cal_len` thay `strlen` vì kernel không có libc strlen tối ưu cho constant string.

---

### 3.4 `#fsave` — thêm 300ms SendOn

**Hiện tại:**
```c
wt_bulk_write(wd, "#fsave\r\n", 8);
msleep(800);
```

**Sửa thành:**
```c
dev_info(&wd->intf->dev, "TX >> #fsave\n");
wt_bulk_write(wd, "#fsave\r\n", 8);
msleep(300 + 800);                                   // SendOn(300) + Delay(800)
```

---

### 3.5 Thêm `#fconfig` lần 2 + purge trước `#freboot`

**Hiện tại:** Không có.

**Thêm vào:**
```c
dev_info(&wd->intf->dev, "TX >> #fconfig (2nd)\n");
wt_bulk_write(wd, "#fconfig\r\n", 10);
msleep(300 + 800);                                   // SendOn(300) + Delay(800)
cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL); // DiscardInBuffer
wd->fbuf_len = 0;
```

**Lý do:** Đây là bước **bị thiếu quan trọng nhất**. Sau `#fsave`, thiết bị có thể thoát config mode. Phải vào config mode lại trước khi `#freboot`, nếu không lệnh reboot bị bỏ qua.

---

### 3.6 `#freboot` — sửa timing + thêm purge sau

**Hiện tại:**
```c
wt_bulk_write(wd, "#freboot\r\n", 10);
msleep(300);
```

**Sửa thành:**
```c
dev_info(&wd->intf->dev, "TX >> #freboot\n");
wt_bulk_write(wd, "#freboot\r\n", 10);
msleep(300 + 1500);                                  // SendOn(300) + Delay(1500)
cp210x_ctrl_out(wd->udev, CP210X_PURGE, PURGE_ALL); // DiscardInBuffer
```

**Lý do:** C# dùng 1500ms delay sau `#freboot` (không phải 300ms). Thiếu purge khiến response của `#freboot` bị firmware xử lý nhầm là data.

---

### 3.7 `y` — sửa timing

**Hiện tại:**
```c
wt_bulk_write(wd, "y\r\n", 3);
msleep(300);
```

**Sửa thành:**
```c
dev_info(&wd->intf->dev, "TX >> y\n");
wt_bulk_write(wd, "y\r\n", 3);
msleep(300 + 800);                                   // SendOn(300) + Delay(800)
```

---

### 3.8 Thêm disable UART + wait 5s + verify stream (hiện tại hoàn toàn thiếu)

**Hiện tại:** Sau `y` báo SUCCESS ngay lập tức.

**Sửa thành:**
```c
/* _port.Close() → disable UART */
cp210x_ctrl_out(wd->udev, CP210X_IFC_ENABLE, UART_DISABLE);

/* Delay(5s) → chờ USB disconnect (IMU đang reboot) */
dev_info(&wd->intf->dev, "tare: waiting up to 15s for USB disconnect/reconnect...\n");
for (i = 0; i < 150; i++) {
    msleep(100);
    if (wd->disconnected) {
        /* probe() sẽ tự restore stream */
        dev_info(&wd->intf->dev, "tare: USB disconnect — probe will restore stream\n");
        atomic_set(&wd->tare_cmd_pending, 0);
        atomic_set(&wd->tare_status, 2);   // SUCCESS
        atomic_set(&wd->tare_pending, 0);
        return;
    }
}

/* Không disconnect → fallback manual reinit (= OpenFreshPort) */
dev_info(&wd->intf->dev, "tare: no disconnect — manual UART reinit\n");
rc = cp210x_setup(wd->udev, default_baud);
...

/* CountFrameHeads 15s */
for (i = 0; i < 150 && !wd->disconnected; i++) {
    msleep(100);
    frames_after = atomic64_read(&wd->raw_frames_ok) + ...;
    if (frames_after > frames_before + 5) {
        dev_info(&wd->intf->dev, "tare: SUCCESS — stream restored\n");
        ...
        return;
    }
}
```

**Lý do:** C# đóng port rồi chờ 5s, sau đó `OpenFreshPort` retry 20s và `CountFrameHeads` 15s. Kernel phải làm tương đương: disable UART, chờ USB disconnect. Nếu không disconnect (một số firmware variant không USB-disconnect khi reboot UART), fallback manual reinit giống pattern trong `wt_reboot_work_fn` đang hoạt động.

---

## 4. Tổng kết các thay đổi

| # | Thay đổi | Mức độ quan trọng |
|---|----------|------------------|
| 1 | Thêm `usb_kill_urb` trước khi disable UART | Cao — tránh race condition |
| 2 | Thêm 300ms vào mỗi step (`#fconfig`, `#fsave`, `#freboot`, `y`) | Trung bình — timing đúng |
| 3 | Thêm `#fconfig` lần 2 trước `#freboot` | **Rất cao — bước thiếu quan trọng nhất** |
| 4 | Thêm purge trước `#freboot` và sau `#freboot` | Cao |
| 5 | Thêm disable UART sau `y` | Trung bình |
| 6 | Thêm wait + detect USB disconnect | Cao — phát hiện reboot thành công |
| 7 | Thêm fallback manual UART reinit | Cao — tương đương `OpenFreshPort` |
| 8 | Thêm verify stream 15s | Cao — xác nhận tare thực sự thành công |

---

## 5. Không thay đổi

- Protocol `tare:<type>:<wait_ms>` từ `/dev/wheeltec_ctrl` — giữ nguyên
- `tare_type`, `tare_wait_ms`, `tare_status`, `tare_pending` — giữ nguyên
- Default `tare_wait_ms = 2500` — giữ nguyên (user có thể override)
- `wt_reboot_work_fn` — không đụng vào
