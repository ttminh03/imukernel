# N100 USB descriptor provisioning

Each N100 board ships with default Silicon Labs CP2102 descriptors:

```
VID  = 0x10c4
PID  = 0xea60
iManufacturer = "Silicon Labs"
iProduct      = "CP2102 USB to UART Bridge Controller"
iSerial       = "0001" (factory default, often duplicated across units)
```

The `wheeltec_imu` kernel driver expects:

```
VID  = 0x10c4
PID  = 0xE100
iManufacturer = "WheelTec"
iProduct      = "N100 IMU"
iSerial       = "WT-N100-XXXX"   (unique per unit; required for distinguishing
                                  multiple N100 on same host)
```

Reflash before deployment.

## Tool A — cp210x-cfg (Linux CLI, recommended for batch)

```bash
# Install (one-time)
git clone https://github.com/DiUS/cp210x-cfg
cd cp210x-cfg && make && sudo make install

# Read current state
sudo cp210x-cfg

# Write new descriptors (assumes only one CP2102 on the bus)
sudo cp210x-cfg \
    -V 0x10c4 \
    -P 0xE100 \
    -m "WheelTec" \
    -p "N100 IMU" \
    -s "WT-N100-$(date +%Y%m%d-%H%M%S)"

# Power-cycle the USB connection (unplug/replug) for the new IDs to take effect.
```

## Tool B — CP21xx Customization Utility (Windows GUI, official)

Download from Silicon Labs website. Connect board, set:
- VID: `10C4`, PID: `E100`
- Manufacturer: `WheelTec`, Product: `N100 IMU`
- Serial: per-unit unique
- Click "Program Device".

## Verify

```bash
lsusb | grep -i wheeltec   # should show: ID 10c4:e100 WheelTec N100 IMU
udevadm info -a -n /dev/wheeltec_imu0_data | grep -E "ID_VENDOR|ID_MODEL|SERIAL"
```

## Recovery (factory reset to 0x10c4:0xea60)

If a board is mis-flashed and the kernel driver no longer binds, the device
still enumerates as a USB device (just without a tty or wheeltec_imu attached).
`cp210x-cfg` can talk to it as long as the host can see USB descriptors:

```bash
sudo cp210x-cfg -m 10c4:e100 -V 0x10c4 -P 0xea60 \
    -N "CP2102 USB to UART Bridge Controller" \
    -S "0001"

```

## OTP warning

Some early CP2102 silicon revisions have **one-time-programmable** descriptor
memory. Verify on a single sacrificial board before flashing a batch.
CP2102N (newer generation) supports unlimited rewrites.
