#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_DIR="$SCRIPT_DIR/wheeltec_imu"
UDEV_RULE="/etc/udev/rules.d/99-wheeltec-imu.rules"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# --- Kiểm tra quyền sudo ---
if [[ $EUID -ne 0 ]]; then
    error "Script phải chạy với sudo: sudo $0"
    exit 1
fi

# --- Kiểm tra kernel headers ---
KERNEL_VER="$(uname -r)"
if [[ ! -d "/lib/modules/$KERNEL_VER/build" ]]; then
    error "Không tìm thấy kernel headers cho $KERNEL_VER"
    error "Chạy: sudo apt install linux-headers-$KERNEL_VER"
    exit 1
fi

# ============================================================
# Bước 1 — Build kernel module
# ============================================================
info "Bước 1/3 — Build kernel module..."
cd "$MODULE_DIR"
make clean 2>/dev/null || true
if ! make; then
    error "Build thất bại."
    exit 1
fi
info "Build thành công: wheeltec_imu_mod.ko"

# ============================================================
# Bước 2 — Cài module vào kernel
# ============================================================
info "Bước 2/3 — Cài module vào kernel..."
mkdir -p "/lib/modules/$KERNEL_VER/extra/"
install -m 0644 wheeltec_imu_mod.ko "/lib/modules/$KERNEL_VER/extra/"
depmod -a
info "Module đã cài vào /lib/modules/$KERNEL_VER/extra/"

# ============================================================
# Bước 3 — Tạo udev rule và cấp quyền
# ============================================================
info "Bước 3/3 — Tạo udev rule..."
cat > "$UDEV_RULE" << 'EOF'
SUBSYSTEM=="misc", KERNEL=="wheeltec_imu*", GROUP="dialout", MODE="0660"
EOF
udevadm control --reload-rules
udevadm trigger --subsystem-match=misc
info "udev rule đã tạo: $UDEV_RULE"

# ============================================================
# Xong
# ============================================================
echo ""
info "Cài đặt hoàn tất."
echo ""
echo "  Load module:    sudo modprobe wheeltec_imu_mod"
echo "  Kiểm tra:       lsmod | grep wheeltec"
echo "  Device nodes:   ls /dev/wheeltec_imu*"
echo ""
warn "Nếu user chưa thuộc group dialout:"
echo "  sudo usermod -aG dialout \$USER   # rồi logout/login lại"
