savedcmd_wheeltec_imu_mod.o := ld -m elf_x86_64 -z noexecstack --no-warn-rwx-segments   -r -o wheeltec_imu_mod.o @wheeltec_imu_mod.mod 
