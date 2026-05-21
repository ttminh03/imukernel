savedcmd_wheeltec_imu_mod.mod := printf '%s\n'   wheeltec_imu.o crc_tables.o | awk '!x[$$0]++ { print("./"$$0) }' > wheeltec_imu_mod.mod
