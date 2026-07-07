#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_MITIGATION_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

KSYMTAB_DATA(cp_crc8_table, "", "");
KSYMTAB_DATA(cp_crc16_table, "", "");

SYMBOL_CRC(cp_crc8_table, 0xbb5f28e7, "");
SYMBOL_CRC(cp_crc16_table, 0x43286ac7, "");

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xe859704b, "usb_alloc_urb" },
	{ 0xe7a02573, "ida_alloc_range" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x2556884a, "misc_deregister" },
	{ 0x6fe8a202, "usb_free_urb" },
	{ 0xedfbc05a, "param_ops_uint" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xb0e602eb, "memmove" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x92540fbf, "finish_wait" },
	{ 0xfa0bde56, "usb_register_driver" },
	{ 0x96848186, "scnprintf" },
	{ 0x69acdf38, "memcpy" },
	{ 0x37a0cba, "kfree" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xe2964344, "__wake_up" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x93c7edeb, "usb_find_common_endpoints" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0x1d24c881, "___ratelimit" },
	{ 0x1000e51, "schedule" },
	{ 0x808fc694, "usb_put_dev" },
	{ 0x97a84c7f, "usb_bulk_msg" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x7682ba4e, "__copy_overflow" },
	{ 0xbc4715dc, "usb_get_dev" },
	{ 0x3173741f, "usb_submit_urb" },
	{ 0xf1e7991b, "_dev_info" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x22efd784, "_dev_err" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0xfd553160, "usb_control_msg" },
	{ 0xffb7c514, "ida_free" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xf61f6a42, "usb_deregister" },
	{ 0xac8d440e, "_dev_warn" },
	{ 0xa27f77ea, "misc_register" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x1ac5d3cb, "strcspn" },
	{ 0x66b4cc41, "kmemdup" },
	{ 0x1103880, "usb_kill_urb" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x56470118, "__warn_printk" },
	{ 0xd0e4bdda, "kmalloc_trace" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0xf9a482f9, "msleep" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xe2c17b5d, "__SCT__might_resched" },
	{ 0xf116693b, "kmalloc_caches" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x907364e, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v10C4pE100d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "AB22203E09FE67295EA2A58");
