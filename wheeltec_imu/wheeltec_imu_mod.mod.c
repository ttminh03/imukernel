#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

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

KSYMTAB_DATA(cp_crc8_table, "", "");
KSYMTAB_DATA(cp_crc16_table, "", "");

SYMBOL_CRC(cp_crc8_table, 0x1690648c, "");
SYMBOL_CRC(cp_crc16_table, 0x85a93ed1, "");

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x0dba6eb8, "usb_alloc_urb" },
	{ 0xd98156e2, "ida_alloc_range" },
	{ 0xa61fd7aa, "__check_object_size" },
	{ 0xf5270aff, "misc_deregister" },
	{ 0xd6d46b9d, "usb_free_urb" },
	{ 0xc2614bbe, "param_ops_uint" },
	{ 0x092a35a2, "_copy_from_user" },
	{ 0xd710adbf, "__kmalloc_noprof" },
	{ 0xa53f4e29, "memmove" },
	{ 0x40a621c5, "snprintf" },
	{ 0x49733ad6, "queue_work_on" },
	{ 0xc87f4bab, "finish_wait" },
	{ 0xaba46e12, "usb_register_driver" },
	{ 0x40a621c5, "scnprintf" },
	{ 0xa53f4e29, "memcpy" },
	{ 0xcb8b6ec6, "kfree" },
	{ 0x0db8d68d, "prepare_to_wait_event" },
	{ 0x16ab4215, "__wake_up" },
	{ 0xd272d446, "__fentry__" },
	{ 0x2bd64ad0, "usb_find_common_endpoints" },
	{ 0x5a844b26, "__x86_indirect_thunk_rax" },
	{ 0xe37614eb, "___ratelimit" },
	{ 0xd272d446, "schedule" },
	{ 0x5d5b6583, "usb_put_dev" },
	{ 0x76a26ca1, "usb_bulk_msg" },
	{ 0xd272d446, "__stack_chk_fail" },
	{ 0xf64ac983, "__copy_overflow" },
	{ 0x9479a1e8, "strnlen" },
	{ 0xa9289d30, "usb_get_dev" },
	{ 0x0819dba7, "usb_submit_urb" },
	{ 0x9878df8a, "_dev_info" },
	{ 0x90a48d82, "__ubsan_handle_out_of_bounds" },
	{ 0x7a5ffe84, "init_wait_entry" },
	{ 0x9878df8a, "_dev_err" },
	{ 0xbd03ed67, "random_kmalloc_seed" },
	{ 0xf46d5bf3, "mutex_lock" },
	{ 0x837202b8, "usb_control_msg" },
	{ 0x4c3d335e, "ida_free" },
	{ 0x173ec8da, "sscanf" },
	{ 0xc1e6c71e, "__mutex_init" },
	{ 0xef4e4365, "usb_deregister" },
	{ 0xe54e0a6b, "__fortify_panic" },
	{ 0x9878df8a, "_dev_warn" },
	{ 0xd33d3223, "misc_register" },
	{ 0xd272d446, "__x86_return_thunk" },
	{ 0x386e4ba3, "kmemdup_noprof" },
	{ 0x092a35a2, "_copy_to_user" },
	{ 0x5403c125, "__init_waitqueue_head" },
	{ 0x6514c3b7, "strcspn" },
	{ 0xf46d5bf3, "mutex_unlock" },
	{ 0x70db3fe4, "__kmalloc_cache_noprof" },
	{ 0xd6d46b9d, "usb_kill_urb" },
	{ 0x2d88a3ab, "cancel_work_sync" },
	{ 0x75738bed, "__warn_printk" },
	{ 0xe4de56b4, "__ubsan_handle_load_invalid_value" },
	{ 0x67628f51, "msleep" },
	{ 0x12ca6142, "ktime_get_with_offset" },
	{ 0x7851be11, "__SCT__might_resched" },
	{ 0xfed1e3bc, "kmalloc_caches" },
	{ 0xaef1f20d, "system_wq" },
	{ 0xba157484, "module_layout" },
};

static const u32 ____version_ext_crcs[]
__used __section("__version_ext_crcs") = {
	0x0dba6eb8,
	0xd98156e2,
	0xa61fd7aa,
	0xf5270aff,
	0xd6d46b9d,
	0xc2614bbe,
	0x092a35a2,
	0xd710adbf,
	0xa53f4e29,
	0x40a621c5,
	0x49733ad6,
	0xc87f4bab,
	0xaba46e12,
	0x40a621c5,
	0xa53f4e29,
	0xcb8b6ec6,
	0x0db8d68d,
	0x16ab4215,
	0xd272d446,
	0x2bd64ad0,
	0x5a844b26,
	0xe37614eb,
	0xd272d446,
	0x5d5b6583,
	0x76a26ca1,
	0xd272d446,
	0xf64ac983,
	0x9479a1e8,
	0xa9289d30,
	0x0819dba7,
	0x9878df8a,
	0x90a48d82,
	0x7a5ffe84,
	0x9878df8a,
	0xbd03ed67,
	0xf46d5bf3,
	0x837202b8,
	0x4c3d335e,
	0x173ec8da,
	0xc1e6c71e,
	0xef4e4365,
	0xe54e0a6b,
	0x9878df8a,
	0xd33d3223,
	0xd272d446,
	0x386e4ba3,
	0x092a35a2,
	0x5403c125,
	0x6514c3b7,
	0xf46d5bf3,
	0x70db3fe4,
	0xd6d46b9d,
	0x2d88a3ab,
	0x75738bed,
	0xe4de56b4,
	0x67628f51,
	0x12ca6142,
	0x7851be11,
	0xfed1e3bc,
	0xaef1f20d,
	0xba157484,
};
static const char ____version_ext_names[]
__used __section("__version_ext_names") =
	"usb_alloc_urb\0"
	"ida_alloc_range\0"
	"__check_object_size\0"
	"misc_deregister\0"
	"usb_free_urb\0"
	"param_ops_uint\0"
	"_copy_from_user\0"
	"__kmalloc_noprof\0"
	"memmove\0"
	"snprintf\0"
	"queue_work_on\0"
	"finish_wait\0"
	"usb_register_driver\0"
	"scnprintf\0"
	"memcpy\0"
	"kfree\0"
	"prepare_to_wait_event\0"
	"__wake_up\0"
	"__fentry__\0"
	"usb_find_common_endpoints\0"
	"__x86_indirect_thunk_rax\0"
	"___ratelimit\0"
	"schedule\0"
	"usb_put_dev\0"
	"usb_bulk_msg\0"
	"__stack_chk_fail\0"
	"__copy_overflow\0"
	"strnlen\0"
	"usb_get_dev\0"
	"usb_submit_urb\0"
	"_dev_info\0"
	"__ubsan_handle_out_of_bounds\0"
	"init_wait_entry\0"
	"_dev_err\0"
	"random_kmalloc_seed\0"
	"mutex_lock\0"
	"usb_control_msg\0"
	"ida_free\0"
	"sscanf\0"
	"__mutex_init\0"
	"usb_deregister\0"
	"__fortify_panic\0"
	"_dev_warn\0"
	"misc_register\0"
	"__x86_return_thunk\0"
	"kmemdup_noprof\0"
	"_copy_to_user\0"
	"__init_waitqueue_head\0"
	"strcspn\0"
	"mutex_unlock\0"
	"__kmalloc_cache_noprof\0"
	"usb_kill_urb\0"
	"cancel_work_sync\0"
	"__warn_printk\0"
	"__ubsan_handle_load_invalid_value\0"
	"msleep\0"
	"ktime_get_with_offset\0"
	"__SCT__might_resched\0"
	"kmalloc_caches\0"
	"system_wq\0"
	"module_layout\0"
;

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v10C4pE100d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "36ACD0B7F78A867F2B5F497");
