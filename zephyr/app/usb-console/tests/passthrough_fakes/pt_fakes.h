/* SPDX-License-Identifier: MIT */
/* Fakes of the Zephyr calls of common/drivers/nina/zephyr/
 * nina_passthrough_os.c and common/shell/nina_shell.c for the host test
 * tests/passthrough.c, which defines the functions (and the fake of
 * nina_passthrough_bus.h).  Every <zephyr/...> header of the files is this
 * one.
 */
#ifndef PT_FAKES_H_
#define PT_FAKES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define ARG_UNUSED(x)           (void)(x)
#define BUILD_ASSERT(cond, msg) _Static_assert(cond, msg)
#define CONFIG_SYSTEM_WORKQUEUE_PRIORITY -1

/* Devices and the devicetree. */
struct device {
	int unused;
};
extern const struct device fake_cdc_dev;
#define DT_NODELABEL(label) 0
#define DEVICE_DT_GET(node) (&fake_cdc_dev)

/* SYS_INIT(): the test calls the functions itself. */
#define SYS_INIT(fn, level, prio)                                                                  \
	static int (*const sys_init_##fn)(void) __attribute__((unused)) = fn

/* The kernel. */
typedef struct {
	int64_t ms; /* -1: K_FOREVER */
} k_timeout_t;
#define K_MSEC(ms) ((k_timeout_t){(ms)})
#define K_NO_WAIT  ((k_timeout_t){0})
#define K_FOREVER  ((k_timeout_t){-1})

struct k_sem {
	unsigned int count;
	unsigned int limit;
};
#define K_SEM_DEFINE(name, initial, max) struct k_sem name = {(initial), (max)}
void k_sem_give(struct k_sem *sem);
int k_sem_take(struct k_sem *sem, k_timeout_t timeout);

struct k_work;
typedef void (*k_work_handler_t)(struct k_work *work);
struct k_work {
	k_work_handler_t handler;
};
struct k_work_delayable {
	struct k_work work;
};
struct k_work_sync {
	int unused;
};
#define K_WORK_DELAYABLE_DEFINE(name, fn) struct k_work_delayable name = {.work = {.handler = (fn)}}
int k_work_schedule(struct k_work_delayable *dwork, k_timeout_t delay);
bool k_work_cancel_delayable_sync(struct k_work_delayable *dwork, struct k_work_sync *sync);

uint32_t k_uptime_get_32(void);
int32_t k_msleep(int32_t ms);

typedef long atomic_t;
typedef long atomic_val_t;
#define ATOMIC_INIT(i) (i)
atomic_val_t atomic_set(atomic_t *target, atomic_val_t value);
bool atomic_cas(atomic_t *target, atomic_val_t old_value, atomic_val_t new_value);

/* Logging: the last line, and how many. */
extern char fake_log[512];
extern unsigned int fake_logs;
#define LOG_MODULE_REGISTER(name, level) extern int fake_log_module_##name
#define LOG_INF(...)                     (fake_logs++, snprintf(fake_log, sizeof(fake_log), __VA_ARGS__))
#define LOG_ERR(...)                     (fake_logs++, snprintf(fake_log, sizeof(fake_log), __VA_ARGS__))

/* The shell: what the command printed, and its sets as arrays. */
struct shell {
	int unused;
};
struct shell_static_entry {
	const char *syntax;
	const char *help;
	const void *subcmd;
	int (*handler)(const struct shell *sh, size_t argc, char **argv);
	uint8_t mandatory;
	uint8_t optional;
};
extern char fake_shell_out[512];
#define shell_print(sh, ...)                                                                       \
	((void)(sh), snprintf(fake_shell_out, sizeof(fake_shell_out), __VA_ARGS__))
#define shell_error(sh, ...)                                                                       \
	((void)(sh), snprintf(fake_shell_out, sizeof(fake_shell_out), __VA_ARGS__))
#define SHELL_CMD_ARG(syn, sub, hlp, fn, mand, opt)                                                \
	{.syntax = #syn, .help = (hlp), .subcmd = (sub), .handler = (fn), .mandatory = (mand),     \
	 .optional = (opt)}
#define SHELL_CMD(syn, sub, hlp, fn) SHELL_CMD_ARG(syn, sub, hlp, fn, 0, 0)
#define SHELL_SUBCMD_SET_END         {.syntax = NULL}
#define SHELL_STATIC_SUBCMD_SET_CREATE(name, ...)                                                  \
	static const struct shell_static_entry name[] = {__VA_ARGS__}
#define SHELL_CMD_REGISTER(syn, sub, hlp, fn)                                                      \
	const struct shell_static_entry *const fake_shell_root_##syn = *(sub)

/* The UART API of the CDC ACM port. */
#define UART_LINE_CTRL_BAUD_RATE 1
#define UART_LINE_CTRL_RTS       2
#define UART_LINE_CTRL_DTR       4
typedef void (*uart_irq_callback_user_data_t)(const struct device *dev, void *user_data);
int uart_line_ctrl_get(const struct device *dev, uint32_t ctrl, uint32_t *val);
int uart_irq_callback_user_data_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				    void *user_data);
void uart_irq_rx_enable(const struct device *dev);
void uart_irq_rx_disable(const struct device *dev);
void uart_irq_tx_enable(const struct device *dev);
void uart_irq_tx_disable(const struct device *dev);
void uart_irq_update(const struct device *dev);
int uart_irq_rx_ready(const struct device *dev);
int uart_irq_tx_ready(const struct device *dev);
int uart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size);
int uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size);

/* The USB device stack's messages. */
struct usbd_context {
	int unused;
};
enum usbd_msg_type {
	USBD_MSG_VBUS_READY,
	USBD_MSG_VBUS_REMOVED,
	USBD_MSG_RESUME,
	USBD_MSG_SUSPEND,
	USBD_MSG_RESET,
	USBD_MSG_CONFIGURATION,
	USBD_MSG_UDC_ERROR,
	USBD_MSG_STACK_ERROR,
	USBD_MSG_CDC_ACM_LINE_CODING,
	USBD_MSG_CDC_ACM_CONTROL_LINE_STATE,
};
struct usbd_msg {
	enum usbd_msg_type type;
	union {
		int status;
		const struct device *dev;
	};
};
typedef void (*usbd_msg_cb_t)(struct usbd_context *const ctx, const struct usbd_msg *const msg);
int usbd_msg_register_cb(struct usbd_context *const ctx, const usbd_msg_cb_t cb);
extern struct usbd_context fake_usbd_contexts[1];
#define STRUCT_SECTION_FOREACH(type, var)                                                          \
	for (struct type *var = fake_##type##s; var < fake_##type##s + 1; var++)

#endif /* PT_FAKES_H_ */
