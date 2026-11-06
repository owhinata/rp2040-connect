#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(usb_console, LOG_LEVEL_INF);

static int cmd_hello(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	LOG_INF("hello command received");
	shell_print(sh, "Hello from Arduino Nano RP2040 Connect!");
	return 0;
}

SHELL_CMD_REGISTER(hello, NULL, "Print a greeting", cmd_hello);

int main(void)
{
	LOG_INF("USB console started");
	return 0;
}
