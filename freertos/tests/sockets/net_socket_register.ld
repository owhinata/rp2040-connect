/* SPDX-License-Identifier: MIT */
/* The iterable section of the layer's socket implementations for the host
 * tests (freertos/script/host_tests.sh links with -Wl,-T,<this file>): the
 * board's common/pico_sdk/drivers/compat/ld/section_extra_post_text.incl inserted
 * into the host linker's default script.
 */
SECTIONS
{
    net_socket_register_area :
    {
        _net_socket_register_list_start = .;
        KEEP(*(SORT_BY_NAME(._net_socket_register.static.*)))
        _net_socket_register_list_end = .;
    }
}
INSERT AFTER .rodata;
