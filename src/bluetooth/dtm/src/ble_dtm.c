/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "dtm.h"

#define STACK_SIZE 1024
#define THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(thread_stack, STACK_SIZE);
struct k_thread thread_data;
LOG_MODULE_REGISTER(MODULE, LOG_LEVEL_INF); // NOLINT

void dtm_thread_function(void *arg)
{
    const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
    bool is_msb_read = false;
    uint8_t rx_byte;
    uint16_t dtm_cmd;
    uint16_t dtm_evt;
    int64_t msb_time;
    int err;

    LOG_INF("Starting Direct Test Mode example\n");

    if (!device_is_ready(uart)) {
        printk("UART device not ready\n");
        return;
    }

    err = dtm_init();
    if (err) {
        LOG_INF("Error during DTM initialization: %d\n", err);
        return;
    }

    for (;;) {
        dtm_wait();

        err = uart_poll_in(uart, &rx_byte);
        if (err) {
            if (err != -1) {
                LOG_INF("UART polling error: %d\n", err);
            }
            /* Nothing read from the UART. */
            continue;
        }

        if (!is_msb_read) {
            /* This is the first byte of the two-byte command. */
            is_msb_read = true;
            dtm_cmd = rx_byte << 8;
            msb_time = k_uptime_get();
            /* Go back and wait for the second byte of the command word. */
            continue;
        }

        /* This is the second byte read; combine it with the first and
         * process the command.
         */
        if ((k_uptime_get() - msb_time) > DTM_UART_SECOND_BYTE_MAX_DELAY) {
            /* More than ~5mS after MSB: Drop old byte, take the
             * new byte as MSB. The variable is_msb_read will
             * remain true.
             */
            dtm_cmd = rx_byte << 8;
            msb_time = k_uptime_get();
            /* Go back and wait for the second byte of the command word. */
            continue;
        }

        /* Two-byte UART command received. */
        is_msb_read = false;
        dtm_cmd |= rx_byte;

        LOG_INF("Sending 0x%04X DTM command\n", dtm_cmd);

        if (dtm_cmd_put(dtm_cmd) != DTM_SUCCESS) {
            /* Extended error handling may be put here.
             * Default behavior is to return the event on the UART;
             * the event report will reflect any lack of success.
             */
        }

        /* Retrieve the result of the operation. This implementation will
         * busy-loop for the duration of the byte transmissions on the
         * UART.
         */
        if (dtm_event_get(&dtm_evt)) {
            LOG_INF("Received 0x%04X DTM event\n", dtm_evt);

            /* Report command status on the UART. */

            /* Transmit MSB of the result. */
            uart_poll_out(uart, (dtm_evt >> 8) & 0xFF);

            /* Transmit LSB of the result. */
            uart_poll_out(uart, dtm_evt & 0xFF);
        }
    }
}

void dtm_thread_init(void)
{
    k_tid_t dtm_thread_id;

    dtm_thread_id = k_thread_create(&thread_data, thread_stack,
                                    K_THREAD_STACK_SIZEOF(thread_stack),
                                    dtm_thread_function, NULL, NULL, NULL,
                                    THREAD_PRIORITY, 0, K_NO_WAIT);

    if (dtm_thread_id == NULL) {
        LOG_INF("Failed to create DTM thread\n");
        return;
    }
    LOG_INF("Direct Test Mode Initialised");

}

static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        const struct module_state_event *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(main), MODULE_STATE_READY))
        {
            dtm_thread_init();
        }
        return false;
    }
}
