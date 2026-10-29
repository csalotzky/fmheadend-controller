/* Run production main/boot with a register-level QN model and captured UART. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#define main firmware_main
#include "../fmheadend_rp2354.c"
#undef main

static bool output[32], latch[32];
static uint8_t addresses[QN_COUNT];
static uint8_t vga[QN_COUNT];
static bool vga_mismatch;
static unsigned operation, enabled_index, idle_ticks;
static unsigned attempts[QN_COUNT], reset_waits[QN_COUNT], startup_delays;
static unsigned power_on_cen_edges, power_on_cen_waits;
static unsigned failed_attempts;
static uint64_t enabled_at[QN_COUNT], reset_at[QN_COUNT];
static qn_bus_status_t injected_status;
static bool address_write_takes_effect, second_cold_failure, cid2_changed;
static int bad_index, fail_operation, invalid_cid_operation;
static bool tx_stuck, all_missing, initialized, led_values[5];
static char log_text[16384];
static jmp_buf idle_done;
static uint64_t now_us;
static bool host_started, debug_pulls_disabled;
void gpio_disable_pulls(unsigned pin) { assert(pin==28); debug_pulls_disabled=true; }
bool spi_audio_init(void) { return true; }
void flash_update_platform_init(bool ready) { assert(ready && initialized && !host_started); }
bool audio_engine_init(void)
{
    assert(!host_started);
    assert(initialized);
    for (unsigned i=0;i<QN_COUNT;++i)
        assert(!qn_boot_results[i].ready || addresses[i]==QN_FIRST_ADDR+i);
    return true;
}
audio_state_t audio_engine_state(void) { return AUDIO_RUNNING; }
void audio_engine_log_fault(void) { assert(false); }
const char *audio_engine_state_name(audio_state_t s) { assert(s==AUDIO_RUNNING); return "RUNNING"; }

void gpio_init(unsigned pin) { output[pin] = latch[pin] = false; }
void gpio_set_dir(unsigned pin, bool value) { output[pin] = value; }
void gpio_put(unsigned pin, bool value)
{
    if (pin == DIAGNOSTIC_LED_PIN) {
        if (host_started && ++idle_ticks == 5) longjmp(idle_done, 1);
        assert(idle_ticks < 5);
        led_values[idle_ticks] = value;
        return;
    }
    assert(pin >= QN_CEN_FIRST_PIN && pin < QN_CEN_FIRST_PIN + QN_COUNT);
    const unsigned index = pin - QN_CEN_FIRST_PIN;
    if (initialized && power_on_cen_waits < 2) {
        assert(startup_delays == 1 && !host_started);
        assert(power_on_cen_edges < 2 * QN_COUNT);
        assert(index == power_on_cen_edges % QN_COUNT);
        assert(value == (power_on_cen_edges < QN_COUNT));
        assert(power_on_cen_waits == (value ? 0 : 1));
        assert(now_us == (value ? 100000 : 200000));
        assert(output[pin] && !attempts[index]);
        latch[pin] = value;
        ++power_on_cen_edges;
        return;
    }
    if (initialized) {
        assert(index >= enabled_index); /* Never revisit earlier devices. */
        assert(!qn_boot_results[index].ready); /* Never reset a ready device. */
        for (unsigned i = 0; i < index; ++i) {
            assert(qn_boot_results[i].attempts != 0);
            assert(latch[QN_CEN_FIRST_PIN + i] == qn_boot_results[i].ready);
            if (qn_boot_results[i].ready) {
                assert(addresses[i] == QN_FIRST_ADDR + i && vga[i] == QN_REG_VGA_DEFAULT);
            }
        }
    }
    if (value) {
        assert(initialized && output[pin]);
        assert(startup_delays == 1 && power_on_cen_waits == 2 && now_us >= 300000);
        if (attempts[index]) {
            assert(!latch[pin] && now_us - reset_at[index] == 250000);
            assert(reset_waits[index] == attempts[index]);
        }
        enabled_index = index;
        enabled_at[index] = now_us;
        ++attempts[index];
        assert(attempts[index] <= 3);
        operation = 0;
        /* Previously assigned QNs may be on; no other default address may be. */
        for (unsigned i = 0; i < QN_COUNT; ++i) {
            assert(!latch[18 + i] || addresses[i] != QN_DEFAULT_ADDR);
        }
        addresses[enabled_index] = QN_DEFAULT_ADDR;
        vga[enabled_index] = 0x60;
    } else if (initialized) {
        assert(latch[pin]); /* Exactly one isolation transition per failed attempt. */
        reset_at[index] = now_us;
    }
    latch[pin] = value;
}
void gpio_set_function(unsigned pin, unsigned function)
{
    assert(DEBUG_UART_ENABLED);
    assert((pin == 28 || pin == 29) && function == GPIO_FUNC_UART);
}
void sleep_ms(uint32_t ms)
{
    if (ms == 100 && startup_delays == 0) {
        assert(initialized && !host_started && ++startup_delays == 1);
        for (unsigned i = 0; i < QN_COUNT; ++i)
            assert(!latch[QN_CEN_FIRST_PIN + i] && !attempts[i]);
    } else if (ms == 100 && power_on_cen_waits < 2) {
        assert(power_on_cen_edges == (power_on_cen_waits + 1) * QN_COUNT);
        assert(now_us == (power_on_cen_waits + 1) * 100000);
        for (unsigned i = 0; i < QN_COUNT; ++i)
            assert(output[QN_CEN_FIRST_PIN + i] &&
                   latch[QN_CEN_FIRST_PIN + i] == (power_on_cen_waits == 0));
        ++power_on_cen_waits;
    } else if (ms == 250) {
        assert(!host_started && !latch[QN_CEN_FIRST_PIN + enabled_index]);
        assert(now_us == reset_at[enabled_index] && attempts[enabled_index] < 3);
        ++reset_waits[enabled_index];
    } else if (ms == 100) {
        assert(latch[QN_CEN_FIRST_PIN + enabled_index] && !operation);
        assert(now_us == enabled_at[enabled_index]);
    } else if (ms == 10) {
        assert(latch[QN_CEN_FIRST_PIN + enabled_index] && operation == 3);
    } else assert(ms == 1 && host_started);
    now_us += (uint64_t)ms * 1000;
}
uint64_t time_us_64(void) { return now_us; }
void host_i2c_init(uint8_t ready_mask)
{
    for (unsigned i = 0; i < QN_COUNT; ++i) {
        assert(!!(ready_mask & (1u << i)) == qn_boot_results[i].ready);
    }
    host_started = true;
}
void host_i2c_poll(void) { assert(host_started); }
bool host_i2c_led(uint64_t now) { (void)now; return boot_hardware_fault && idle_ticks % 2 == 0; }
void uart_init(unsigned uart, unsigned baud)
{
    assert(DEBUG_UART_ENABLED);
    assert(uart == uart0 && baud == 115200);
    for (unsigned i = 0; i < QN_COUNT; ++i)
        assert(output[QN_CEN_FIRST_PIN + i] && !latch[QN_CEN_FIRST_PIN + i]);
}
void uart_set_hw_flow(unsigned uart, bool cts, bool rts)
{
    assert(DEBUG_UART_ENABLED);
    assert(uart == uart0 && !cts && !rts);
}
void uart_set_format(unsigned uart, unsigned bits, unsigned stop, unsigned parity)
{
    assert(DEBUG_UART_ENABLED);
    assert(uart == uart0 && bits == 8 && stop == 1 && parity == UART_PARITY_NONE);
}
void uart_puts(unsigned uart, const char *text)
{
    assert(DEBUG_UART_ENABLED);
    assert(uart == uart0 && strlen(log_text) + strlen(text) < sizeof(log_text));
    strcat(log_text, text);
}
void qn_bus_init(void) { initialized = true; }
const char *qn_bus_status_name(qn_bus_status_t status)
{
    assert(status > QN_BUS_OK && status <= QN_BUS_INVALID_ARGUMENT);
    static const char *names[] = {"OK", "NACK", "SCL timeout", "SDA stuck low",
                                  "transaction timeout", "invalid argument"};
    return names[status];
}

static bool failure_active(void)
{
    return (int)enabled_index == bad_index && attempts[enabled_index] <= failed_attempts;
}

static qn_bus_status_t check_operation(uint8_t address, uint8_t reg, bool write)
{
    static const uint8_t registers[] = {6, 0, 2, 6, 0, 0, 4, 4};
    static const bool writes[] = {false, true, true, false, true, false, true, false};
    assert(power_on_cen_waits == 2); /* No I2C during the startup pulse. */
    assert(operation < 8 && reg == registers[operation] && write == writes[operation]);
    assert(address == addresses[enabled_index]);
    assert(latch[QN_CEN_FIRST_PIN + enabled_index] && !host_started);
    assert(now_us >= enabled_at[enabled_index] + 100000); /* Per-attempt CEN settle. */
    bool previous_fault = false;
    for (unsigned i = 0; i < enabled_index; ++i)
        previous_fault |= !qn_boot_results[i].ready;
    assert(boot_hardware_fault == previous_fault); /* No premature permanent fault. */
    const bool fail = all_missing || (failure_active() && (int)operation == fail_operation) ||
                     (second_cold_failure && enabled_index == 1 &&
                      attempts[enabled_index] == 1 && operation == 0);
    ++operation;
    return fail ? injected_status : QN_BUS_OK;
}

qn_bus_status_t qn_bus_read_reg(uint8_t address, uint8_t reg, uint8_t *value)
{
    const qn_bus_status_t status = check_operation(address, reg, false);
    if (status != QN_BUS_OK) return status;
    if (reg == 6) {
        *value = (failure_active() &&
                  (int)(operation - 1) == invalid_cid_operation) ?
                  (cid2_changed ? 0x12 : 0xff) : 0x11;
    } else if (reg == 4) {
        *value = vga[enabled_index] ^ (vga_mismatch && failure_active() ? 1 : 0);
    } else {
        *value = tx_stuck && failure_active() ? 0x45 : 0x05;
    }
    return QN_BUS_OK;
}

qn_bus_status_t qn_bus_write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    const qn_bus_status_t status = check_operation(address, reg, true);
    if (reg == 2) {
        assert(value == QN_FIRST_ADDR + enabled_index);
        /* Deliberately model a write that takes effect even when ACK is lost. */
        if (status == QN_BUS_OK || address_write_takes_effect) addresses[enabled_index] = value;
    } else if (reg == 4) {
        assert(value == 0x45 && value == QN_REG_VGA_DEFAULT);
        vga[enabled_index] = value;
    } else {
        assert(reg == 0 && value == 0x05); /* No RF-enable/SWRST writes. */
    }
    return status;
}

static void run(int bad, int fail, int invalid_cid, bool stuck, bool missing,
                unsigned failures)
{
    memset(output, 0, sizeof(output));
    memset(latch, 0, sizeof(latch));
    memset(qn_boot_results, 0, sizeof(qn_boot_results));
    memset(led_values, 0, sizeof(led_values));
    memset(attempts, 0, sizeof(attempts));
    memset(reset_waits, 0, sizeof(reset_waits));
    log_text[0] = '\0';
    boot_hardware_fault = initialized = false;
    idle_ticks = 0;
    enabled_index = startup_delays = 0;
    power_on_cen_edges = power_on_cen_waits = 0;
    failed_attempts = failures;
    now_us = 0;
    host_started = false;
    debug_pulls_disabled = false;
    bad_index = bad;
    fail_operation = fail;
    invalid_cid_operation = invalid_cid;
    tx_stuck = stuck;
    all_missing = missing;
    if (setjmp(idle_done) == 0) firmware_main();

    const bool fault = (bad >= 0 && failures >= 3) || missing;
    assert(boot_hardware_fault == fault && !led_values[0]);
    for (unsigned tick = 1; tick < 5; ++tick) {
        assert(led_values[tick] == (fault && tick % 2 == 1));
    }
    for (unsigned i = 0; i < QN_COUNT; ++i) {
        const bool ready = !missing && !((int)i == bad && failures >= 3);
        const unsigned expected_attempts = missing ? 3 : (int)i == bad ?
            (failures >= 3 ? 3 : failures + 1) : second_cold_failure && i == 1 ? 2 : 1;
        assert(attempts[i] == expected_attempts);
        assert(qn_boot_results[i].attempts == expected_attempts);
        assert(reset_waits[i] == expected_attempts - 1);
        assert(qn_boot_results[i].ready == ready && latch[18 + i] == ready);
        assert(qn_boot_results[i].bus_status == (ready ? QN_BUS_OK :
               missing || fail >= 0 ? injected_status : QN_BUS_OK));
        char expected[160];
        if (ready) {
            assert(addresses[i] == 0x21 + i);
            assert(vga[i] == 0x45);
            assert(qn_boot_results[i].cid2_read && qn_boot_results[i].cid2 == 0x11);
            assert(strcmp(qn_boot_results[i].stage, "ready") == 0);
            snprintf(expected, sizeof(expected), "QN%u @ 0x%02X: CID2=0x11 OK, RF OFF", i, 0x21 + i);
        } else snprintf(expected, sizeof(expected), "QN%u target 0x%02X: FAIL", i, 0x21 + i);
        if (DEBUG_UART_ENABLED) assert(strstr(log_text, expected));
        if (DEBUG_UART_ENABLED) {
            const unsigned failed = expected_attempts - (ready ? 1 : 0);
            for (unsigned attempt = 1; attempt <= failed; ++attempt) {
                snprintf(expected, sizeof(expected), "QN%u target 0x%02X: FAIL attempt %u/3 ",
                         i, 0x21 + i, attempt);
                assert(strstr(log_text, expected));
            }
            snprintf(expected, sizeof(expected), "QN%u target 0x%02X: FAIL attempt %u/3 ",
                     i, 0x21 + i, failed + 1);
            assert(!strstr(log_text, expected));
        }
    }
    assert(startup_delays == 1 && power_on_cen_edges == 2 * QN_COUNT && power_on_cen_waits == 2);
    if (DEBUG_UART_ENABLED) {
        assert(strstr(log_text, FMHEADEND_FIRMWARE_VERSION));
        char expected[96];
        snprintf(expected, sizeof(expected), "BOOT: %u/%u ready; RF OFF; %s",
                 missing ? 0 : fault ? QN_COUNT - 1 : QN_COUNT, QN_COUNT,
                 fault ? "HARDWARE FAULT" : "IDLE");
        assert(strstr(log_text, expected));
        assert(!!strstr(log_text, "exhausted") == fault);
        if (bad >= 0 && fail >= 0) {
            static const char *stages[] = {"default CID2 read", "default RF OFF write",
                "address assignment", "assigned CID2 read", "assigned RF OFF write",
                "RF OFF readback", "crystal load write", "crystal load readback"};
            snprintf(expected, sizeof(expected), "%s (%s, bus=%u)", stages[fail],
                     qn_bus_status_name(injected_status), (unsigned)injected_status);
            assert(strstr(log_text, expected));
        }
    } else {
        assert(!log_text[0] && !output[28] && debug_pulls_disabled);
    }
}

int main(void)
{
    injected_status = QN_BUS_NACK;
    run(-1, -1, -1, false, false, 0);
    for (int index = 0; index < QN_COUNT; ++index) {
        for (unsigned failures = 1; failures <= 3; ++failures) {
            for (int status = QN_BUS_NACK; status <= QN_BUS_INVALID_ARGUMENT; ++status) {
                injected_status = (qn_bus_status_t)status;
                for (int op = 0; op < 8; ++op) {
                    for (unsigned effect = 0; effect < (op == 2 ? 2u : 1u); ++effect) {
                        address_write_takes_effect = effect;
                        run(index, op, -1, false, false, failures);
                    }
                }
            }
            run(index, -1, 0, false, false, failures);
            run(index, -1, 3, false, false, failures);
            cid2_changed = true;
            run(index, -1, 3, false, false, failures);
            cid2_changed = false;
            run(index, -1, -1, true, false, failures);
            vga_mismatch = true;
            run(index, -1, -1, false, false, failures);
            if (DEBUG_UART_ENABLED) assert(strstr(log_text, "crystal load verification"));
            vga_mismatch = false;
        }
    }
    injected_status = QN_BUS_NACK;
    run(-1, -1, -1, false, true, 3);
    /* Reproduce the captured cold boot: TX0 and TX1 initially hold SDA low. */
    injected_status = QN_BUS_SDA_STUCK;
    second_cold_failure = QN_COUNT > 1;
    run(0, 0, -1, false, false, 1);
    const qn_boot_result_t invalid = qn_boot_device(QN_COUNT);
    assert(!invalid.ready && !invalid.attempts && invalid.bus_status == QN_BUS_INVALID_ARGUMENT);
    puts("Safe boot: retries, timing, CEN isolation, RF OFF, UART and LED PASS");
}
