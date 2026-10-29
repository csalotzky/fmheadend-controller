#ifndef FMHEADEND_QN_BOOT_H
#define FMHEADEND_QN_BOOT_H

#include <stdbool.h>
#include "qn_bus.h"

#define QN_DEFAULT_ADDR 0x2Au
#define QN_FIRST_ADDR 0x21u
#define QN_BOOT_MAX_ATTEMPTS 3u

typedef struct {
    uint8_t address; /* Assigned only when ready; otherwise intended address. */
    uint8_t cid2;
    bool cid2_read;
    bool ready;
    uint8_t attempts; /* Boot caller accumulates attempts, including failures. */
    const char *stage;
    qn_bus_status_t bus_status;
} qn_boot_result_t;

/* Configure all populated CEN outputs LOW before bus initialization. */
void qn_boot_hold_all_reset(void);
/* One complete attempt, in population index order. Failure lowers only this
 * QN's CEN; caller must hold it low for 250 ms before retrying this index.
 */
qn_boot_result_t qn_boot_device(unsigned index);

#endif
