#ifndef __FIRM_UPDATER_H
#define __FIRM_UPDATER_H

#include <stdint.h>

typedef struct {
  uint32_t start_addr;
  uint32_t bytelen;
} update_info_t;

void fu_main(void);
#endif
