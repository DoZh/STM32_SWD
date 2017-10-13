#include <stdint.h>
#include <target_internal.h>



#ifndef __TARGET_STM32F4_H
#define __TARGET_STM32F4_H


static const char stm32f4_driver_str[] = "STM32F4xx";
static const char stm32f7_driver_str[] = "STM32F7xx";
static const char stm32f2_driver_str[] = "STM32F2xx";

/* Flash Program ad Erase Controller Register Map */
#define FPEC_BASE	0x40023C00
#define FLASH_ACR	(FPEC_BASE+0x00)
#define FLASH_KEYR	(FPEC_BASE+0x04)
#define FLASH_OPTKEYR	(FPEC_BASE+0x08)
#define FLASH_SR	(FPEC_BASE+0x0C)
#define FLASH_CR	(FPEC_BASE+0x10)
#define FLASH_OPTCR	(FPEC_BASE+0x14)

#define FLASH_CR_PG		(1 << 0)
#define FLASH_CR_SER		(1 << 1)
#define FLASH_CR_MER		(1 << 2)
#define FLASH_CR_PSIZE8		(0 << 8)
#define FLASH_CR_PSIZE16	(1 << 8)
#define FLASH_CR_PSIZE32	(2 << 8)
#define FLASH_CR_PSIZE64	(3 << 8)
#define FLASH_CR_STRT		(1 << 16)
#define FLASH_CR_EOPIE		(1 << 24)
#define FLASH_CR_ERRIE		(1 << 25)
#define FLASH_CR_STRT		(1 << 16)
#define FLASH_CR_LOCK		(1 << 31)

#define FLASH_SR_BSY		(1 << 16)

#define FLASH_OPTCR_OPTLOCK	(1 << 0)
#define FLASH_OPTCR_OPTSTRT	(1 << 1)

#define KEY1 0x45670123
#define KEY2 0xCDEF89AB

#define OPTKEY1 0x08192A3B
#define OPTKEY2 0x4C5D6E7F

#define SR_ERROR_MASK	0xF2
#define SR_EOP		0x01

#define DBGMCU_IDCODE	0xE0042000
#define ARM_CPUID	0xE000ED00

#define DBGMCU_CR		0xE0042004
#define DBG_STANDBY		(1 << 0)
#define DBG_STOP		(1 << 1)
#define DBG_SLEEP		(1 << 2)

#define DBGMCU_APB1_FZ	0xE0042008
#define DBG_WWDG_STOP	(1 << 11)
#define DBG_IWDG_STOP	(1 << 12)

/* This routine uses word access.  Only usable on target voltage >2.7V */
static const uint16_t stm32f4_flash_write_x32_stub[] = {
#include "flashstub/stm32f4_x32.stub"
};

/* This routine uses byte access. Usable on target voltage <2.2V */
static const uint16_t stm32f4_flash_write_x8_stub[] = {
#include "flashstub/stm32f4_x8.stub"
};

#define SRAM_BASE 0x20000000
#define STUB_BUFFER_BASE \
	ALIGN(SRAM_BASE + MAX(sizeof(stm32f4_flash_write_x8_stub), \
			      sizeof(stm32f4_flash_write_x32_stub)), 4)

#define AXIM_BASE 0x8000000
#define ITCM_BASE 0x0200000

struct stm32f4_flash {
	struct target_flash f;
	uint8_t base_sector;
	uint8_t psize;
};

enum ID_STM32F47 {
	ID_STM32F20X  = 0x411,
	ID_STM32F40X  = 0x413,
	ID_STM32F42X  = 0x419,
	ID_STM32F446  = 0x421,
	ID_STM32F401C = 0x423,
	ID_STM32F411  = 0x431,
	ID_STM32F401E = 0x433,
	ID_STM32F46X  = 0x434,
	ID_STM32F412  = 0x441,
	ID_STM32F74X  = 0x449,
	ID_STM32F76X  = 0x451,
	ID_STM32F72X  = 0x452,
	ID_STM32F410  = 0x458,
	ID_STM32F413  = 0x463
};

bool stm32f4_cmd_erase_mass(target *t);
bool stm32f4_cmd_option(target *t, int argc, char *argv[]);
bool stm32f4_cmd_psize(target *t, int argc, char *argv[]);

const struct command_s stm32f4_cmd_list[] = {
	{"erase_mass", (cmd_handler)stm32f4_cmd_erase_mass, "Erase entire flash memory"},
	{"option", (cmd_handler)stm32f4_cmd_option, "Manipulate option bytes"},
	{"psize", (cmd_handler)stm32f4_cmd_psize, "Configure flash write parallelism: (x8|x32)"},
	{NULL, NULL, NULL}
};

int stm32f4_flash_erase(struct target_flash *f, target_addr addr, size_t len);
int stm32f4_flash_write(struct target_flash *f,
                               target_addr dest, const void *src, size_t len);
void stm32f4_add_flash(target *t,
                              uint32_t addr, size_t length, size_t blocksize,
                              uint8_t base_sector);
bool stm32f4_probe(target *t);
void stm32f4_flash_unlock(target *t);
#endif
