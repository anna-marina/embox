/**
 * @file imx6_usdhc.c
 * @brief i.MX6 uSDHC driver for MMC card
 * @author Denis Deryugin <deryugin.denis@gmail.com>
 * @version
 * @date 01.05.2017
 */

#include <drivers/block_dev.h>
#include <drivers/common/memory.h>
#include <framework/mod/options.h>
#include <hal/reg.h>
#include <string.h>
#include <net/util/show_packet.h>
#include <util/log.h>

#define BASE_ADDR OPTION_GET(NUMBER, base_addr)

#define USDHC_DS_ADDR               (BASE_ADDR + 0x00)
#define USDHC_BLK_ATT               (BASE_ADDR + 0x04)
#define USDHC_CMD_ARG               (BASE_ADDR + 0x08)
#define USDHC_CMD_XFR_TYP           (BASE_ADDR + 0x0C)
#define USDHC_CMD_RSP0              (BASE_ADDR + 0x10)
#define USDHC_CMD_RSP1              (BASE_ADDR + 0x14)
#define USDHC_CMD_RSP2              (BASE_ADDR + 0x18)
#define USDHC_CMD_RSP3              (BASE_ADDR + 0x1C)
#define USDHC_DATA_BUFF_ACC_PORT    (BASE_ADDR + 0x20)
#define USDHC_PRES_STATE            (BASE_ADDR + 0x24)
#define USDHC_PROT_CTRL             (BASE_ADDR + 0x28)
#define USDHC_SYS_CTRL              (BASE_ADDR + 0x2C)
# define USDHC_SYS_CTRL_RSTA        (1 << 24)
# define USDHC_SYS_CTRL_RSTC        (1 << 25)
# define USDHC_SYS_CTRL_RSTD        (1 << 26)
#define USDHC_INT_STATUS            (BASE_ADDR + 0x30)
#define USDHC_INT_STATUS_EN         (BASE_ADDR + 0x34)
#define USDHC_INT_SIGNAL_EN         (BASE_ADDR + 0x38)
#define USDHC_AUTOCMD12_ERR_STATUS  (BASE_ADDR + 0x3C)
#define USDHC_HOST_CTRL_CAP         (BASE_ADDR + 0x40)
#define USDHC_WTMK_LVL              (BASE_ADDR + 0x44)
#define USDHC_MIX_CTRL              (BASE_ADDR + 0x48)
#define USDHC_FORCE_EVENT           (BASE_ADDR + 0x50)
#define USDHC_ADMA_ERR_STATUS       (BASE_ADDR + 0x54)
#define USDHC_ADMA_SYS_ADDR         (BASE_ADDR + 0x58)
#define USDHC_DLL_CTRl              (BASE_ADDR + 0x60)
#define USDHC_DLL_STATUS            (BASE_ADDR + 0x64)
#define USDHC_CLK_TUNE_CTRL_STATUS  (BASE_ADDR + 0x68)
#define USDHC_VEND_SPEC             (BASE_ADDR + 0xC0)
#define USDHC_MMC_BOOT              (BASE_ADDR + 0xC4)
#define USDHC_VEND_SPEC2            (BASE_ADDR + 0xC8)

#define BLK_LEN 512

#define TIMEOUT 0x1FFFFF

#define SDHCI_IRQ_EN_BITS (1 | 2 | 0x100 | 0x10000 | 0x20000 | 0x40000 | 0x80000 | 0x100000 | 0x400000 | 0x20 | 0x10 | 0x8)

static void imx6_usdhc_send_cmd(int cmd_idx, uint32_t arg);
static void imx6_usdhc_send_app_cmd(int cmd_idx, uint32_t arg, uint32_t rca);

static void _resp_dump(void) {
	log_debug("===================");
	log_debug("USDHC_CMD_RSP0             =0x%08x", REG32_LOAD(BASE_ADDR + 0x10));
	log_debug("USDHC_CMD_RSP1             =0x%08x", REG32_LOAD(BASE_ADDR + 0x14));
	log_debug("USDHC_CMD_RSP2             =0x%08x", REG32_LOAD(BASE_ADDR + 0x18));
	log_debug("USDHC_CMD_RSP3             =0x%08x", REG32_LOAD(BASE_ADDR + 0x1C));
	log_debug("===================");
}

static void _reg_dump(void) {
	log_debug("===================");
	log_debug("REG DUMP");
	log_debug("===================");
	log_debug("USDHC_DS_ADDR              =0x%08x", REG32_LOAD(BASE_ADDR + 0x00));
	log_debug("USDHC_BLK_ATT              =0x%08x", REG32_LOAD(BASE_ADDR + 0x04));
	log_debug("USDHC_CMD_ARG              =0x%08x", REG32_LOAD(BASE_ADDR + 0x08));
	log_debug("USDHC_CMD_XFR_TYP          =0x%08x", REG32_LOAD(BASE_ADDR + 0x0C));
	log_debug("USDHC_CMD_RSP0             =0x%08x", REG32_LOAD(BASE_ADDR + 0x10));
	log_debug("USDHC_CMD_RSP1             =0x%08x", REG32_LOAD(BASE_ADDR + 0x14));
	log_debug("USDHC_CMD_RSP2             =0x%08x", REG32_LOAD(BASE_ADDR + 0x18));
	log_debug("USDHC_CMD_RSP3             =0x%08x", REG32_LOAD(BASE_ADDR + 0x1C));
	log_debug("USDHC_DATA_BUFF_ACC_PORT   =0x%08x", REG32_LOAD(BASE_ADDR + 0x20));
	log_debug("USDHC_PRES_STATE           =0x%08x", REG32_LOAD(BASE_ADDR + 0x24));
	log_debug("USDHC_PROT_CTRL            =0x%08x", REG32_LOAD(BASE_ADDR + 0x28));
	log_debug("USDHC_SYS_CTRL             =0x%08x", REG32_LOAD(BASE_ADDR + 0x2C));
	log_debug("USDHC_INT_STATUS           =0x%08x", REG32_LOAD(BASE_ADDR + 0x30));
	log_debug("USDHC_INT_STATUS_EN        =0x%08x", REG32_LOAD(BASE_ADDR + 0x34));
	log_debug("USDHC_INT_SIGNAL_EN        =0x%08x", REG32_LOAD(BASE_ADDR + 0x38));
	log_debug("USDHC_AUTOCMD12_ERR_STATUS =0x%08x", REG32_LOAD(BASE_ADDR + 0x3C));
	log_debug("USDHC_HOST_CTRL_CAP        =0x%08x", REG32_LOAD(BASE_ADDR + 0x40));
	log_debug("USDHC_WTMK_LVL             =0x%08x", REG32_LOAD(BASE_ADDR + 0x44));
	log_debug("USDHC_MIX_CTRL             =0x%08x", REG32_LOAD(BASE_ADDR + 0x48));
	log_debug("USDHC_FORCE_EVENT          =0x%08x", REG32_LOAD(BASE_ADDR + 0x50));
	log_debug("USDHC_ADMA_ERR_STATUS      =0x%08x", REG32_LOAD(BASE_ADDR + 0x54));
	log_debug("USDHC_ADMA_SYS_ADDR        =0x%08x", REG32_LOAD(BASE_ADDR + 0x58));
	log_debug("USDHC_DLL_CTRl             =0x%08x", REG32_LOAD(BASE_ADDR + 0x60));
	log_debug("USDHC_DLL_STATUS           =0x%08x", REG32_LOAD(BASE_ADDR + 0x64));
	log_debug("USDHC_CLK_TUNE_CTRL_STATUS =0x%08x", REG32_LOAD(BASE_ADDR + 0x68));
	log_debug("USDHC_VEND_SPEC            =0x%08x", REG32_LOAD(BASE_ADDR + 0xC0));
	log_debug("USDHC_MMC_BOOT             =0x%08x", REG32_LOAD(BASE_ADDR + 0xC4));
	log_debug("USDHC_VEND_SPEC2           =0x%08x", REG32_LOAD(BASE_ADDR + 0xC8));
}

#define RSP_136         (1 << 16)
#define RSP_PRESENT     (2 << 16)
#define RSP_BUSY        (3 << 16)
#define RSP_CRC         (1 << 19)
#define RSP_OPCODE      (1 << 20)

#define RSP_NONE        0
#define RSP_R1          (RSP_PRESENT | RSP_CRC | RSP_OPCODE)
#define RSP_R1b         (RSP_CRC | RSP_OPCODE|RSP_BUSY)
#define RSP_R2          (RSP_136 | RSP_CRC)
#define RSP_R3          (RSP_PRESENT)
#define RSP_R4          (RSP_PRESENT)
#define RSP_R5          (RSP_PRESENT | RSP_CRC | RSP_OPCODE)
#define RSP_R6          (RSP_PRESENT | RSP_CRC | RSP_OPCODE)
#define RSP_R7          (RSP_PRESENT | RSP_CRC | RSP_OPCODE)

void imx6_usdhc_reset_data(void) {
	REG32_STORE(USDHC_SYS_CTRL, REG32_LOAD(USDHC_SYS_CTRL) | USDHC_SYS_CTRL_RSTD);

	int timeout = TIMEOUT;
	while(timeout--);

	if (REG32_LOAD(USDHC_SYS_CTRL) & USDHC_SYS_CTRL_RSTD) {
		log_error("Reset timeout");
	}
}

void imx6_usdhc_reset_cmd(void) {
	REG32_STORE(USDHC_SYS_CTRL, REG32_LOAD(USDHC_SYS_CTRL) | USDHC_SYS_CTRL_RSTC);

	int timeout = TIMEOUT;
	while(timeout--);

	if (REG32_LOAD(USDHC_SYS_CTRL) & USDHC_SYS_CTRL_RSTC) {
		log_error("Reset timeout");
	}
}

static void imx6_usdhc_send_cmd(int cmd_idx, uint32_t arg) {
	uint32_t wcmd = 0;
	uint32_t mix = 0;
	int t = TIMEOUT;

	wcmd = ((cmd_idx & 0x3f) << 24);// | (1 << 20) | (1 << 19);

	switch (cmd_idx) {
	case 41:
		wcmd |= RSP_R3;
		break;
	case 17:
		wcmd |= (1 << 20) | (1 << 21) | (1 << 19) | (3 << 16) | 0x10 | 1;
		mix |= 1 | (1 << 4);
		break;
	case 9:
		wcmd |= RSP_R2;
		break;
	case 55:
	case 8:
	case 6:
		wcmd |= RSP_R1;
		break;
	case 7:
		wcmd |= RSP_R1b;
		break;
	case 3:
		wcmd |= RSP_R6;
		break;
	case 13:
	case 2:
		wcmd |= RSP_R2;
		break;
	case 1:
		wcmd |= RSP_R3;
		break;
	case 0:
		arg = 0;
		wcmd = 0;
		break;
	default:
		break;
	}

	REG32_STORE(USDHC_CMD_ARG, arg);
	REG32_STORE(USDHC_MIX_CTRL, (1 << 24) | mix | ((REG32_LOAD(USDHC_MIX_CTRL) & 0xFFFFFF80) | (wcmd & 0x7F)));
	REG32_STORE(USDHC_CMD_XFR_TYP, wcmd);
	log_debug("\nsend cmd: idx=%d; wcmd=0x%08x, mix=0x%08x\n", cmd_idx, wcmd, mix);

	/* Wait command to be completed */
	while(!(REG32_LOAD(USDHC_INT_STATUS) & -1) && --t);

	if (!t) {
		log_error("Command timeout!\n");
		_reg_dump();
	}

	t = REG32_LOAD(USDHC_INT_STATUS);
	log_debug("INTERRUPT STATUS=0x%08x\n", t);


	REG32_STORE(USDHC_INT_STATUS, t);
}

static void imx6_usdhc_send_app_cmd(int cmd_idx, uint32_t arg, uint32_t rca) {
	imx6_usdhc_send_cmd(55, rca << 16);
	_resp_dump();
	imx6_usdhc_send_cmd(cmd_idx, arg);
}

static void imx6_usdhc_reset(void) {
	_reg_dump();
	REG32_STORE(USDHC_SYS_CTRL, REG32_LOAD(USDHC_SYS_CTRL) | USDHC_SYS_CTRL_RSTA);

	int timeout = TIMEOUT;
	while(timeout--);

	if (REG32_LOAD(USDHC_SYS_CTRL) & USDHC_SYS_CTRL_RSTA) {
		log_error("Reset timeout");
	}

	REG32_STORE(USDHC_VEND_SPEC, REG32_LOAD(USDHC_VEND_SPEC) | (1 << 31) | (1 << 14) | (1 << 13));
	REG32_STORE(USDHC_VEND_SPEC, 0x20007809);
	_reg_dump();
}

static void imx6_usdhc_set_block_len(size_t len) {
	uint32_t tmp;
	imx6_usdhc_send_cmd(16, len);

	tmp = len | (1 << 16);
	REG32_STORE(USDHC_BLK_ATT, tmp);
}

uint8_t _buf[1024 * 1024] __attribute__ ((aligned(1024)));

uint8_t corr_block[] = {
0xEB, 0x3C, 0x90, 0x6D, 0x6B, 0x66, 0x73, 0x2E, 0x66, 0x61, 0x74, 0x00, 0x02, 0x40, 0x01, 0x00,
0x02, 0x00 ,0x04 ,0x00 ,0x00 ,0xF8 ,0x40 ,0x00 ,0x3E ,0x00 ,0x7C ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,
0x00, 0x00 ,0x02 ,0x00 ,0x80 ,0x01 ,0x29 ,0x7F ,0x87 ,0xC2 ,0x10 ,0x45 ,0x6D ,0x62 ,0x6F ,0x78,
0x46, 0x41 ,0x54 ,0x20 ,0x20 ,0x20 ,0x46 ,0x41 ,0x54 ,0x31 ,0x32 ,0x20 ,0x20 ,0x20 ,0x0E ,0x1F,
0xBE, 0x5B ,0x7C ,0xAC ,0x22 ,0xC0 ,0x74 ,0x0B ,0x56 ,0xB4 ,0x0E ,0xBB ,0x07 ,0x00 ,0xCD ,0x10,
0x5E, 0xEB ,0xF0 ,0x32 ,0xE4 ,0xCD ,0x16 ,0xCD ,0x19 ,0xEB ,0xFE ,0x54 ,0x68 ,0x69 ,0x73 ,0x20,
0x69, 0x73 ,0x20 ,0x6E ,0x6F ,0x74 ,0x20 ,0x61 ,0x20 ,0x62 ,0x6F ,0x6F ,0x74 ,0x61 ,0x62 ,0x6C,
0x65, 0x20 ,0x64 ,0x69 ,0x73 ,0x6B ,0x2E ,0x20 ,0x20 ,0x50 ,0x6C ,0x65 ,0x61 ,0x73 ,0x65 ,0x20,
0x69, 0x6E ,0x73 ,0x65 ,0x72 ,0x74 ,0x20 ,0x61 ,0x20 ,0x62 ,0x6F ,0x6F ,0x74 ,0x61 ,0x62 ,0x6C,
0x65, 0x20 ,0x66 ,0x6C ,0x6F ,0x70 ,0x70 ,0x79 ,0x20 ,0x61 ,0x6E ,0x64 ,0x0D ,0x0A ,0x70 ,0x72,
0x65, 0x73 ,0x73 ,0x20 ,0x61 ,0x6E ,0x79 ,0x20 ,0x6B ,0x65 ,0x79 ,0x20 ,0x74 ,0x6F ,0x20 ,0x74,
0x72, 0x79 ,0x20 ,0x61 ,0x67 ,0x61 ,0x69 ,0x6E ,0x20 ,0x2E ,0x2E ,0x2E ,0x20 ,0x0D ,0x0A ,0x00,
};

static int imx6_usdhc_read(struct block_dev *bdev, char *buffer, size_t count,
		blkno_t blkno) {
	buffer = (char*)_buf;

	int timeout = TIMEOUT;

	_reg_dump();

	log_debug("sdhc read, addr=0x%x, count=%d", blkno * bdev->block_size, count);

	memset(buffer, 0, count * BLK_LEN);

	REG32_STORE(USDHC_INT_STATUS, -1);

	log_debug("SET DS ADDR=%p\n", buffer);
	REG32_STORE(USDHC_DS_ADDR, (uint32_t) buffer);
	log_debug("got=%p\n", (void*) REG32_LOAD(USDHC_DS_ADDR));
	imx6_usdhc_send_cmd(17, blkno * BLK_LEN);

	timeout = TIMEOUT;
	while(REG32_LOAD(USDHC_PRES_STATE) & (1 << 9) && --timeout);

	if (timeout == 0) {
		log_debug("transfer active poll timeout");
	}

	show_packet((void*)buffer, 12 * 16, "");

	int waste = 0;

	if (1)
	for (int i = 0; i < 16 * 12; i++) {
		if ((_buf[i] & (~corr_block[i])) & 0xff) {
			log_debug("zero to one on pos %d", i);
		}

		_buf[i] ^= corr_block[i];
		int t = _buf[i];
		while(t) {
			waste += t % 2;
			t >>= 1;
		}
	}

	log_debug("TOTAL WASTE=%d", waste);

	show_packet((void*)buffer, 12 * 16, "");
	//_reg_dump();

	timeout = REG32_LOAD(USDHC_INT_STATUS);
	log_debug("INTERRUPT STATUS=0x%08x\n", timeout);


	REG32_STORE(USDHC_INT_STATUS, timeout);

	//_reg_dump();

	while(1);

	return count;
}

static int imx6_usdhc_write(struct block_dev *bdev, char *buffer, size_t count,
		blkno_t blkno) {
	log_debug("sdhc write, addr=0x%x, count=%d", blkno * bdev->block_size, count);

	REG32_STORE(USDHC_INT_STATUS, -1);

	REG32_STORE(USDHC_DS_ADDR, (uint32_t) buffer);

	imx6_usdhc_send_cmd(24, blkno * BLK_LEN);

	return count;
}

static int imx6_usdhc_probe(void *args);

struct block_dev_driver imx6_usdhc_driver = {
	.name  = "imx6_usdhc_driver",
	.probe = imx6_usdhc_probe,
	.read  = imx6_usdhc_read,
	.write = imx6_usdhc_write
};

BLOCK_DEV_DEF("imx6_usdhc", &imx6_usdhc_driver);

static struct periph_memory_desc imx6_usdhc_mem = {
	.start = BASE_ADDR,
	.len   = 0x4000,
};

PERIPH_MEMORY_DEFINE(imx6_usdhc_mem);

static uint32_t sdhc_get_size() {
	uint32_t t;
	/* Assume it's not high-capacity SD card */
	imx6_usdhc_send_cmd(9, 0x4567 << 16);
	t = (REG32_LOAD(USDHC_CMD_RSP1) >> 6) & 0x3;
	t |= (REG32_LOAD(USDHC_CMD_RSP1) >> 24) << 2;
	t |= (REG32_LOAD(USDHC_CMD_RSP2) & 0x3) << 10;

	return 256 * 1024 * (t + 1);
}

static int imx6_usdhc_probe(void *args) {
	struct block_dev *bdev;

	log_debug("SDHC probe");

	_reg_dump();

	bdev = block_dev_create("sdhc", &imx6_usdhc_driver, NULL);
	bdev->privdata = NULL;
	bdev->block_size = BLK_LEN;


#if 1
	/* SDHC initialization */
	imx6_usdhc_reset();

	REG32_STORE(USDHC_SYS_CTRL, 0xb011f);
	REG32_STORE(USDHC_INT_STATUS_EN, -1);

	REG32_STORE(USDHC_PROT_CTRL, 0x00000020);

	imx6_usdhc_set_block_len(BLK_LEN);
#endif
	/* SD initialization */
#if 0
	imx6_usdhc_send_cmd(0, 0);
	imx6_usdhc_send_cmd(8, 0x1aa);
	imx6_usdhc_send_app_cmd(41, 0xC01f8000, 0);
	imx6_usdhc_send_app_cmd(41, 0xC01f8000, 0);
	imx6_usdhc_send_cmd(2, 0);
	imx6_usdhc_send_cmd(3, 0);

	uint32_t rca = (REG32_LOAD(USDHC_CMD_RSP0) & 0xFFFF0000) >> 16;

	log_debug("MMC RCA=0x%04x", rca);
	imx6_usdhc_send_cmd(9, rca << 16);
	imx6_usdhc_send_cmd(7, rca << 16);

	imx6_usdhc_send_app_cmd(6, 0, rca); /* Transfer width */
	//REG32_CLEAR(USDHC_PROT_CTRL, 0x3 << 1);
	//REG32_ORIN(USDHC_PROT_CTRL, 0 << 1);
	_reg_dump();
#if 0
	uint32_t conf = 0x0000e000| (1 << 16) | (1 << 15) | (2 << 6) | (5 << 3) | 1;

	REG32_STORE(0x20e06b4, conf);
	REG32_STORE(0x20e06b0, conf);
	REG32_STORE(0x20e06ac, conf);
	REG32_STORE(0x20e06a8, conf);

	REG32_STORE(0x20e069c, conf);
	REG32_STORE(0x20e0698, conf);
	REG32_STORE(0x20e0694, conf);
	REG32_STORE(0x20e0690, conf);
#endif
#endif

	bdev->size = 1024 * 1024 * 2;
	if (0) {
	imx6_usdhc_reset();

	imx6_usdhc_send_app_cmd(41, 0xC0ff8000, 0);
	imx6_usdhc_set_block_len(BLK_LEN);
		sdhc_get_size();
		_resp_dump();
	}
	log_debug("bdev size is %d\n", bdev->size);

	_reg_dump();

	return 0;
}
