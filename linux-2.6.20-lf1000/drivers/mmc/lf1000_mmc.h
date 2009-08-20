#ifndef LF1000_MMC_HH
#define LF1000_MMC_HH

/* Command FSM States */
enum sdio_state {
	IDLE			= 0,
	SEND_INIT_SEQUENCE	= 1,
	TX_CMD_START_BIT	= 2,
	TX_CMD_TX_BIT		= 3,
	TX_CMD_INDEX_ARG	= 4,
	TX_CMD_CRC7		= 5,
	TX_CMD_END_BIT		= 6,
	TX_RESP_START_BIT	= 7,
	RX_RESP_IRQ_RESPONSE	= 8,
	RX_RESP_TX_BIT		= 9,
	RX_RESP_CMD_IDX		= 10,
	RX_RESP_DATA		= 11,
	RX_RESP_CRC7		= 12,
	RX_RESP_END_BIT		= 13,
	CMD_PATH_WITH_NCC	= 14,
	WAIT			= 15
};

/* Registers (as offsets from LF1000_SDIOn_BASE) */
#define SDI_CTRL	0x000
#define SDI_PWREN	0x004
#define SDI_CLKDIV	0x008
#define SDI_CLKENA	0x010
#define SDI_TMOUT	0x014
#define SDI_CTYPE	0x018
#define SDI_BLKSIZ	0x01C
#define SDI_BYTCNT	0x020
#define SDI_INTMASK	0x024
#define SDI_CMD		0x028
#define SDI_RESP0	0x030
#define SDI_RESP1	0x034
#define SDI_RESP2	0x038
#define SDI_RESP3	0x03C
#define SDI_MINTSTS	0x040
#define SDI_RINTSTS	0x044
#define SDI_STATUS	0x048
#define SDI_FIFOTH	0x04C
#define SDI_TCBCNT	0x05C
#define SDI_TBBCNT	0x060
#define SDI_DEBNCE	0x064
#define SDI_DAT		0x100
#define SDI_CICTRL	0x200
#define SDI_SYSCLKENB	0x7C0
#define SDI_CLKGEN	0x7C4

/* SDI Control Register (CTRL) */
#define ABORT_DATA	8
#define SEND_IRQ_RESP	7
#define READ_WAIT	6
#define DMA_ENA		5
#define INT_ENA		4
#define DMARST		2
#define FIFORST		1
#define CTRLRST		0

/* SDI Clock Enable Register (CLKENA) */
#define LOWPWR		16
#define CLKENA		15

/* SDI Timeout Register (TMOUT) */
#define DTMOUT		8
#define RSPTMOUT	0

/* SDI Card Type Register (CTYPE) */
#define C8BIT		16	/* must be 0, data book calls this 8BIT */
#define WIDTH		0	/* 0: 1-bit, 1: 4-bit */

/* SDI Interrupt Mask Register (INTMASK) */
#define MSKSDIOINT	16
#define MSKEBE		15
#define MSKACD		14
#define MSKSBE		13
#define MSKHLE		12
#define MSKFRUN		11
#define MSKHTO		10
#define MSKDRTO		9
#define MSKRTO		8
#define MSKDCRC		7
#define MSKRCRC		6
#define MSKRXDR		5
#define MSKTXDR		4
#define MSKDTO		3
#define MSKCD		2
#define MSKRE		1
#define MSKCDET		0	/* Note: data book calls this MSKCD */

/* SDI Command Register (CMD) */
#define STARTCMD	31
#define UPDATECLKONLY	21
#define CARDNUM		16	/* must be 0, bits 16:20 */
#define SENDINIT	15
#define STOPABORT	14
#define WAITPRVDAT	13
#define SENDAUTOSTOP	12
#define TRMODE		11
#define RW		10
#define DATEXP		9
#define CHKRSPCRC	8
#define RSPLEN		7
#define RSPEXP		6
#define CMDINDEX	0

/* SDI Masked Interrupt Status Register */
#define SDIOINT		16
#define EBEINT		15
#define ACDINT		14
#define SBEINT		13
#define HLEINT		12
#define FRUNINT		11
#define HTOINT		10
#define DRTOINT		9
#define RTOINT		8
#define DCRCINT		7
#define RCRCINT		6
#define RXDRINT		5
#define TXDRINT		4
#define DTOINT		3
#define CDINT		2
#define REINT		1
#define CDETINT		0	/* Note: data book calls this CDINT */

/* SDI Status Register (STATUS) */
#define DMAREQ		31
#define DMAACK		30
#define FIFOCOUNT	17
#define RSPINDEX	11
#define FSMBUSY		10
#define DATBUSY		9
#define CPRESENT	8
#define CMDFSM		4
#define FIFOFULL	3
#define FIFOEMPTY	2
#define TXWMARK		1
#define RXWMARK		0

/* SDI FIFO Threshold Watermark Register (FIFOTH) */
#define RXTH		16
#define TXTH		0

/* SDI Card Interface Control Register (CICTRL) */
#define CDETECT		2
#define WRTPRT		0

/* SDI System Clock Enable Register (SYSCLKENB) */
#define PCLKMODE	3
#define CLKGENENB	2

/* SDI Clock Generator Register (CLKGEN) */
#define CLKDIV0		4
#define CLKSRCSEL0	1
#define OUTCLKINV0	0

#endif
