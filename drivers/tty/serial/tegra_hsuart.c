/*
 * drivers/serial/tegra_hsuart.c
 *
 * High-speed serial driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009-2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/termios.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/serial_reg.h>
#include <linux/serial_8250.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/tegra_uart.h>

#include <mach/dma.h>
#include <mach/clk.h>

#define TEGRA_UART_TYPE "TEGRA_UART"

#define TX_EMPTY_STATUS (UART_LSR_TEMT | UART_LSR_THRE)

#define BYTES_TO_ALIGN(x) ((unsigned long)(ALIGN((x), sizeof(u32))) - \
	(unsigned long)(x))

#define UART_RX_DMA_BUFFER_SIZE    (2048*8)

#define UART_LSR_FIFOE		0x80
#define UART_LSR_TXFIFO_FULL	0x100
#define UART_IER_EORD		0x20
#define UART_MCR_RTS_EN		0x40
#define UART_MCR_CTS_EN		0x20
#define UART_LSR_ANY		(UART_LSR_OE | UART_LSR_BI | \
				UART_LSR_PE | UART_LSR_FE)

#define TX_FORCE_PIO 0
#define RX_FORCE_PIO 0

const int dma_req_sel[] = {
	TEGRA_DMA_REQ_SEL_UARTA,
	TEGRA_DMA_REQ_SEL_UARTB,
	TEGRA_DMA_REQ_SEL_UARTC,
	TEGRA_DMA_REQ_SEL_UARTD,
	TEGRA_DMA_REQ_SEL_UARTE,
};

#define TEGRA_TX_PIO			1
#define TEGRA_TX_DMA			2

#define TEGRA_UART_MIN_DMA		16
#define TEGRA_UART_FIFO_SIZE		8

#define TEGRA_UART_CLOSED    0
#define TEGRA_UART_OPENED    1
#define TEGRA_UART_CLOCK_OFF 2
#define TEGRA_UART_SUSPEND   3

/* Tx fifo trigger level setting in tegra uart is in
 * reverse way then conventional uart */
#define TEGRA_UART_TX_TRIG_16B 0x00
#define TEGRA_UART_TX_TRIG_8B  0x10
#define TEGRA_UART_TX_TRIG_4B  0x20
#define TEGRA_UART_TX_TRIG_1B  0x30

struct tegra_uart_port {
	struct uart_port	uport;
	char			port_name[32];

	/* Module info */
	unsigned long		size;
	struct clk		*clk;
	unsigned int		baud;

	/* Register shadow */
	unsigned char		fcr_shadow;
	unsigned char		mcr_shadow;
	unsigned char		lcr_shadow;
	unsigned char		ier_shadow;
	bool			use_cts_control;
	bool			rts_active;

	int			tx_in_progress;
	unsigned int		tx_bytes;

	dma_addr_t		xmit_dma_addr;

	/* TX DMA */
	struct tegra_dma_req	tx_dma_req;
	struct tegra_dma_channel *tx_dma;

	/* RX DMA */
	struct tegra_dma_req	rx_dma_req;
	struct tegra_dma_channel *rx_dma;

	bool			use_rx_dma;
	bool			use_tx_dma;
	int			uart_state;
	bool			rx_timeout;
	int			rx_in_progress;
};

static void tegra_set_baudrate(struct tegra_uart_port *t, unsigned int baud);
static void do_handle_rx_pio(struct tegra_uart_port *t);
static void do_handle_rx_dma(struct tegra_uart_port *t);
static void set_rts(struct tegra_uart_port *t, bool active);

static inline u8 uart_readb(struct tegra_uart_port *t, unsigned long reg)
{
	u8 val = readb(t->uport.membase + (reg << t->uport.regshift));
	dev_vdbg(t->uport.dev, "%s: %p %03lx = %02x\n", __func__,
		t->uport.membase, reg << t->uport.regshift, val);
	return val;
}

static inline u32 uart_readl(struct tegra_uart_port *t, unsigned long reg)
{
	u32 val = readl(t->uport.membase + (reg << t->uport.regshift));
	dev_vdbg(t->uport.dev, "%s: %p %03lx = %02x\n", __func__,
		t->uport.membase, reg << t->uport.regshift, val);
	return val;
}

static inline void uart_writeb(struct tegra_uart_port *t, u8 val,
	unsigned long reg)
{
	dev_vdbg(t->uport.dev, "%s: %p %03lx %02x\n",
		__func__, t->uport.membase, reg << t->uport.regshift, val);
	writeb(val, t->uport.membase + (reg << t->uport.regshift));
}

static inline void uart_writel(struct tegra_uart_port *t, u32 val,
	unsigned long reg)
{
	dev_vdbg(t->uport.dev, "%s: %p %03lx %08x\n",
		__func__, t->uport.membase, reg << t->uport.regshift, val);
	writel(val, t->uport.membase + (reg << t->uport.regshift));
}

static void fill_tx_fifo(struct tegra_uart_port *t, int max_bytes)
{
	int i;
	struct circ_buf *xmit = &t->uport.state->xmit;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	unsigned long lsr;
#endif

	for (i = 0; i < max_bytes; i++) {
		BUG_ON(uart_circ_empty(xmit));
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		lsr = uart_readl(t, UART_LSR);
		if ((lsr & UART_LSR_TXFIFO_FULL))
			break;
#endif
		uart_writeb(t, xmit->buf[xmit->tail], UART_TX);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		t->uport.icount.tx++;
	}
}

static void tegra_start_pio_tx(struct tegra_uart_port *t, unsigned int bytes)
{
	if (bytes > TEGRA_UART_FIFO_SIZE)
		bytes = TEGRA_UART_FIFO_SIZE;

	t->fcr_shadow &= ~UART_FCR_T_TRIG_11;
	t->fcr_shadow |= TEGRA_UART_TX_TRIG_8B;
	uart_writeb(t, t->fcr_shadow, UART_FCR);
	t->tx_in_progress = TEGRA_TX_PIO;
	t->tx_bytes = bytes;
	t->ier_shadow |= UART_IER_THRI;
	uart_writeb(t, t->ier_shadow, UART_IER);
}

static void tegra_start_dma_tx(struct tegra_uart_port *t, unsigned long bytes)
{
	struct circ_buf *xmit;
	xmit = &t->uport.state->xmit;

	dma_sync_single_for_device(t->uport.dev, t->xmit_dma_addr,
		UART_XMIT_SIZE, DMA_TO_DEVICE);

	t->fcr_shadow &= ~UART_FCR_T_TRIG_11;
	t->fcr_shadow |= TEGRA_UART_TX_TRIG_4B;
	uart_writeb(t, t->fcr_shadow, UART_FCR);

	t->tx_bytes = bytes & ~(sizeof(u32)-1);
	t->tx_dma_req.source_addr = t->xmit_dma_addr + xmit->tail;
	t->tx_dma_req.size = t->tx_bytes;

	t->tx_in_progress = TEGRA_TX_DMA;

	tegra_dma_enqueue_req(t->tx_dma, &t->tx_dma_req);
}

/* Called with u->lock taken */
static void tegra_start_next_tx(struct tegra_uart_port *t)
{
	unsigned long tail;
	unsigned long count;

	struct circ_buf *xmit;

	xmit = &t->uport.state->xmit;
	tail = (unsigned long)&xmit->buf[xmit->tail];
	count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);


	dev_vdbg(t->uport.dev, "+%s %lu %d\n", __func__, count,
		t->tx_in_progress);

	if (count == 0)
		goto out;

	if (!t->use_tx_dma || count < TEGRA_UART_MIN_DMA)
		tegra_start_pio_tx(t, count);
	else if (BYTES_TO_ALIGN(tail) > 0)
		tegra_start_pio_tx(t, BYTES_TO_ALIGN(tail));
	else
		tegra_start_dma_tx(t, count);

out:
	dev_vdbg(t->uport.dev, "-%s", __func__);
}

/* Called by serial core driver with u->lock taken. */
static void tegra_start_tx(struct uart_port *u)
{
	struct tegra_uart_port *t;
	struct circ_buf *xmit;

	t = container_of(u, struct tegra_uart_port, uport);
	xmit = &u->state->xmit;

	if (!uart_circ_empty(xmit) && !t->tx_in_progress)
		tegra_start_next_tx(t);
}

static int tegra_start_dma_rx(struct tegra_uart_port *t)
{
	wmb();
	dma_sync_single_for_device(t->uport.dev, t->rx_dma_req.dest_addr,
			t->rx_dma_req.size, DMA_TO_DEVICE);
	if (tegra_dma_enqueue_req(t->rx_dma, &t->rx_dma_req)) {
		dev_err(t->uport.dev, "Could not enqueue Rx DMA req\n");
		return -EINVAL;
	}
	return 0;
}

static void tegra_rx_dma_threshold_callback(struct tegra_dma_req *req)
{
	struct tegra_uart_port *t = req->dev;
	struct uart_port *u = &t->uport;
	unsigned long flags;

	spin_lock_irqsave(&u->lock, flags);

	do_handle_rx_dma(t);

	spin_unlock_irqrestore(&u->lock, flags);
}

/*
 * It is expected that the callers take the UART lock when this API is called.
 *
 * There are 2 contexts when this function is called:
 *
 * 1. DMA ISR - DMA ISR triggers the threshold complete calback, which calls the
 * dequue API which in-turn calls this callback. UART lock is taken during
 * the call to the threshold callback.
 *
 * 2. UART ISR - UART calls the dequue API which in-turn will call this API.
 * In this case, UART ISR takes the UART lock.
 */
static void tegra_rx_dma_complete_callback(struct tegra_dma_req *req)
{
	struct tegra_uart_port *t = req->dev;
	struct uart_port *u = &t->uport;
	struct tty_struct *tty = u->state->port.tty;
	int copied;

	/* If we are here, DMA is stopped */

	dev_dbg(t->uport.dev, "%s: %d %d\n", __func__, req->bytes_transferred,
		req->status);
	if (req->bytes_transferred) {
		t->uport.icount.rx += req->bytes_transferred;
		dma_sync_single_for_cpu(t->uport.dev, req->dest_addr,
				req->size, DMA_FROM_DEVICE);
		copied = tty_insert_flip_string(tty,
			((unsigned char *)(req->virt_addr)),
			req->bytes_transferred);
		if (copied != req->bytes_transferred) {
			WARN_ON(1);
			dev_err(t->uport.dev, "Not able to copy uart data "
				"to tty layer Req %d and coped %d\n",
				req->bytes_transferred, copied);
		}
		dma_sync_single_for_device(t->uport.dev, req->dest_addr,
				req->size, DMA_TO_DEVICE);
	}

	do_handle_rx_pio(t);

	/* Push the read data later in caller place. */
	if (req->status == -TEGRA_DMA_REQ_ERROR_ABORTED)
		return;

	spin_unlock(&u->lock);
	tty_flip_buffer_push(u->state->port.tty);
	spin_lock(&u->lock);
}

/* Lock already taken */
static void do_handle_rx_dma(struct tegra_uart_port *t)
{
	struct uart_port *u = &t->uport;
	if (t->rts_active)
		set_rts(t, false);
	tegra_dma_dequeue_req(t->rx_dma, &t->rx_dma_req);
	tty_flip_buffer_push(u->state->port.tty);
	/* enqueue the request again */
	tegra_start_dma_rx(t);
	if (t->rts_active)
		set_rts(t, true);
}

/* Wait for a symbol-time. */
static void wait_sym_time(struct tegra_uart_port *t, unsigned int syms)
{

	/* Definitely have a start bit. */
	unsigned int bits = 1;
	switch (t->lcr_shadow & 3) {
	case UART_LCR_WLEN5:
		bits += 5;
		break;
	case UART_LCR_WLEN6:
		bits += 6;
		break;
	case UART_LCR_WLEN7:
		bits += 7;
		break;
	default:
		bits += 8;
		break;
	}

	/* Technically 5 bits gets 1.5 bits of stop... */
	if (t->lcr_shadow & UART_LCR_STOP)
		bits += 2;
	else
		bits++;

	if (t->lcr_shadow & UART_LCR_PARITY)
		bits++;

	if (likely(t->baud))
		udelay(DIV_ROUND_UP(syms * bits * 1000000, t->baud));
}

/* Flush desired FIFO. */
static void tegra_fifo_reset(struct tegra_uart_port *t, u8 fcr_bits)
{
	unsigned char fcr = t->fcr_shadow;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	fcr |= fcr_bits & (UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	uart_writeb(t, fcr, UART_FCR);
#else
	/*Hw issue: Resetting tx fifo with non-fifo
	mode to avoid any extra character to be sent*/
	fcr &= ~UART_FCR_ENABLE_FIFO;
	uart_writeb(t, fcr, UART_FCR);
	udelay(60);
	fcr |= fcr_bits & (UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	uart_writeb(t, fcr, UART_FCR);
	fcr |= UART_FCR_ENABLE_FIFO;
	uart_writeb(t, fcr, UART_FCR);
#endif
	uart_readb(t, UART_SCR); /* Dummy read to ensure the write is posted */
	wait_sym_time(t, 1); /* Wait for the flush to propagate. */
}

static char do_decode_rx_error(struct tegra_uart_port *t, u8 lsr)
{
	char flag = TTY_NORMAL;

	if (unlikely(lsr & UART_LSR_ANY)) {
		if (lsr & UART_LSR_OE) {
			/* Overrrun error  */
			flag |= TTY_OVERRUN;
			t->uport.icount.overrun++;
			dev_err(t->uport.dev, "Got overrun errors\n");
		} else if (lsr & UART_LSR_PE) {
			/* Parity error */
			flag |= TTY_PARITY;
			t->uport.icount.parity++;
			dev_err(t->uport.dev, "Got Parity errors\n");
		} else if (lsr & UART_LSR_FE) {
			flag |= TTY_FRAME;
			t->uport.icount.frame++;
			dev_err(t->uport.dev, "Got frame errors\n");
		} else if (lsr & UART_LSR_BI) {
			dev_err(t->uport.dev, "Got Break\n");
			t->uport.icount.brk++;
			/* If FIFO read error without any data, reset Rx FIFO */
			if (!(lsr & UART_LSR_DR) && (lsr & UART_LSR_FIFOE))
				tegra_fifo_reset(t, UART_FCR_CLEAR_RCVR);
		}
	}
	return flag;
}

static void do_handle_rx_pio(struct tegra_uart_port *t)
{
	int count = 0;
	do {
		char flag = TTY_NORMAL;
		unsigned char lsr = 0;
		unsigned char ch;


		lsr = uart_readb(t, UART_LSR);
		if (!(lsr & UART_LSR_DR))
			break;

		flag =  do_decode_rx_error(t, lsr);
		ch = uart_readb(t, UART_RX);
		t->uport.icount.rx++;
		count++;

		if (!uart_handle_sysrq_char(&t->uport, c))
			uart_insert_char(&t->uport, lsr, UART_LSR_OE, ch, flag);
	} while (1);

	dev_dbg(t->uport.dev, "PIO received %d bytes\n", count);
	return;
}

static void do_handle_modem_signal(struct uart_port *u)
{
	unsigned char msr;
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);
	msr = uart_readb(t, UART_MSR);
	if (msr & UART_MSR_CTS)
		dev_dbg(u->dev, "CTS triggered\n");
	if (msr & UART_MSR_DSR)
		dev_dbg(u->dev, "DSR enabled\n");
	if (msr & UART_MSR_DCD)
		dev_dbg(u->dev, "CD enabled\n");
	if (msr & UART_MSR_RI)
		dev_dbg(u->dev, "RI enabled\n");
	return;
}

static void do_handle_tx_pio(struct tegra_uart_port *t)
{
	struct circ_buf *xmit = &t->uport.state->xmit;

	fill_tx_fifo(t, t->tx_bytes);

	t->tx_in_progress = 0;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&t->uport);

	tegra_start_next_tx(t);
	return;
}

static void tegra_tx_dma_complete_callback(struct tegra_dma_req *req)
{
	struct tegra_uart_port *t = req->dev;
	struct circ_buf *xmit = &t->uport.state->xmit;
	int count = req->bytes_transferred;
	unsigned long flags;

	dev_vdbg(t->uport.dev, "%s: %d\n", __func__, count);

	/* Update xmit pointers without lock if dma aborted. */
	if (req->status == -TEGRA_DMA_REQ_ERROR_ABORTED) {
		xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
		t->tx_in_progress = 0;
		return;
	}

	spin_lock_irqsave(&t->uport.lock, flags);
	xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
	t->tx_in_progress = 0;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&t->uport);

	tegra_start_next_tx(t);

	spin_unlock_irqrestore(&t->uport.lock, flags);
}

static irqreturn_t tegra_uart_isr(int irq, void *data)
{
	struct tegra_uart_port *t = data;
	struct uart_port *u = &t->uport;
	unsigned char iir;
	unsigned char ier;
	bool is_rx_int = false;
	unsigned long flags;

	spin_lock_irqsave(&u->lock, flags);
	t  = container_of(u, struct tegra_uart_port, uport);
	while (1) {
		iir = uart_readb(t, UART_IIR);
		if (iir & UART_IIR_NO_INT) {
			if (likely(t->use_rx_dma) && is_rx_int) {
				do_handle_rx_dma(t);

				if (t->rx_in_progress) {
					ier = t->ier_shadow;
					ier |= (UART_IER_RLSI | UART_IER_RTOIE |
								UART_IER_EORD);
					t->ier_shadow = ier;
					uart_writeb(t, ier, UART_IER);
				}
			}
			spin_unlock_irqrestore(&u->lock, flags);
			return IRQ_HANDLED;
		}

		dev_dbg(u->dev, "tegra_uart_isr iir = 0x%x (%d)\n", iir,
			(iir >> 1) & 0x7);
		switch ((iir >> 1) & 0x7) {
		case 0: /* Modem signal change interrupt */
			do_handle_modem_signal(u);
			break;
		case 1: /* Transmit interrupt only triggered when using PIO */
			t->ier_shadow &= ~UART_IER_THRI;
			uart_writeb(t, t->ier_shadow, UART_IER);
			do_handle_tx_pio(t);
			break;
		case 4: /* End of data */
		case 6: /* Rx timeout */
		case 2: /* Receive */
			if (likely(t->use_rx_dma)) {
				if (!is_rx_int) {
					is_rx_int = true;
					/* Disable interrups */
					ier = t->ier_shadow;
					ier |= UART_IER_RDI;
					uart_writeb(t, ier, UART_IER);
					ier &= ~(UART_IER_RDI | UART_IER_RLSI |
						UART_IER_RTOIE | UART_IER_EORD);
					t->ier_shadow = ier;
					uart_writeb(t, ier, UART_IER);
				}
			} else {
				do_handle_rx_pio(t);

				spin_unlock_irqrestore(&u->lock, flags);
				tty_flip_buffer_push(u->state->port.tty);
				spin_lock_irqsave(&u->lock, flags);
			}
			break;
		case 3: /* Receive error */
			/* FIXME how to handle this? Why do we get here */
			do_decode_rx_error(t, uart_readb(t, UART_LSR));
			break;
		case 5: /* break nothing to handle */
		case 7: /* break nothing to handle */
			break;
		}
	}
}

static void tegra_stop_rx(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned char ier;

	t = container_of(u, struct tegra_uart_port, uport);

	if (t->rts_active)
		set_rts(t, false);

	if (t->rx_in_progress) {
		wait_sym_time(t, 1); /* wait a character interval */

		ier = t->ier_shadow;
		ier &= ~(UART_IER_RDI | UART_IER_RLSI | UART_IER_RTOIE |
					UART_IER_EORD);
		t->ier_shadow = ier;
		uart_writeb(t, ier, UART_IER);
		t->rx_in_progress = 0;

		if (t->use_rx_dma && t->rx_dma)
			tegra_dma_dequeue_req(t->rx_dma, &t->rx_dma_req);
		else
			do_handle_rx_pio(t);

		tty_flip_buffer_push(u->state->port.tty);
	}

	return;
}

static void tegra_uart_hw_deinit(struct tegra_uart_port *t)
{
	unsigned long flags;
	unsigned long char_time = DIV_ROUND_UP(10000000, t->baud);
	unsigned long fifo_empty_time = t->uport.fifosize * char_time;
	unsigned long wait_time;
	unsigned char lsr;
	unsigned char msr;
	unsigned char mcr;

	/* Disable interrupts */
	uart_writeb(t, 0, UART_IER);

	lsr = uart_readb(t, UART_LSR);
	if ((lsr & UART_LSR_TEMT) != UART_LSR_TEMT) {
		msr = uart_readb(t, UART_MSR);
		mcr = uart_readb(t, UART_MCR);
		if ((mcr & UART_MCR_CTS_EN) && (msr & UART_MSR_CTS))
			dev_err(t->uport.dev, "%s: Tx fifo not empty and "
				"slave disabled CTS, Waiting for slave to"
				" be ready\n", __func__);

		/* Wait for Tx fifo to be empty */
		while ((lsr & UART_LSR_TEMT) != UART_LSR_TEMT) {
			wait_time = min(fifo_empty_time, 100lu);
			udelay(wait_time);
			fifo_empty_time -= wait_time;
			if (!fifo_empty_time) {
				msr = uart_readb(t, UART_MSR);
				mcr = uart_readb(t, UART_MCR);
				if ((mcr & UART_MCR_CTS_EN) &&
					(msr & UART_MSR_CTS))
					dev_err(t->uport.dev, "%s: Slave is "
					"still not ready!\n", __func__);
				break;
			}
			lsr = uart_readb(t, UART_LSR);
		}
	}

	spin_lock_irqsave(&t->uport.lock, flags);

	/* Reset the Rx and Tx FIFOs */
	tegra_fifo_reset(t, UART_FCR_CLEAR_XMIT | UART_FCR_CLEAR_RCVR);

	t->baud = 0;
	t->uart_state = TEGRA_UART_CLOSED;

	spin_unlock_irqrestore(&t->uport.lock, flags);

	clk_disable(t->clk);
}

static void tegra_uart_free_rx_dma_buffer(struct tegra_uart_port *t)
{
	if (likely(t->rx_dma_req.dest_addr))
		dma_free_coherent(t->uport.dev, t->rx_dma_req.size,
			t->rx_dma_req.virt_addr, t->rx_dma_req.dest_addr);
	t->rx_dma_req.dest_addr = 0;
	t->rx_dma_req.virt_addr = NULL;
}

static void tegra_uart_free_rx_dma(struct tegra_uart_port *t)
{
	if (!t->use_rx_dma)
		return;

	tegra_dma_free_channel(t->rx_dma);
	t->rx_dma = NULL;
	t->use_rx_dma = false;
}

static int tegra_uart_hw_init(struct tegra_uart_port *t)
{
	unsigned char ier;

	dev_vdbg(t->uport.dev, "+tegra_uart_hw_init\n");

	t->fcr_shadow = 0;
	t->mcr_shadow = 0;
	t->lcr_shadow = 0;
	t->ier_shadow = 0;
	t->baud = 0;

	clk_enable(t->clk);

	/* Reset the UART controller to clear all previous status.*/
	tegra_periph_reset_assert(t->clk);
	udelay(100);
	tegra_periph_reset_deassert(t->clk);
	udelay(100);

	t->rx_in_progress = 0;

	/*
	 * Set the trigger level
	 *
	 * For PIO mode:
	 *
	 * For receive, this will interrupt the CPU after that many number of
	 * bytes are received, for the remaining bytes the receive timeout
	 * interrupt is received.
	 *
	 *  Rx high watermark is set to 4.
	 *
	 * For transmit, if the trasnmit interrupt is enabled, this will
	 * interrupt the CPU when the number of entries in the FIFO reaches the
	 * low watermark.
	 *
	 *  Tx low watermark is set to 8.
	 *
	 *  For DMA mode:
	 *
	 *  Set the Tx trigger to 4. This should match the DMA burst size that
	 *  programmed in the DMA registers.
	 */
	t->fcr_shadow = UART_FCR_ENABLE_FIFO;
	t->fcr_shadow |= UART_FCR_R_TRIG_01;
	t->fcr_shadow |= TEGRA_UART_TX_TRIG_8B;
	uart_writeb(t, t->fcr_shadow, UART_FCR);

	if (t->use_rx_dma) {
		/*
		 * Initialize the UART for a simple default configuration
		 * so that the receive DMA buffer may be enqueued */
		t->lcr_shadow = 3;  /* no parity, stop, 8 data bits */
		tegra_set_baudrate(t, 115200);
		t->fcr_shadow |= UART_FCR_DMA_SELECT;
		uart_writeb(t, t->fcr_shadow, UART_FCR);
		if (tegra_start_dma_rx(t)) {
			dev_err(t->uport.dev, "Rx DMA enqueue failed\n");
			tegra_uart_free_rx_dma(t);
			t->fcr_shadow &= ~UART_FCR_DMA_SELECT;
			uart_writeb(t, t->fcr_shadow, UART_FCR);
		}
	} else {
		uart_writeb(t, t->fcr_shadow, UART_FCR);
	}

	t->rx_in_progress = 1;

	/*
	 *  Enable IE_RXS for the receive status interrupts like line errros.
	 *  Enable IE_RX_TIMEOUT to get the bytes which cannot be DMA'd.
	 *
	 *  If using DMA mode, enable EORD instead of receive interrupt which
	 *  will interrupt after the UART is done with the receive instead of
	 *  the interrupt when the FIFO "threshold" is reached.
	 *
	 *  EORD is different interrupt than RX_TIMEOUT - RX_TIMEOUT occurs when
	 *  the DATA is sitting in the FIFO and couldn't be transferred to the
	 *  DMA as the DMA size alignment(4 bytes) is not met. EORD will be
	 *  triggered when there is a pause of the incomming data stream for 4
	 *  characters long.
	 *
	 *  For pauses in the data which is not aligned to 4 bytes, we get
	 *  both the EORD as well as RX_TIMEOUT - SW sees RX_TIMEOUT first
	 *  then the EORD.
	 *
	 *  Don't get confused, believe in the magic of nvidia hw...:-)
	 */
	ier = 0;
	ier |= UART_IER_RLSI | UART_IER_RTOIE;
	if (t->use_rx_dma)
		ier |= UART_IER_EORD;
	else
		ier |= UART_IER_RDI;
	t->ier_shadow = ier;
	uart_writeb(t, ier, UART_IER);

	t->uart_state = TEGRA_UART_OPENED;
	dev_vdbg(t->uport.dev, "-tegra_uart_hw_init\n");
	return 0;
}

static int tegra_uart_init_rx_dma_buffer(struct tegra_uart_port *t)
{
	dma_addr_t rx_dma_phys;
	void *rx_dma_virt;

	t->rx_dma_req.size = UART_RX_DMA_BUFFER_SIZE;
	rx_dma_virt = dma_alloc_coherent(t->uport.dev,
		t->rx_dma_req.size, &rx_dma_phys, GFP_KERNEL);
	if (!rx_dma_virt) {
		dev_err(t->uport.dev, "DMA buffers allocate failed\n");
		return -ENOMEM;
	}
	t->rx_dma_req.dest_addr = rx_dma_phys;
	t->rx_dma_req.virt_addr = rx_dma_virt;

	t->rx_dma_req.source_addr = (unsigned long)t->uport.mapbase;
	t->rx_dma_req.source_wrap = 4;
	t->rx_dma_req.dest_wrap = 0;
	t->rx_dma_req.to_memory = 1;
	t->rx_dma_req.source_bus_width = 8;
	t->rx_dma_req.dest_bus_width = 32;
	t->rx_dma_req.req_sel = dma_req_sel[t->uport.line];
	t->rx_dma_req.complete = tegra_rx_dma_complete_callback;
	t->rx_dma_req.threshold = tegra_rx_dma_threshold_callback;
	t->rx_dma_req.dev = t;

	return 0;
}

static int tegra_uart_init_rx_dma(struct tegra_uart_port *t)
{
	t->rx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_CONTINUOUS,
					"uart_rx_%d", t->uport.line);
	if (!t->rx_dma) {
		dev_err(t->uport.dev, "%s: failed to allocate RX DMA.\n",
				__func__);
		return -ENODEV;
	}
	return 0;
}

static int tegra_startup(struct uart_port *u)
{
	struct tegra_uart_port *t = container_of(u,
		struct tegra_uart_port, uport);
	int ret = 0;
	struct tegra_uart_platform_data *pdata;

	t = container_of(u, struct tegra_uart_port, uport);
	sprintf(t->port_name, "tegra_uart_%d", u->line);

	t->use_tx_dma = false;
	if (!TX_FORCE_PIO) {
		t->tx_dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT,
					"uart_tx_%d", u->line);
		if (t->tx_dma)
			t->use_tx_dma = true;
		else
			pr_err("%s: failed to allocate TX DMA.\n", __func__);
	}
	if (t->use_tx_dma) {
		t->tx_dma_req.instance = u->line;
		t->tx_dma_req.complete = tegra_tx_dma_complete_callback;
		t->tx_dma_req.to_memory = 0;

		t->tx_dma_req.dest_addr = (unsigned long)t->uport.mapbase;
		t->tx_dma_req.dest_wrap = 4;
		t->tx_dma_req.source_wrap = 0;
		t->tx_dma_req.source_bus_width = 32;
		t->tx_dma_req.dest_bus_width = 8;
		t->tx_dma_req.req_sel = dma_req_sel[t->uport.line];
		t->tx_dma_req.dev = t;
		t->tx_dma_req.size = 0;
		t->xmit_dma_addr = dma_map_single(t->uport.dev,
			t->uport.state->xmit.buf, UART_XMIT_SIZE,
			DMA_TO_DEVICE);
	}
	t->tx_in_progress = 0;

	t->use_rx_dma = false;
	if (!RX_FORCE_PIO && t->rx_dma_req.virt_addr) {
		if (!tegra_uart_init_rx_dma(t))
			t->use_rx_dma = true;
	}

	ret = tegra_uart_hw_init(t);
	if (ret)
		goto fail;

	pdata = u->dev->platform_data;
	if (pdata->is_loopback)
		t->mcr_shadow |= UART_MCR_LOOP;

	dev_dbg(u->dev, "Requesting IRQ %d\n", u->irq);
	ret = request_irq(u->irq, tegra_uart_isr, IRQF_DISABLED,
				t->port_name, t);
	if (ret) {
		dev_err(u->dev, "Failed to register ISR for IRQ %d\n", u->irq);
		goto fail;
	}

	dev_dbg(u->dev, "Started UART port %d\n", u->line);
	return 0;
fail:
	dev_err(u->dev, "Tegra UART startup failed\n");
	return ret;
}

static void tegra_shutdown(struct uart_port *u)
{
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(u->dev, "+tegra_shutdown\n");

	tegra_uart_hw_deinit(t);

	t->rx_in_progress = 0;
	t->tx_in_progress = 0;

	tegra_uart_free_rx_dma(t);
	if (t->use_tx_dma) {
		tegra_dma_free_channel(t->tx_dma);
		t->tx_dma = NULL;
		t->use_tx_dma = false;
		dma_unmap_single(t->uport.dev, t->xmit_dma_addr, UART_XMIT_SIZE,
				DMA_TO_DEVICE);
		t->xmit_dma_addr = 0;
	}

	free_irq(u->irq, t);
	dev_vdbg(u->dev, "-tegra_shutdown\n");
}

static void tegra_wake_peer(struct uart_port *u)
{
	struct tegra_uart_platform_data *pdata = u->dev->platform_data;

	if (pdata && pdata->wake_peer)
		pdata->wake_peer(u);
}

static unsigned int tegra_get_mctrl(struct uart_port *u)
{
	/*
	 * RI - Ring detector is active
	 * CD/DCD/CAR - Carrier detect is always active. For some reason
	 *			  linux has different names for carrier detect.
	 * DSR - Data Set ready is active as the hardware doesn't support it.
	 *	   Don't know if the linux support this yet?
	 * CTS - Clear to send. Always set to active, as the hardware handles
	 *	   CTS automatically.
	 */
	return TIOCM_RI | TIOCM_CD | TIOCM_DSR | TIOCM_CTS;
}

static void set_rts(struct tegra_uart_port *t, bool active)
{
	unsigned char mcr;
	mcr = t->mcr_shadow;
	if (active)
		mcr |= UART_MCR_RTS_EN;
	else
		mcr &= ~UART_MCR_RTS_EN;
	if (mcr != t->mcr_shadow) {
		uart_writeb(t, mcr, UART_MCR);
		t->mcr_shadow = mcr;
	}
	return;
}

static void set_dtr(struct tegra_uart_port *t, bool active)
{
	unsigned char mcr;
	mcr = t->mcr_shadow;
	if (active)
		mcr |= UART_MCR_DTR;
	else
		mcr &= ~UART_MCR_DTR;
	if (mcr != t->mcr_shadow) {
		uart_writeb(t, mcr, UART_MCR);
		t->mcr_shadow = mcr;
	}
	return;
}

static void tegra_set_mctrl(struct uart_port *u, unsigned int mctrl)
{
	unsigned char mcr;
	struct tegra_uart_port *t;

	dev_dbg(u->dev, "tegra_set_mctrl called with %d\n", mctrl);
	t = container_of(u, struct tegra_uart_port, uport);

	mcr = t->mcr_shadow;
	if (mctrl & TIOCM_RTS) {
		t->rts_active = true;
		set_rts(t, true);
	} else {
		t->rts_active = false;
		set_rts(t, false);
	}

	if (mctrl & TIOCM_DTR)
		set_dtr(t, true);
	else
		set_dtr(t, false);
	return;
}

static void tegra_break_ctl(struct uart_port *u, int break_ctl)
{
	struct tegra_uart_port *t;
	unsigned char lcr;

	t = container_of(u, struct tegra_uart_port, uport);
	lcr = t->lcr_shadow;
	if (break_ctl)
		lcr |= UART_LCR_SBC;
	else
		lcr &= ~UART_LCR_SBC;
	uart_writeb(t, lcr, UART_LCR);
	t->lcr_shadow = lcr;
}

static int tegra_request_port(struct uart_port *u)
{
	return 0;
}

static void tegra_release_port(struct uart_port *u)
{
	/* Nothing to do here */
}

static unsigned int tegra_tx_empty(struct uart_port *u)
{
	struct tegra_uart_port *t;
	unsigned int ret = 0;
	unsigned long flags;
	unsigned char lsr;

	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(u->dev, "+tegra_tx_empty\n");

	spin_lock_irqsave(&u->lock, flags);
	if (!t->tx_in_progress) {
		lsr = uart_readb(t, UART_LSR);
		if ((lsr & TX_EMPTY_STATUS) == TX_EMPTY_STATUS)
			ret = TIOCSER_TEMT;
	}
	spin_unlock_irqrestore(&u->lock, flags);

	dev_vdbg(u->dev, "-tegra_tx_empty\n");
	return ret;
}

static void tegra_stop_tx(struct uart_port *u)
{
	struct tegra_uart_port *t;

	t = container_of(u, struct tegra_uart_port, uport);

	if (t->use_tx_dma)
		tegra_dma_dequeue_req(t->tx_dma, &t->tx_dma_req);

	return;
}

static void tegra_enable_ms(struct uart_port *u)
{
}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int clk_div71_get_divider(unsigned long parent_rate,
		unsigned long rate)
{
	s64 divider_u71 = parent_rate;
	if (!rate)
		return -EINVAL;

	divider_u71 *= 2;
	divider_u71 += rate - 1;
	do_div(divider_u71, rate);

	if ((divider_u71 - 2) < 0)
		return 0;

	if ((divider_u71 - 2) > 255)
		return -EINVAL;

	return divider_u71 - 2;
}
#endif

static int clk_div16_get_divider(unsigned long parent_rate, unsigned long rate)
{
	s64 divider_u16;

	divider_u16 = parent_rate;
	if (!rate)
		return -EINVAL;
	divider_u16 += rate - 1;
	do_div(divider_u16, rate);

	if (divider_u16 > 0xFFFF)
		return -EINVAL;

	return divider_u16;
}

static unsigned long find_best_clock_source(struct tegra_uart_port *t,
		unsigned long rate)
{
	struct uart_port *u = &t->uport;
	struct tegra_uart_platform_data *pdata;
	int i;
	int divider;
	unsigned long parent_rate;
	unsigned long new_rate;
	unsigned long err_rate;
	unsigned int fin_err = rate;
	unsigned long fin_rate = rate;
	int final_index = -1;
	int count;
	unsigned long error_2perc;

	pdata = u->dev->platform_data;
	if (!pdata || !pdata->parent_clk_count)
		return fin_rate;

	error_2perc = (rate / 50);

	for (count = 0; count < pdata->parent_clk_count; ++count) {
		parent_rate = pdata->parent_clk_list[count].fixed_clk_rate;

		if (parent_rate < rate)
			continue;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		divider = clk_div71_get_divider(parent_rate, rate);

		/* Get the best divider around calculated value */
		if (divider > 2) {
			for (i = divider - 2; i < (divider + 2); ++i) {
				new_rate = ((parent_rate << 1) + i + 1) /
								(i + 2);
				err_rate = abs(new_rate - rate);
				if (err_rate < fin_err) {
					final_index = count;
					fin_err = err_rate;
					fin_rate = new_rate;
					if (fin_err < error_2perc)
						break;
				}
			}
			if (fin_err < error_2perc)
				break;
		}
#endif
		/* Get the divisor by uart controller dll/dlm */
		divider = clk_div16_get_divider(parent_rate, rate);

		/* Get the best divider around calculated value */
		if (divider > 2) {
			for (i = divider - 2; i < (divider + 2); ++i) {
				new_rate = parent_rate/i;
				err_rate = abs(new_rate - rate);
				if (err_rate < fin_err) {
					final_index = count;
					fin_err = err_rate;
					fin_rate = parent_rate;
					if (fin_err < error_2perc)
						break;
				}
			}
			if (fin_err < error_2perc)
				break;
		}
	}

	if (final_index >= 0) {
		dev_info(t->uport.dev, "Setting clk_src %s\n",
				pdata->parent_clk_list[final_index].name);
		clk_set_parent(t->clk,
			pdata->parent_clk_list[final_index].parent_clk);
	}
	return fin_rate;
}

#define UART_CLOCK_ACCURACY 5
static void tegra_set_baudrate(struct tegra_uart_port *t, unsigned int baud)
{
	unsigned long rate;
	unsigned int divisor;
	unsigned char lcr;
	unsigned int baud_actual;
	unsigned int baud_delta;
	unsigned long best_rate;

	if (t->baud == baud)
		return;

	rate = baud * 16;
	best_rate = find_best_clock_source(t, rate);
	clk_set_rate(t->clk, best_rate);

	rate = clk_get_rate(t->clk);

	divisor = rate;
	do_div(divisor, 16);
	divisor += baud/2;
	do_div(divisor, baud);

	/* The allowable baudrate error from desired baudrate is 5% */
	baud_actual = divisor ? rate / (16 * divisor) : 0;
	baud_delta = abs(baud_actual - baud);
	if (WARN_ON(baud_delta * 20 > baud)) {
		dev_err(t->uport.dev, "requested baud %u, actual %u\n",
				baud, baud_actual);
	}

	lcr = t->lcr_shadow;
	lcr |= UART_LCR_DLAB;
	uart_writeb(t, lcr, UART_LCR);

	uart_writel(t, divisor & 0xFF, UART_TX);
	uart_writel(t, ((divisor >> 8) & 0xFF), UART_IER);

	lcr &= ~UART_LCR_DLAB;
	uart_writeb(t, lcr, UART_LCR);
	uart_readb(t, UART_SCR); /* Dummy read to ensure the write is posted */

	t->baud = baud;
	wait_sym_time(t, 2); /* wait two character intervals at new rate */
	dev_dbg(t->uport.dev, "Baud %u clock freq %lu and divisor of %u\n",
		baud, rate, divisor);
}

static void tegra_set_termios(struct uart_port *u, struct ktermios *termios,
					   struct ktermios *oldtermios)
{
	struct tegra_uart_port *t;
	unsigned int baud;
	unsigned long flags;
	unsigned int lcr;
	unsigned int c_cflag = termios->c_cflag;
	unsigned char mcr;

	t = container_of(u, struct tegra_uart_port, uport);
	dev_vdbg(t->uport.dev, "+tegra_set_termios\n");

	spin_lock_irqsave(&u->lock, flags);

	/* Changing configuration, it is safe to stop any rx now */
	if (t->rts_active)
		set_rts(t, false);

	/* Parity */
	lcr = t->lcr_shadow;
	lcr &= ~UART_LCR_PARITY;
	if (PARENB == (c_cflag & PARENB)) {
		if (CMSPAR == (c_cflag & CMSPAR)) {
			/* FIXME What is space parity? */
			/* data |= SPACE_PARITY; */
		} else if (c_cflag & PARODD) {
			lcr |= UART_LCR_PARITY;
			lcr &= ~UART_LCR_EPAR;
			lcr &= ~UART_LCR_SPAR;
		} else {
			lcr |= UART_LCR_PARITY;
			lcr |= UART_LCR_EPAR;
			lcr &= ~UART_LCR_SPAR;
		}
	}

	lcr &= ~UART_LCR_WLEN8;
	switch (c_cflag & CSIZE) {
	case CS5:
		lcr |= UART_LCR_WLEN5;
		break;
	case CS6:
		lcr |= UART_LCR_WLEN6;
		break;
	case CS7:
		lcr |= UART_LCR_WLEN7;
		break;
	default:
		lcr |= UART_LCR_WLEN8;
		break;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB)
		lcr |= UART_LCR_STOP;
	else
		lcr &= ~UART_LCR_STOP;

	uart_writeb(t, lcr, UART_LCR);
	t->lcr_shadow = lcr;

	/* Baud rate. */
	baud = uart_get_baud_rate(u, termios, oldtermios, 200, 4000000);
	spin_unlock_irqrestore(&u->lock, flags);
	tegra_set_baudrate(t, baud);
	spin_lock_irqsave(&u->lock, flags);

	/* Flow control */
	if (termios->c_cflag & CRTSCTS)	{
		mcr = t->mcr_shadow;
		mcr |= UART_MCR_CTS_EN;
		mcr &= ~UART_MCR_RTS_EN;
		t->mcr_shadow = mcr;
		uart_writeb(t, mcr, UART_MCR);
		t->use_cts_control = true;
		/* if top layer has asked to set rts active then do so here */
		if (t->rts_active)
			set_rts(t, true);
	} else {
		mcr = t->mcr_shadow;
		mcr &= ~UART_MCR_CTS_EN;
		mcr &= ~UART_MCR_RTS_EN;
		t->mcr_shadow = mcr;
		uart_writeb(t, mcr, UART_MCR);
		t->use_cts_control = false;
	}

	/* update the port timeout based on new settings */
	uart_update_timeout(u, termios->c_cflag, baud);

	spin_unlock_irqrestore(&u->lock, flags);
	dev_vdbg(t->uport.dev, "-tegra_set_termios\n");
	return;
}

/*
 * Flush any TX data submitted for DMA and PIO. Called when the
 * TX circular buffer is reset.
 */
static void tegra_flush_buffer(struct uart_port *u)
{
	struct tegra_uart_port *t;

	dev_vdbg(u->dev, "%s called", __func__);

	t = container_of(u, struct tegra_uart_port, uport);

	t->tx_bytes = 0;

	if (t->use_tx_dma) {
		tegra_dma_dequeue_req(t->tx_dma, &t->tx_dma_req);
		t->tx_dma_req.size = 0;
	}
	return;
}


static void tegra_pm(struct uart_port *u, unsigned int state,
	unsigned int oldstate)
{

}

static const char *tegra_type(struct uart_port *u)
{
	return TEGRA_UART_TYPE;
}

static struct uart_ops tegra_uart_ops = {
	.tx_empty	= tegra_tx_empty,
	.set_mctrl	= tegra_set_mctrl,
	.get_mctrl	= tegra_get_mctrl,
	.stop_tx	= tegra_stop_tx,
	.start_tx	= tegra_start_tx,
	.stop_rx	= tegra_stop_rx,
	.flush_buffer	= tegra_flush_buffer,
	.enable_ms	= tegra_enable_ms,
	.break_ctl	= tegra_break_ctl,
	.startup	= tegra_startup,
	.shutdown	= tegra_shutdown,
	.wake_peer	= tegra_wake_peer,
	.set_termios	= tegra_set_termios,
	.pm		= tegra_pm,
	.type		= tegra_type,
	.request_port	= tegra_request_port,
	.release_port	= tegra_release_port,
};

static struct uart_driver tegra_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "tegra_uart",
	.dev_name	= "ttyHS",
	.cons		= 0,
	.nr		= 5,
};

static int tegra_uart_probe(struct platform_device *pdev)
{
	struct tegra_uart_port *t;
	struct uart_port *u;
	struct resource *resource;
	int ret;
	char name[64];
	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr) {
		pr_err("Invalid Uart instance (%d)\n", pdev->id);
		return -ENODEV;
	}

	t = kzalloc(sizeof(struct tegra_uart_port), GFP_KERNEL);
	if (!t) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	u = &t->uport;
	u->dev = &pdev->dev;
	platform_set_drvdata(pdev, u);
	u->line = pdev->id;
	u->ops = &tegra_uart_ops;
	u->type = PORT_TEGRA;
	u->fifosize = 32;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource)) {
		ret = -ENXIO;
		goto fail;
	}

	u->mapbase = resource->start;
	u->membase = IO_ADDRESS(u->mapbase);
	if (unlikely(!u->membase)) {
		ret = -ENOMEM;
		goto fail;
	}
	u->iotype = UPIO_MEM32;

	u->irq = platform_get_irq(pdev, 0);
	if (unlikely(u->irq < 0)) {
		ret = -ENXIO;
		goto fail;
	}

	u->regshift = 2;

	t->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(t->clk)) {
		dev_err(&pdev->dev, "Couldn't get the clock\n");
		ret = -ENODEV;
		goto fail;
	}

	ret = uart_add_one_port(&tegra_uart_driver, u);
	if (ret) {
		pr_err("%s: Failed(%d) to add uart port %s%d\n",
			__func__, ret, tegra_uart_driver.dev_name, u->line);
		goto fail;
	}

	snprintf(name, sizeof(name), "tegra_hsuart_%d", u->line);
	pr_info("Registered UART port %s%d\n",
		tegra_uart_driver.dev_name, u->line);
	t->uart_state = TEGRA_UART_CLOSED;

	if (!RX_FORCE_PIO) {
		ret = tegra_uart_init_rx_dma_buffer(t);
		if (ret < 0) {
			pr_err("%s: Failed(%d) to allocate rx dma buffer "
				"%s%d\n", __func__, ret,
				tegra_uart_driver.dev_name, u->line);
			goto rx_dma_buff_fail;
		}
	}
	return ret;

rx_dma_buff_fail:
	uart_remove_one_port(&tegra_uart_driver, u);
fail:
	if (t->clk)
		clk_put(t->clk);
	platform_set_drvdata(pdev, NULL);
	kfree(t);
	return ret;
}

static int __devexit tegra_uart_remove(struct platform_device *pdev)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr)
		pr_err("Invalid Uart instance (%d)\n", pdev->id);

	u = &t->uport;
	uart_remove_one_port(&tegra_uart_driver, u);

	tegra_uart_free_rx_dma_buffer(t);

	platform_set_drvdata(pdev, NULL);

	pr_info("Unregistered UART port %s%d\n",
		tegra_uart_driver.dev_name, u->line);
	kfree(t);
	return 0;
}

static int tegra_uart_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr)
		pr_err("Invalid Uart instance (%d)\n", pdev->id);

	u = &t->uport;
	dev_dbg(t->uport.dev, "tegra_uart_suspend called\n");

	/* enable clock before calling suspend so that controller
	   register can be accessible */
	if (t->uart_state == TEGRA_UART_CLOCK_OFF) {
		clk_enable(t->clk);
		t->uart_state = TEGRA_UART_OPENED;
	}

	uart_suspend_port(&tegra_uart_driver, u);
	t->uart_state = TEGRA_UART_SUSPEND;

	return 0;
}

static int tegra_uart_resume(struct platform_device *pdev)
{
	struct tegra_uart_port *t = platform_get_drvdata(pdev);
	struct uart_port *u;

	if (pdev->id < 0 || pdev->id > tegra_uart_driver.nr)
		pr_err("Invalid Uart instance (%d)\n", pdev->id);

	u = &t->uport;
	dev_dbg(t->uport.dev, "tegra_uart_resume called\n");

	if (t->uart_state == TEGRA_UART_SUSPEND)
		uart_resume_port(&tegra_uart_driver, u);
	return 0;
}

/* Switch off the clock of the uart controller. */
void tegra_uart_request_clock_off(struct uart_port *uport)
{
	unsigned long flags;
	struct tegra_uart_port *t;
	bool is_clk_disable = false;

	if (IS_ERR_OR_NULL(uport))
		BUG();

	dev_vdbg(uport->dev, "tegra_uart_request_clock_off");

	t = container_of(uport, struct tegra_uart_port, uport);
	spin_lock_irqsave(&uport->lock, flags);
	if (t->uart_state == TEGRA_UART_OPENED) {
		is_clk_disable = true;
		t->uart_state = TEGRA_UART_CLOCK_OFF;
	}
	spin_unlock_irqrestore(&uport->lock, flags);

	if (is_clk_disable)
		clk_disable(t->clk);

	return;
}

/* Switch on the clock of the uart controller */
void tegra_uart_request_clock_on(struct uart_port *uport)
{
	unsigned long flags;
	struct tegra_uart_port *t;
	bool is_clk_enable = false;

	if (IS_ERR_OR_NULL(uport))
		BUG();

	t = container_of(uport, struct tegra_uart_port, uport);
	spin_lock_irqsave(&uport->lock, flags);
	if (t->uart_state == TEGRA_UART_CLOCK_OFF) {
		is_clk_enable = true;
		t->uart_state = TEGRA_UART_OPENED;
	}
	spin_unlock_irqrestore(&uport->lock, flags);

	if (is_clk_enable)
		clk_enable(t->clk);

	return;
}

/* Set the modem control signals state of uart controller. */
void tegra_uart_set_mctrl(struct uart_port *uport, unsigned int mctrl)
{
	unsigned long flags;
	struct tegra_uart_port *t;

	t = container_of(uport, struct tegra_uart_port, uport);
	if (t->uart_state != TEGRA_UART_OPENED) {
		dev_err(t->uport.dev, "Uart is in invalid state\n");
		return;
	}

	spin_lock_irqsave(&uport->lock, flags);
	if (mctrl & TIOCM_RTS) {
		t->rts_active = true;
		set_rts(t, true);
	} else {
		t->rts_active = false;
		set_rts(t, false);
	}

	if (mctrl & TIOCM_DTR)
		set_dtr(t, true);
	else
		set_dtr(t, false);
	spin_unlock_irqrestore(&uport->lock, flags);
	return;
}

/*
 * Return the status of the transmit fifo whether empty or not.
 * Return 0 if tx fifo is not empty.
 * Return TIOCSER_TEMT if tx fifo is empty.
 */
int tegra_uart_is_tx_empty(struct uart_port *uport)
{
	return tegra_tx_empty(uport);
}

static struct platform_driver tegra_uart_platform_driver = {
	.probe		= tegra_uart_probe,
	.remove		= __devexit_p(tegra_uart_remove),
	.suspend	= tegra_uart_suspend,
	.resume		= tegra_uart_resume,
	.driver		= {
		.name	= "tegra_uart"
	}
};

static int __init tegra_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&tegra_uart_driver);
	if (unlikely(ret)) {
		pr_err("Could not register %s driver\n",
			tegra_uart_driver.driver_name);
		return ret;
	}

	ret = platform_driver_register(&tegra_uart_platform_driver);
	if (unlikely(ret)) {
		pr_err("Could not register the UART platfrom driver\n");
		uart_unregister_driver(&tegra_uart_driver);
		return ret;
	}

	pr_info("Initialized tegra uart driver\n");
	return 0;
}

static void __exit tegra_uart_exit(void)
{
	pr_info("Unloading tegra uart driver\n");
	platform_driver_unregister(&tegra_uart_platform_driver);
	uart_unregister_driver(&tegra_uart_driver);
}

module_init(tegra_uart_init);
module_exit(tegra_uart_exit);
MODULE_DESCRIPTION("High speed UART driver for tegra chipset");
