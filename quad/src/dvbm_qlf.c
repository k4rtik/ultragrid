/* dvbm_qlf.c
 *
 * Linux driver functions for Linear Systems Ltd. DVB Master Q/i RoHS.
 *
 * Copyright (C) 2003-2008 Linear Systems Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either Version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public Licence for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Linear Systems can be contacted at <http://www.linsys.ca/>.
 *
 */

#include <linux/kernel.h> /* KERN_INFO */
#include <linux/module.h> /* THIS_MODULE */

#include <linux/fs.h> /* inode, file, file_operations */
#include <linux/sched.h> /* pt_regs */
#include <linux/pci.h> /* pci_dev */
#include <linux/slab.h> /* kmalloc () */
#include <linux/list.h> /* INIT_LIST_HEAD () */
#include <linux/spinlock.h> /* spin_lock_init () */
#include <linux/init.h> /* __devinit */
#include <linux/errno.h> /* error codes */
#include <linux/interrupt.h> /* irqreturn_t */
#include <linux/device.h> /* class_device_create_file () */

#include <asm/semaphore.h> /* sema_init () */
#include <asm/uaccess.h> /* put_user () */
#include <asm/bitops.h> /* set_bit () */

#include "asicore.h"
#include "../include/master.h"
#include "miface.h"
#include "mdev.h"
#include "dvbm.h"
#include "plx9080.h"
#include "masterplx.h"
#include "lsdma.h"
#include "masterlsdma.h"
#include "dvbm_qlf.h"

static const char dvbm_qlf_name[] = DVBM_NAME_QLF;
static const char dvbm_qie_name[] = DVBM_NAME_QIE;
static const char dvbm_lpqlf_name[] = DVBM_NAME_LPQLF;

/* Static function prototypes */
static ssize_t dvbm_qlf_show_uid (struct class_device *cd, char *buf);
static irqreturn_t IRQ_HANDLER(dvbm_qlf_irq_handler,irq,dev_id,regs);
static void dvbm_qlf_init (struct master_iface *iface);
static void dvbm_qlf_start (struct master_iface *iface);
static void dvbm_qlf_stop (struct master_iface *iface);
static void dvbm_qlf_exit (struct master_iface *iface);
static int dvbm_qlf_open (struct inode *inode, struct file *filp);
static long dvbm_qlf_unlocked_ioctl (struct file *filp,
	unsigned int cmd,
	unsigned long arg);
static int dvbm_qlf_ioctl (struct inode *inode,
	struct file *filp,
	unsigned int cmd,
	unsigned long arg);
static int dvbm_qlf_fsync (struct file *filp,
	struct dentry *dentry,
	int datasync);
static int dvbm_qlf_release (struct inode *inode, struct file *filp);

struct file_operations dvbm_qlf_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = masterlsdma_read,
	.poll = masterlsdma_rxpoll,
	.ioctl = dvbm_qlf_ioctl,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = dvbm_qlf_unlocked_ioctl,
#endif
#ifdef HAVE_COMPAT_IOCTL
	.compat_ioctl = asi_compat_ioctl,
#endif
	.open = dvbm_qlf_open,
	.release = dvbm_qlf_release,
	.fsync = dvbm_qlf_fsync,
	.fasync = NULL
};

/**
 * dvbm_qlf_show_uid - interface attribute read handler
 * @cd: class_device being read
 * @buf: output buffer
 **/
static ssize_t
dvbm_qlf_show_uid (struct class_device *cd,
	char *buf)
{
	struct master_dev *card = to_master_dev(cd);

	return snprintf (buf, PAGE_SIZE, "0x%08X%08X\n",
		readl (card->core.addr + DVBM_QLF_UIDR_HI),
		readl (card->core.addr + DVBM_QLF_UIDR_LO));
}

static CLASS_DEVICE_ATTR(uid,S_IRUGO,
	dvbm_qlf_show_uid,NULL);

/**
 * dvbm_qlf_pci_probe - PCI insertion handler for a DVB Master Q/i RoHS
 * @dev: PCI device
 *
 * Handle the insertion of a DVB Master Q/i RoHS.
 * Returns a negative error code on failure and 0 on success.
 **/
int __devinit
dvbm_qlf_pci_probe (struct pci_dev *dev)
{
	int err;
	unsigned int i, cap, transport;
	struct master_dev *card;
	const char *name;

	/* Allocate a board info structure */
	if ((card = (struct master_dev *)
		kmalloc (sizeof (*card), GFP_KERNEL)) == NULL) {
		err = -ENOMEM;
		goto NO_MEM;
	}

	/* Initialize the board info structure */
	memset (card, 0, sizeof (*card));
	/* PLX 9056 */
	switch (dev->device) {
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQIE:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF4:
		card->bridge_addr = ioremap_nocache
			(pci_resource_start (dev, 0),
			pci_resource_len (dev, 0));
		break;
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_MINIBNC:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_HEADER:
		card->bridge_addr = ioremap_nocache
			(pci_resource_start (dev, 2),
			pci_resource_len (dev, 2));
		break;
	}

	/* ASI Core */
	switch (dev->device) {
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQIE:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF4:
		card->core.addr = ioremap_nocache
			(pci_resource_start (dev, 2),
			pci_resource_len (dev, 2));
		break;
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_MINIBNC:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_HEADER:
		card->core.addr = ioremap_nocache
			(pci_resource_start (dev, 0),
			pci_resource_len (dev, 0));
		break;
	}
	card->version = readl (card->core.addr + DVBM_QLF_FPGAID) & 0xffff;
	switch (dev->device) {
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF4:
		name = dvbm_qlf_name;
		break;
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQIE:
		name = dvbm_qie_name;
		break;
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_MINIBNC:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_HEADER:
		name = dvbm_lpqlf_name;
		break;
	default:
		name = "";
		break;
	}
	card->name = name;
	card->irq_handler = dvbm_qlf_irq_handler;
	INIT_LIST_HEAD(&card->iface_list);
	switch (dev->device) {
	default:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQIE:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF4:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_MINIBNC:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_HEADER:
		card->capabilities = MASTER_CAP_UID;
		break;
	}
	/* Lock for LSDMA_CSR, ICSR */
	spin_lock_init (&card->irq_lock);
	/* Lock for PFLUT, RCR */
	spin_lock_init (&card->reg_lock);
	sema_init (&card->users_sem, 1);
	card->pdev = dev;

	/* Print the firmware version */
	printk (KERN_INFO "%s: %s detected, firmware version %u.%u (0x%04X)\n",
		dvbm_driver_name, name,
		card->version >> 8, card->version & 0x00ff, card->version);

	/* Store the pointer to the board info structure
	 * in the PCI info structure */
	pci_set_drvdata (dev, card);

	switch (dev->device) {
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQIE:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF4:
		/* Reset the PCI 9056 */
		masterplx_reset_bridge (card);

		/* Setup the PCI 9056 */
		writel (PLX_INTCSR_PCIINT_ENABLE |
			PLX_INTCSR_PCILOCINT_ENABLE,
			card->bridge_addr + PLX_INTCSR);
		/* Dummy read to flush PCI posted writes */
		readl (card->bridge_addr + PLX_INTCSR);

		/* Remap bridge address to the DMA controller */
		iounmap (card->bridge_addr);
		/* LS DMA Controller */
		card->bridge_addr = ioremap_nocache (pci_resource_start (dev, 3),
			pci_resource_len (dev, 3));
		break;
	default:
		break;
	}

	/* Reset the FPGA */
	for (i = 0; i < 4; i++) {
		writel (DVBM_QLF_RCR_RST, card->core.addr + DVBM_QLF_RCR(i));
	}

	/* Setup the LS DMA controller */
	writel (LSDMA_INTMSK_CH(0) | LSDMA_INTMSK_CH(1) | LSDMA_INTMSK_CH(2)
		| LSDMA_INTMSK_CH(3),
		card->bridge_addr + LSDMA_INTMSK);
	for (i = 0; i < 4; i++) {
		writel (LSDMA_CH_CSR_INTDONEENABLE |
			LSDMA_CH_CSR_INTSTOPENABLE |
			LSDMA_CH_CSR_DIRECTION,
			card->bridge_addr + LSDMA_CSR(i));
	}

	/* Dummy read to flush PCI posted writes */
	readl (card->bridge_addr + LSDMA_INTMSK);

	/* Register a Master device */
	if ((err = mdev_register (card,
		&dvbm_card_list,
		dvbm_driver_name,
		&dvbm_class)) < 0) {
		goto NO_DEV;
	}

	/* Add class_device attributes */
	if (card->capabilities & MASTER_CAP_UID) {
		if ((err = class_device_create_file (&card->class_dev,
			&class_device_attr_uid)) < 0) {
			printk (KERN_WARNING
				"%s: Unable to create file 'uid'\n",
				dvbm_driver_name);
		}
	}

	/* Setup device capabilities */
	cap = ASI_CAP_RX_DATA;
	switch (dev->device) {
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQIE:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBQLF4:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_MINIBNC:
	case DVBM_PCI_DEVICE_ID_LINSYS_DVBLPQLF_HEADER:
		cap |= ASI_CAP_RX_SYNC | ASI_CAP_RX_CD | ASI_CAP_RX_MAKE188 |
		ASI_CAP_RX_BYTECOUNTER | ASI_CAP_RX_PIDFILTER |
		ASI_CAP_RX_TIMESTAMPS | ASI_CAP_RX_INVSYNC |
		ASI_CAP_RX_PTIMESTAMPS | ASI_CAP_RX_PIDCOUNTER |
		ASI_CAP_RX_4PIDCOUNTER | ASI_CAP_RX_NULLPACKETS |
		ASI_CAP_RX_27COUNTER;
		break;
	default:
		transport = 0xff;
		break;
	}
	/* Register receiver interfaces */
	for (i = 0; i < 4; i++) {
		if ((err = asi_register_iface (card,
			MASTER_DIRECTION_RX,
			&dvbm_qlf_fops,
			cap,
			4,
			ASI_CTL_TRANSPORT_DVB_ASI)) < 0) {
			goto NO_IFACE;
		}
	}

	return 0;

NO_IFACE:
	dvbm_pci_remove (dev);
NO_DEV:
NO_MEM:
	return err;
}

/**
 * dvbm_qlf_pci_remove - PCI removal handler for a DVB Master Q/i RoHS
 * @card: Master device
 *
 * Handle the removal of a DVB Master Q/i RoHS.
 **/
void
dvbm_qlf_pci_remove (struct master_dev *card)
{
	iounmap (card->core.addr);
	return;
}

/**
 * dvbm_qlf_irq_handler - DVB Master Q/i RoHS interrupt service routine
 * @irq: interrupt number
 * @dev_id: pointer to the device data structure
 * @regs: processor context
 **/
static irqreturn_t
IRQ_HANDLER(dvbm_qlf_irq_handler,irq,dev_id,regs)
{
	struct master_dev *card = dev_id;
	struct list_head *p = &card->iface_list;
	struct master_iface *iface;
	unsigned int dmaintsrc = readl (card->bridge_addr + LSDMA_INTSRC);
	unsigned int status, interrupting_iface = 0, i;

	for (i = 0; i < 4; i++) {
		p = p->next;
		iface = list_entry (p, struct master_iface, list);

		/* Clear ASI interrupts */
		spin_lock (&card->irq_lock);
		status = readl (card->core.addr + DVBM_QLF_ICSR(i));
		if ((status & DVBM_QLF_ICSR_ISMASK) != 0) {
			writel (status, card->core.addr + DVBM_QLF_ICSR(i));
			spin_unlock (&card->irq_lock);
			if (status & DVBM_QLF_ICSR_RXCDIS) {
				set_bit (ASI_EVENT_RX_CARRIER_ORDER,
					&iface->events);
				interrupting_iface |= (0x1 << i);
			}
			if (status & DVBM_QLF_ICSR_RXAOSIS) {
				set_bit (ASI_EVENT_RX_AOS_ORDER,
					&iface->events);
				interrupting_iface |= (0x1 << i);
			}
			if (status & DVBM_QLF_ICSR_RXLOSIS) {
				set_bit (ASI_EVENT_RX_LOS_ORDER,
					&iface->events);
				interrupting_iface |= (0x1 << i);
			}
			if (status & DVBM_QLF_ICSR_RXOIS) {
				set_bit (ASI_EVENT_RX_FIFO_ORDER,
					&iface->events);
				interrupting_iface |= (0x1 << i);
			}
			if (status & DVBM_QLF_ICSR_RXDIS) {
				set_bit (ASI_EVENT_RX_DATA_ORDER,
					&iface->events);
				interrupting_iface |= (0x1 << i);
			}
		} else {
			spin_unlock (&card->irq_lock);
		}

		/* Clear DMA interrupts */
		if (dmaintsrc & LSDMA_INTSRC_CH(i)) {
			/* Read the interrupt type and clear it */
			spin_lock (&card->irq_lock);
			status = readl (card->bridge_addr + LSDMA_CSR(i));
			writel (status, card->bridge_addr + LSDMA_CSR(i));
			spin_unlock (&card->irq_lock);

			/* Increment the buffer pointer */
			if (status & LSDMA_CH_CSR_INTSRCBUFFER) {
				lsdma_advance (iface->dma);
				if (lsdma_rx_isempty (iface->dma)) {
					set_bit (ASI_EVENT_RX_BUFFER_ORDER,
						&iface->events);
				}
			}

			/* Flag end-of-chain */
			if (status & LSDMA_CH_CSR_INTSRCDONE) {
				set_bit (0, &iface->dma_done);
			}

			/* Flag DMA abort */
			if (status & LSDMA_CH_CSR_INTSRCSTOP) {
				set_bit (0, &iface->dma_done);
			}

			interrupting_iface |= (0x1 << i);
		}

		if (interrupting_iface & (0x1 << i)) {
			wake_up (&iface->queue);
		}
	}

	if (interrupting_iface) {
		/* Dummy read to flush PCI posted writes */
		readl (card->bridge_addr + LSDMA_INTMSK);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/**
 * dvbm_qlf_init - Initialize a DVB Master Q/i RoHS receiver
 * @iface: interface
 **/
static void
dvbm_qlf_init (struct master_iface *iface)
{
	struct master_dev *card = iface->card;
	const unsigned int channel = mdev_index (card, &iface->list);
	unsigned int i, reg = (iface->null_packets ? DVBM_QLF_RCR_RNP : 0);

	switch (iface->timestamps) {
	default:
	case ASI_CTL_TSTAMP_NONE:
		reg |= 0;
		break;
	case ASI_CTL_TSTAMP_APPEND:
		reg |= DVBM_QLF_RCR_TSE;
		break;
	case ASI_CTL_TSTAMP_PREPEND:
		reg |= DVBM_QLF_RCR_PTSE;
		break;
	}
	switch (iface->mode) {
	default:
	case ASI_CTL_RX_MODE_RAW:
		reg |= 0;
		break;
	case ASI_CTL_RX_MODE_188:
		reg |= DVBM_QLF_RCR_188 | DVBM_QLF_RCR_PFE;
		break;
	case ASI_CTL_RX_MODE_204:
		reg |= DVBM_QLF_RCR_204 | DVBM_QLF_RCR_PFE;
		break;
	case ASI_CTL_RX_MODE_AUTO:
		reg |= DVBM_QLF_RCR_AUTO | DVBM_QLF_RCR_PFE;
		break;
	case ASI_CTL_RX_MODE_AUTOMAKE188:
		reg |= DVBM_QLF_RCR_AUTO | DVBM_QLF_RCR_RSS |
			DVBM_QLF_RCR_PFE;
		break;
	case ASI_CTL_RX_MODE_204MAKE188:
		reg |= DVBM_QLF_RCR_204 | DVBM_QLF_RCR_RSS |
			DVBM_QLF_RCR_PFE;
		break;
	}

	/* There will be no races on RCR
	 * until this code returns, so we don't need to lock it */
	writel (reg | DVBM_QLF_RCR_RST,
		card->core.addr + DVBM_QLF_RCR(channel));
	wmb ();
	writel (reg, card->core.addr + DVBM_QLF_RCR(channel));
	wmb ();
	writel (DVBM_QLF_RDMATL, card->core.addr + DVBM_QLF_RDMATLR(channel));

	/* Reset the byte counter */
	readl (card->core.addr + DVBM_QLF_RXBCOUNTR(channel));

	/* Reset the PID filter.
	 * There will be no races on PFLUT
	 * until this code returns, so we don't need to lock it */
	for (i = 0; i < 256; i++) {
		writel (i, card->core.addr + DVBM_QLF_PFLUTAR(channel));
		/* Dummy read to flush PCI posted writes */
		readl (card->core.addr + DVBM_QLF_FPGAID);
		writel (0xffffffff, card->core.addr + DVBM_QLF_PFLUTR(channel));
		/* Dummy read to flush PCI posted writes */
		readl (card->core.addr + DVBM_QLF_FPGAID);
	}

	/* Clear PID registers */
	writel (0, card->core.addr + DVBM_QLF_PIDR0(channel));
	writel (0, card->core.addr + DVBM_QLF_PIDR1(channel));
	writel (0, card->core.addr + DVBM_QLF_PIDR2(channel));
	writel (0, card->core.addr + DVBM_QLF_PIDR3(channel));

	/* Reset PID counters */
	readl (card->core.addr + DVBM_QLF_PIDCOUNTR0(channel));
	readl (card->core.addr + DVBM_QLF_PIDCOUNTR1(channel));
	readl (card->core.addr + DVBM_QLF_PIDCOUNTR2(channel));
	readl (card->core.addr + DVBM_QLF_PIDCOUNTR3(channel));

	return;
}

/**
 * dvbm_qlf_start - Activate the DVB Master Q/i RoHS receiver
 * @iface: interface
 **/
static void
dvbm_qlf_start (struct master_iface *iface)
{
	struct master_dev *card = iface->card;
	const unsigned int channel = mdev_index (card, &iface->list);
	unsigned int reg;

	/* Enable and start DMA */
	writel (lsdma_dma_to_desc_low (lsdma_head_desc_bus_addr (iface->dma)),
		card->bridge_addr + LSDMA_DESC(channel));
	clear_bit (0, &iface->dma_done);
	wmb ();
	writel (LSDMA_CH_CSR_INTDONEENABLE | LSDMA_CH_CSR_INTSTOPENABLE |
		LSDMA_CH_CSR_DIRECTION | LSDMA_CH_CSR_ENABLE,
		card->bridge_addr + LSDMA_CSR(channel));
	/* Dummy read to flush PCI posted writes */
	readl (card->bridge_addr + LSDMA_INTMSK);

	/* Enable receiver interrupts */
	spin_lock_irq (&card->irq_lock);
	writel (DVBM_QLF_ICSR_RXCDIE | DVBM_QLF_ICSR_RXAOSIE |
		DVBM_QLF_ICSR_RXLOSIE | DVBM_QLF_ICSR_RXOIE |
		DVBM_QLF_ICSR_RXDIE,
		card->core.addr + DVBM_QLF_ICSR(channel));
	spin_unlock_irq (&card->irq_lock);

	/* Enable the receiver */
	spin_lock (&card->reg_lock);
	reg = readl (card->core.addr + DVBM_QLF_RCR(channel));
	writel (reg | DVBM_QLF_RCR_EN,
		card->core.addr + DVBM_QLF_RCR(channel));
	spin_unlock (&card->reg_lock);

	return;
}

/**
 * dvbm_qlf_stop - Deactivate the DVB Master Q/i RoHS receiver
 * @iface: interface
 **/
static void
dvbm_qlf_stop (struct master_iface *iface)
{
	struct master_dev *card = iface->card;
	const unsigned int channel = mdev_index (card, &iface->list);
	unsigned int reg;

	/* Disable the receiver */
	spin_lock (&card->reg_lock);
	reg = readl (card->core.addr + DVBM_QLF_RCR(channel));
	writel (reg & ~DVBM_QLF_RCR_EN,
		card->core.addr + DVBM_QLF_RCR(channel));
	spin_unlock (&card->reg_lock);

	/* Disable receiver interrupts */
	spin_lock_irq (&card->irq_lock);
	writel (DVBM_QLF_ICSR_RXCDIS | DVBM_QLF_ICSR_RXAOSIS |
		DVBM_QLF_ICSR_RXLOSIS | DVBM_QLF_ICSR_RXOIS |
		DVBM_QLF_ICSR_RXDIS,
		card->core.addr + DVBM_QLF_ICSR(channel));

	/* Disable and abort DMA */
	writel (LSDMA_CH_CSR_INTDONEENABLE | LSDMA_CH_CSR_INTSTOPENABLE |
		LSDMA_CH_CSR_DIRECTION,
		card->bridge_addr + LSDMA_CSR(channel));
	wmb ();
	writel (LSDMA_CH_CSR_INTDONEENABLE | LSDMA_CH_CSR_INTSTOPENABLE |
		LSDMA_CH_CSR_DIRECTION | LSDMA_CH_CSR_STOP,
		card->bridge_addr + LSDMA_CSR(channel));
	/* Dummy read to flush PCI posted writes */
	readl (card->bridge_addr + LSDMA_INTMSK);
	spin_unlock_irq (&card->irq_lock);
	wait_event (iface->queue, test_bit (0, &iface->dma_done));
	writel (LSDMA_CH_CSR_INTDONEENABLE | LSDMA_CH_CSR_INTSTOPENABLE |
		LSDMA_CH_CSR_DIRECTION,
		card->bridge_addr + LSDMA_CSR(channel));

	return;
}

/**
 * dvbm_qlf_exit - Clean up the DVB Master Q/i RoHS receiver
 * @iface: interface
 **/
static void
dvbm_qlf_exit (struct master_iface *iface)
{
	struct master_dev *card = iface->card;
	const unsigned int channel = mdev_index (card, &iface->list);

	/* Reset the receiver.
	 * There will be no races on RCR here,
	 * so we don't need to lock it */
	writel (DVBM_QLF_RCR_RST, card->core.addr + DVBM_QLF_RCR(channel));

	return;
}

/**
 * dvbm_qlf_open - DVB Master Q/i RoHS receiver open() method
 * @inode: inode
 * @filp: file
 *
 * Returns a negative error code on failure and 0 on success.
 **/
static int
dvbm_qlf_open (struct inode *inode, struct file *filp)
{
	return masterlsdma_open (inode,
		filp,
		dvbm_qlf_init,
		dvbm_qlf_start,
		0,
		0);
}

/**
 * dvbm_qlf_unlocked_ioctl - DVB Master Q/i RoHS receiver unlocked_ioctl() method
 * @filp: file
 * @cmd: ioctl command
 * @arg: ioctl argument
 *
 * Returns a negative error code on failure and 0 on success.
 **/
static long
dvbm_qlf_unlocked_ioctl (struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	struct master_iface *iface = filp->private_data;
	struct master_dev *card = iface->card;
	const unsigned int channel = mdev_index (card, &iface->list);
	int val;
	unsigned int reg = 0, pflut[256], i;

	switch (cmd) {
	case ASI_IOC_RXGETBUFLEVEL:
		if (put_user (lsdma_rx_buflevel (iface->dma),
			(unsigned int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXGETSTATUS:
		/* Atomic reads of ICSR and RCR, so we don't need to lock */
		reg = readl (card->core.addr + DVBM_QLF_ICSR(channel));
		switch (readl (card->core.addr + DVBM_QLF_RCR(channel)) &
			DVBM_QLF_RCR_SYNC_MASK) {
		case 0:
			val = 1;
			break;
		case DVBM_QLF_RCR_188:
			val = (reg & DVBM_QLF_ICSR_RXPASSING) ? 188 : 0;
			break;
		case DVBM_QLF_RCR_204:
			val = (reg & DVBM_QLF_ICSR_RXPASSING) ? 204 : 0;
			break;
		case DVBM_QLF_RCR_AUTO:
			if (reg & DVBM_QLF_ICSR_RXPASSING) {
				val = (reg & DVBM_QLF_ICSR_RX204) ? 204 : 188;
			} else {
				val = 0;
			}
			break;
		default:
			return -EIO;
		}
		if (put_user (val, (int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXGETBYTECOUNT:
		if (!(iface->capabilities & ASI_CAP_RX_BYTECOUNTER)) {
			return -ENOTTY;
		}
		if (put_user (readl (card->core.addr +
			DVBM_QLF_RXBCOUNTR(channel)),
			(unsigned int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXSETINVSYNC:
		if (get_user (val, (int *)arg)) {
			return -EFAULT;
		}
		switch (val) {
		case 0:
			reg |= 0;
			break;
		case 1:
			reg |= DVBM_QLF_RCR_INVSYNC;
			break;
		default:
			return -EINVAL;
		}
		spin_lock (&card->reg_lock);
		writel ((readl (card->core.addr + DVBM_QLF_RCR(channel)) &
			~DVBM_QLF_RCR_INVSYNC) | reg,
			card->core.addr + DVBM_QLF_RCR(channel));
		spin_unlock (&card->reg_lock);
		break;
	case ASI_IOC_RXGETCARRIER:
		/* Atomic read of ICSR, so we don't need to lock */
		if (put_user (
			(readl (card->core.addr + DVBM_QLF_ICSR(channel)) &
			DVBM_QLF_ICSR_RXCD) ? 1 : 0, (int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXGETRXD:
		/* Atomic read of ICSR, so we don't need to lock */
		if (put_user (
			(readl (card->core.addr + DVBM_QLF_ICSR(channel)) &
			DVBM_QLF_ICSR_RXD) ? 1 : 0, (int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXSETPF:
		if (!(iface->capabilities & ASI_CAP_RX_PIDFILTER)) {
			return -ENOTTY;
		}
		if (copy_from_user (pflut, (unsigned int *)arg,
			sizeof (unsigned int [256]))) {
			return -EFAULT;
		}
		spin_lock (&card->reg_lock);
		for (i = 0; i < 256; i++) {
			writel (i, card->core.addr + DVBM_QLF_PFLUTAR(channel));
			/* Dummy read to flush PCI posted writes */
			readl (card->core.addr + DVBM_QLF_FPGAID);
			writel (pflut[i], card->core.addr + DVBM_QLF_PFLUTR(channel));
			/* Dummy read to flush PCI posted writes */
			readl (card->core.addr + DVBM_QLF_FPGAID);
		}
		spin_unlock (&card->reg_lock);
		break;
	case ASI_IOC_RXSETPID0:
		if (!(iface->capabilities & ASI_CAP_RX_PIDCOUNTER)) {
			return -ENOTTY;
		}
		if (get_user (val, (int *)arg)) {
			return -EFAULT;
		}
		if ((val < 0) || (val > 0x00001fff)) {
			return -EINVAL;
		}
		writel (val, card->core.addr + DVBM_QLF_PIDR0(channel));
		/* Reset PID count */
		readl (card->core.addr + DVBM_QLF_PIDCOUNTR0(channel));
		break;
	case ASI_IOC_RXGETPID0COUNT:
		if (!(iface->capabilities & ASI_CAP_RX_PIDCOUNTER)) {
			return -ENOTTY;
		}
		if (put_user (readl (card->core.addr +
			DVBM_QLF_PIDCOUNTR0(channel)),
			(unsigned int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXSETPID1:
		if (!(iface->capabilities & ASI_CAP_RX_4PIDCOUNTER)) {
			return -ENOTTY;
		}
		if (get_user (val, (int *)arg)) {
			return -EFAULT;
		}
		if ((val < 0) || (val > 0x00001fff)) {
			return -EINVAL;
		}
		writel (val, card->core.addr + DVBM_QLF_PIDR1(channel));
		/* Reset PID count */
		readl (card->core.addr + DVBM_QLF_PIDCOUNTR1(channel));
		break;
	case ASI_IOC_RXGETPID1COUNT:
		if (!(iface->capabilities & ASI_CAP_RX_4PIDCOUNTER)) {
			return -ENOTTY;
		}
		if (put_user (readl (card->core.addr +
			DVBM_QLF_PIDCOUNTR1(channel)),
			(unsigned int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXSETPID2:
		if (!(iface->capabilities & ASI_CAP_RX_4PIDCOUNTER)) {
			return -ENOTTY;
		}
		if (get_user (val, (int *)arg)) {
			return -EFAULT;
		}
		if ((val < 0) || (val > 0x00001fff)) {
			return -EINVAL;
		}
		writel (val, card->core.addr + DVBM_QLF_PIDR2(channel));
		/* Reset PID count */
		readl (card->core.addr + DVBM_QLF_PIDCOUNTR2(channel));
		break;
	case ASI_IOC_RXGETPID2COUNT:
		if (!(iface->capabilities & ASI_CAP_RX_4PIDCOUNTER)) {
			return -ENOTTY;
		}
		if (put_user (readl (card->core.addr +
			DVBM_QLF_PIDCOUNTR2(channel)),
			(unsigned int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXSETPID3:
		if (!(iface->capabilities & ASI_CAP_RX_4PIDCOUNTER)) {
			return -ENOTTY;
		}
		if (get_user (val, (int *)arg)) {
			return -EFAULT;
		}
		if ((val < 0) || (val > 0x00001fff)) {
			return -EINVAL;
		}
		writel (val, card->core.addr + DVBM_QLF_PIDR3(channel));
		/* Reset PID count */
		readl (card->core.addr + DVBM_QLF_PIDCOUNTR3(channel));
		break;
	case ASI_IOC_RXGETPID3COUNT:
		if (!(iface->capabilities & ASI_CAP_RX_4PIDCOUNTER)) {
			return -ENOTTY;
		}
		if (put_user (readl (card->core.addr +
			DVBM_QLF_PIDCOUNTR3(channel)),
			(unsigned int *)arg)) {
			return -EFAULT;
		}
		break;
	case ASI_IOC_RXGET27COUNT:
		if (!(iface->capabilities & ASI_CAP_RX_27COUNTER)) {
			return -ENOTTY;
		}
		if (put_user (readl (card->core.addr + DVBM_QLF_27COUNTR),
			(unsigned int *)arg)) {
			return -EFAULT;
		}
		break;
	default:
		return asi_rxioctl (iface, cmd, arg);
	}
	return 0;
}

/**
 * dvbm_qlf_ioctl - DVB Master Q/i RoHS receiver ioctl() method
 * @inode: inode
 * @filp: file
 * @cmd: ioctl command
 * @arg: ioctl argument
 *
 * Returns a negative error code on failure and 0 on success.
 **/
static int
dvbm_qlf_ioctl (struct inode *inode,
	struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	return dvbm_qlf_unlocked_ioctl (filp, cmd, arg);
}

/**
 * dvbm_qlf_fsync - DVB Master Q/i RoHS receiver fsync() method
 * @filp: file to flush
 * @dentry: directory entry associated with the file
 * @datasync: used by filesystems
 *
 * Returns a negative error code on failure and 0 on success.
 **/
static int
dvbm_qlf_fsync (struct file *filp,
	struct dentry *dentry,
	int datasync)
{
	struct master_iface *iface = filp->private_data;
	struct master_dev *card = iface->card;
	const unsigned int channel = mdev_index (card, &iface->list);
	unsigned int reg;

	if (down_interruptible (&iface->buf_sem)) {
		return -ERESTARTSYS;
	}

	/* Stop the receiver */
	dvbm_qlf_stop (iface);

	/* Reset the onboard FIFO and driver buffers */
	spin_lock (&card->reg_lock);
	reg = readl (card->core.addr + DVBM_QLF_RCR(channel));
	writel (reg | DVBM_QLF_RCR_RST,
		card->core.addr + DVBM_QLF_RCR(channel));
	wmb ();
	writel (reg, card->core.addr + DVBM_QLF_RCR(channel));
	spin_unlock (&card->reg_lock);
	iface->events = 0;
	lsdma_reset (iface->dma);

	/* Start the receiver */
	dvbm_qlf_start (iface);

	up (&iface->buf_sem);
	return 0;
}

/**
 * dvbm_qlf_release - DVB Master Q/i RoHS receiver release() method
 * @inode: inode
 * @filp: file
 *
 * Returns a negative error code on failure and 0 on success.
 **/
static int
dvbm_qlf_release (struct inode *inode, struct file *filp)
{
	struct master_iface *iface = filp->private_data;

	return masterlsdma_release (iface, dvbm_qlf_stop, dvbm_qlf_exit);
}
