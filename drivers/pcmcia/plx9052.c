/*======================================================================

    Device driver for PLX9052 PCI-to-PCMCIA bridges.

    (C) 2003 Pavel Roskin <proski@gnu.org>
    (C) 2011 Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>

    The contents of this file are subject to the Mozilla Public
    License Version 1.1 (the "License"); you may not use this file
    except in compliance with the License. You may obtain a copy of
    the License at http://www.mozilla.org/MPL/

    Software distributed under the License is distributed on an "AS
    IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
    implied. See the License for the specific language governing
    rights and limitations under the License.

    Alternatively, the contents of this file may be used under the
    terms of the GNU General Public License version 2 (the "GPL"), in
    which case the provisions of the GPL are applicable instead of the
    above.  If you wish to allow the use of your version of this file
    only under the terms of the GPL and not to allow others to use
    your version of this file under the MPL, indicate your decision
    by deleting the provisions above and replace them with the notice
    and other provisions required by the GPL.  If you do not delete
    the provisions above, a recipient may use your version of this
    file under either the MPL or the GPL.

======================================================================*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

#include <pcmcia/ss.h>

/*====================================================================*/

/* Module parameters */

MODULE_AUTHOR("Pavel Roskin <proski@gnu.org>");
MODULE_DESCRIPTION("PLX9052 PCMCIA socket driver");
MODULE_LICENSE("Dual MPL/GPL");

/*====================================================================*/

#define DEVICE_NAME "plx9052"

#define PLX_IOWIN_MIN	0x04	/* Minimal I/O window */
#define PLX_IOWIN_MAX	0x1000	/* Maximal I/O window */
#define PLX_MEMWIN_MIN	0x10	/* Minimal memory window */
#define PLX_MEMWIN_MAX	0x80000	/* Maximal memory window */

#define PLX_CIS_START	0x03000000	/* Start of CIS in PLX local memory */

/*
 * PLX9052 has 4 local address spaces.
 * The driver allocates them dynamically.
 */
#define PLX_AREAS	4	/* Number of local address spaces (areas) */
#define PLX_AREA_USED	0x20	/* Area is used */
#define PLX_AREA_IO	0x10	/* Area is used for I/O window */
#define PLX_AREA_MAP_MASK	0x0f	/* Map number */


/* PCI base address for corresponding local address space */
#define PLX_PCI_BASE(x)	(PCI_BASE_ADDRESS_2 + (x << 2))


/* PLX9052 registers */

/* Local address space range (0 <= x < PLX_AREAS) */
#define PLX_LASRR(x)	(0x00 + (x << 2))
#define PLX_LASRR_IO	1	/* Range is used as I/O space */
#define PLX_LAS_MEMMASK	0x0ffffff0	/* Mask for memory address lines */
#define PLX_LAS_IOMASK	0x0ffffffc	/* Mask for I/O address lines */

/* Local address space base address (0 <= x < PLX_AREAS) */
#define PLX_LASBA(x)	(0x14 + (x << 2))
#define PLX_LASBA_DISABLE 0	/* Disable address decoding */
#define PLX_LASBA_ENABLE  1	/* Enable address decoding */

/* Local address space bus region descriptors (0 <= x < PLX_AREAS) */
#define PLX_LASBRD(x)	(0x28 + (x << 2))
#define PLX_LASBRD_WIDTH_MASK	0x00c00000
#define PLX_LASBRD_WIDTH_8	0x00000000
#define PLX_LASBRD_WIDTH_16	0x00400000
#define PLX_LASBRD_WIDTH_32	0x00800000
#define PLX_LASBRD_WS_MASK	0x003fffc0
#define PLX_LASBRD_WS0		0x00000000
#define PLX_LASBRD_WS1		0x0010a840

/* Chip select base address (0 <= x < PLX_AREAS) */
#define PLX_CSBASE(x)	(0x3c + (x << 2))
#define PLX_CSBASE_ENABLE	1

#define PLX_INTCSR	0x4c	/* Interrupt Control and Status Register */

#define PLX_INT1_ENABLE	0x40	/* Interrupt 1 Enable bit */
#define PLX_INT1_ACTIVE	0x04	/* Interrupt 1 Active bit */

#define PLX_CNTRL	0x50	/* Control register */
#define PLX_CNTRL_RESET	0x40000000	/* Reset local bus */


#ifdef __IN_PCMCIA_PACKAGE__
socket_state_t dead_socket = {
	0, SS_DETECT, 0, 0, 0
};
#endif


struct plx9052_socket {
	void (*handler) (void *info, u_int events);
	void *info;

	socket_state_t state;
	struct pci_dev *pdev;
	struct pcmcia_socket socket;

	/* PCI resource 1, PLX9052 registers */
	unsigned int plxctl_addr;
	unsigned int plxctl_len;	/* should be 0x80 */

	/* PCI resource 2, mapped to PCMCIA memory space */
	unsigned long mem_phys;
	unsigned long mem_len;	/* should be 0x1000 */
	u8 *mem_virt;

	char area_used[PLX_AREAS];	/* Used local address spaces */

	/* Event processing */
	u_int event;
	spinlock_t event_lock;
	struct work_struct event_work;
};


/* Forward declarations */
static void plx9052_close(struct pci_dev *pdev);
static int plx9052_set_socket(struct pcmcia_socket *sock,
			      socket_state_t * state);


/* Check that an area is size-aligned.
 * Return 0 for good areas, -1 for bad ones.  */
static int plx9052_align_check(u32 base, u32 len)
{
	int power = 0;
	int l = len;

	if (!len)
		return -1;

	while (l >>= 1)
		power++;

	if (len != (1 << power))
		return -1;

	if (base & ((1 << power) - 1))
		return -1;

	return 0;
}


static inline void plx9052_outl(struct plx9052_socket *socket, u32 val,
				unsigned addr)
{
	outl_p(val, socket->plxctl_addr + addr);
}


static inline u32 plx9052_inl(struct plx9052_socket *socket, unsigned addr)
{
	return inl_p(socket->plxctl_addr + addr);
}


static inline void plx9052_enable_irq(struct plx9052_socket *socket)
{
	u32 reg;

	reg = plx9052_inl(socket, PLX_INTCSR);
	if (!(reg & PLX_INT1_ENABLE)) {
		reg |= PLX_INT1_ENABLE;
		plx9052_outl(socket, reg, PLX_INTCSR);
	}
}


static inline void plx9052_disable_irq(struct plx9052_socket *socket)
{
	u32 reg;

	reg = plx9052_inl(socket, PLX_INTCSR);
	if (reg & PLX_INT1_ENABLE) {
		reg &= ~PLX_INT1_ENABLE;
		plx9052_outl(socket, reg, PLX_INTCSR);
	}
}


static inline int plx9052_irq_active(struct plx9052_socket *socket)
{
	u32 reg;

	reg = plx9052_inl(socket, PLX_INTCSR);
	return (reg & PLX_INT1_ACTIVE);
}


/* Hack - determine card presence by the first byte of CIS */
static inline int plx9052_card_present(struct plx9052_socket *socket)
{
	return (((volatile u8 *) socket->mem_virt)[0] == 1);
}


static void plx9052_event(struct work_struct *work)
{
	struct plx9052_socket *socket =
		container_of(work, struct plx9052_socket, event_work);
	u_int event;

	if (!socket->handler)
		return;

	spin_lock_irq(&socket->event_lock);
	event = socket->event;
	socket->event = 0;
	spin_unlock_irq(&socket->event_lock);

	if (!event)
		return;

	/* To make sure it'socket an insertion, wait 1 second before CIS is ready.
	 * If the card is currently active, report ejection immediately.  */
	if (!(socket->state.flags & SS_OUTPUT_ENA)) {
		current->state = TASK_INTERRUPTIBLE;
		schedule_timeout(HZ);
	}

	socket->handler(socket->info, event);

	if (!(socket->state.flags & SS_RESET))
		plx9052_enable_irq(socket);
}


static int plx9052_get_area(struct plx9052_socket *socket, int is_io,
			    int map)
{
	int i;

	/* Area 0 is used internally for CIS */
	for (i = 1; i < PLX_AREAS; i++) {
		if (!socket->area_used[i]) {
			socket->area_used[i] = PLX_AREA_USED | map
			    | (is_io ? PLX_AREA_IO : 0);
			return i;
		}
	}

	return -1;
}


static void plx9052_disable_areas(struct plx9052_socket *socket, int is_io,
				  int map)
{
	int i;
	int type = PLX_AREA_USED | map | (is_io ? PLX_AREA_IO : 0);

	/* Note that there may be more than one area for the map */
	for (i = 1; i < PLX_AREAS; i++) {
		if (socket->area_used[i] == type) {
			socket->area_used[i] = 0;
			plx9052_outl(socket, PLX_LASBA_DISABLE,
				     PLX_LASBA(i));
		}
	}
}


/* program a size-aligned area */
static int plx9052_program_area(struct plx9052_socket *socket, int is_io,
				int map, unsigned long sysbase,
				unsigned long syslen, u32 cardbase,
				unsigned int flags)
{
	u32 brd;
	u32 mask;
	int area;

	area = plx9052_get_area(socket, is_io, map);
	if (area < 0) {
		printk(KERN_ERR "No free areas left\n");
		return -EINVAL;
	}

	pci_write_config_dword(socket->pdev, PLX_PCI_BASE(area),
			       sysbase | is_io);

	mask = ~(syslen - 1);
	if (is_io)
		mask = (mask & PLX_LAS_IOMASK) | PLX_LASRR_IO;
	else
		mask &= PLX_LAS_MEMMASK;

	brd = plx9052_inl(socket, PLX_LASBRD(area));
	brd &= ~PLX_LASBRD_WIDTH_MASK;
	if (flags & MAP_16BIT)
		brd |= PLX_LASBRD_WIDTH_16;

	brd &= ~PLX_LASBRD_WS_MASK;
	if (!(flags & MAP_0WS))
		brd |= PLX_LASBRD_WS1;

	plx9052_outl(socket, brd, PLX_LASBRD(area));

	plx9052_outl(socket, mask, PLX_LASRR(area));
	plx9052_outl(socket, cardbase | PLX_LASBA_ENABLE, PLX_LASBA(area));

	return 0;
}


static int plx9052_init(struct pcmcia_socket *sock)
{
	struct plx9052_socket *socket =
	    container_of(sock, struct plx9052_socket, socket);

#if 0
	/* Dump all registers */
	int i;
	for (i = 0; i < 0x20; i++) {
		printk("%08x ", plx9052_inl(socket, i << 2));
		if ((i & 7) == 7)
			printk("\n");
	}
#endif

	/* Area 0 is used to access CIS from the driver */
	plx9052_outl(socket, PLX_LASBA_ENABLE | PLX_CIS_START,
		     PLX_LASBA(0));
	socket->area_used[0] = 1;

	plx9052_outl(socket,
		     PLX_CIS_START | (PLX_MEMWIN_MAX >> 1) |
		     PLX_CSBASE_ENABLE, PLX_CSBASE(0));
	plx9052_outl(socket, (PLX_IOWIN_MAX >> 1) | PLX_CSBASE_ENABLE,
		     PLX_CSBASE(1));

	plx9052_enable_irq(socket);
	socket->state.csc_mask |= SS_DETECT;

	return 0;
}


static int plx9052_suspend(struct pcmcia_socket *sock)
{
	plx9052_set_socket(sock, &dead_socket);
	return 0;
}


static int plx9052_get_status(struct pcmcia_socket *sock, u_int *value)
{
	struct plx9052_socket *socket =
	    container_of(sock, struct plx9052_socket, socket);

	*value = 0;

	if (plx9052_card_present(socket))
		*value |= SS_READY | SS_POWERON | SS_IOCARD | SS_DETECT;

	return 0;
}


static int plx9052_set_socket(struct pcmcia_socket *sock,
			      socket_state_t *state)
{
	u32 reg;
	struct plx9052_socket *socket =
	    container_of(sock, struct plx9052_socket, socket);

	if (state->flags & SS_RESET) {
		plx9052_disable_irq(socket);
		reg = plx9052_inl(socket, PLX_CNTRL);
		reg |= PLX_CNTRL_RESET;
		plx9052_outl(socket, PLX_CNTRL, reg);
	} else {
		reg = plx9052_inl(socket, PLX_CNTRL);
		reg &= ~PLX_CNTRL_RESET;
		plx9052_outl(socket, PLX_CNTRL, reg);
		plx9052_enable_irq(socket);
	}

	socket->state = *state;
	return 0;
}


static int plx9052_set_io_map(struct pcmcia_socket *sock,
			      struct pccard_io_map *io)
{
	struct plx9052_socket *socket =
	    container_of(sock, struct plx9052_socket, socket);
	unsigned int len;	/* requested length */
	unsigned int len2;	/* length adjusted to power of two */
	unsigned int start2;	/* start adjusted to power of two */
	unsigned int split;	/* split point for unaligned windows */
	unsigned int tmp;
	int err;

	/* Disable mapping before changing it */
	plx9052_disable_areas(socket, 1, io->map);

	if (!(io->flags & MAP_ACTIVE))
		return 0;

	len = io->stop + 1 - io->start;
	if (len > PLX_IOWIN_MAX) {
		printk(KERN_ERR
		       "Requested I/O area 0x%Lx-0x%Lx is too long\n",
		       io->start, io->stop);
		return -EINVAL;
	}

	/* Simplest case - size aligned window */
	if (!plx9052_align_check(io->start, len)) {
		err =
		    plx9052_program_area(socket, 1, io->map, io->start,
					 len, 0, io->flags);
		return err;
	}

	/* Find the highest address line that needs to be opened */
	tmp = io->stop ^ io->start;
	for (len2 = PLX_IOWIN_MIN; len2 <= PLX_IOWIN_MAX; len2 <<= 1) {
		if (len2 > tmp)
			break;
	}

	/* Split the requested window into at most two size-aligned windows */
	start2 = ~(len2 - 1) & io->start;
	split = start2 + (len2 >> 1);

	if (plx9052_align_check(io->start, split - io->start) ||
	    plx9052_align_check(split, io->stop + 1 - split)) {
		printk(KERN_ERR
		       "I/O area 0x%Lx-0x%Lx is too badly unaligned\n",
		       io->start, io->stop);
		return -ENOTSUPP;
	}

	err = plx9052_program_area(socket, 1, io->map, io->start,
				   split - io->start, 0, io->flags);

	if (err)
		return err;

	err = plx9052_program_area(socket, 1, io->map, split,
				   io->stop + 1 - split, 0, io->flags);

	return err;
}


static int plx9052_set_mem_map(struct pcmcia_socket *sock,
			       struct pccard_mem_map *mem)
{
	struct plx9052_socket *socket =
	    container_of(sock, struct plx9052_socket, socket);
	unsigned long len;	/* requested length */
	int err;

	/* Disable mapping before changing it */
	plx9052_disable_areas(socket, 0, mem->map);

	if (!(mem->flags & MAP_ACTIVE))
		return 0;

	/* Memory allocation in the first megabyte is problematic on
	 * some machines with Intel chipset.  */
	if (mem->res->start < 0x100000)
		return -EINVAL;

	len = resource_size(mem->res);
	if (len > PLX_MEMWIN_MAX) {
		printk(KERN_ERR "Memory map %pR is too long\n",
		       mem->res);
		return -EINVAL;
	}

	if (plx9052_align_check(mem->res->start, len)) {
		printk(KERN_ERR
		       "Memory map %pR is not size-aligned\n",
		       mem->res);
		return -EINVAL;
	}

	err =
	    plx9052_program_area(socket, 0, mem->map, mem->res->start, len,
				 mem->card_start | PLX_CIS_START,
				 mem->flags);

	return err;
}


static irqreturn_t plx9052_interrupt(int irq, void *dev_id)
{
	struct plx9052_socket *socket = (struct plx9052_socket *) dev_id;

	if (!plx9052_irq_active(socket))
		return IRQ_NONE;

	if (!(socket->state.csc_mask & SS_DETECT))
		return IRQ_HANDLED;

	if (!plx9052_card_present(socket)) {
		socket->event |= SS_DETECT;
		plx9052_disable_irq(socket);
		schedule_work(&socket->event_work);
	}

	return IRQ_HANDLED;
}


static struct pccard_operations plx9052_operations = {
	.init = plx9052_init,
	.suspend = plx9052_suspend,
	.get_status = plx9052_get_status,
	.set_socket = plx9052_set_socket,
	.set_io_map = plx9052_set_io_map,
	.set_mem_map = plx9052_set_mem_map,
};


static int plx9052_probe(struct pci_dev *pdev,
			 const struct pci_device_id *ent)
{
	int err = 0;
	struct plx9052_socket *socket = NULL;

	socket = kmalloc(sizeof(struct plx9052_socket), GFP_KERNEL);
	if (!socket)
		return -ENOMEM;

	memset(socket, 0, sizeof(struct plx9052_socket));

	socket->pdev = pdev;
	pci_set_drvdata(pdev, socket);

	spin_lock_init(&socket->event_lock);
	INIT_WORK(&socket->event_work, plx9052_event);

	err = pci_enable_device(pdev);
	if (err)
		return -EIO;

	/* Resource 1 is control registers of PLX9052 */
	socket->plxctl_addr = pci_resource_start(pdev, 1);
	socket->plxctl_len = pci_resource_len(pdev, 1);
	if (!request_region
	    (socket->plxctl_addr, socket->plxctl_len, DEVICE_NAME)) {
		printk(KERN_ERR "plx9052: I/O at 0x%x-0x%x busy\n",
		       socket->plxctl_addr,
		       socket->plxctl_addr + socket->plxctl_len - 1);
		socket->plxctl_addr = 0;
		err = -EBUSY;
		goto fail;
	}

	/* Resource 2 is mapped to the PCMCIA memory space, starting with CIS */
	socket->mem_phys = pci_resource_start(pdev, 2);
	socket->mem_len = pci_resource_len(pdev, 2);
	if (!request_mem_region
	    (socket->mem_phys, socket->mem_len, DEVICE_NAME)) {
		printk(KERN_ERR "plx9052: memory at 0x%lx-0x%lx busy\n",
		       socket->mem_phys,
		       socket->mem_phys + socket->mem_len - 1);
		socket->mem_phys = 0;
		err = -EBUSY;
		goto fail;
	}

	socket->mem_virt = ioremap(socket->mem_phys, socket->mem_len);
	if (!socket->mem_virt) {
		printk(KERN_ERR
		       "plx9052: cannot map memory at 0x%lx-0x%lx\n",
		       socket->mem_phys,
		       socket->mem_phys + socket->mem_len - 1);
		err = -ENOMEM;
		goto fail;
	}

	err =
	    request_irq(pdev->irq, plx9052_interrupt, IRQF_SHARED,
			DEVICE_NAME, socket);
	if (err) {
		printk(KERN_ERR "plx9052: cannot allocate IRQ %d.\n",
		       pdev->irq);
		err = -EBUSY;
		goto fail;
	}

	socket->socket.ops = &plx9052_operations;
	socket->socket.dev.parent = &pdev->dev;
	socket->socket.driver_data = socket;
	socket->socket.owner = THIS_MODULE;

	socket->socket.features =
	    SS_CAP_PCCARD | SS_CAP_MEM_ALIGN | SS_CAP_PAGE_REGS;
	socket->socket.map_size = PLX_MEMWIN_MIN;	/* minimum window size */
	socket->socket.pci_irq = pdev->irq;

	pcmcia_register_socket(&socket->socket);

	printk(KERN_INFO "plx9052: socket enabled, IRQ %d\n",
	       socket->pdev->irq);

	return 0;		/* succeeded */

      fail:
	plx9052_close(pdev);

	return err;
}


static void plx9052_close(struct pci_dev *pdev)
{
	struct plx9052_socket *socket = pci_get_drvdata(pdev);

	pcmcia_unregister_socket(&socket->socket);

	if (pdev->irq)
		free_irq(pdev->irq, socket);

	if (socket->plxctl_addr)
		release_region(socket->plxctl_addr, socket->plxctl_len);

	if (socket->mem_virt)
		iounmap(socket->mem_virt);

	if (socket->mem_phys)
		release_mem_region(socket->mem_phys, socket->mem_len);

	socket->pdev = NULL;

	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	kfree(socket);
}


/* PCI ID table, from orinoco_plx.c */
static struct pci_device_id plx9052_pci_id_table[] = {
	{0x111a, 0x1023, PCI_ANY_ID, PCI_ANY_ID,},	/* Siemens SpeedStream SS1023 */
	{0x10b5, 0x9050, PCI_ANY_ID, PCI_ANY_ID,},	/* PLX PCI Bridge */
	{0x1385, 0x4100, PCI_ANY_ID, PCI_ANY_ID,},	/* Netgear MA301 */
	{0x15e8, 0x0130, PCI_ANY_ID, PCI_ANY_ID,},	/* Correga  - does this work? */
	{0x1638, 0x1100, PCI_ANY_ID, PCI_ANY_ID,},	/* SMC EZConnect SMC2602W,
							   Eumitcom PCI WL11000,
							   Addtron AWA-100 */
	{0x16ab, 0x1100, PCI_ANY_ID, PCI_ANY_ID,},	/* Global Sun Tech GL24110P */
	{0x16ab, 0x1101, PCI_ANY_ID, PCI_ANY_ID,},	/* Reported working, but unknown */
	{0x16ab, 0x1102, PCI_ANY_ID, PCI_ANY_ID,},	/* Linksys WDT11 */
	{0x16ec, 0x3685, PCI_ANY_ID, PCI_ANY_ID,},	/* USR 2415 */
	{0xec80, 0xec00, PCI_ANY_ID, PCI_ANY_ID,},	/* Belkin F5D6000 tested by
							   Brendan W. McAdams <rit@jacked-in.org> */
	{0x10b7, 0x7770, PCI_ANY_ID, PCI_ANY_ID,},	/* 3Com AirConnect PCI tested by
							   Damien Persohn <damien@persohn.net> */
	{0,},
};

MODULE_DEVICE_TABLE(pci, plx9052_pci_id_table);

static struct pci_driver plx9052_driver = {
	.name = DEVICE_NAME,
	.id_table = plx9052_pci_id_table,
	.probe = plx9052_probe,
	.remove = plx9052_close,
};


static int __init init_plx9052(void)
{
	return pci_register_driver(&plx9052_driver);
}

extern void __exit exit_plx9052(void)
{
	pci_unregister_driver(&plx9052_driver);
}


module_init(init_plx9052);
module_exit(exit_plx9052);
