

/******************************************************************************

  Copyright (c) 2001-2015, Intel Corporation 
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:
  
   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.
  
   2. Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in the 
      documentation and/or other materials provided with the distribution.
  
   3. Neither the name of the Intel Corporation nor the names of its 
      contributors may be used to endorse or promote products derived from 
      this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/
/*$FreeBSD$*/

/*
 * Uncomment the following extensions for better performance in a VM,
 * especially if you have support in the hypervisor.
 * See http://info.iet.unipi.it/~luigi/netmap/
 */
// #define BATCH_DISPATCH
// #define NIC_SEND_COMBINING
// #define NIC_PARAVIRT	/* enable virtio-like synchronization */

#include "opt_inet.h"
#include "opt_inet6.h"

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_device_polling.h"
#endif

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf_ring.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/eventhandler.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>

#include <machine/in_cksum.h>
#include <dev/led/led.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include "e1000_api.h"
#include "if_lem.h"
#include "ifdi_if.h"


/*********************************************************************
 *  Legacy Em Driver version:
 *********************************************************************/
char lem_driver_version[] = "1.1.0";

/*********************************************************************
 *  PCI Device ID Table
 *
 *  Used by probe to select devices to load on
 *  Last field stores an index into e1000_strings
 *  Last entry must be all 0s
 *
 *  { Vendor ID, Device ID, SubVendor ID, SubDevice ID, String Index }
 *********************************************************************/

static pci_vendor_info_t lem_vendor_info_array[] =
{
	/* Intel(R) PRO/1000 Network Connection */
    PVID(0x8086, E1000_DEV_ID_82540EM, "Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82540EM_LOM,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82540EP,		"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82540EP_LOM,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82540EP_LP,	"Intel(R) PRO/1000 Legacy Network Connection"),

	PVID(0x8086, E1000_DEV_ID_82541EI,		"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82541ER,		"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82541ER_LOM,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82541EI_MOBILE,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82541GI,		"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82541GI_LF,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82541GI_MOBILE,	"Intel(R) PRO/1000 Legacy Network Connection"),

	PVID(0x8086, E1000_DEV_ID_82542,		"Intel(R) PRO/1000 Legacy Network Connection"),

	PVID(0x8086, E1000_DEV_ID_82543GC_FIBER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82543GC_COPPER,	"Intel(R) PRO/1000 Legacy Network Connection"),

	PVID(0x8086, E1000_DEV_ID_82544EI_COPPER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82544EI_FIBER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82544GC_COPPER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82544GC_LOM,	"Intel(R) PRO/1000 Legacy Network Connection"),

	PVID(0x8086, E1000_DEV_ID_82545EM_COPPER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82545EM_FIBER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82545GM_COPPER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82545GM_FIBER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82545GM_SERDES,	"Intel(R) PRO/1000 Legacy Network Connection"),

	PVID(0x8086, E1000_DEV_ID_82546EB_COPPER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82546EB_FIBER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82546EB_QUAD_COPPER, "Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82546GB_COPPER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82546GB_FIBER,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82546GB_SERDES,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82546GB_PCIE,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82546GB_QUAD_COPPER, "Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3, "Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82547EI,		"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82547EI_MOBILE,	"Intel(R) PRO/1000 Legacy Network Connection"),
	PVID(0x8086, E1000_DEV_ID_82547GI,		"Intel(R) PRO/1000 Legacy Network Connection"),
	/* required last entry */
    PVID_END
};

/*********************************************************************
 *  Function prototypes
 *********************************************************************/
static void *lem_register(device_t dev); 
static int	lem_if_attach_pre(if_ctx_t ctx);
static int	lem_if_attach_post(if_ctx_t ctx);
static int  lem_if_suspend(if_ctx_t ctx);
static int  lem_if_shutdown(if_ctx_t ctx);
static int  lem_if_resume(if_ctx_t ctx);
static int  lem_if_detach(if_ctx_t ctx); 
static int  lem_if_sysctl_int_delay(if_ctx_t ctx, if_int_delay_info_t info);
static int  lem_if_msix_intr_assign(if_ctx_t ctx, int msix);
static int  lem_if_mtu_set(if_ctx_t ctx, uint32_t mtu);
static int  lem_if_promisc_set(if_ctx_t ctx, int flags);
static void lem_if_init(if_ctx_t ctx);
static void lem_if_intr_disable(if_ctx_t ctx);
static void lem_if_intr_enable(if_ctx_t ctx);
static void lem_if_multi_set(if_ctx_t ctx);
static void lem_if_media_set(if_ctx_t ctx);
static uint64_t lem_if_get_counter(if_ctx_t ctx, ift_counter cnt);
static void lem_if_stop(if_ctx_t ctx);
static int lem_if_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs, int nqs);
static void lem_if_queues_free(if_ctx_t ctx);
static int lem_if_media_change(if_ctx_t ctx);
static void lem_if_media_status(if_ctx_t ctx, struct ifmediareq *ifmr);
static void lem_if_timer(if_ctx_t ctx, uint16_t qid); 
static void lem_if_led_func(if_ctx_t ctx, int onoff); 

static int	lem_setup_interface(device_t, struct adapter *);
static void	lem_initialize_transmit_unit(struct adapter *);
static void	lem_initialize_receive_unit(struct adapter *);
static void	lem_identify_hardware(struct adapter *);
static int	lem_allocate_pci_resources(struct adapter *);
static int	lem_hardware_init(struct adapter *);
static void	lem_release_manageability(struct adapter *);
static void	lem_free_pci_resources(struct adapter *);
void	lem_update_link_status(struct adapter *);
static void	lem_update_stats_counters(struct adapter *);
static void	lem_add_hw_stats(struct adapter *adapter);
static void lem_if_vlan_register(if_ctx_t ctx, u16 vtag);
static void lem_if_vlan_unregister(if_ctx_t ctx, u16 vtag);
static void	lem_add_rx_process_limit(struct adapter *, const char *,
		    const char *, int *, int);

static void lem_get_hw_control(struct adapter *);
static void	lem_get_wakeup(if_ctx_t ctx); 
static void	lem_smartspeed(struct adapter *);
static int	lem_irq_fast(void *);
int lem_intr(void *arg);
static void	lem_setup_vlan_hw_support(struct adapter *);
static int 	lem_is_valid_ether_addr(u8 *);

/* Management and WOL Support */
static void	lem_init_manageability(struct adapter *);
static void	lem_release_manageability(struct adapter *);
static void lem_release_hw_control(struct adapter *);
static void lem_enable_wakeup(struct adapter *adapter);

static void lem_add_int_delay_sysctl(struct adapter *adapter, const char *name,
            const char *description, struct if_int_delay_info *info, int offset, int value);
static int	lem_sysctl_nvm_info(SYSCTL_HANDLER_ARGS);
static void	lem_print_nvm_info(struct adapter *);
static void	lem_set_flow_cntrl(struct adapter *, const char *,
							   const char *, int *, int);

static int	lem_enable_phy_wakeup(struct adapter *);


/*********************************************************************
 *  FreeBSD Device Interface Entry Points
 *********************************************************************/

static device_method_t lem_methods[] = {
	/* Device interface */
	DEVMETHOD(device_register, lem_register),
	DEVMETHOD(device_probe, iflib_device_probe),
	DEVMETHOD(device_attach, iflib_device_attach),
	DEVMETHOD(device_detach, iflib_device_detach),
	DEVMETHOD(device_shutdown, iflib_device_shutdown),
	DEVMETHOD(device_suspend, iflib_device_suspend),
	DEVMETHOD(device_resume, iflib_device_resume),
	DEVMETHOD_END
};

static driver_t lem_driver = {
	"em", lem_methods, sizeof(struct adapter),
};

extern devclass_t em_devclass;
DRIVER_MODULE(lem, pci, lem_driver, em_devclass, 0, 0);

MODULE_DEPEND(lem, pci, 1, 1, 1);
MODULE_DEPEND(lem, ether, 1, 1, 1);
MODULE_DEPEND(lem, iflib, 1, 1, 1); 

static device_method_t lem_if_methods[] =
{
	DEVMETHOD(ifdi_attach_pre, lem_if_attach_pre),
	DEVMETHOD(ifdi_attach_post, lem_if_attach_post),
	DEVMETHOD(ifdi_suspend, lem_if_suspend),
	DEVMETHOD(ifdi_shutdown, lem_if_shutdown),
	DEVMETHOD(ifdi_resume, lem_if_resume), 
	DEVMETHOD(ifdi_msix_intr_assign, lem_if_msix_intr_assign),
	DEVMETHOD(ifdi_detach, lem_if_detach),
	DEVMETHOD(ifdi_sysctl_int_delay, lem_if_sysctl_int_delay),
	DEVMETHOD(ifdi_queues_alloc, lem_if_queues_alloc),
	DEVMETHOD(ifdi_mtu_set, lem_if_mtu_set),
	DEVMETHOD(ifdi_intr_disable, lem_if_intr_disable),
	DEVMETHOD(ifdi_intr_enable, lem_if_intr_enable),
	DEVMETHOD(ifdi_led_func, lem_if_led_func), 
	DEVMETHOD(ifdi_multi_set, lem_if_multi_set),
	DEVMETHOD(ifdi_media_set, lem_if_media_set),
	DEVMETHOD(ifdi_media_change, lem_if_media_change),
	DEVMETHOD(ifdi_media_status, lem_if_media_status),
	DEVMETHOD(ifdi_promisc_set, lem_if_promisc_set), 
	DEVMETHOD(ifdi_get_counter, lem_if_get_counter),
	DEVMETHOD(ifdi_init, lem_if_init), 
    DEVMETHOD(ifdi_stop, lem_if_stop),
	DEVMETHOD(ifdi_queues_free, lem_if_queues_free),
	DEVMETHOD(ifdi_timer, lem_if_timer),
	DEVMETHOD(ifdi_vlan_register, lem_if_vlan_register),
	DEVMETHOD(ifdi_vlan_unregister, lem_if_vlan_unregister),
	DEVMETHOD_END
};


static driver_t lem_if_driver = {
	"lem_if", lem_if_methods, sizeof(struct adapter)
};

/*********************************************************************
 *  Tunable default values.
 *********************************************************************/

#define EM_TICKS_TO_USECS(ticks)	((1024 * (ticks) + 500) / 1000)
#define EM_USECS_TO_TICKS(usecs)	((1000 * (usecs) + 512) / 1024)

#define MAX_INTS_PER_SEC	8000
#define DEFAULT_ITR		(1000000000/(MAX_INTS_PER_SEC * 256))

static int lem_tx_int_delay_dflt = EM_TICKS_TO_USECS(EM_TIDV);
static int lem_rx_int_delay_dflt = EM_TICKS_TO_USECS(EM_RDTR);
static int lem_tx_abs_int_delay_dflt = EM_TICKS_TO_USECS(EM_TADV);
static int lem_rx_abs_int_delay_dflt = EM_TICKS_TO_USECS(EM_RADV);
/*
 * increase lem_rxd and lem_txd to at least 2048 in netmap mode
 * for better performance.
 */
static int lem_rxd = EM_DEFAULT_RXD;
static int lem_txd = EM_DEFAULT_TXD;
static int lem_smart_pwr_down = FALSE;

/* Controls whether promiscuous also shows bad packets */
static int lem_debug_sbp = FALSE;

TUNABLE_INT("hw.em.tx_int_delay", &lem_tx_int_delay_dflt);
TUNABLE_INT("hw.em.rx_int_delay", &lem_rx_int_delay_dflt);
TUNABLE_INT("hw.em.tx_abs_int_delay", &lem_tx_abs_int_delay_dflt);
TUNABLE_INT("hw.em.rx_abs_int_delay", &lem_rx_abs_int_delay_dflt);
TUNABLE_INT("hw.em.rxd", &lem_rxd);
TUNABLE_INT("hw.em.txd", &lem_txd);
TUNABLE_INT("hw.em.smart_pwr_down", &lem_smart_pwr_down);
TUNABLE_INT("hw.em.sbp", &lem_debug_sbp);

/* Interrupt style - default to fast */
static int lem_use_legacy_irq = 0;
TUNABLE_INT("hw.em.use_legacy_irq", &lem_use_legacy_irq);

/* How many packets rxeof tries to clean at a time */
static int lem_rx_process_limit = 100;
TUNABLE_INT("hw.em.rx_process_limit", &lem_rx_process_limit);

/* Flow control setting - default to FULL */
static int lem_fc_setting = e1000_fc_full;
TUNABLE_INT("hw.em.fc_setting", &lem_fc_setting);

/* Global used in WOL setup with multiport cards */
static int global_quad_port_a = 0;

extern struct if_txrx lem_txrx;

static struct if_shared_ctx lem_sctx_init = {
	.isc_magic = IFLIB_MAGIC,
	.isc_q_align = PAGE_SIZE,/* max(DBA_ALIGN, PAGE_SIZE) */
	.isc_tx_maxsize = EM_TSO_SIZE,
	.isc_tx_maxsegsize = PAGE_SIZE*4,
	.isc_rx_maxsize = PAGE_SIZE*4,
	.isc_rx_nsegments = 1,
	.isc_rx_maxsegsize = PAGE_SIZE*4,
	.isc_ntxd = EM_DEFAULT_TXD,
	.isc_nrxd = EM_DEFAULT_RXD,
	.isc_nfl = 1,
	.isc_qsizes[0] = roundup2((EM_DEFAULT_TXD * sizeof(struct e1000_tx_desc)) +
							  sizeof(u32), EM_DBA_ALIGN),
	.isc_qsizes[1] = roundup2(EM_DEFAULT_RXD *
							  sizeof(struct e1000_rx_desc), EM_DBA_ALIGN),
	.isc_nqs = 2,
	.isc_admin_intrcnt = 0,
	.isc_vendor_info = lem_vendor_info_array,
	.isc_driver_version = lem_driver_version,
	.isc_txrx = &lem_txrx,
	.isc_driver = &lem_if_driver,	
};

if_shared_ctx_t lem_sctx = &lem_sctx_init; 

static int
lem_if_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs, int nqs)
{
  	struct adapter *adapter = iflib_get_softc(ctx);
    struct em_buffer *transmit_buf;
	struct em_buffer *receive_buf;

	MPASS(nqs == 2);

    /*Allocate tx_buffer_area & rx_buffer_area */
	if ((transmit_buf = malloc(sizeof(*transmit_buf)*lem_sctx->isc_ntxd, M_DEVBUF, M_WAITOK|M_ZERO)) == NULL) {
		device_printf(iflib_get_dev(ctx), "Unable to allocate em_buffer for transmit\n");
		return (ENOMEM); 
	}

	if ((receive_buf = malloc(sizeof(*receive_buf)*lem_sctx->isc_nrxd, M_DEVBUF, M_WAITOK|M_ZERO)) == NULL) {
		device_printf(iflib_get_dev(ctx), "Unable to allocate em_buffer for receive\n");
		return (ENOMEM); 
	}
	
	adapter->tx_desc_base = (struct e1000_tx_desc *)vaddrs[0];
	adapter->rx_desc_base = (struct e1000_rx_desc *)vaddrs[1];

    adapter->tx_desc_base->buffer_addr = htole64(paddrs); 
    adapter->rx_desc_base->buffer_addr = htole64(paddrs + 1); 
	
    adapter->tx_buffer_area = transmit_buf;
	adapter->rx_buffer_area = receive_buf;

    device_printf(iflib_get_dev(ctx), "allocated tx and rx buffer areas\n"); 
	
	return (0); 
}

static void
lem_if_queues_free(if_ctx_t ctx)
{
	device_printf(iflib_get_dev(ctx), "calling lem_if_queues_free\n"); 
	struct adapter *adapter = iflib_get_softc(ctx);

	if (adapter->tx_buffer_area != NULL) {
		free(adapter->tx_buffer_area, M_DEVBUF);
		adapter->tx_buffer_area = NULL;
	}

	if (adapter->rx_buffer_area != NULL) {
		free(adapter->rx_buffer_area, M_DEVBUF);
		adapter->rx_buffer_area = NULL;
	}

	if (adapter->mta != NULL) {
		free(adapter->mta, M_DEVBUF);
		adapter->mta = NULL;
	}
}

static void *
lem_register(device_t dev)
{
    lem_sctx->isc_ntxd = lem_txd;
	lem_sctx->isc_nrxd = lem_rxd;
	lem_sctx->isc_qsizes[0] = roundup2((lem_txd * sizeof(struct e1000_tx_desc)) +
									   sizeof(u32), EM_DBA_ALIGN);
	lem_sctx->isc_qsizes[1] = roundup2(lem_rxd *
									   sizeof(struct e1000_rx_desc), EM_DBA_ALIGN);


	return (lem_sctx);
}


/*********************************************************************
 *  Device initialization routine
 *
 *  The attach entry point is called when the driver is being loaded.
 *  This routine identifies the type of hardware, allocates all resources
 *  and initializes the hardware.
 *
 *  return 0 on success, positive on failure
 *********************************************************************/

static int
lem_if_attach_pre(if_ctx_t ctx)
{
	device_t dev; 
	struct adapter	*adapter;
	int		error = 0;

	INIT_DEBUGOUT("lem_attach: begin");

	dev = iflib_get_dev(ctx);
	adapter = iflib_get_softc(ctx);
	adapter->dev = adapter->osdep.dev = dev;
	adapter->ctx = ctx;
	adapter->shared = iflib_get_softc_ctx(ctx); 
    adapter->media = iflib_get_media(ctx); 

    /* Allocate multicast array memory. */
	adapter->mta = malloc(sizeof(u8) * ETH_ADDR_LEN *
	    MAX_NUM_MULTICAST_ADDRESSES, M_DEVBUF, M_NOWAIT);
	if (adapter->mta == NULL) {
		device_printf(dev, "Can not allocate multicast setup array\n");
		error = ENOMEM;
		goto err_hw_init;
	}
	
	/* SYSCTL stuff */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "nvm", CTLTYPE_INT|CTLFLAG_RW, adapter, 0,
	    lem_sysctl_nvm_info, "I", "NVM Information");


	/* Determine hardware and mac info */
	lem_identify_hardware(adapter);
	
    adapter->shared->isc_tx_nsegments = EM_MAX_SCATTER;
	adapter->shared->isc_msix_bar = 0; 
	
	/* Setup PCI resources */
	if (lem_allocate_pci_resources(adapter)) {
		device_printf(dev, "Allocation of PCI resources failed\n");
		error = ENXIO;
		goto err_pci;
	}

	/* Do Shared Code initialization */
	if (e1000_setup_init_funcs(&adapter->hw, TRUE)) {
		device_printf(dev, "Setup of Shared code failed\n");
		error = ENXIO;
		goto err_pci;
	}

	e1000_get_bus_info(&adapter->hw);

	/* Set up some sysctls for the tunable interrupt delays */
    	lem_add_int_delay_sysctl(adapter, "rx_int_delay",
	    "receive interrupt delay in usecs", &adapter->rx_int_delay,
	    E1000_REGISTER(&adapter->hw, E1000_RDTR), lem_rx_int_delay_dflt);
	   lem_add_int_delay_sysctl(adapter, "tx_int_delay",
	    "transmit interrupt delay in usecs", &adapter->tx_int_delay,
	    E1000_REGISTER(&adapter->hw, E1000_TIDV), lem_tx_int_delay_dflt);
	if (adapter->hw.mac.type >= e1000_82540) {
		lem_add_int_delay_sysctl(adapter, "rx_abs_int_delay",
		    "receive interrupt delay limit in usecs",
		    &adapter->rx_abs_int_delay,
		    E1000_REGISTER(&adapter->hw, E1000_RADV),
		    lem_rx_abs_int_delay_dflt);
		lem_add_int_delay_sysctl(adapter, "tx_abs_int_delay",
		    "transmit interrupt delay limit in usecs",
		    &adapter->tx_abs_int_delay,
		    E1000_REGISTER(&adapter->hw, E1000_TADV),
		    lem_tx_abs_int_delay_dflt);
		lem_add_int_delay_sysctl(adapter, "itr",
		    "interrupt delay limit in usecs/4",
		    &adapter->tx_itr,
		    E1000_REGISTER(&adapter->hw, E1000_ITR),
		    DEFAULT_ITR);
	}

	/* Sysctls for limiting the amount of work done in the taskqueue */
	lem_add_rx_process_limit(adapter, "rx_processing_limit",
	    "max number of rx packets to process", &adapter->rx_process_limit,
	    lem_rx_process_limit);

#ifdef NIC_SEND_COMBINING
	/* Sysctls to control mitigation */
	lem_add_rx_process_limit(adapter, "sc_enable",
	    "driver TDT mitigation", &adapter->sc_enable, 0);
#endif /* NIC_SEND_COMBINING */
#ifdef BATCH_DISPATCH
	lem_add_rx_process_limit(adapter, "batch_enable",
	    "driver rx batch", &adapter->batch_enable, 0);
#endif /* BATCH_DISPATCH */
#ifdef NIC_PARAVIRT
	lem_add_rx_process_limit(adapter, "rx_retries",
	    "driver rx retries", &adapter->rx_retries, 0);
#endif /* NIC_PARAVIRT */

        /* Sysctl for setting the interface flow control */
	lem_set_flow_cntrl(adapter, "flow_control",
	    "flow control setting",
	    &adapter->fc_setting, lem_fc_setting);

	/*
	 * Validate number of transmit and receive descriptors. It
	 * must not exceed hardware maximum, and must be multiple
	 * of E1000_DBA_ALIGN.
	 */
	if (((lem_txd * sizeof(struct e1000_tx_desc)) % EM_DBA_ALIGN) != 0 ||
	    (adapter->hw.mac.type >= e1000_82544 && lem_txd > EM_MAX_TXD) ||
	    (adapter->hw.mac.type < e1000_82544 && lem_txd > EM_MAX_TXD_82543) ||
	    (lem_txd < EM_MIN_TXD)) {
		device_printf(dev, "Using %d TX descriptors instead of %d!\n",
		    EM_DEFAULT_TXD, lem_txd);
		adapter->num_tx_desc = EM_DEFAULT_TXD;
	} else
		adapter->num_tx_desc = lem_txd;
	if (((lem_rxd * sizeof(struct e1000_rx_desc)) % EM_DBA_ALIGN) != 0 ||
	    (adapter->hw.mac.type >= e1000_82544 && lem_rxd > EM_MAX_RXD) ||
	    (adapter->hw.mac.type < e1000_82544 && lem_rxd > EM_MAX_RXD_82543) ||
	    (lem_rxd < EM_MIN_RXD)) {
		device_printf(dev, "Using %d RX descriptors instead of %d!\n",
		    EM_DEFAULT_RXD, lem_rxd);
		adapter->num_rx_desc = EM_DEFAULT_RXD;
	} else
		adapter->num_rx_desc = lem_rxd;

	adapter->hw.mac.autoneg = DO_AUTO_NEG;
	adapter->hw.phy.autoneg_wait_to_complete = FALSE;
	adapter->hw.phy.autoneg_advertised = AUTONEG_ADV_DEFAULT;
	adapter->rx_buffer_len = 2048;

	e1000_init_script_state_82541(&adapter->hw, TRUE);
	e1000_set_tbi_compatibility_82543(&adapter->hw, TRUE);

	/* Copper options */
	if (adapter->hw.phy.media_type == e1000_media_type_copper) {
		adapter->hw.phy.mdix = AUTO_ALL_MODES;
		adapter->hw.phy.disable_polarity_correction = FALSE;
		adapter->hw.phy.ms_type = EM_MASTER_SLAVE;
	}

	/*
	 * Set the frame limits assuming
	 * standard ethernet sized frames.
	 */
	adapter->max_frame_size = ETHERMTU + ETHER_HDR_LEN + ETHERNET_FCS_SIZE;
	adapter->min_frame_size = ETH_ZLEN + ETHERNET_FCS_SIZE;

	/*
	 * This controls when hardware reports transmit completion
	 * status.
	 */
	adapter->hw.mac.report_tx_early = 1;

#ifdef NIC_PARAVIRT
	device_printf(dev, "driver supports paravirt, subdev 0x%x\n",
		adapter->hw.subsystem_device_id);
	if (adapter->hw.subsystem_device_id == E1000_PARA_SUBDEV) {
		uint64_t bus_addr;

		device_printf(dev, "paravirt support on dev %p\n", adapter);
		tsize = 4096; // XXX one page for the csb
		}
		/* Setup the Base of the CSB */
		adapter->csb = (struct paravirt_csb *)adapter->csb_mem.dma_vaddr;
		/* force the first kick */
		adapter->csb->host_need_txkick = 1; /* txring empty */
		adapter->csb->guest_need_rxkick = 1; /* no rx packets */
		bus_addr = adapter->csb_mem.dma_paddr;
		lem_add_rx_process_limit(adapter, "csb_on",
		    "enable paravirt.", &adapter->csb->guest_csb_on, 0);
		lem_add_rx_process_limit(adapter, "txc_lim",
		    "txc_lim", &adapter->csb->host_txcycles_lim, 1);

		/* some stats */
#define PA_SC(name, var, val)		\
	lem_add_rx_process_limit(adapter, name, name, var, val)
		PA_SC("host_need_txkick",&adapter->csb->host_need_txkick, 1);
		PA_SC("host_rxkick_at",&adapter->csb->host_rxkick_at, ~0);
		PA_SC("guest_need_txkick",&adapter->csb->guest_need_txkick, 0);
		PA_SC("guest_need_rxkick",&adapter->csb->guest_need_rxkick, 1);
		PA_SC("tdt_reg_count",&adapter->tdt_reg_count, 0);
		PA_SC("tdt_csb_count",&adapter->tdt_csb_count, 0);
		PA_SC("tdt_int_count",&adapter->tdt_int_count, 0);
		PA_SC("guest_need_kick_count",&adapter->guest_need_kick_count, 0);
		/* tell the host where the block is */
		E1000_WRITE_REG(&adapter->hw, E1000_CSBAH,
			(u32)(bus_addr >> 32));
		E1000_WRITE_REG(&adapter->hw, E1000_CSBAL,
			(u32)bus_addr);
#endif /* NIC_PARAVIRT */

	/*
	** Start from a known state, this is
	** important in reading the nvm and
	** mac from that.
	*/
	e1000_reset_hw(&adapter->hw);

	/* Make sure we have a good EEPROM before we read from it */
	if (e1000_validate_nvm_checksum(&adapter->hw) < 0) {
		/*
		** Some PCI-E parts fail the first check due to
		** the link being in sleep state, call it again,
		** if it fails a second time its a real issue.
		*/
		if (e1000_validate_nvm_checksum(&adapter->hw) < 0) {
			device_printf(dev,
			    "The EEPROM Checksum Is Not Valid\n");
			error = EIO;
			goto err_hw_init;
		}
	}

	/* Copy the permanent MAC address out of the EEPROM */
	if (e1000_read_mac_addr(&adapter->hw) < 0) {
		device_printf(dev, "EEPROM read error while reading MAC"
		    " address\n");
		error = EIO;
		goto err_hw_init;
	}

	if (!lem_is_valid_ether_addr(adapter->hw.mac.addr)) {
		device_printf(dev, "Invalid MAC address\n");
		error = EIO;
		goto err_hw_init;
	}

	/* Initialize the hardware */
	if (lem_hardware_init(adapter)) {
		device_printf(dev, "Unable to initialize the hardware\n");
		error = EIO;
		goto err_hw_init;
	}

	/*
	 * Get Wake-on-Lan and Management info for later use
	 */
     lem_get_wakeup(ctx);

/* Setup OS specific network interface */
	if (lem_setup_interface(dev, adapter) != 0)
		goto err_hw_init;

    return (0); 

    err_hw_init:
	   lem_release_hw_control(adapter);

    err_pci:
	   lem_free_pci_resources(adapter);
	   free(adapter->mta, M_DEVBUF);

	return (error);
}

static int
lem_if_attach_post(if_ctx_t ctx)
{
	device_t dev = iflib_get_dev(ctx);
	struct adapter *adapter = iflib_get_softc(ctx); 
    struct ifnet *ifp = iflib_get_ifp(ctx); 
	
/* Initialize statistics */
	lem_update_stats_counters(adapter);

	adapter->hw.mac.get_link_status = 1;
	lem_update_link_status(adapter);

	/* Indicate SOL/IDER usage */
	if (e1000_check_reset_block(&adapter->hw))
		device_printf(dev,
		    "PHY reset is blocked due to SOL/IDER session.\n");

	/* Do we need workaround for 82544 PCI-X adapter? */
	if (adapter->hw.bus.type == e1000_bus_type_pcix &&
	    adapter->hw.mac.type == e1000_82544)
		adapter->pcix_82544 = TRUE;
	else
		adapter->pcix_82544 = FALSE;

	lem_add_hw_stats(adapter);

	/* Non-AMT based hardware can now take control from firmware */
	if (adapter->has_manage && !adapter->has_amt)
		lem_get_hw_control(adapter);

	/* Tell the stack that the interface is not active */
	if_setdrvflagbits(ifp, 0, IFF_DRV_OACTIVE | IFF_DRV_RUNNING);

	iflib_led_create(ctx); 

	INIT_DEBUGOUT("lem_attach: end");

	return (0);
}


/*********************************************************************
 *  Device removal routine
 *
 *  The detach entry point is called when the driver is being removed.
 *  This routine stops the adapter and deallocates all the resources
 *  that were allocated for driver operation.
 *
 *  return 0 on success, positive on failure
 *********************************************************************/

static int
lem_if_detach(if_ctx_t ctx)
{
	struct adapter	*adapter = iflib_get_softc(ctx);

	INIT_DEBUGOUT("em_detach: begin");

	adapter->in_detach = 1;
	e1000_phy_hw_reset(&adapter->hw);

	lem_release_manageability(adapter);
	lem_free_pci_resources(adapter);

	lem_release_hw_control(adapter);

	return (0);
}

/*********************************************************************
 *
 *  Shutdown entry point
 *
 **********************************************************************/

static int
lem_if_shutdown(if_ctx_t ctx)
{
	return lem_if_suspend(ctx);
}

/*
 * Suspend/resume device methods.
 */
static int
lem_if_suspend(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx);

	INIT_DEBUGOUT("lem_suspend: begin"); 
	
	lem_release_manageability(adapter);
	lem_release_hw_control(adapter);
	lem_enable_wakeup(adapter);
	return (0);
}

static int
lem_if_resume(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx);

	lem_if_init(ctx);
	lem_init_manageability(adapter);
	return (0);
}

static void
lem_if_led_func(if_ctx_t ctx, int onoff) 
{
	struct adapter *adapter = iflib_get_softc(ctx);
	if (onoff) {
		e1000_setup_led(&adapter->hw);
		e1000_led_on(&adapter->hw);
	} else {
		e1000_led_off(&adapter->hw);
		e1000_cleanup_led(&adapter->hw);
	}	
}

/*********************************************************************
 *  Ioctl entry point
 *
 *  em_ioctl is called when the user wants to configure the
 *  interface.
 *
 *  return 0 on success, positive on failure
 **********************************************************************/

static int
lem_if_mtu_set(if_ctx_t ctx, uint32_t mtu)
{
	struct adapter	*adapter = iflib_get_softc(ctx); 
    if_t ifp = iflib_get_ifp(ctx); 
    int max_frame_size; 
	
	IOCTL_DEBUGOUT("ioctl rcv'd: SIOCSIFMTU (Set Interface MTU)");
	
	switch (adapter->hw.mac.type) {
	case e1000_82542:
		max_frame_size = ETHER_MAX_LEN;
		break;
	default:
		max_frame_size = MAX_JUMBO_FRAME_SIZE;
	}
	
	if (mtu > max_frame_size - ETHER_HDR_LEN - ETHER_CRC_LEN) {
		return(EINVAL);
	}
	
	adapter->max_frame_size =
		if_getmtu(ifp) + ETHER_HDR_LEN + ETHER_CRC_LEN;

	if (adapter->hw.mac.type == e1000_82542 && 
		adapter->hw.revision_id == E1000_REVISION_2) {
		lem_initialize_receive_unit(adapter);
	}
	
	return(0); 
}

static void
lem_if_media_set(if_ctx_t ctx)
{
	struct adapter	*adapter = iflib_get_softc(ctx); 
     device_t dev = iflib_get_dev(ctx); 
	
	if (e1000_check_reset_block(&adapter->hw)) {
			device_printf(dev, "Media change is"
			    " blocked due to SOL/IDER session.\n");
		}
}
		
/*********************************************************************
 *  Init entry point
 *
 *  This routine is used in two ways. It is used by the stack as
 *  init entry point in network interface structure. It is also used
 *  by the driver as a hw/sw initialization routine to get to a
 *  consistent state.
 *
 *  return 0 on success, positive on failure
 **********************************************************************/

static void
lem_if_init(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	if_t ifp = iflib_get_ifp(ctx); 
	device_t	dev = iflib_get_dev(ctx); 
	u32		pba;

	INIT_DEBUGOUT("lem_init: begin");

	/*
	 * Packet Buffer Allocation (PBA)
	 * Writing PBA sets the receive portion of the buffer
	 * the remainder is used for the transmit buffer.
	 *
	 * Devices before the 82547 had a Packet Buffer of 64K.
	 *   Default allocation: PBA=48K for Rx, leaving 16K for Tx.
	 * After the 82547 the buffer was reduced to 40K.
	 *   Default allocation: PBA=30K for Rx, leaving 10K for Tx.
	 *   Note: default does not leave enough room for Jumbo Frame >10k.
	 */
	switch (adapter->hw.mac.type) {
	case e1000_82547:
	case e1000_82547_rev_2: /* 82547: Total Packet Buffer is 40K */
		if (adapter->max_frame_size > 8192)
			pba = E1000_PBA_22K; /* 22K for Rx, 18K for Tx */
		else
			pba = E1000_PBA_30K; /* 30K for Rx, 10K for Tx */
		adapter->tx_fifo_head = 0;
		adapter->tx_head_addr = pba << EM_TX_HEAD_ADDR_SHIFT;
		adapter->tx_fifo_size =
		    (E1000_PBA_40K - pba) << EM_PBA_BYTES_SHIFT;
		break;
	default:
		/* Devices before 82547 had a Packet Buffer of 64K.   */
		if (adapter->max_frame_size > 8192)
			pba = E1000_PBA_40K; /* 40K for Rx, 24K for Tx */
		else
			pba = E1000_PBA_48K; /* 48K for Rx, 16K for Tx */
	}

	INIT_DEBUGOUT1("lem_init: pba=%dK",pba);
	E1000_WRITE_REG(&adapter->hw, E1000_PBA, pba);
	
	/* Get the latest mac address, User can use a LAA */
        bcopy(if_getlladdr(ifp), adapter->hw.mac.addr,
              ETHER_ADDR_LEN);

	/* Put the address into the Receive Address Array */
	e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	/* Initialize the hardware */
	if (lem_hardware_init(adapter)) {
		device_printf(dev, "Unable to initialize the hardware\n");
		return;
	}
	lem_update_link_status(adapter);

	/* Setup VLAN support, basic and offload if available */
	E1000_WRITE_REG(&adapter->hw, E1000_VET, ETHERTYPE_VLAN);

	/* Set hardware offload abilities */
	if_clearhwassist(ifp);
	if (adapter->hw.mac.type >= e1000_82543) {
		if (if_getcapenable(ifp) & IFCAP_TXCSUM)
			if_sethwassistbits(ifp, CSUM_TCP | CSUM_UDP, 0);
	}

	/* Configure for OS presence */
	lem_init_manageability(adapter);

	/* Prepare transmit descriptors and buffers */
	lem_initialize_transmit_unit(adapter);
	lem_initialize_receive_unit(adapter);

	/* Use real VLAN Filter support? */
	if (if_getcapenable(ifp) & IFCAP_VLAN_HWTAGGING) {
		if (if_getcapenable(ifp) & IFCAP_VLAN_HWFILTER)
			/* Use real VLAN Filter support */
			lem_setup_vlan_hw_support(adapter);
		else {
			u32 ctrl;
			ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
			ctrl |= E1000_CTRL_VME;
			E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
                }
	}

	if_setdrvflagbits(ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);

	e1000_clear_hw_cntrs_base_generic(&adapter->hw);

	lem_if_intr_enable(ctx);

	/* AMT based hardware can now take control from firmware */
	if (adapter->has_manage && adapter->has_amt)
		lem_get_hw_control(adapter);
}

/*********************************************************************
 *
 *  Fast Legacy/MSI Combined Interrupt Service routine  
 *
 *********************************************************************/
static int
lem_irq_fast(void *arg)
{
	struct adapter	*adapter = arg;
	u32		reg_icr;

	reg_icr = E1000_READ_REG(&adapter->hw, E1000_ICR);

	/* Hot eject?  */
	if (reg_icr == 0xffffffff)
		return FILTER_STRAY;

	/* Definitely not our interrupt.  */
	if (reg_icr == 0x0)
		return FILTER_STRAY;

	/* Link status change */
	if (reg_icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		adapter->hw.mac.get_link_status = 1;
		//lem_update_link_status(adapter);
	}

	if (reg_icr & E1000_ICR_RXO)
		adapter->rx_overruns++;

	return FILTER_HANDLED;
}

/*********************************************************************
 *
 *  Legacy Interrupt Service routine
 *
 **********************************************************************/
int 
lem_intr(void *arg)
{
   	struct adapter *adapter = arg;
	if_t ifp = iflib_get_ifp(adapter->ctx); 
    u32 reg_icr;

		if ((if_getcapenable(ifp) & IFCAP_POLLING) ||
	    ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0))
			return (FILTER_HANDLED);

	reg_icr = E1000_READ_REG(&adapter->hw, E1000_ICR);
	if (reg_icr & E1000_ICR_RXO)
		adapter->rx_overruns++;

	if ((reg_icr == 0xffffffff) || (reg_icr == 0)) {
		return (FILTER_HANDLED);
	}

	if (reg_icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		adapter->hw.mac.get_link_status = 1;
		lem_update_link_status(adapter);
		return (FILTER_HANDLED);
	}
	
	return (FILTER_SCHEDULE_THREAD); 
}

/*********************************************************************
 *
 *  Media Ioctl callback
 *
 *  This routine is called whenever the user queries the status of
 *  the interface using ifconfig.
 *
 **********************************************************************/
static void
lem_if_media_status(if_ctx_t ctx, struct ifmediareq *ifmr)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	u_char fiber_type = IFM_1000_SX;

	INIT_DEBUGOUT("lem_media_status: begin");

	lem_update_link_status(adapter);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (!adapter->link_active) {
		return;
	}

	ifmr->ifm_status |= IFM_ACTIVE;

	if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
	    (adapter->hw.phy.media_type == e1000_media_type_internal_serdes)) {
		if (adapter->hw.mac.type == e1000_82545)
			fiber_type = IFM_1000_LX;
		ifmr->ifm_active |= fiber_type | IFM_FDX;
	} else {
		switch (adapter->link_speed) {
		case 10:
			ifmr->ifm_active |= IFM_10_T;
			break;
		case 100:
			ifmr->ifm_active |= IFM_100_TX;
			break;
		case 1000:
			ifmr->ifm_active |= IFM_1000_T;
			break;
		}
		if (adapter->link_duplex == FULL_DUPLEX)
			ifmr->ifm_active |= IFM_FDX;
		else
			ifmr->ifm_active |= IFM_HDX;
	}
}

/*********************************************************************
 *
 *  Media Ioctl callback
 *
 *  This routine is called when the user changes speed/duplex using
 *  media/mediopt option with ifconfig.
 *
 **********************************************************************/
static int
	lem_if_media_change(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	device_t dev = iflib_get_dev(ctx); 
	struct ifmedia  *ifm = adapter->media;

	INIT_DEBUGOUT("lem_media_change: begin");

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

	switch (IFM_SUBTYPE(ifm->ifm_media)) {
	case IFM_AUTO:
		adapter->hw.mac.autoneg = DO_AUTO_NEG;
		adapter->hw.phy.autoneg_advertised = AUTONEG_ADV_DEFAULT;
		break;
	case IFM_1000_LX:
	case IFM_1000_SX:
	case IFM_1000_T:
		adapter->hw.mac.autoneg = DO_AUTO_NEG;
		adapter->hw.phy.autoneg_advertised = ADVERTISE_1000_FULL;
		break;
	case IFM_100_TX:
		adapter->hw.mac.autoneg = FALSE;
		adapter->hw.phy.autoneg_advertised = 0;
		if ((ifm->ifm_media & IFM_GMASK) == IFM_FDX)
			adapter->hw.mac.forced_speed_duplex = ADVERTISE_100_FULL;
		else
			adapter->hw.mac.forced_speed_duplex = ADVERTISE_100_HALF;
		break;
	case IFM_10_T:
		adapter->hw.mac.autoneg = FALSE;
		adapter->hw.phy.autoneg_advertised = 0;
		if ((ifm->ifm_media & IFM_GMASK) == IFM_FDX)
			adapter->hw.mac.forced_speed_duplex = ADVERTISE_10_FULL;
		else
			adapter->hw.mac.forced_speed_duplex = ADVERTISE_10_HALF;
		break;
	default:
		device_printf(dev, "Unsupported media type\n");
	}

	return (0);
}

static int
lem_if_promisc_set(if_ctx_t ctx, int flags)
{
    struct adapter *adapter =  iflib_get_softc(ctx);
    if_t ifp = iflib_get_ifp(ctx); 
	u32		reg_rctl;
	int		mcnt = 0;
    
	/* Disable */
	reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	reg_rctl &=  (~E1000_RCTL_UPE);
	if (if_getflags(ifp) & IFF_ALLMULTI)
		mcnt = MAX_NUM_MULTICAST_ADDRESSES;
	else
		mcnt = if_multiaddr_count(ifp, MAX_NUM_MULTICAST_ADDRESSES);

	/* Don't disable if in MAX groups */
	if (mcnt < MAX_NUM_MULTICAST_ADDRESSES)
		reg_rctl &=  (~E1000_RCTL_MPE);
	reg_rctl &=  (~E1000_RCTL_SBP);
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);

    /* Enable */
	reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);

	if (if_getflags(ifp) & IFF_PROMISC) {
		reg_rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		/* Turn this on if you want to see bad packets */
		if (lem_debug_sbp)
			reg_rctl |= E1000_RCTL_SBP;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
	} else if (if_getflags(ifp) & IFF_ALLMULTI) {
		reg_rctl |= E1000_RCTL_MPE;
		reg_rctl &= ~E1000_RCTL_UPE;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
	}
	
	return (0); 
}


/*********************************************************************
 *  Multicast Update
 *
 *  This routine is called whenever multicast address list is updated.
 *
 **********************************************************************/

static void
lem_if_multi_set(if_ctx_t ctx)
{
	struct adapter *adapter =  iflib_get_softc(ctx);
	if_t ifp = iflib_get_ifp(ctx); 
	u32 reg_rctl = 0;
	u8  *mta; /* Multicast array memory */
	int mcnt = 0;

	IOCTL_DEBUGOUT("lem_set_multi: begin");

	MPASS(adapter->mta != NULL);
	mta = adapter->mta;
	bzero(mta, sizeof(u8) * ETH_ADDR_LEN * MAX_NUM_MULTICAST_ADDRESSES);

	if (adapter->hw.mac.type == e1000_82542 && 
	    adapter->hw.revision_id == E1000_REVISION_2) {
		reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		if (adapter->hw.bus.pci_cmd_word & CMD_MEM_WRT_INVALIDATE)
			e1000_pci_clear_mwi(&adapter->hw);
		reg_rctl |= E1000_RCTL_RST;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
		msec_delay(5);
	}

	if_multiaddr_array(ifp, mta, &mcnt, MAX_NUM_MULTICAST_ADDRESSES);

	if (mcnt >= MAX_NUM_MULTICAST_ADDRESSES) {
		reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		reg_rctl |= E1000_RCTL_MPE;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
	} else
		e1000_update_mc_addr_list(&adapter->hw, mta, mcnt);

	if (adapter->hw.mac.type == e1000_82542 && 
	    adapter->hw.revision_id == E1000_REVISION_2) {
		reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		reg_rctl &= ~E1000_RCTL_RST;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
		msec_delay(5);
		if (adapter->hw.bus.pci_cmd_word & CMD_MEM_WRT_INVALIDATE)
			e1000_pci_set_mwi(&adapter->hw);
	}
}

/*********************************************************************
 *  Timer routine
 *
 *  This routine checks for link status and updates statistics.
 *
 **********************************************************************/
static void
	lem_if_timer(if_ctx_t ctx, uint16_t qid)
{
	struct adapter	*adapter = iflib_get_softc(ctx); 

	lem_update_link_status(adapter);
	lem_update_stats_counters(adapter);

	lem_smartspeed(adapter);
}

void
lem_update_link_status(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	if_t ifp = iflib_get_ifp(adapter->ctx); 
	device_t dev = iflib_get_dev(adapter->ctx); 
	u32 link_check = 0;

	/* Get the cached link value or read phy for real */
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (hw->mac.get_link_status) {
			/* Do the work to read phy */
			e1000_check_for_link(hw);
			link_check = !hw->mac.get_link_status;
			if (link_check) /* ESB2 fix */
				e1000_cfg_on_link_up(hw);
		} else
			link_check = TRUE;
		break;
	case e1000_media_type_fiber:
		e1000_check_for_link(hw);
		link_check = (E1000_READ_REG(hw, E1000_STATUS) &
                                 E1000_STATUS_LU);
		break;
	case e1000_media_type_internal_serdes:
		e1000_check_for_link(hw);
		link_check = adapter->hw.mac.serdes_has_link;
		break;
	default:
	case e1000_media_type_unknown:
		break;
	}

	/* Now check for a transition */
	if (link_check && (adapter->link_active == 0)) {
		e1000_get_speed_and_duplex(hw, &adapter->link_speed,
		    &adapter->link_duplex);
		if (bootverbose)
			device_printf(dev, "Link is up %d Mbps %s\n",
			    adapter->link_speed,
			    ((adapter->link_duplex == FULL_DUPLEX) ?
			    "Full Duplex" : "Half Duplex"));
		adapter->link_active = 1;
		adapter->smartspeed = 0;
		if_setbaudrate(ifp, adapter->link_speed * 1000000);
		iflib_link_state_change(adapter->ctx, LINK_STATE_UP);
	} else if (!link_check && (adapter->link_active == 1)) {
		if_setbaudrate(ifp, 0);
		adapter->link_speed = 0;
		adapter->link_duplex = 0;
		if (bootverbose)
			device_printf(dev, "Link is Down\n");
		adapter->link_active = 0;
		/* Link down, disable watchdog */
		adapter->watchdog_check = FALSE;
		iflib_link_state_change(adapter->ctx, LINK_STATE_DOWN);
	}
}

/*********************************************************************
 *
 *  This routine disables all traffic on the adapter by issuing a
 *  global reset on the MAC and deallocates TX/RX buffers.
 *
 *  This routine should always be called with BOTH the CORE
 *  and TX locks.
 **********************************************************************/

static void
lem_if_stop(if_ctx_t ctx)
{
	struct adapter	*adapter = iflib_get_softc(ctx);

	INIT_DEBUGOUT("lem_stop: begin");

	e1000_reset_hw(&adapter->hw);
	if (adapter->hw.mac.type >= e1000_82544)
		E1000_WRITE_REG(&adapter->hw, E1000_WUC, 0);

	e1000_led_off(&adapter->hw);
	e1000_cleanup_led(&adapter->hw);
}


/*********************************************************************
 *
 *  Determine hardware revision.
 *
 **********************************************************************/
static void
lem_identify_hardware(struct adapter *adapter)
{
	 device_t dev = iflib_get_dev(adapter->ctx); 

	/* Make sure our PCI config space has the necessary stuff set */
	/* pci_enable_busmaster(dev); HANDLED IN IFLIB_DEVICE_ATTACH */
	adapter->hw.bus.pci_cmd_word = pci_read_config(dev, PCIR_COMMAND, 2);

	/* Save off the information about this board */
	adapter->hw.vendor_id = pci_get_vendor(dev);
	adapter->hw.device_id = pci_get_device(dev);
	adapter->hw.revision_id = pci_read_config(dev, PCIR_REVID, 1);
	adapter->hw.subsystem_vendor_id =
	    pci_read_config(dev, PCIR_SUBVEND_0, 2);
	adapter->hw.subsystem_device_id =
	    pci_read_config(dev, PCIR_SUBDEV_0, 2);

	/* Do Shared Code Init and Setup */
	if (e1000_set_mac_type(&adapter->hw)) {
		device_printf(dev, "Setup init failure\n");
		return;
	}
}

static int
lem_allocate_pci_resources(struct adapter *adapter)
{
	device_t	dev = iflib_get_dev(adapter->ctx); 
	int		val, rid, error = E1000_SUCCESS;

	rid = PCIR_BAR(0);
	adapter->memory = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);
	if (adapter->memory == NULL) {
		device_printf(dev, "Unable to allocate bus resource: memory\n");
		return (ENXIO);
	}
	adapter->osdep.mem_bus_space_tag =
	    rman_get_bustag(adapter->memory);
	adapter->osdep.mem_bus_space_handle =
	    rman_get_bushandle(adapter->memory);
	adapter->hw.hw_addr = (u8 *)&adapter->osdep.mem_bus_space_handle;

	/* Only older adapters use IO mapping */
	if (adapter->hw.mac.type > e1000_82543) {
		/* Figure our where our IO BAR is ? */
		for (rid = PCIR_BAR(0); rid < PCIR_CIS;) {
			val = pci_read_config(dev, rid, 4);
			if (EM_BAR_TYPE(val) == EM_BAR_TYPE_IO) {
				adapter->io_rid = rid;
				break;
			}
			rid += 4;
			/* check for 64bit BAR */
			if (EM_BAR_MEM_TYPE(val) == EM_BAR_MEM_TYPE_64BIT)
				rid += 4;
		}
		if (rid >= PCIR_CIS) {
			device_printf(dev, "Unable to locate IO BAR\n");
			return (ENXIO);
		}
		adapter->ioport = bus_alloc_resource_any(dev,
		    SYS_RES_IOPORT, &adapter->io_rid, RF_ACTIVE);
		if (adapter->ioport == NULL) {
			device_printf(dev, "Unable to allocate bus resource: "
			    "ioport\n");
			return (ENXIO);
		}
		adapter->hw.io_base = 0;
		adapter->osdep.io_bus_space_tag =
		    rman_get_bustag(adapter->ioport);
		adapter->osdep.io_bus_space_handle =
		    rman_get_bushandle(adapter->ioport);
	}

	adapter->hw.back = &adapter->osdep;

	return (error);
}

/*********************************************************************
 *
 *  Setup the Legacy or MSI Interrupt handler
 *
 **********************************************************************/

static int
lem_if_msix_intr_assign(if_ctx_t ctx, int msix)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	int error, rid = 0;

	/* Manually turn off all interrupts */
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, 0xffffffff);


    /* Do Legacy Setup ? */
	if (lem_use_legacy_irq) {
		error = iflib_irq_alloc_generic(ctx, &adapter->irq, rid, IFLIB_INTR_RX,
										lem_intr, adapter, 0, "irq-legacy");
	} else {
		error = iflib_irq_alloc_generic(ctx, &adapter->irq, rid, IFLIB_INTR_RX,
										lem_irq_fast, adapter, 0, "irq");
    }
		
	if (error) {
		iflib_irq_free(ctx, &adapter->irq); 
		device_printf(iflib_get_dev(ctx), "Failed to allocate interrupt err: %d", error);
        return (error);                 		
	}

    iflib_softirq_alloc_generic(ctx, rid, IFLIB_INTR_TX, adapter, 0, "irq");

	return 0; 
}


static void
lem_free_pci_resources(struct adapter *adapter)
{
	device_t dev = iflib_get_dev(adapter->ctx); 
	
	/* Release all msix resources */
	iflib_irq_free(adapter->ctx, &adapter->irq); 
	
	if (adapter->memory != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(0), adapter->memory);

	if (adapter->ioport != NULL)
		bus_release_resource(dev, SYS_RES_IOPORT,
		    adapter->io_rid, adapter->ioport);
}


/*********************************************************************
 *
 *  Initialize the hardware to a configuration
 *  as specified by the adapter structure.
 *
 **********************************************************************/
static int
lem_hardware_init(struct adapter *adapter)
{
	device_t dev = iflib_get_dev(adapter->ctx); 
	u16 	rx_buffer_size;

	INIT_DEBUGOUT("lem_hardware_init: begin");

	/* Issue a global reset */
	e1000_reset_hw(&adapter->hw);

	/* When hardware is reset, fifo_head is also reset */
	adapter->tx_fifo_head = 0;

	/*
	 * These parameters control the automatic generation (Tx) and
	 * response (Rx) to Ethernet PAUSE frames.
	 * - High water mark should allow for at least two frames to be
	 *   received after sending an XOFF.
	 * - Low water mark works best when it is very near the high water mark.
	 *   This allows the receiver to restart by sending XON when it has
	 *   drained a bit. Here we use an arbitary value of 1500 which will
	 *   restart after one full frame is pulled from the buffer. There
	 *   could be several smaller frames in the buffer and if so they will
	 *   not trigger the XON until their total number reduces the buffer
	 *   by 1500.
	 * - The pause time is fairly large at 1000 x 512ns = 512 usec.
	 */
	rx_buffer_size = ((E1000_READ_REG(&adapter->hw, E1000_PBA) &
	    0xffff) << 10 );

	adapter->hw.fc.high_water = rx_buffer_size -
	    roundup2(adapter->max_frame_size, 1024);
	adapter->hw.fc.low_water = adapter->hw.fc.high_water - 1500;

	adapter->hw.fc.pause_time = EM_FC_PAUSE_TIME;
	adapter->hw.fc.send_xon = TRUE;

        /* Set Flow control, use the tunable location if sane */
        if ((lem_fc_setting >= 0) && (lem_fc_setting < 4))
                adapter->hw.fc.requested_mode = lem_fc_setting;
        else
                adapter->hw.fc.requested_mode = e1000_fc_none;

	if (e1000_init_hw(&adapter->hw) < 0) {
		device_printf(dev, "Hardware Initialization Failed\n");
		return (EIO);
	}

	e1000_check_for_link(&adapter->hw);

	return (0);
}

/*********************************************************************
 *
 *  Setup networking device structure and register an interface.
 *
 **********************************************************************/
static int
lem_setup_interface(device_t dev, struct adapter *adapter)
{
	if_t ifp = iflib_get_ifp(adapter->ctx);

	INIT_DEBUGOUT("lem_setup_interface: begin");

	if_setsendqlen(ifp, adapter->num_tx_desc - 1);
	if_setsendqready(ifp);

	if (adapter->hw.mac.type >= e1000_82543) {
		if_setcapabilitiesbit(ifp, IFCAP_HWCSUM | IFCAP_VLAN_HWCSUM, 0);
		if_setcapenablebit(ifp, IFCAP_HWCSUM | IFCAP_VLAN_HWCSUM, 0);
	}

	/*
	 * Tell the upper layer(s) we support long frames.
	 */
	if_setifheaderlen(ifp, sizeof(struct ether_vlan_header));
	if_setcapabilitiesbit(ifp, IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_MTU, 0);
	if_setcapenablebit(ifp, IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_MTU, 0);

	/*
	** Dont turn this on by default, if vlans are
	** created on another pseudo device (eg. lagg)
	** then vlan events are not passed thru, breaking
	** operation, but with HW FILTER off it works. If
	** using vlans directly on the em driver you can
	** enable this and get full hardware tag filtering.
	*/
	if_setcapabilitiesbit(ifp, IFCAP_VLAN_HWFILTER, 0);

	/* Enable only WOL MAGIC by default */
	if (adapter->wol) {
		if_setcapabilitiesbit(ifp, IFCAP_WOL, 0);
		if_setcapenablebit(ifp, IFCAP_WOL_MAGIC, 0);
	}
		
	/*
	 * Specify the media types supported by this adapter and register
	 * callbacks to update media and link information
	 */
	if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
	    (adapter->hw.phy.media_type == e1000_media_type_internal_serdes)) {
		u_char fiber_type = IFM_1000_SX;	/* default type */

		if (adapter->hw.mac.type == e1000_82545)
			fiber_type = IFM_1000_LX;
		ifmedia_add(adapter->media, IFM_ETHER | fiber_type | IFM_FDX, 
			    0, NULL);
		ifmedia_add(adapter->media, IFM_ETHER | fiber_type, 0, NULL);
	} else {
		ifmedia_add(adapter->media, IFM_ETHER | IFM_10_T, 0, NULL);
		ifmedia_add(adapter->media, IFM_ETHER | IFM_10_T | IFM_FDX,
			    0, NULL);
		ifmedia_add(adapter->media, IFM_ETHER | IFM_100_TX,
			    0, NULL);
		ifmedia_add(adapter->media, IFM_ETHER | IFM_100_TX | IFM_FDX,
			    0, NULL);
		if (adapter->hw.phy.type != e1000_phy_ife) {
			ifmedia_add(adapter->media,
				IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
			ifmedia_add(adapter->media,
				IFM_ETHER | IFM_1000_T, 0, NULL);
		}
	}
	ifmedia_add(adapter->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(adapter->media, IFM_ETHER | IFM_AUTO);
	return (0);
}


/*********************************************************************
 *
 *  Workaround for SmartSpeed on 82541 and 82547 controllers
 *
 **********************************************************************/
static void
lem_smartspeed(struct adapter *adapter)
{
	u16 phy_tmp;

	if (adapter->link_active || (adapter->hw.phy.type != e1000_phy_igp) ||
	    adapter->hw.mac.autoneg == 0 ||
	    (adapter->hw.phy.autoneg_advertised & ADVERTISE_1000_FULL) == 0)
		return;

	if (adapter->smartspeed == 0) {
		/* If Master/Slave config fault is asserted twice,
		 * we assume back-to-back */
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_STATUS, &phy_tmp);
		if (!(phy_tmp & SR_1000T_MS_CONFIG_FAULT))
			return;
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_STATUS, &phy_tmp);
		if (phy_tmp & SR_1000T_MS_CONFIG_FAULT) {
			e1000_read_phy_reg(&adapter->hw,
			    PHY_1000T_CTRL, &phy_tmp);
			if(phy_tmp & CR_1000T_MS_ENABLE) {
				phy_tmp &= ~CR_1000T_MS_ENABLE;
				e1000_write_phy_reg(&adapter->hw,
				    PHY_1000T_CTRL, phy_tmp);
				adapter->smartspeed++;
				if(adapter->hw.mac.autoneg &&
				   !e1000_copper_link_autoneg(&adapter->hw) &&
				   !e1000_read_phy_reg(&adapter->hw,
				    PHY_CONTROL, &phy_tmp)) {
					phy_tmp |= (MII_CR_AUTO_NEG_EN |
						    MII_CR_RESTART_AUTO_NEG);
					e1000_write_phy_reg(&adapter->hw,
					    PHY_CONTROL, phy_tmp);
				}
			}
		}
		return;
	} else if(adapter->smartspeed == EM_SMARTSPEED_DOWNSHIFT) {
		/* If still no link, perhaps using 2/3 pair cable */
		e1000_read_phy_reg(&adapter->hw, PHY_1000T_CTRL, &phy_tmp);
		phy_tmp |= CR_1000T_MS_ENABLE;
		e1000_write_phy_reg(&adapter->hw, PHY_1000T_CTRL, phy_tmp);
		if(adapter->hw.mac.autoneg &&
		   !e1000_copper_link_autoneg(&adapter->hw) &&
		   !e1000_read_phy_reg(&adapter->hw, PHY_CONTROL, &phy_tmp)) {
			phy_tmp |= (MII_CR_AUTO_NEG_EN |
				    MII_CR_RESTART_AUTO_NEG);
			e1000_write_phy_reg(&adapter->hw, PHY_CONTROL, phy_tmp);
		}
	}
	/* Restart process after EM_SMARTSPEED_MAX iterations */
	if(adapter->smartspeed++ == EM_SMARTSPEED_MAX)
		adapter->smartspeed = 0;
}

/*********************************************************************
 *
 *  Enable transmit unit.
 *
 **********************************************************************/
static void
lem_initialize_transmit_unit(struct adapter *adapter)
{
	u32	tctl, tipg = 0;
	u64	bus_addr;

	 INIT_DEBUGOUT("lem_initialize_transmit_unit: begin");
	/* Setup the Base and Length of the Tx Descriptor Ring */
	bus_addr = adapter->txdma.dma_paddr;
	E1000_WRITE_REG(&adapter->hw, E1000_TDLEN(0),
	    adapter->num_tx_desc * sizeof(struct e1000_tx_desc));
	E1000_WRITE_REG(&adapter->hw, E1000_TDBAH(0),
	    (u32)(bus_addr >> 32));
	E1000_WRITE_REG(&adapter->hw, E1000_TDBAL(0),
	    (u32)bus_addr);
	/* Setup the HW Tx Head and Tail descriptor pointers */
	E1000_WRITE_REG(&adapter->hw, E1000_TDT(0), 0);
	E1000_WRITE_REG(&adapter->hw, E1000_TDH(0), 0);

	HW_DEBUGOUT2("Base = %x, Length = %x\n",
	    E1000_READ_REG(&adapter->hw, E1000_TDBAL(0)),
	    E1000_READ_REG(&adapter->hw, E1000_TDLEN(0)));

	/* Set the default values for the Tx Inter Packet Gap timer */
	switch (adapter->hw.mac.type) {
	case e1000_82542:
		tipg = DEFAULT_82542_TIPG_IPGT;
		tipg |= DEFAULT_82542_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82542_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
		break;
	default:
		if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
		    (adapter->hw.phy.media_type ==
		    e1000_media_type_internal_serdes))
			tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
		else
			tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
		tipg |= DEFAULT_82543_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82543_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
	}

	E1000_WRITE_REG(&adapter->hw, E1000_TIPG, tipg);
	E1000_WRITE_REG(&adapter->hw, E1000_TIDV, adapter->tx_int_delay.iidi_value);
	if(adapter->hw.mac.type >= e1000_82540)
		E1000_WRITE_REG(&adapter->hw, E1000_TADV,
		    adapter->tx_abs_int_delay.iidi_value);

	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		   (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);

	/* Setup Transmit Descriptor Base Settings */   
	adapter->txd_cmd = E1000_TXD_CMD_IFCS;

	if (adapter->tx_int_delay.iidi_value > 0)
		adapter->txd_cmd |= E1000_TXD_CMD_IDE;
}

/*********************************************************************
 *
 *  Enable receive unit.
 *
 **********************************************************************/

static void
lem_initialize_receive_unit(struct adapter *adapter)
{
	if_t ifp = iflib_get_ifp(adapter->ctx); 
	u64	bus_addr;
	u32	rctl, rxcsum;

	INIT_DEBUGOUT("lem_initialize_receive_unit: begin");

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring
	 */
	rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);

	if (adapter->hw.mac.type >= e1000_82540) {
		E1000_WRITE_REG(&adapter->hw, E1000_RADV,
		    adapter->rx_abs_int_delay.iidi_value);
		/*
		 * Set the interrupt throttling rate. Value is calculated
		 * as DEFAULT_ITR = 1/(MAX_INTS_PER_SEC * 256ns)
		 */
		E1000_WRITE_REG(&adapter->hw, E1000_ITR, DEFAULT_ITR);
	}

	/* Setup the Base and Length of the Rx Descriptor Ring */
	bus_addr = adapter->rxdma.dma_paddr;
	E1000_WRITE_REG(&adapter->hw, E1000_RDLEN(0),
	    adapter->num_rx_desc * sizeof(struct e1000_rx_desc));
	E1000_WRITE_REG(&adapter->hw, E1000_RDBAH(0),
	    (u32)(bus_addr >> 32));
	E1000_WRITE_REG(&adapter->hw, E1000_RDBAL(0),
	    (u32)bus_addr);

	/* Setup the Receive Control Register */
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO |
		   E1000_RCTL_RDMTS_HALF |
		   (adapter->hw.mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/* Make sure VLAN Filters are off */
	rctl &= ~E1000_RCTL_VFE;

	if (e1000_tbi_sbp_enabled_82543(&adapter->hw))
		rctl |= E1000_RCTL_SBP;
	else
		rctl &= ~E1000_RCTL_SBP;

	switch (adapter->rx_buffer_len) {
	default:
	case 2048:
		rctl |= E1000_RCTL_SZ_2048;
		break;
	case 4096:
		rctl |= E1000_RCTL_SZ_4096 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;
	case 8192:
		rctl |= E1000_RCTL_SZ_8192 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;
	case 16384:
		rctl |= E1000_RCTL_SZ_16384 |
		    E1000_RCTL_BSEX | E1000_RCTL_LPE;
		break;
	}

	if (if_getmtu(ifp) > ETHERMTU)
		rctl |= E1000_RCTL_LPE;
	else
		rctl &= ~E1000_RCTL_LPE;

	/* Enable 82543 Receive Checksum Offload for TCP and UDP */
	if ((adapter->hw.mac.type >= e1000_82543) &&
	    (if_getcapenable(ifp) & IFCAP_RXCSUM)) {
		rxcsum = E1000_READ_REG(&adapter->hw, E1000_RXCSUM);
		rxcsum |= (E1000_RXCSUM_IPOFL | E1000_RXCSUM_TUOFL);
		E1000_WRITE_REG(&adapter->hw, E1000_RXCSUM, rxcsum);
	}

	/* Enable Receives */
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);

	/*
	 * Setup the HW Rx Head and
	 * Tail Descriptor Pointers
	 */
	E1000_WRITE_REG(&adapter->hw, E1000_RDH(0), 0);
	rctl = adapter->num_rx_desc - 1; /* default RDT value */

	E1000_WRITE_REG(&adapter->hw, E1000_RDT(0), rctl);

	return;
}

/*
 * This routine is run via an vlan
 * config EVENT
 */
static void
lem_if_vlan_register(if_ctx_t ctx, u16 vtag)
{
	struct adapter	*adapter = iflib_get_softc(ctx);
	u32		index, bit;

	if ((vtag == 0) || (vtag > 4095))       /* Invalid ID */
                return;

	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	adapter->shadow_vfta[index] |= (1 << bit);
	++adapter->num_vlans;
}

/*
 * This routine is run via an vlan
 * unconfig EVENT
 */
static void
lem_if_vlan_unregister(if_ctx_t ctx, u16 vtag)
{
	struct adapter	*adapter = iflib_get_softc(ctx);
	u32		index, bit;

	if ((vtag == 0) || (vtag > 4095))       /* Invalid */
                return;

	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	adapter->shadow_vfta[index] &= ~(1 << bit);
	--adapter->num_vlans;
}

static void
lem_setup_vlan_hw_support(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32             reg;

	/*
	** We get here thru init_locked, meaning
	** a soft reset, this has already cleared
	** the VFTA and other state, so if there
	** have been no vlan's registered do nothing.
	*/
	if (adapter->num_vlans == 0)
                return;

	/*
	** A soft reset zero's out the VFTA, so
	** we need to repopulate it now.
	*/
	for (int i = 0; i < EM_VFTA_SIZE; i++)
                if (adapter->shadow_vfta[i] != 0)
			E1000_WRITE_REG_ARRAY(hw, E1000_VFTA,
                            i, adapter->shadow_vfta[i]);

	reg = E1000_READ_REG(hw, E1000_CTRL);
	reg |= E1000_CTRL_VME;
	E1000_WRITE_REG(hw, E1000_CTRL, reg);

	/* Enable the Filter Table */
	reg = E1000_READ_REG(hw, E1000_RCTL);
	reg &= ~E1000_RCTL_CFIEN;
	reg |= E1000_RCTL_VFE;
	E1000_WRITE_REG(hw, E1000_RCTL, reg);
}

static void
lem_if_intr_enable(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct e1000_hw *hw = &adapter->hw;
	u32 ims_mask = IMS_ENABLE_MASK;

	E1000_WRITE_REG(hw, E1000_IMS, ims_mask);
}

static void
lem_if_intr_disable(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct e1000_hw *hw = &adapter->hw;

	E1000_WRITE_REG(hw, E1000_IMC, 0xffffffff);
}

/*
 * Bit of a misnomer, what this really means is
 * to enable OS management of the system... aka
 * to disable special hardware management features 
 */
static void
lem_init_manageability(struct adapter *adapter)
{
	/* A shared code workaround */
	if (adapter->has_manage) {
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);
		/* disable hardware interception of ARP */
		manc &= ~(E1000_MANC_ARP_EN);
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*
 * Give control back to hardware management
 * controller if there is one.
 */
static void
lem_release_manageability(struct adapter *adapter)
{
	if (adapter->has_manage) {
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);

		/* re-enable hardware interception of ARP */
		manc |= E1000_MANC_ARP_EN;
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*
 * lem_get_hw_control sets the {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means
 * that the driver is loaded. For AMT version type f/w
 * this means that the network i/f is open.
 */
static void
lem_get_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext;

	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
	return;
}

/*
 * lem_release_hw_control resets {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is no longer loaded. For AMT versions of the
 * f/w this means that the network i/f is closed.
 */
static void
lem_release_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext;

	if (!adapter->has_manage)
		return;

	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
	return;
}

static int
lem_is_valid_ether_addr(u8 *addr)
{
	char zero_addr[6] = { 0, 0, 0, 0, 0, 0 };

	if ((addr[0] & 1) || (!bcmp(addr, zero_addr, ETHER_ADDR_LEN))) {
		return (FALSE);
	}

	return (TRUE);
}

/*
** Parse the interface capabilities with regard
** to both system management and wake-on-lan for
** later use.
*/
static void
lem_get_wakeup(if_ctx_t ctx)
{
	struct adapter	*adapter = iflib_get_softc(ctx);
	device_t dev = iflib_get_dev(ctx); 
	u16		eeprom_data = 0, device_id, apme_mask;

	adapter->has_manage = e1000_enable_mng_pass_thru(&adapter->hw);
	apme_mask = EM_EEPROM_APME;

	switch (adapter->hw.mac.type) {
	case e1000_82542:
	case e1000_82543:
		break;
	case e1000_82544:
		e1000_read_nvm(&adapter->hw,
		    NVM_INIT_CONTROL2_REG, 1, &eeprom_data);
		apme_mask = EM_82544_APME;
		break;
	case e1000_82546:
	case e1000_82546_rev_3:
		if (adapter->hw.bus.func == 1) {
			e1000_read_nvm(&adapter->hw,
			    NVM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
			break;
		} else
			e1000_read_nvm(&adapter->hw,
			    NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	default:
		e1000_read_nvm(&adapter->hw,
		    NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	}
	if (eeprom_data & apme_mask)
		adapter->wol = (E1000_WUFC_MAG | E1000_WUFC_MC);
	/*
         * We have the eeprom settings, now apply the special cases
         * where the eeprom may be wrong or the board won't support
         * wake on lan on a particular port
	 */
	device_id = pci_get_device(dev);
        switch (device_id) {
	case E1000_DEV_ID_82546GB_PCIE:
		adapter->wol = 0;
		break;
	case E1000_DEV_ID_82546EB_FIBER:
	case E1000_DEV_ID_82546GB_FIBER:
		/* Wake events only supported on port A for dual fiber
		 * regardless of eeprom setting */
		if (E1000_READ_REG(&adapter->hw, E1000_STATUS) &
		    E1000_STATUS_FUNC_1)
			adapter->wol = 0;
		break;
	case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
                /* if quad port adapter, disable WoL on all but port A */
		if (global_quad_port_a != 0)
			adapter->wol = 0;
		/* Reset for multiple quad port adapters */
		if (++global_quad_port_a == 4)
			global_quad_port_a = 0;
                break;
	}
	return;
}


/*
 * Enable PCI Wake On Lan capability
 */
static void
lem_enable_wakeup(struct adapter *adapter)
{
    device_t dev = iflib_get_dev(adapter->ctx);
	if_t ifp = iflib_get_ifp(adapter->ctx); 
	u32		pmc, ctrl, ctrl_ext, rctl;
	u16     	status;

	if ((pci_find_cap(dev, PCIY_PMG, &pmc) != 0))
		return;

	/* Advertise the wakeup capability */
	ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
	ctrl |= (E1000_CTRL_SWDPIN2 | E1000_CTRL_SWDPIN3);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
	E1000_WRITE_REG(&adapter->hw, E1000_WUC, E1000_WUC_PME_EN);

	/* Keep the laser running on Fiber adapters */
	if (adapter->hw.phy.media_type == e1000_media_type_fiber ||
	    adapter->hw.phy.media_type == e1000_media_type_internal_serdes) {
		ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
		ctrl_ext |= E1000_CTRL_EXT_SDP3_DATA;
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT, ctrl_ext);
	}

	/*
	** Determine type of Wakeup: note that wol
	** is set with all bits on by default.
	*/
	if ((if_getcapenable(ifp) & IFCAP_WOL_MAGIC) == 0)
		adapter->wol &= ~E1000_WUFC_MAG;

	if ((if_getcapenable(ifp) & IFCAP_WOL_MCAST) == 0)
		adapter->wol &= ~E1000_WUFC_MC;
	else {
		rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		rctl |= E1000_RCTL_MPE;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
	}

	if (adapter->hw.mac.type == e1000_pchlan) {
		if (lem_enable_phy_wakeup(adapter))
			return;
	} else {
		E1000_WRITE_REG(&adapter->hw, E1000_WUC, E1000_WUC_PME_EN);
		E1000_WRITE_REG(&adapter->hw, E1000_WUFC, adapter->wol);
	}


        /* Request PME */
        status = pci_read_config(dev, pmc + PCIR_POWER_STATUS, 2);
	status &= ~(PCIM_PSTAT_PME | PCIM_PSTAT_PMEENABLE);
	if (if_getcapenable(ifp) & IFCAP_WOL)
		status |= PCIM_PSTAT_PME | PCIM_PSTAT_PMEENABLE;
        pci_write_config(dev, pmc + PCIR_POWER_STATUS, status, 2);

	return;
}

/*
** WOL in the newer chipset interfaces (pchlan)
** require thing to be copied into the phy
*/
static int
lem_enable_phy_wakeup(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 mreg, ret = 0;
	u16 preg;

	/* copy MAC RARs to PHY RARs */
	for (int i = 0; i < adapter->hw.mac.rar_entry_count; i++) {
		mreg = E1000_READ_REG(hw, E1000_RAL(i));
		e1000_write_phy_reg(hw, BM_RAR_L(i), (u16)(mreg & 0xFFFF));
		e1000_write_phy_reg(hw, BM_RAR_M(i),
		    (u16)((mreg >> 16) & 0xFFFF));
		mreg = E1000_READ_REG(hw, E1000_RAH(i));
		e1000_write_phy_reg(hw, BM_RAR_H(i), (u16)(mreg & 0xFFFF));
		e1000_write_phy_reg(hw, BM_RAR_CTRL(i),
		    (u16)((mreg >> 16) & 0xFFFF));
	}

	/* copy MAC MTA to PHY MTA */
	for (int i = 0; i < adapter->hw.mac.mta_reg_count; i++) {
		mreg = E1000_READ_REG_ARRAY(hw, E1000_MTA, i);
		e1000_write_phy_reg(hw, BM_MTA(i), (u16)(mreg & 0xFFFF));
		e1000_write_phy_reg(hw, BM_MTA(i) + 1,
		    (u16)((mreg >> 16) & 0xFFFF));
	}

	/* configure PHY Rx Control register */
	e1000_read_phy_reg(&adapter->hw, BM_RCTL, &preg);
	mreg = E1000_READ_REG(hw, E1000_RCTL);
	if (mreg & E1000_RCTL_UPE)
		preg |= BM_RCTL_UPE;
	if (mreg & E1000_RCTL_MPE)
		preg |= BM_RCTL_MPE;
	preg &= ~(BM_RCTL_MO_MASK);
	if (mreg & E1000_RCTL_MO_3)
		preg |= (((mreg & E1000_RCTL_MO_3) >> E1000_RCTL_MO_SHIFT)
				<< BM_RCTL_MO_SHIFT);
	if (mreg & E1000_RCTL_BAM)
		preg |= BM_RCTL_BAM;
	if (mreg & E1000_RCTL_PMCF)
		preg |= BM_RCTL_PMCF;
	mreg = E1000_READ_REG(hw, E1000_CTRL);
	if (mreg & E1000_CTRL_RFCE)
		preg |= BM_RCTL_RFCE;
	e1000_write_phy_reg(&adapter->hw, BM_RCTL, preg);

	/* enable PHY wakeup in MAC register */
	E1000_WRITE_REG(hw, E1000_WUC,
	    E1000_WUC_PHY_WAKE | E1000_WUC_PME_EN);
	E1000_WRITE_REG(hw, E1000_WUFC, adapter->wol);

	/* configure and enable PHY wakeup in PHY registers */
	e1000_write_phy_reg(&adapter->hw, BM_WUFC, adapter->wol);
	e1000_write_phy_reg(&adapter->hw, BM_WUC, E1000_WUC_PME_EN);

	/* activate PHY wakeup */
	ret = hw->phy.ops.acquire(hw);
	if (ret) {
		printf("Could not acquire PHY\n");
		return ret;
	}
	e1000_write_phy_reg_mdic(hw, IGP01E1000_PHY_PAGE_SELECT,
	                         (BM_WUC_ENABLE_PAGE << IGP_PAGE_SHIFT));
	ret = e1000_read_phy_reg_mdic(hw, BM_WUC_ENABLE_REG, &preg);
	if (ret) {
		printf("Could not read PHY page 769\n");
		goto out;
	}
	preg |= BM_WUC_ENABLE_BIT | BM_WUC_HOST_WU_BIT;
	ret = e1000_write_phy_reg_mdic(hw, BM_WUC_ENABLE_REG, preg);
	if (ret)
		printf("Could not set PHY Host Wakeup bit\n");
out:
	hw->phy.ops.release(hw);

	return ret;
}

/**********************************************************************
 *
 *  Update the board statistics counters.
 *
 **********************************************************************/
static void
lem_update_stats_counters(struct adapter *adapter)
{

	if(adapter->hw.phy.media_type == e1000_media_type_copper ||
	   (E1000_READ_REG(&adapter->hw, E1000_STATUS) & E1000_STATUS_LU)) {
		adapter->stats.symerrs += E1000_READ_REG(&adapter->hw, E1000_SYMERRS);
		adapter->stats.sec += E1000_READ_REG(&adapter->hw, E1000_SEC);
	}
	adapter->stats.crcerrs += E1000_READ_REG(&adapter->hw, E1000_CRCERRS);
	adapter->stats.mpc += E1000_READ_REG(&adapter->hw, E1000_MPC);
	adapter->stats.scc += E1000_READ_REG(&adapter->hw, E1000_SCC);
	adapter->stats.ecol += E1000_READ_REG(&adapter->hw, E1000_ECOL);

	adapter->stats.mcc += E1000_READ_REG(&adapter->hw, E1000_MCC);
	adapter->stats.latecol += E1000_READ_REG(&adapter->hw, E1000_LATECOL);
	adapter->stats.colc += E1000_READ_REG(&adapter->hw, E1000_COLC);
	adapter->stats.dc += E1000_READ_REG(&adapter->hw, E1000_DC);
	adapter->stats.rlec += E1000_READ_REG(&adapter->hw, E1000_RLEC);
	adapter->stats.xonrxc += E1000_READ_REG(&adapter->hw, E1000_XONRXC);
	adapter->stats.xontxc += E1000_READ_REG(&adapter->hw, E1000_XONTXC);
	adapter->stats.xoffrxc += E1000_READ_REG(&adapter->hw, E1000_XOFFRXC);
	adapter->stats.xofftxc += E1000_READ_REG(&adapter->hw, E1000_XOFFTXC);
	adapter->stats.fcruc += E1000_READ_REG(&adapter->hw, E1000_FCRUC);
	adapter->stats.prc64 += E1000_READ_REG(&adapter->hw, E1000_PRC64);
	adapter->stats.prc127 += E1000_READ_REG(&adapter->hw, E1000_PRC127);
	adapter->stats.prc255 += E1000_READ_REG(&adapter->hw, E1000_PRC255);
	adapter->stats.prc511 += E1000_READ_REG(&adapter->hw, E1000_PRC511);
	adapter->stats.prc1023 += E1000_READ_REG(&adapter->hw, E1000_PRC1023);
	adapter->stats.prc1522 += E1000_READ_REG(&adapter->hw, E1000_PRC1522);
	adapter->stats.gprc += E1000_READ_REG(&adapter->hw, E1000_GPRC);
	adapter->stats.bprc += E1000_READ_REG(&adapter->hw, E1000_BPRC);
	adapter->stats.mprc += E1000_READ_REG(&adapter->hw, E1000_MPRC);
	adapter->stats.gptc += E1000_READ_REG(&adapter->hw, E1000_GPTC);

	/* For the 64-bit byte counters the low dword must be read first. */
	/* Both registers clear on the read of the high dword */

	adapter->stats.gorc += E1000_READ_REG(&adapter->hw, E1000_GORCL) +
	    ((u64)E1000_READ_REG(&adapter->hw, E1000_GORCH) << 32);
	adapter->stats.gotc += E1000_READ_REG(&adapter->hw, E1000_GOTCL) +
	    ((u64)E1000_READ_REG(&adapter->hw, E1000_GOTCH) << 32);

	adapter->stats.rnbc += E1000_READ_REG(&adapter->hw, E1000_RNBC);
	adapter->stats.ruc += E1000_READ_REG(&adapter->hw, E1000_RUC);
	adapter->stats.rfc += E1000_READ_REG(&adapter->hw, E1000_RFC);
	adapter->stats.roc += E1000_READ_REG(&adapter->hw, E1000_ROC);
	adapter->stats.rjc += E1000_READ_REG(&adapter->hw, E1000_RJC);

	adapter->stats.tor += E1000_READ_REG(&adapter->hw, E1000_TORH);
	adapter->stats.tot += E1000_READ_REG(&adapter->hw, E1000_TOTH);

	adapter->stats.tpr += E1000_READ_REG(&adapter->hw, E1000_TPR);
	adapter->stats.tpt += E1000_READ_REG(&adapter->hw, E1000_TPT);
	adapter->stats.ptc64 += E1000_READ_REG(&adapter->hw, E1000_PTC64);
	adapter->stats.ptc127 += E1000_READ_REG(&adapter->hw, E1000_PTC127);
	adapter->stats.ptc255 += E1000_READ_REG(&adapter->hw, E1000_PTC255);
	adapter->stats.ptc511 += E1000_READ_REG(&adapter->hw, E1000_PTC511);
	adapter->stats.ptc1023 += E1000_READ_REG(&adapter->hw, E1000_PTC1023);
	adapter->stats.ptc1522 += E1000_READ_REG(&adapter->hw, E1000_PTC1522);
	adapter->stats.mptc += E1000_READ_REG(&adapter->hw, E1000_MPTC);
	adapter->stats.bptc += E1000_READ_REG(&adapter->hw, E1000_BPTC);

	if (adapter->hw.mac.type >= e1000_82543) {
		adapter->stats.algnerrc += 
		E1000_READ_REG(&adapter->hw, E1000_ALGNERRC);
		adapter->stats.rxerrc += 
		E1000_READ_REG(&adapter->hw, E1000_RXERRC);
		adapter->stats.tncrs += 
		E1000_READ_REG(&adapter->hw, E1000_TNCRS);
		adapter->stats.cexterr += 
		E1000_READ_REG(&adapter->hw, E1000_CEXTERR);
		adapter->stats.tsctc += 
		E1000_READ_REG(&adapter->hw, E1000_TSCTC);
		adapter->stats.tsctfc += 
		E1000_READ_REG(&adapter->hw, E1000_TSCTFC);
	}
}

static uint64_t
lem_if_get_counter(if_ctx_t ctx, ift_counter cnt)
{
	struct adapter *adapter = iflib_get_softc(ctx);
    if_t ifp = iflib_get_ifp(ctx); 
	
	switch (cnt) {
	case IFCOUNTER_COLLISIONS:
		return (adapter->stats.colc);
	case IFCOUNTER_IERRORS:
		return (adapter->dropped_pkts + adapter->stats.rxerrc +
		    adapter->stats.crcerrs + adapter->stats.algnerrc +
		    adapter->stats.ruc + adapter->stats.roc +
		    adapter->stats.mpc + adapter->stats.cexterr);
	case IFCOUNTER_OERRORS:
		return (adapter->stats.ecol + adapter->stats.latecol +
		    adapter->watchdog_events);
	default:
		return (if_get_counter_default(ifp, cnt));
	}
}

/* Export a single 32-bit register via a read-only sysctl. */
static int
lem_sysctl_reg_handler(SYSCTL_HANDLER_ARGS)
{
	struct adapter *adapter;
	u_int val;

	adapter = oidp->oid_arg1;
	val = E1000_READ_REG(&adapter->hw, oidp->oid_arg2);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

/*
 * Add sysctl variables, one per statistic, to the system.
 */
static void
lem_add_hw_stats(struct adapter *adapter)
{
	device_t dev = iflib_get_dev(adapter->ctx);

	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(dev);
	struct sysctl_oid_list *child = SYSCTL_CHILDREN(tree);
	struct e1000_hw_stats *stats = &adapter->stats;

	struct sysctl_oid *stat_node;
	struct sysctl_oid_list *stat_list;

	/* Driver Statistics */
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "mbuf_alloc_fail", 
			 CTLFLAG_RD, &adapter->mbuf_alloc_failed,
			 "Std mbuf failed");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "cluster_alloc_fail", 
			 CTLFLAG_RD, &adapter->mbuf_cluster_failed,
			 "Std mbuf cluster failed");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "dropped", 
			CTLFLAG_RD, &adapter->dropped_pkts,
			"Driver dropped packets");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "tx_dma_fail", 
			CTLFLAG_RD, &adapter->no_tx_dma_setup,
			"Driver tx dma failure in xmit");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "tx_desc_fail1",
			CTLFLAG_RD, &adapter->no_tx_desc_avail1,
			"Not enough tx descriptors failure in xmit");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "tx_desc_fail2",
			CTLFLAG_RD, &adapter->no_tx_desc_avail2,
			"Not enough tx descriptors failure in xmit");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "rx_overruns",
			CTLFLAG_RD, &adapter->rx_overruns,
			"RX overruns");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "watchdog_timeouts",
			CTLFLAG_RD, &adapter->watchdog_events,
			"Watchdog timeouts");

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "device_control",
			CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_CTRL,
			lem_sysctl_reg_handler, "IU",
			"Device Control Register");
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "rx_control",
			CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_RCTL,
			lem_sysctl_reg_handler, "IU",
			"Receiver Control Register");
	SYSCTL_ADD_UINT(ctx, child, OID_AUTO, "fc_high_water",
			CTLFLAG_RD, &adapter->hw.fc.high_water, 0,
			"Flow Control High Watermark");
	SYSCTL_ADD_UINT(ctx, child, OID_AUTO, "fc_low_water", 
			CTLFLAG_RD, &adapter->hw.fc.low_water, 0,
			"Flow Control Low Watermark");
	SYSCTL_ADD_UQUAD(ctx, child, OID_AUTO, "fifo_workaround",
			CTLFLAG_RD, &adapter->tx_fifo_wrk_cnt,
			"TX FIFO workaround events");
	SYSCTL_ADD_UQUAD(ctx, child, OID_AUTO, "fifo_reset",
			CTLFLAG_RD, &adapter->tx_fifo_reset_cnt,
			"TX FIFO resets");

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "txd_head", 
			CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_TDH(0),
			lem_sysctl_reg_handler, "IU",
 			"Transmit Descriptor Head");
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "txd_tail", 
			CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_TDT(0),
			lem_sysctl_reg_handler, "IU",
 			"Transmit Descriptor Tail");
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "rxd_head", 
			CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_RDH(0),
			lem_sysctl_reg_handler, "IU",
			"Receive Descriptor Head");
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "rxd_tail", 
			CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_RDT(0),
			lem_sysctl_reg_handler, "IU",
			"Receive Descriptor Tail");
	

	/* MAC stats get their own sub node */

	stat_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "mac_stats", 
				    CTLFLAG_RD, NULL, "Statistics");
	stat_list = SYSCTL_CHILDREN(stat_node);

	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "excess_coll",
			CTLFLAG_RD, &stats->ecol,
			"Excessive collisions");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "single_coll",
			CTLFLAG_RD, &stats->scc,
			"Single collisions");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "multiple_coll",
			CTLFLAG_RD, &stats->mcc,
			"Multiple collisions");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "late_coll",
			CTLFLAG_RD, &stats->latecol,
			"Late collisions");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "collision_count",
			CTLFLAG_RD, &stats->colc,
			"Collision Count");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "symbol_errors",
			CTLFLAG_RD, &adapter->stats.symerrs,
			"Symbol Errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "sequence_errors",
			CTLFLAG_RD, &adapter->stats.sec,
			"Sequence Errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "defer_count",
			CTLFLAG_RD, &adapter->stats.dc,
			"Defer Count");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "missed_packets",
			CTLFLAG_RD, &adapter->stats.mpc,
			"Missed Packets");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_no_buff",
			CTLFLAG_RD, &adapter->stats.rnbc,
			"Receive No Buffers");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_undersize",
			CTLFLAG_RD, &adapter->stats.ruc,
			"Receive Undersize");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_fragmented",
			CTLFLAG_RD, &adapter->stats.rfc,
			"Fragmented Packets Received ");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_oversize",
			CTLFLAG_RD, &adapter->stats.roc,
			"Oversized Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_jabber",
			CTLFLAG_RD, &adapter->stats.rjc,
			"Recevied Jabber");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_errs",
			CTLFLAG_RD, &adapter->stats.rxerrc,
			"Receive Errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "crc_errs",
			CTLFLAG_RD, &adapter->stats.crcerrs,
			"CRC errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "alignment_errs",
			CTLFLAG_RD, &adapter->stats.algnerrc,
			"Alignment Errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "coll_ext_errs",
			CTLFLAG_RD, &adapter->stats.cexterr,
			"Collision/Carrier extension errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "xon_recvd",
			CTLFLAG_RD, &adapter->stats.xonrxc,
			"XON Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "xon_txd",
			CTLFLAG_RD, &adapter->stats.xontxc,
			"XON Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "xoff_recvd",
			CTLFLAG_RD, &adapter->stats.xoffrxc,
			"XOFF Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "xoff_txd",
			CTLFLAG_RD, &adapter->stats.xofftxc,
			"XOFF Transmitted");

	/* Packet Reception Stats */
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "total_pkts_recvd",
			CTLFLAG_RD, &adapter->stats.tpr,
			"Total Packets Received ");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_pkts_recvd",
			CTLFLAG_RD, &adapter->stats.gprc,
			"Good Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "bcast_pkts_recvd",
			CTLFLAG_RD, &adapter->stats.bprc,
			"Broadcast Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_recvd",
			CTLFLAG_RD, &adapter->stats.mprc,
			"Multicast Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_64",
			CTLFLAG_RD, &adapter->stats.prc64,
			"64 byte frames received ");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_65_127",
			CTLFLAG_RD, &adapter->stats.prc127,
			"65-127 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_128_255",
			CTLFLAG_RD, &adapter->stats.prc255,
			"128-255 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_256_511",
			CTLFLAG_RD, &adapter->stats.prc511,
			"256-511 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_512_1023",
			CTLFLAG_RD, &adapter->stats.prc1023,
			"512-1023 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_1024_1522",
			CTLFLAG_RD, &adapter->stats.prc1522,
			"1023-1522 byte frames received");
 	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_octets_recvd",
 			CTLFLAG_RD, &adapter->stats.gorc, 
 			"Good Octets Received");

	/* Packet Transmission Stats */
 	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_octets_txd",
 			CTLFLAG_RD, &adapter->stats.gotc, 
 			"Good Octets Transmitted"); 
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "total_pkts_txd",
			CTLFLAG_RD, &adapter->stats.tpt,
			"Total Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_pkts_txd",
			CTLFLAG_RD, &adapter->stats.gptc,
			"Good Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "bcast_pkts_txd",
			CTLFLAG_RD, &adapter->stats.bptc,
			"Broadcast Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_txd",
			CTLFLAG_RD, &adapter->stats.mptc,
			"Multicast Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_64",
			CTLFLAG_RD, &adapter->stats.ptc64,
			"64 byte frames transmitted ");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_65_127",
			CTLFLAG_RD, &adapter->stats.ptc127,
			"65-127 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_128_255",
			CTLFLAG_RD, &adapter->stats.ptc255,
			"128-255 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_256_511",
			CTLFLAG_RD, &adapter->stats.ptc511,
			"256-511 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_512_1023",
			CTLFLAG_RD, &adapter->stats.ptc1023,
			"512-1023 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_1024_1522",
			CTLFLAG_RD, &adapter->stats.ptc1522,
			"1024-1522 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tso_txd",
			CTLFLAG_RD, &adapter->stats.tsctc,
			"TSO Contexts Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tso_ctx_fail",
			CTLFLAG_RD, &adapter->stats.tsctfc,
			"TSO Contexts Failed");
}

/**********************************************************************
 *
 *  This routine provides a way to dump out the adapter eeprom,
 *  often a useful debug/service tool. This only dumps the first
 *  32 words, stuff that matters is in that extent.
 *
 **********************************************************************/

static int
lem_sysctl_nvm_info(SYSCTL_HANDLER_ARGS)
{
	struct adapter *adapter;
	int error;
	int result;

	result = -1;
	error = sysctl_handle_int(oidp, &result, 0, req);

	if (error || !req->newptr)
		return (error);

	/*
	 * This value will cause a hex dump of the
	 * first 32 16-bit words of the EEPROM to
	 * the screen.
	 */
	if (result == 1) {
		adapter = (struct adapter *)arg1;
		lem_print_nvm_info(adapter);
        }

	return (error);
}

static void
lem_print_nvm_info(struct adapter *adapter)
{
	u16	eeprom_data;
	int	i, j, row = 0;

	/* Its a bit crude, but it gets the job done */
	printf("\nInterface EEPROM Dump:\n");
	printf("Offset\n0x0000  ");
	for (i = 0, j = 0; i < 32; i++, j++) {
		if (j == 8) { /* Make the offset block */
			j = 0; ++row;
			printf("\n0x00%x0  ",row);
		}
		e1000_read_nvm(&adapter->hw, i, 1, &eeprom_data);
		printf("%04x ", eeprom_data);
	}
	printf("\n");
}

static int
	lem_if_sysctl_int_delay(if_ctx_t ctx, if_int_delay_info_t info)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	u32 regval;
	int error;
	int usecs;
	int ticks;

	usecs = info->iidi_value;
	error = sysctl_handle_int(info->iidi_oidp, &usecs, 0, info->iidi_req);
	if (error != 0 || info->iidi_req->newptr == NULL)
		return (error);
	if (usecs < 0 || usecs > EM_TICKS_TO_USECS(65535))
		return (EINVAL);
	ticks = EM_USECS_TO_TICKS(usecs);
	if (info->iidi_offset == E1000_ITR)	/* units are 256ns here */
		ticks *= 4;

	regval = E1000_READ_OFFSET(&adapter->hw, info->iidi_offset);
	regval = (regval & ~0xffff) | (ticks & 0xffff);
	/* Handle a few special cases. */
	switch (info->iidi_offset) {
	case E1000_RDTR:
		break;
	case E1000_TIDV:
		if (ticks == 0) {
			adapter->txd_cmd &= ~E1000_TXD_CMD_IDE;
			/* Don't write 0 into the TIDV register. */
			regval++;
		} else
			adapter->txd_cmd |= E1000_TXD_CMD_IDE;
		break;
	}
	E1000_WRITE_OFFSET(&adapter->hw, info->iidi_offset, regval);
	return (0);
}

static void
lem_add_int_delay_sysctl(struct adapter *adapter, const char *name,
	const char *description, struct if_int_delay_info *info,
	int offset, int value)
{
	if_ctx_t ctx = adapter->ctx;
	
	info->iidi_ctx = ctx;
	info->iidi_offset = offset;
	info->iidi_value = value;
    iflib_add_int_delay_sysctl(ctx, name, description, info, offset, value); 
}


static void
lem_set_flow_cntrl(struct adapter *adapter, const char *name,
        const char *description, int *limit, int value)
{
	device_t dev = iflib_get_dev(adapter->ctx); 
		
	*limit = value;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, name, CTLFLAG_RW, limit, value, description);
}

static void
lem_add_rx_process_limit(struct adapter *adapter, const char *name,
	const char *description, int *limit, int value)
{
     device_t dev = iflib_get_dev(adapter->ctx); 
	
	*limit = value;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, name, CTLFLAG_RW, limit, value, description);
}
