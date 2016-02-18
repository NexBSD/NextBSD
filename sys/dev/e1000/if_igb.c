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


#include "opt_inet.h"
#include "opt_inet6.h"
#include "opt_rss.h"

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_device_polling.h"
#include "opt_altq.h"
#endif

#include "if_igb.h"
#include "ifdi_if.h"

#include <net/netmap.h>
#include <sys/selinfo.h>
#include <dev/netmap/netmap_kern.h>

/*********************************************************************
 *  Driver version:
 *********************************************************************/
char igb_driver_version[] = "2.5.3-k";

/*********************************************************************
 *  PCI Device ID Table
 *
 *  Used by probe to select devices to load on
 *  Last field stores an index into e1000_strings
 *  Last entry must be all 0s
 *
 *  { Vendor ID, Device ID, SubVendor ID, SubDevice ID, String Index }
 *********************************************************************/

static pci_vendor_info_t igb_vendor_info_array[] =
{
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82575EB_COPPER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82575EB_FIBER_SERDES, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82575GB_QUAD_COPPER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576_NS, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576_NS_SERDES, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576_FIBER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576_SERDES, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576_SERDES_QUAD, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576_QUAD_COPPER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576_QUAD_COPPER_ET2, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82576_VF, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82580_COPPER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82580_FIBER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82580_SERDES, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82580_SGMII, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82580_COPPER_DUAL, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_82580_QUAD_FIBER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_DH89XXCC_SERDES, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_DH89XXCC_SGMII, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_DH89XXCC_SFP, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_DH89XXCC_BACKPLANE, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I350_COPPER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I350_FIBER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I350_SERDES, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I350_SGMII, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I350_VF, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I210_COPPER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I210_COPPER_IT, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I210_COPPER_OEM1, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I210_COPPER_FLASHLESS, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I210_SERDES_FLASHLESS, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I210_FIBER, "Intel(R) PRO/10GbE PCI-Express Network Driver"), 
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I210_SERDES, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I210_SGMII, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I211_COPPER, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I354_BACKPLANE_1GBPS, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I354_BACKPLANE_2_5GBPS, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	PVID(IGB_INTEL_VENDOR_ID, E1000_DEV_ID_I354_SGMII, "Intel(R) PRO/10GbE PCI-Express Network Driver"),
	/* required last entry */
        PVID_END
};

/*********************************************************************
 *  Function prototypes
 *********************************************************************/
static void *igb_register(device_t dev);
static int	igb_if_attach_pre(if_ctx_t);
static int  igb_if_attach_post(if_ctx_t ctx);
static int	igb_if_detach(if_ctx_t);
static int	igb_if_shutdown(if_ctx_t);
static int	igb_if_suspend(if_ctx_t);
static int      igb_if_resume(if_ctx_t); 

static void igb_if_stop(if_ctx_t ctx);
static void	igb_if_init(if_ctx_t ctx);
static void igb_if_enable_intr(if_ctx_t ctx); 
static void	igb_if_disable_intr(if_ctx_t ctx);
static int igb_if_media_change(if_ctx_t ctx);
static int igb_if_msix_intr_assign(if_ctx_t, int);
static void igb_if_update_admin_status(if_ctx_t ctx); 
static void	igb_if_media_status(if_ctx_t ctx, struct ifmediareq *ifmr);
static void igb_if_multi_set(if_ctx_t ctx);
static int igb_if_mtu_set(if_ctx_t ctx, uint32_t mtu);

static int igb_if_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs, int nqs);
static void igb_if_queues_free(if_ctx_t ctx);
static uint64_t	igb_if_get_counter(if_ctx_t, ift_counter);
static void	igb_identify_hardware(if_ctx_t ctx);
static int	igb_allocate_pci_resources(if_ctx_t ctx);
static void	igb_free_pci_resources(if_ctx_t ctx);
static void	igb_reset(if_ctx_t ctx); 
static int	igb_setup_interface(if_ctx_t ctx);

static void	igb_configure_queues(struct adapter *);
static void igb_initialize_rss_mapping(struct adapter *adapter);
static void	igb_initialize_transmit_units(if_ctx_t ctx);
static void	igb_initialize_receive_units(if_ctx_t ctx); 
static void igb_if_timer(if_ctx_t ctx, uint16_t qid); 
static void	igb_update_stats_counters(struct adapter *);

static int 	igb_if_set_promisc(if_ctx_t ctx, int flags); 
static void	igb_disable_promisc(if_ctx_t ctx); 

static void	igb_if_vlan_register(if_ctx_t, u16);
static void	igb_if_vlan_unregister(if_ctx_t, u16);
static void	igb_setup_vlan_hw_support(if_ctx_t ctx);

       int       igb_intr(void *arg);
static int	igb_sysctl_nvm_info(SYSCTL_HANDLER_ARGS);
static void	igb_print_nvm_info(struct adapter *);
static int 	igb_is_valid_ether_addr(u8 *);
static void igb_add_hw_stats(struct adapter *); 

static void	igb_vf_init_stats(struct adapter *);
static void	igb_update_vf_stats_counters(struct adapter *);
static void igb_init_dmac(if_ctx_t ctx, u32 pba);
static void igb_handle_link(void *context, int pending);

/* Management and WOL Support */
static void	igb_init_manageability(struct adapter *);
static void	igb_release_manageability(struct adapter *);
static void igb_get_hw_control(struct adapter *);
static void igb_release_hw_control(struct adapter *);
static void igb_enable_wakeup(device_t);

static int	igb_msix_que(void *);
static int	igb_msix_link(void *);
static void	igb_set_sysctl_value(struct adapter *, const char *,
		    const char *, int *, int);
static int	igb_set_flowcntl(SYSCTL_HANDLER_ARGS);
static int	igb_sysctl_dmac(SYSCTL_HANDLER_ARGS);
static int	igb_sysctl_eee(SYSCTL_HANDLER_ARGS);

#ifdef DEVICE_POLLING
static poll_handler_t igb_poll;
#endif /* POLLING */

/*********************************************************************
 *  FreeBSD Device Interface Entry Points
 *********************************************************************/
static device_method_t igb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_register, igb_register),
	DEVMETHOD(device_probe, iflib_device_probe),
	DEVMETHOD(device_attach, iflib_device_attach),
	DEVMETHOD(device_detach, iflib_device_detach),
	DEVMETHOD(device_shutdown, iflib_device_shutdown),
	DEVMETHOD(device_suspend, iflib_device_suspend),
	DEVMETHOD(device_resume, iflib_device_resume),
	DEVMETHOD_END
};

static driver_t igb_driver = {
	"igb", igb_methods, sizeof(struct adapter),
};

static devclass_t igb_devclass;
DRIVER_MODULE(igb, pci, igb_driver, igb_devclass, 0, 0);
MODULE_DEPEND(igb, pci, 1, 1, 1);
MODULE_DEPEND(igb, ether, 1, 1, 1);
#ifdef DEV_NETMAP
MODULE_DEPEND(igb, netmap, 1, 1, 1);
#endif /* DEV_NETMAP */

static device_method_t igb_if_methods[] = {
	DEVMETHOD(ifdi_attach_pre, igb_if_attach_pre),
	DEVMETHOD(ifdi_attach_post, igb_if_attach_post), 
	DEVMETHOD(ifdi_detach, igb_if_detach),
	DEVMETHOD(ifdi_shutdown, igb_if_shutdown),
	DEVMETHOD(ifdi_suspend, igb_if_suspend),
	DEVMETHOD(ifdi_resume, igb_if_resume), 
	DEVMETHOD(ifdi_init, igb_if_init),
	DEVMETHOD(ifdi_stop, igb_if_stop),
	DEVMETHOD(ifdi_msix_intr_assign, igb_if_msix_intr_assign),
	DEVMETHOD(ifdi_intr_enable, igb_if_enable_intr), 
	DEVMETHOD(ifdi_intr_disable, igb_if_disable_intr),
	DEVMETHOD(ifdi_queues_alloc, igb_if_queues_alloc),
	DEVMETHOD(ifdi_queues_free, igb_if_queues_free),
	DEVMETHOD(ifdi_update_admin_status, igb_if_update_admin_status),
	DEVMETHOD(ifdi_multi_set, igb_if_multi_set),
	DEVMETHOD(ifdi_media_status, igb_if_media_status),
	DEVMETHOD(ifdi_media_change, igb_if_media_change),
	DEVMETHOD(ifdi_mtu_set, igb_if_mtu_set),
	DEVMETHOD(ifdi_promisc_set, igb_if_set_promisc),
	DEVMETHOD(ifdi_timer, igb_if_timer),
	DEVMETHOD(ifdi_vlan_register, igb_if_vlan_register),
	DEVMETHOD(ifdi_vlan_unregister, igb_if_vlan_unregister),
	DEVMETHOD(ifdi_get_counter, igb_if_get_counter), 
	DEVMETHOD_END
};

/*
 * note that if (adapter->msix_mem) is replaced by:
 * if (adapter->intr_type == IFLIB_INTR_MSIX)
 */
static driver_t igb_if_driver = {
  "igb_if", igb_if_methods, sizeof(struct adapter)
};

/*********************************************************************
 *  Tunable default values.
 *********************************************************************/

static SYSCTL_NODE(_hw, OID_AUTO, igb, CTLFLAG_RD, 0, "IGB driver parameters");

/* Descriptor defaults */
static int igb_rxd = IGB_DEFAULT_RXD;
static int igb_txd = IGB_DEFAULT_TXD;
SYSCTL_INT(_hw_igb, OID_AUTO, rxd, CTLFLAG_RDTUN, &igb_rxd, 0,
    "Number of receive descriptors per queue");
SYSCTL_INT(_hw_igb, OID_AUTO, txd, CTLFLAG_RDTUN, &igb_txd, 0,
    "Number of transmit descriptors per queue");

/*
** AIM: Adaptive Interrupt Moderation
** which means that the interrupt rate
** is varied over time based on the
** traffic for that interrupt vector
*/
static int igb_enable_aim = TRUE;
SYSCTL_INT(_hw_igb, OID_AUTO, enable_aim, CTLFLAG_RWTUN, &igb_enable_aim, 0,
    "Enable adaptive interrupt moderation");

/*
 * MSIX should be the default for best performance,
 * but this allows it to be forced off for testing.
 */         
static int igb_enable_msix = 1;
SYSCTL_INT(_hw_igb, OID_AUTO, enable_msix, CTLFLAG_RDTUN, &igb_enable_msix, 0,
    "Enable MSI-X interrupts");

/*
** Tuneable Interrupt rate
*/
static int igb_max_interrupt_rate = 8000;
SYSCTL_INT(_hw_igb, OID_AUTO, max_interrupt_rate, CTLFLAG_RDTUN,
    &igb_max_interrupt_rate, 0, "Maximum interrupts per second");

#ifndef IGB_LEGACY_TX
/*
** Tuneable number of buffers in the buf-ring (drbr_xxx)
*/
static int igb_buf_ring_size = IGB_BR_SIZE;
SYSCTL_INT(_hw_igb, OID_AUTO, buf_ring_size, CTLFLAG_RDTUN,
    &igb_buf_ring_size, 0, "Size of the bufring");
#endif

/*
** Header split causes the packet header to
** be dma'd to a seperate mbuf from the payload.
** this can have memory alignment benefits. But
** another plus is that small packets often fit
** into the header and thus use no cluster. Its
** a very workload dependent type feature.
*/
static int igb_header_split = FALSE;
SYSCTL_INT(_hw_igb, OID_AUTO, header_split, CTLFLAG_RDTUN, &igb_header_split, 0,
    "Enable receive mbuf header split");

/*
** This will autoconfigure based on the
** number of CPUs and max supported
** MSIX messages if left at 0.
*/
#if 0
/* determined by iflib */
static int igb_num_queues = 0;
SYSCTL_INT(_hw_igb, OID_AUTO, num_queues, CTLFLAG_RDTUN, &igb_num_queues, 0,
    "Number of queues to configure, 0 indicates autoconfigure");
#endif
/*
** Global variable to store last used CPU when binding queues
** to CPUs in igb_allocate_msix.  Starts at CPU_FIRST and increments when a
** queue is bound to a cpu.
*/
int igb_last_bind_cpu = -1;

/* How many packets rxeof tries to clean at a time */
static int igb_rx_process_limit = 100;
SYSCTL_INT(_hw_igb, OID_AUTO, rx_process_limit, CTLFLAG_RDTUN,
    &igb_rx_process_limit, 0,
    "Maximum number of received packets to process at a time, -1 means unlimited");

/* How many packets txeof tries to clean at a time */
static int igb_tx_process_limit = -1;
SYSCTL_INT(_hw_igb, OID_AUTO, tx_process_limit, CTLFLAG_RDTUN,
    &igb_tx_process_limit, 0,
    "Maximum number of sent packets to process at a time, -1 means unlimited");

extern struct if_txrx igb_txrx; 

static struct if_shared_ctx igb_sctx_init = {
   	.isc_magic = IFLIB_MAGIC,
	.isc_q_align =  PAGE_SIZE,/* max(DBA_ALIGN, PAGE_SIZE) */
	.isc_tx_maxsize = IGB_TSO_SIZE,
	.isc_tx_maxsegsize = PAGE_SIZE*4,
	.isc_rx_maxsize = PAGE_SIZE*4,
	.isc_rx_nsegments = 1,
	.isc_rx_maxsegsize = PAGE_SIZE*4,
	.isc_ntxd = IGB_DEFAULT_TXD,
	.isc_nrxd = IGB_DEFAULT_RXD,
	.isc_nfl = 1,
	.isc_qsizes[0] = roundup2((IGB_DEFAULT_TXD * sizeof(union e1000_adv_tx_desc)) +
							  sizeof(u32), IGB_DBA_ALIGN),
	.isc_qsizes[1] = roundup2(IGB_DEFAULT_RXD *
							  sizeof(union e1000_adv_rx_desc), IGB_DBA_ALIGN),
	.isc_nqs = 2,
	.isc_admin_intrcnt = 1,
	.isc_vendor_info = igb_vendor_info_array,
	.isc_driver_version = igb_driver_version,
	.isc_txrx = &igb_txrx,
	.isc_driver = &igb_if_driver,
};

if_shared_ctx_t igb_sctx = &igb_sctx_init; 

static void *
igb_register(device_t dev)
{
	/* Do descriptor calc and sanity checks */
	if (((igb_txd * sizeof(union e1000_adv_tx_desc)) % IGB_DBA_ALIGN) != 0 ||
		igb_txd < IGB_MIN_TXD || igb_txd > IGB_MAX_TXD) {
		device_printf(dev, "TXD config issue, using default!\n");
		igb_sctx->isc_ntxd = IGB_DEFAULT_TXD;
	} else {
		igb_sctx->isc_ntxd = igb_txd;
	}

	igb_tx_process_limit = min(igb_sctx->isc_ntxd, igb_txd);
	igb_sctx->isc_qsizes[0] = roundup2(igb_sctx->isc_ntxd *
	    sizeof(union e1000_adv_tx_desc), IGB_DBA_ALIGN);

    igb_sctx->isc_qsizes[1] = roundup2(igb_sctx->isc_nrxd *
									   sizeof(union e1000_adv_rx_desc), IGB_DBA_ALIGN);
	
	return (igb_sctx); 
}

static int
igb_if_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs, int nqs)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	device_t dev = iflib_get_dev(ctx);
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	struct igb_queue	*que;
	int error = E1000_SUCCESS;
        int i;
	
	MPASS(adapter->num_queues > 0);
	MPASS(nqs == 2); 

	/* First allocate the top level queue structs */
	if (!(adapter->queues =
	    (struct igb_queue *) malloc(sizeof(struct igb_queue) *
	    adapter->num_queues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate queue memory\n");
		return(ENOMEM);
	}

	for (i = 0, que = adapter->queues; i < adapter->num_queues; i++, que++) {
		/* Set up some basics */
		struct tx_ring *txr = &que->txr;
		struct rx_ring *rxr = &que->rxr;
		struct lro_ctrl *lro = &rxr->lro; 
		txr->adapter = rxr->adapter = que->adapter = adapter;
		txr->que = rxr->que = que; 
		que->me = txr->me = rxr->me =  i;

		 /* Allocate transmit buffer memory */
		 if (!(txr->tx_buffers = (struct igb_tx_buf *) malloc(sizeof(struct igb_tx_buf) * igb_sctx->isc_ntxd, M_DEVBUF, M_NOWAIT | M_ZERO))) {
	        device_printf(iflib_get_dev(ctx), "failed to allocate tx_buffer memory\n");
			error = ENOMEM;
			goto fail; 
		 }

		 /*
		 ** Now set up the LRO interface, we
		 ** also only do head split when LRO
		 ** is enabled, since so often they
		 ** are undesireable in similar setups.
		 */
		 if (ifp->if_capenable & IFCAP_LRO) {
			 error = tcp_lro_init(lro);
			 if (error) {
				 device_printf(dev, "LRO Initialization failed!\n");
				 goto fail;
			 }
			 INIT_DEBUGOUT("RX LRO Initialized\n");
			 rxr->lro_enabled = TRUE;
			 lro->ifp = iflib_get_ifp(ctx);
		 }

		 /* get the virtual and physical address of the hardware queues */
		 txr->tx_base = (union e1000_adv_tx_desc *)vaddrs[i*2]; 
		 rxr->rx_base = (union e1000_adv_rx_desc *)vaddrs[i*2+1];
		 
		 txr->tx_paddr = paddrs[i*2];
		 rxr->rx_paddr = paddrs[i*2+1];
		 
		 txr->tail = E1000_TDT(que->me);
		 rxr->tail = E1000_RDT(que->me); 

		 txr->bytes = rxr->bytes = 0;
		 txr->total_packets = 0;
		 txr->tx_buffers->eop = NULL; 
	}

	iflib_config_gtask_init(ctx, &adapter->link_task, igb_handle_link, "link_task");
	
	device_printf(iflib_get_dev(ctx), "allocated for %d queues\n", adapter->num_queues);
	return (0);

fail:
	igb_if_queues_free(ctx);
	return(error); 
}

static void
igb_if_queues_free(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	struct igb_queue *que = adapter->queues;

	if (que == NULL)
		return;

	for (int i = 0; i < adapter->num_queues; i++, que++) {
		struct rx_ring *rxr = &que->rxr;
		struct tx_ring *txr = &que->txr; 		
		struct lro_ctrl	*lro = &rxr->lro;

		if (txr->tx_buffers == NULL)
			break; 

		tcp_lro_free(lro);
		free(txr->tx_buffers, M_DEVBUF);
		txr->tx_buffers = NULL; 
	}

	free(adapter->queues, M_DEVBUF);
	adapter->queues = NULL; 
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
igb_if_attach_pre(if_ctx_t ctx)
{
	printf("attach pre called\n"); 
	device_t                dev = iflib_get_dev(ctx);  
	struct adapter	        *adapter = iflib_get_softc(ctx); 
	int		        error = 0;

	INIT_DEBUGOUT("igb_if_attach: begin");
	adapter->hw.mac.get_link_status = 0;
	
	if (resource_disabled("igb", device_get_unit(dev))) {
		device_printf(dev, "Disabled by device hint\n");
		return (ENXIO);
	}

	adapter->ctx = ctx; 
	adapter->dev = adapter->osdep.dev = dev;
	adapter->shared = iflib_get_softc_ctx(ctx);
	adapter->media = iflib_get_media(ctx);
	adapter->num_tx_desc = igb_sctx->isc_ntxd; 
	adapter->num_rx_desc = igb_sctx->isc_nrxd;
	
	/* SYSCTLs */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "nvm", CTLTYPE_INT|CTLFLAG_RW, adapter, 0,
	    igb_sysctl_nvm_info, "I", "NVM Information");

	igb_set_sysctl_value(adapter, "enable_aim",
	    "Interrupt Moderation", &adapter->enable_aim,
	    igb_enable_aim);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "fc", CTLTYPE_INT|CTLFLAG_RW,
	    adapter, 0, igb_set_flowcntl, "I", "Flow Control");

	/* Determine hardware and mac info & set isc_msix_bar */
	igb_identify_hardware(ctx);
	adapter->shared->isc_msix_bar = PCIR_BAR(IGB_MSIX_BAR);
	adapter->shared->isc_tx_nsegments = IGB_MAX_SCATTER; 
	
	/* Setup PCI resources */
	if (igb_allocate_pci_resources(ctx)) {
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

	/* Sysctls for limiting the amount of work done in the taskqueues */
	igb_set_sysctl_value(adapter, "rx_processing_limit",
	    "max number of rx packets to process",
	    &adapter->rx_process_limit, igb_rx_process_limit);

	igb_set_sysctl_value(adapter, "tx_processing_limit",
	    "max number of tx packets to process",
	    &adapter->tx_process_limit, igb_tx_process_limit);

	adapter->hw.mac.autoneg = DO_AUTO_NEG;
	adapter->hw.phy.autoneg_wait_to_complete = FALSE;
	adapter->hw.phy.autoneg_advertised = AUTONEG_ADV_DEFAULT;

	/* Copper options */
	if (adapter->hw.phy.media_type == e1000_media_type_copper) {
		adapter->hw.phy.mdix = AUTO_ALL_MODES;
		adapter->hw.phy.disable_polarity_correction = FALSE;
		adapter->hw.phy.ms_type = IGB_MASTER_SLAVE;
	}

	/*Set the frame limits assuming
	 * standard ethernet sized frames.
	 */
	adapter->shared->isc_max_frame_size = ETHERMTU + ETHER_HDR_LEN + ETHERNET_FCS_SIZE;

   	if (((igb_rxd * sizeof(union e1000_adv_rx_desc)) % IGB_DBA_ALIGN) != 0 ||
		igb_rxd < IGB_MIN_RXD || igb_rxd > IGB_MAX_RXD) {
		device_printf(dev, "RXD config issue, using default!\n");
		igb_sctx->isc_nrxd = IGB_DEFAULT_RXD;
	} else {
		igb_sctx->isc_nrxd = igb_rxd; 
	}
	
	/* Allocate the appropriate stats memory */
	if (adapter->vf_ifp) {
		adapter->stats =
		    (struct e1000_vf_stats *)malloc(sizeof \
		    (struct e1000_vf_stats), M_DEVBUF, M_NOWAIT | M_ZERO);
		igb_vf_init_stats(adapter);
	} else
		adapter->stats =
		    (struct e1000_hw_stats *)malloc(sizeof \
		    (struct e1000_hw_stats), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (adapter->stats == NULL) {
		device_printf(dev, "Can not allocate stats memory\n");
		error = ENOMEM;
		goto err_late;
	}

	/* Allocate multicast array memory. */
	adapter->mta = malloc(sizeof(u8) * ETH_ADDR_LEN *
	    MAX_NUM_MULTICAST_ADDRESSES, M_DEVBUF, M_NOWAIT);
	if (adapter->mta == NULL) {
		device_printf(dev, "Can not allocate multicast setup array\n");
		error = ENOMEM;
		goto err_late;
	}

	/* Some adapter-specific advanced features */
	if (adapter->hw.mac.type >= e1000_i350) {
		SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
		    OID_AUTO, "dmac", CTLTYPE_INT|CTLFLAG_RW,
		    adapter, 0, igb_sysctl_dmac, "I", "DMA Coalesce");
		SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
		    OID_AUTO, "eee_disabled", CTLTYPE_INT|CTLFLAG_RW,
		    adapter, 0, igb_sysctl_eee, "I",
		    "Disable Energy Efficient Ethernet");
		if (adapter->hw.phy.media_type == e1000_media_type_copper) {
			if (adapter->hw.mac.type == e1000_i354)
				e1000_set_eee_i354(&adapter->hw, TRUE, TRUE);
			else
				e1000_set_eee_i350(&adapter->hw, TRUE, TRUE);
		}
	}

	/*
	** Start from a known state, this is
	** important in reading the nvm and
	** mac from that.
	*/
	e1000_reset_hw(&adapter->hw);

	/* Make sure we have a good EEPROM before we read from it */
	if (((adapter->hw.mac.type != e1000_i210) &&
	    (adapter->hw.mac.type != e1000_i211)) &&
	    (e1000_validate_nvm_checksum(&adapter->hw) < 0)) {
		/*
		** Some PCI-E parts fail the first check due to
		** the link being in sleep state, call it again,
		** if it fails a second time its a real issue.
		*/
		if (e1000_validate_nvm_checksum(&adapter->hw) < 0) {
			device_printf(dev,
			    "The EEPROM Checksum Is Not Valid\n");
			error = EIO;
			goto err_late;
		}
	}

	/*
	** Copy the permanent MAC address out of the EEPROM
	*/
	if (e1000_read_mac_addr(&adapter->hw) < 0) {
		device_printf(dev, "EEPROM read error while reading MAC"
		    " address\n");
		error = EIO;
		goto err_late;
	}
	/* Check its sanity */
	if (!igb_is_valid_ether_addr(adapter->hw.mac.addr)) {
		device_printf(dev, "Invalid MAC address\n");
		error = EIO;
		goto err_late;
	}

	adapter->link_active = 0;
	iflib_link_state_change(ctx, LINK_STATE_DOWN);
	return(0);

err_late:
    free(adapter->mta, M_DEVBUF);
	free(adapter->stats, M_DEVBUF); 
	igb_release_hw_control(adapter);
err_pci:
	igb_free_pci_resources(ctx);

	return (error);
}

static int
igb_if_attach_post(if_ctx_t ctx)
{
    device_t dev = iflib_get_dev(ctx);
	struct adapter *adapter = iflib_get_softc(ctx);
    int error;
	int eeprom_data = 0; 
	
	/* Setup OS specific network interface */
	error = igb_setup_interface(ctx); 
	if (error) {
		device_printf(dev, "Error in igb_setup_interface");
		goto err_late; 
	}
	
	/* Now get a good starting state */
	igb_reset(ctx); 

	/* Initialize statistics */
	igb_update_stats_counters(adapter);

	/* Indicate SOL/IDER usage */
	if (e1000_check_reset_block(&adapter->hw))
		device_printf(dev,
		    "PHY reset is blocked due to SOL/IDER session.\n");

	/* Determine if we have to control management hardware */
	adapter->has_manage = e1000_enable_mng_pass_thru(&adapter->hw);

	/*
	 * Setup Wake-on-Lan
	 */
	/* APME bit in EEPROM is mapped to WUC.APME */
	eeprom_data = E1000_READ_REG(&adapter->hw, E1000_WUC) & E1000_WUC_APME;
	if (eeprom_data)
		adapter->wol = E1000_WUFC_MAG;

	igb_add_hw_stats(adapter);

	return (0);

err_late:
    free(adapter->mta, M_DEVBUF);
	free(adapter->stats, M_DEVBUF); 
	igb_release_hw_control(adapter);
	igb_free_pci_resources(ctx);

	return (error);
	
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
igb_if_detach(if_ctx_t ctx)
{
	struct adapter	*adapter = iflib_get_softc(ctx);
	device_t dev    = iflib_get_dev(ctx); 
	
	INIT_DEBUGOUT("igb_detach: begin");

#ifdef DEVICE_POLLING
	if (ifp->if_capenable & IFCAP_POLLING)
		ether_poll_deregister(ifp);
#endif

	e1000_phy_hw_reset(&adapter->hw);

	/* Give control back to firmware */
	igb_release_manageability(adapter);
	igb_release_hw_control(adapter);

	if (adapter->wol) {
		E1000_WRITE_REG(&adapter->hw, E1000_WUC, E1000_WUC_PME_EN);
		E1000_WRITE_REG(&adapter->hw, E1000_WUFC, adapter->wol);
		igb_enable_wakeup(dev);
	}

	igb_free_pci_resources(ctx);

	if (adapter->mta != NULL)
		free(adapter->mta, M_DEVBUF);

	if (adapter->stats != NULL)
		free(adapter->stats, M_DEVBUF); 
	
	return (0);
}

/*********************************************************************
 *
 *  Shutdown entry point
 *
 **********************************************************************/

static int
igb_if_shutdown(if_ctx_t ctx)
{
	return igb_if_suspend(ctx); 
}

/*
 * Suspend/resume device methods.
 */
static int
igb_if_suspend(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx);
        device_t dev = iflib_get_dev(ctx); 

	igb_if_stop(ctx); 
        igb_release_manageability(adapter);
	igb_release_hw_control(adapter);

        if (adapter->wol) {
                E1000_WRITE_REG(&adapter->hw, E1000_WUC, E1000_WUC_PME_EN);
                E1000_WRITE_REG(&adapter->hw, E1000_WUFC, adapter->wol);
                igb_enable_wakeup(dev);
        }

	return 0;
}

static int
igb_if_resume(if_ctx_t ctx)
{
        struct adapter *adapter = iflib_get_softc(ctx);
	struct ifnet *ifp = iflib_get_ifp(ctx); 
	
        igb_if_init(ctx);
	igb_init_manageability(adapter);

	return (0); 
}

/*********************************************************************         *  Ioctl mtu entry point                                                     *  return 0 on success, EINVAL on failure                                     **********************************************************************/
static int
igb_if_mtu_set(if_ctx_t ctx, uint32_t mtu)
{
	int maximum_frame_size = 9234;
	int error = 0; 
	struct adapter *adapter = iflib_get_softc(ctx);

	IOCTL_DEBUGOUT("ioctl rcv'd: SIOCSIFMTU (Set Interface MTU)");

	if (mtu > maximum_frame_size - ETHER_HDR_LEN - ETHER_CRC_LEN) {
		error = EINVAL;
	} else {
		adapter->max_frame_size = mtu + ETHER_HDR_LEN + ETHER_CRC_LEN;
	}

	return error; 
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
igb_if_init(if_ctx_t ctx)
{
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	struct adapter *adapter = iflib_get_softc(ctx); 

	INIT_DEBUGOUT("igb_init: begin");

	/* Get the latest mac address, User can use a LAA */
        bcopy(IF_LLADDR(ifp), adapter->hw.mac.addr,
              ETHER_ADDR_LEN);

	/* Put the address into the Receive Address Array */
	e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	igb_reset(ctx); 

	E1000_WRITE_REG(&adapter->hw, E1000_VET, ETHERTYPE_VLAN);

	/* Set hardware offload abilities */
	ifp->if_hwassist = 0;
	if (ifp->if_capenable & IFCAP_TXCSUM) {
		ifp->if_hwassist |= (CSUM_TCP | CSUM_UDP);
#if __FreeBSD_version >= 800000
		if ((adapter->hw.mac.type == e1000_82576) ||
		    (adapter->hw.mac.type == e1000_82580))
			ifp->if_hwassist |= CSUM_SCTP;
#endif
	}

	if (ifp->if_capenable & IFCAP_TSO)
		ifp->if_hwassist |= CSUM_TSO;

	/* Configure for OS presence */
	igb_init_manageability(adapter);

	/* Prepare transmit descriptors and buffers */
	igb_initialize_transmit_units(ctx);

	/* Setup Multicast table */
	igb_if_multi_set(ctx);

	/*
	** Figure out the desired mbuf pool
	** for doing jumbo/packetsplit
	*/
	if (adapter->max_frame_size <= 2048)
		adapter->rx_mbuf_sz = MCLBYTES;
	else if (adapter->max_frame_size <= 4096)
		adapter->rx_mbuf_sz = MJUMPAGESIZE;
	else
		adapter->rx_mbuf_sz = MJUM9BYTES;

	/* Prepare receive descriptors and buffers */
	igb_initialize_receive_units(ctx);
	e1000_rx_fifo_flush_82575(&adapter->hw);

        /* Enable VLAN support */
	if (ifp->if_capenable & IFCAP_VLAN_HWTAGGING)
		igb_setup_vlan_hw_support(ctx);
                                
	/* Don't lose promiscuous settings */
	igb_disable_promisc(ctx); 
	igb_if_set_promisc(ctx, if_getflags(ifp));

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	e1000_clear_hw_cntrs_base_generic(&adapter->hw);

	if (adapter->msix > 1) /* Set up queue routing */
		igb_configure_queues(adapter);

	/* this clears any pending interrupts */
	E1000_READ_REG(&adapter->hw, E1000_ICR);

	if (ifp->if_capenable &~ IFCAP_POLLING) {
		E1000_WRITE_REG(&adapter->hw, E1000_ICS, E1000_ICS_LSC);
	}

	/* Set Energy Efficient Ethernet */
	if (adapter->hw.phy.media_type == e1000_media_type_copper) {
		if (adapter->hw.mac.type == e1000_i354)
			e1000_set_eee_i354(&adapter->hw, TRUE, TRUE);
		else
			e1000_set_eee_i350(&adapter->hw, TRUE, TRUE);
	}
}

static void
igb_handle_link(void *context, int pending)
{
	struct adapter *adapter = context; 

	printf("link status set to 1: igb_handle_link_called\n"); 
	adapter->hw.mac.get_link_status = 1;
	iflib_admin_intr_deferred(adapter->ctx); 
}

/*********************************************************************
 *
 *  MSI/Legacy Deferred
 *  Interrupt Service routine  
 *
 *********************************************************************/
int
igb_intr(void *arg)
{
	struct adapter		*adapter = arg;
	u32			reg_icr;

	reg_icr = E1000_READ_REG(&adapter->hw, E1000_ICR);

	/* Hot eject?  */
	if (reg_icr == 0xffffffff)
		return FILTER_STRAY;

	/* Definitely not our interrupt.  */
	if (reg_icr == 0x0)
		return FILTER_STRAY;

	if ((reg_icr & E1000_ICR_INT_ASSERTED) == 0)
		return FILTER_STRAY;

	/*
	 * Mask interrupts until the taskqueue is finished running.  This is
	 * cheap, just assume that it is needed.  This also works around the
	 * MSI message reordering errata on certain systems.
	 */
	igb_if_disable_intr(adapter->ctx);

	/* Link status change */
	if (reg_icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC))
		GROUPTASK_ENQUEUE(&adapter->link_task); 

	if (reg_icr & E1000_ICR_RXO)
		adapter->rx_overruns++;

	return FILTER_HANDLED;
}

/*********************************************************************
 *
 *  MSIX Que Interrupt Service routine
 *
 **********************************************************************/
static int
igb_msix_que(void *arg)
{
	struct igb_queue *que = arg;
	struct adapter *adapter = que->adapter;
	struct ifnet   *ifp = iflib_get_ifp(adapter->ctx); 
	struct tx_ring *txr = &que->txr;
	struct rx_ring *rxr = &que->rxr;
	u32		newitr = 0;

	/* Ignore spurious interrupts */
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return 0;

	E1000_WRITE_REG(&adapter->hw, E1000_EIMC, que->eims);
	++que->irqs;

	if (adapter->enable_aim == FALSE)
		goto no_calc;
	/*
	** Do Adaptive Interrupt Moderation:
        **  - Write out last calculated setting
	**  - Calculate based on average size over
	**    the last interval.
	*/
        if (que->eitr_setting)
                E1000_WRITE_REG(&adapter->hw,
                    E1000_EITR(que->msix), que->eitr_setting);
 
        que->eitr_setting = 0;

        /* Idle, do nothing */
        if ((txr->bytes == 0) && (rxr->bytes == 0))
                goto no_calc;
                                
        /* Used half Default if sub-gig */
        if (adapter->link_speed != 1000)
                newitr = IGB_DEFAULT_ITR / 2;
        else {
		if ((txr->bytes) && (txr->packets))
                	newitr = txr->bytes/txr->packets;
		if ((rxr->bytes) && (rxr->packets))
			newitr = max(newitr,
			    (rxr->bytes / rxr->packets));
                newitr += 24; /* account for hardware frame, crc */
		/* set an upper boundary */
		newitr = min(newitr, 3000);
		/* Be nice to the mid range */
                if ((newitr > 300) && (newitr < 1200))
                        newitr = (newitr / 3);
                else
                        newitr = (newitr / 2);
        }
        newitr &= 0x7FFC;  /* Mask invalid bits */
        if (adapter->hw.mac.type == e1000_82575)
                newitr |= newitr << 16;
        else
                newitr |= E1000_EITR_CNT_IGNR;
                 
        /* save for next interrupt */
        que->eitr_setting = newitr;

        /* Reset state */
        txr->bytes = 0;
        txr->packets = 0;
        rxr->bytes = 0;
		rxr->rx_bytes = 0; 
        rxr->packets = 0;

no_calc:
		return (FILTER_SCHEDULE_THREAD); 
}


/*********************************************************************
 *
 *  MSIX Link Interrupt Service routine
 *
 **********************************************************************/

static int
igb_msix_link(void *arg)
{
	struct adapter	*adapter = arg;
	struct e1000_hw *hw = &adapter->hw;
	u32       	icr;

	++adapter->link_irq;
	MPASS(hw->back != NULL); 

	icr = E1000_READ_REG(&adapter->hw, E1000_ICR);
	if (!(icr & E1000_ICR_LSC))
		goto spurious;

	printf("link status set to 1 - msix_link called\n"); 
	adapter->hw.mac.get_link_status = 1;
	iflib_admin_intr_deferred(adapter->ctx); 

spurious:
	/* Rearm */
	E1000_WRITE_REG(&adapter->hw, E1000_IMS, E1000_IMS_LSC);
	E1000_WRITE_REG(&adapter->hw, E1000_EIMS, adapter->link_mask);
	return (FILTER_HANDLED); 
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
igb_if_media_status(if_ctx_t ctx, struct ifmediareq *ifmr)
{
	struct adapter *adapter = iflib_get_softc(ctx); 

	INIT_DEBUGOUT("igb_media_status: begin");

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (!adapter->link_active) {
		return;
	}
	
	ifmr->ifm_status |= IFM_ACTIVE;

	switch (adapter->link_speed) {
	case 10:
		ifmr->ifm_active |= IFM_10_T;
		break;
	case 100:
		/*
		** Support for 100Mb SFP - these are Fiber 
		** but the media type appears as serdes
		*/
		if (adapter->hw.phy.media_type ==
		    e1000_media_type_internal_serdes)
			ifmr->ifm_active |= IFM_100_FX;
		else
			ifmr->ifm_active |= IFM_100_TX;
		break;
	case 1000:
		ifmr->ifm_active |= IFM_1000_T;
		break;
	case 2500:
		ifmr->ifm_active |= IFM_2500_SX;
		break;
	}

	if (adapter->link_duplex == FULL_DUPLEX)
		ifmr->ifm_active |= IFM_FDX;
	else
		ifmr->ifm_active |= IFM_HDX;
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
igb_if_media_change(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct ifmedia  *ifm = iflib_get_media(ctx); 

	INIT_DEBUGOUT("igb_media_change: begin");

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
		device_printf(adapter->dev, "Unsupported media type\n");
	}

	return (0);
}



static int
igb_if_set_promisc(if_ctx_t ctx, int flags)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct ifnet	*ifp = iflib_get_ifp(ctx);
	struct e1000_hw *hw = &adapter->hw;
	u32		reg;

	if (adapter->vf_ifp) {
		e1000_promisc_set_vf(hw, e1000_promisc_enabled);
		return (0);
	}

	reg = E1000_READ_REG(hw, E1000_RCTL);
	if (ifp->if_flags & IFF_PROMISC) {
		reg |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		E1000_WRITE_REG(hw, E1000_RCTL, reg);
	} else if (ifp->if_flags & IFF_ALLMULTI) {
		reg |= E1000_RCTL_MPE;
		reg &= ~E1000_RCTL_UPE;
		E1000_WRITE_REG(hw, E1000_RCTL, reg);
	}
	return (0); 
}

static void
igb_disable_promisc(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct e1000_hw *hw = &adapter->hw;
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	u32		reg;
	int		mcnt = 0;

	if (adapter->vf_ifp) {
		e1000_promisc_set_vf(hw, e1000_promisc_disabled);
		return;
	}
	reg = E1000_READ_REG(hw, E1000_RCTL);
	reg &=  (~E1000_RCTL_UPE);
	if (ifp->if_flags & IFF_ALLMULTI)
		mcnt = MAX_NUM_MULTICAST_ADDRESSES;
	else {
		struct  ifmultiaddr *ifma;

		TAILQ_FOREACH(ifma, &ifp->if_multiaddrs, ifma_link) {
			if (ifma->ifma_addr->sa_family != AF_LINK)
				continue;
			if (mcnt == MAX_NUM_MULTICAST_ADDRESSES)
				break;
			mcnt++;
		}

	}
	/* Don't disable if in MAX groups */
	if (mcnt < MAX_NUM_MULTICAST_ADDRESSES)
		reg &=  (~E1000_RCTL_MPE);
	E1000_WRITE_REG(hw, E1000_RCTL, reg);
}


/*********************************************************************
 *  Multicast Update
 *
 *  This routine is called whenever multicast address list is updated.
 *
 **********************************************************************/

static void
igb_if_multi_set(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	struct ifmultiaddr *ifma;
	u32 reg_rctl = 0;
	u8  *mta;

	int mcnt = 0;

	IOCTL_DEBUGOUT("igb_set_multi: begin");

	mta = adapter->mta;
	bzero(mta, sizeof(uint8_t) * ETH_ADDR_LEN *
	    MAX_NUM_MULTICAST_ADDRESSES);

	TAILQ_FOREACH(ifma, &ifp->if_multiaddrs, ifma_link) {
		if (ifma->ifma_addr->sa_family != AF_LINK)
			continue;

		if (mcnt == MAX_NUM_MULTICAST_ADDRESSES)
			break;

		bcopy(LLADDR((struct sockaddr_dl *)ifma->ifma_addr),
		    &mta[mcnt * ETH_ADDR_LEN], ETH_ADDR_LEN);
		mcnt++;
	}

	if (mcnt >= MAX_NUM_MULTICAST_ADDRESSES) {
		reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		reg_rctl |= E1000_RCTL_MPE;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
	} else
		e1000_update_mc_addr_list(&adapter->hw, mta, mcnt);
}


/*********************************************************************
 *  Timer routine:
 *  	This routine checks for link status,
 *	updates statistics, and does the watchdog.
 *
 **********************************************************************/

static void
igb_if_timer(if_ctx_t ctx, uint16_t qid) 
{
	struct adapter		*adapter = iflib_get_softc(ctx);

	if (qid != 0)
		return;
	igb_update_stats_counters(adapter);

#ifndef DEVICE_POLLING
	/* Schedule all queue interrupts - deadlock protection */
	E1000_WRITE_REG(&adapter->hw, E1000_EICS, adapter->que_mask);
#endif
	igb_if_update_admin_status(ctx);
	iflib_admin_intr_deferred(ctx); 
}

static void
igb_if_update_admin_status(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct e1000_hw		*hw = &adapter->hw;
	struct e1000_fc_info	*fc = &hw->fc;
	struct ifnet		*ifp = iflib_get_ifp(ctx); 
	device_t		dev = iflib_get_dev(ctx); 
	u32			link_check, thstat, ctrl;
	char			*flowctl = NULL;

	link_check = thstat = ctrl = 0;

	/* Get the cached link value or read for real */
        switch (hw->phy.media_type) {
        case e1000_media_type_copper:
                if (hw->mac.get_link_status) {
			/* Do the work to read phy */
                        e1000_check_for_link(hw);
                        link_check = !hw->mac.get_link_status;
                } else {
			link_check = TRUE;
		}
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
	/* VF device is type_unknown */
        case e1000_media_type_unknown:
                e1000_check_for_link(hw);
		link_check = !hw->mac.get_link_status;
		/* Fall thru */
        default:
                break;
        }

	/* Check for thermal downshift or shutdown */
	if (hw->mac.type == e1000_i350) {
		thstat = E1000_READ_REG(hw, E1000_THSTAT);
		ctrl = E1000_READ_REG(hw, E1000_CTRL_EXT);
	}

	/* Get the flow control for display */
	switch (fc->current_mode) {
	case e1000_fc_rx_pause:
		flowctl = "RX";
		break;	
	case e1000_fc_tx_pause:
		flowctl = "TX";
		break;	
	case e1000_fc_full:
		flowctl = "Full";
		break;	
	case e1000_fc_none:
	default:
		flowctl = "None";
		break;	
	}

	/* Now we check if a transition has happened */
	if (link_check && (adapter->link_active == 0)) {
		printf("link check & link_active == 0\n"); 
		e1000_get_speed_and_duplex(&adapter->hw, 
		    &adapter->link_speed, &adapter->link_duplex);
		if (bootverbose)
			device_printf(dev, "Link is up %d Mbps %s,"
			    " Flow Control: %s\n",
			    adapter->link_speed,
			    ((adapter->link_duplex == FULL_DUPLEX) ?
			    "Full Duplex" : "Half Duplex"), flowctl);
		adapter->link_active = 1;
		ifp->if_baudrate = adapter->link_speed * 1000000;
		if ((ctrl & E1000_CTRL_EXT_LINK_MODE_GMII) &&
		    (thstat & E1000_THSTAT_LINK_THROTTLE))
			device_printf(dev, "Link: thermal downshift\n");
		/* Delay Link Up for Phy update */
		if (((hw->mac.type == e1000_i210) ||
		    (hw->mac.type == e1000_i211)) &&
		    (hw->phy.id == I210_I_PHY_ID))
			msec_delay(I210_LINK_DELAY);
		/* Reset if the media type changed. */
		if (hw->dev_spec._82575.media_changed) {
			hw->dev_spec._82575.media_changed = false;
			adapter->flags |= IGB_MEDIA_RESET;
			igb_reset(ctx); 
		}	
		/* This can sleep */
		iflib_link_state_change(ctx, LINK_STATE_UP); 
	} else if (!link_check && (adapter->link_active == 1)) {
		printf("no link check and adapter link active\n"); 
		ifp->if_baudrate = adapter->link_speed = 0;
		adapter->link_duplex = 0;
		if (bootverbose)
			device_printf(dev, "Link is Down\n");
		if ((ctrl & E1000_CTRL_EXT_LINK_MODE_GMII) &&
		    (thstat & E1000_THSTAT_PWR_DOWN))
			device_printf(dev, "Link: thermal shutdown\n");
		adapter->link_active = 0;
		/* This can sleep */
		iflib_link_state_change(ctx, LINK_STATE_DOWN); 
	}
}

/*********************************************************************
 *
 *  This routine disables all traffic on the adapter by issuing a
 *  global reset on the MAC and deallocates TX/RX buffers.
 *
 **********************************************************************/

static void
igb_if_stop(if_ctx_t ctx)
{
	struct adapter	*adapter = iflib_get_softc(ctx); 

	INIT_DEBUGOUT("igb_stop: begin");

	e1000_reset_hw(&adapter->hw);
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
igb_identify_hardware(if_ctx_t ctx)
{
	device_t dev = iflib_get_dev(ctx); 
	struct adapter *adapter = iflib_get_softc(ctx);
	
	/* Save off the information about this board */
	adapter->hw.vendor_id = pci_get_vendor(dev);
	adapter->hw.device_id = pci_get_device(dev);
	adapter->hw.revision_id = pci_read_config(dev, PCIR_REVID, 1);
	adapter->hw.subsystem_vendor_id =
	    pci_read_config(dev, PCIR_SUBVEND_0, 2);
	adapter->hw.subsystem_device_id =
	    pci_read_config(dev, PCIR_SUBDEV_0, 2);

	/* Set MAC type early for PCI setup */
	e1000_set_mac_type(&adapter->hw);

	/* Are we a VF device? */
	if ((adapter->hw.mac.type == e1000_vfadapt) ||
	    (adapter->hw.mac.type == e1000_vfadapt_i350))
		adapter->vf_ifp = 1;
	else
		adapter->vf_ifp = 0;
}

static int
igb_allocate_pci_resources(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	device_t	dev = iflib_get_dev(ctx);
	int         rid;

	rid = PCIR_BAR(0);
	adapter->pci_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
						  &rid, RF_ACTIVE);
	if (adapter->pci_mem == NULL) {
		device_printf(dev, "Unable to allocate bus resource: memory\n");
		return (ENXIO);
	}
	
	adapter->osdep.mem_bus_space_tag =
	    rman_get_bustag(adapter->pci_mem);
	adapter->osdep.mem_bus_space_handle =
	    rman_get_bushandle(adapter->pci_mem);
	adapter->hw.hw_addr = (u8 *)&adapter->osdep.mem_bus_space_handle;
	adapter->hw.back = &adapter->osdep;

	return (0);
}

/*********************************************************************
 *
 *  Setup the MSIX Queue Interrupt handlers: 
 *
 **********************************************************************/
static int
igb_if_msix_intr_assign(if_ctx_t ctx, int msix) 
{
	struct              adapter *adapter = iflib_get_softc(ctx); 
	struct igb_queue	*que = adapter->queues;
	int			        error, rid, vector = 0;
	int			        cpu_id = 0;

	/* Be sure to start with all interrupts disabled */
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, ~0);
	E1000_WRITE_FLUSH(&adapter->hw);

    #ifdef	RSS
	/*
	 * If we're doing RSS, the number of queues needs to
	 * match the number of RSS buckets that are configured.
	 *
	 * + If there's more queues than RSS buckets, we'll end
	 *   up with queues that get no traffic.
	 *
	 * + If there's more RSS buckets than queues, we'll end
	 *   up having multiple RSS buckets map to the same queue,
	 *   so there'll be some contention.
	 */
	if (adapter->num_queues != rss_getnumbuckets()) {
		device_printf(dev,
		    "%s: number of queues (%d) != number of RSS buckets (%d)"
		    "; performance will be impacted.\n",
		    __func__,
		    adapter->num_queues,
		    rss_getnumbuckets());
	}
#endif
	
	for (int i = 0; i < adapter->num_queues; i++, vector++, que++) {
		char buf[16];
		rid = vector +1;

		snprintf(buf, sizeof(buf), "rxq%d", i); 
		error = iflib_irq_alloc_generic(ctx, &que->que_irq, rid, IFLIB_INTR_RX, igb_msix_que, que, que->me, buf);  

		if (error) {
			device_printf(iflib_get_dev(ctx), "Failed to allocate que int %d err: %d", i, error);
			adapter->num_queues = i + 1;
			goto fail;
		}
	
		snprintf(buf, sizeof(buf), "txq%d", i);
		iflib_softirq_alloc_generic(ctx, rid, IFLIB_INTR_TX, que, que->me, buf);
			
		que->msix = vector;
		if (adapter->hw.mac.type == e1000_82575)
			que->eims = E1000_EICR_TX_QUEUE0 << i;
		else
			que->eims = 1 << vector;

#ifdef	RSS
		/*
		 * The queue ID is used as the RSS layer bucket ID.
		 * We look up the queue ID -> RSS CPU ID and select
		 * that.
		 */
		cpu_id = rss_getcpu(i % rss_getnumbuckets());
#else
		/*
		 * Bind the msix vector, and thus the
		 * rings to the corresponding cpu.
		 *
		 * This just happens to match the default RSS round-robin
		 * bucket -> queue -> CPU allocation.
		 */
		if (adapter->num_queues > 1) {
			if (igb_last_bind_cpu < 0)
				igb_last_bind_cpu = CPU_FIRST();
			cpu_id = igb_last_bind_cpu;
		}
#endif
	}
	rid = vector + 1;
        error = iflib_irq_alloc_generic(ctx, &adapter->irq, rid, IFLIB_INTR_ADMIN, igb_msix_link, adapter, 0, "aq");

	if (error) {
		device_printf(iflib_get_dev(ctx), "Failed to register admin handler");
		goto fail;
	}

	adapter->linkvec = vector;
	return (0);	
fail:
	iflib_irq_free(ctx, &adapter->irq);
	que = adapter->queues;
	for (int i = 0; i < adapter->num_queues; i++, que++)
		iflib_irq_free(ctx, &que->que_irq);
	return (error);
}


static void
	igb_configure_queues(struct adapter *adapter)
{
	struct	e1000_hw	*hw = &adapter->hw;
	struct	igb_queue	*que;
	u32			tmp, ivar = 0, newitr = 0;

	/* First turn on RSS capability */
	if (adapter->hw.mac.type != e1000_82575)
		E1000_WRITE_REG(hw, E1000_GPIE,
		    E1000_GPIE_MSIX_MODE | E1000_GPIE_EIAME |
		    E1000_GPIE_PBA | E1000_GPIE_NSICR);

	/* Turn on MSIX */
	switch (adapter->hw.mac.type) {
	case e1000_82580:
	case e1000_i350:
	case e1000_i354:
	case e1000_i210:
	case e1000_i211:
	case e1000_vfadapt:
	case e1000_vfadapt_i350:
		/* RX entries */
		for (int i = 0; i < adapter->num_queues; i++) {
			u32 index = i >> 1;
			ivar = E1000_READ_REG_ARRAY(hw, E1000_IVAR0, index);
			que = &adapter->queues[i];
			if (i & 1) {
				ivar &= 0xFF00FFFF;
				ivar |= (que->msix | E1000_IVAR_VALID) << 16;
			} else {
				ivar &= 0xFFFFFF00;
				ivar |= que->msix | E1000_IVAR_VALID;
			}
			E1000_WRITE_REG_ARRAY(hw, E1000_IVAR0, index, ivar);
		}
		/* TX entries */
		for (int i = 0; i < adapter->num_queues; i++) {
			u32 index = i >> 1;
			ivar = E1000_READ_REG_ARRAY(hw, E1000_IVAR0, index);
			que = &adapter->queues[i];
			if (i & 1) {
				ivar &= 0x00FFFFFF;
				ivar |= (que->msix | E1000_IVAR_VALID) << 24;
			} else {
				ivar &= 0xFFFF00FF;
				ivar |= (que->msix | E1000_IVAR_VALID) << 8;
			}
			E1000_WRITE_REG_ARRAY(hw, E1000_IVAR0, index, ivar);
			adapter->que_mask |= que->eims;
		}

		/* And for the link interrupt */
		ivar = (adapter->linkvec | E1000_IVAR_VALID) << 8;
		adapter->link_mask = 1 << adapter->linkvec;
		E1000_WRITE_REG(hw, E1000_IVAR_MISC, ivar);
		break;
	case e1000_82576:
		/* RX entries */
		for (int i = 0; i < adapter->num_queues; i++) {
			u32 index = i & 0x7; /* Each IVAR has two entries */
			ivar = E1000_READ_REG_ARRAY(hw, E1000_IVAR0, index);
			que = &adapter->queues[i];
			if (i < 8) {
				ivar &= 0xFFFFFF00;
				ivar |= que->msix | E1000_IVAR_VALID;
			} else {
				ivar &= 0xFF00FFFF;
				ivar |= (que->msix | E1000_IVAR_VALID) << 16;
			}
			E1000_WRITE_REG_ARRAY(hw, E1000_IVAR0, index, ivar);
			adapter->que_mask |= que->eims;
		}
		/* TX entries */
		for (int i = 0; i < adapter->num_queues; i++) {
			u32 index = i & 0x7; /* Each IVAR has two entries */
			ivar = E1000_READ_REG_ARRAY(hw, E1000_IVAR0, index);
			que = &adapter->queues[i];
			if (i < 8) {
				ivar &= 0xFFFF00FF;
				ivar |= (que->msix | E1000_IVAR_VALID) << 8;
			} else {
				ivar &= 0x00FFFFFF;
				ivar |= (que->msix | E1000_IVAR_VALID) << 24;
			}
			E1000_WRITE_REG_ARRAY(hw, E1000_IVAR0, index, ivar);
			adapter->que_mask |= que->eims;
		}

		/* And for the link interrupt */
		ivar = (adapter->linkvec | E1000_IVAR_VALID) << 8;
		adapter->link_mask = 1 << adapter->linkvec;
		E1000_WRITE_REG(hw, E1000_IVAR_MISC, ivar);
		break;

	case e1000_82575:
                /* enable MSI-X support*/
		tmp = E1000_READ_REG(hw, E1000_CTRL_EXT);
                tmp |= E1000_CTRL_EXT_PBA_CLR;
                /* Auto-Mask interrupts upon ICR read. */
                tmp |= E1000_CTRL_EXT_EIAME;
                tmp |= E1000_CTRL_EXT_IRCA;
                E1000_WRITE_REG(hw, E1000_CTRL_EXT, tmp);

		/* Queues */
		for (int i = 0; i < adapter->num_queues; i++) {
			que = &adapter->queues[i];
			tmp = E1000_EICR_RX_QUEUE0 << i;
			tmp |= E1000_EICR_TX_QUEUE0 << i;
			que->eims = tmp;
			E1000_WRITE_REG_ARRAY(hw, E1000_MSIXBM(0),
			    i, que->eims);
			adapter->que_mask |= que->eims;
		}

		/* Link */
		E1000_WRITE_REG(hw, E1000_MSIXBM(adapter->linkvec),
		    E1000_EIMS_OTHER);
		adapter->link_mask |= E1000_EIMS_OTHER;
	default:
		break;
	}

	/* Set the starting interrupt rate */
	if (igb_max_interrupt_rate > 0)
		newitr = (4000000 / igb_max_interrupt_rate) & 0x7FFC;

        if (hw->mac.type == e1000_82575)
                newitr |= newitr << 16;
        else
                newitr |= E1000_EITR_CNT_IGNR;

	for (int i = 0; i < adapter->num_queues; i++) {
		que = &adapter->queues[i];
		E1000_WRITE_REG(hw, E1000_EITR(que->msix), newitr);
	}

	return;
}


static void
igb_free_pci_resources(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct		igb_queue *que = adapter->queues;
	device_t	dev = iflib_get_dev(ctx); 

    if (adapter->intr_type == IFLIB_INTR_MSIX) {
		iflib_irq_free(ctx, &adapter->irq); 
	}

	/* First release all the interrupt resources */
	for (int i = 0; i < adapter->num_queues; i++, que++) {
		iflib_irq_free(ctx, &que->que_irq);
	}

    /* Free link/admin interrupt */
	if (adapter->pci_mem != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(0), adapter->pci_mem);
    }
}

static int
igb_set_num_queues(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
    int maxqueues = adapter->num_queues; 
	
	/* Sanity check based on HW */
	switch (adapter->hw.mac.type) {
		case e1000_82575:
			maxqueues = 4;
			break;
		case e1000_82576:
		case e1000_82580:
		case e1000_i350:
		case e1000_i354:
			maxqueues = 8;
			break;
		case e1000_i210:
			maxqueues = 4;
			break;
		case e1000_i211:
			maxqueues = 2;
			break;
		default:  /* VF interfaces */
			maxqueues = 1;
			break;
	}

	return maxqueues;
}

/*********************************************************************
 *
 *  Initialize the DMA Coalescing feature
 *
 **********************************************************************/
static void
igb_init_dmac(if_ctx_t ctx, u32 pba)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	device_t	dev = iflib_get_dev(ctx); 
	struct e1000_hw *hw = &adapter->hw;
	u32 		dmac, reg = ~E1000_DMACR_DMAC_EN;
	u16		hwm;

	if (hw->mac.type == e1000_i211)
		return;

	if (hw->mac.type > e1000_82580) {

		if (adapter->dmac == 0) { /* Disabling it */
			E1000_WRITE_REG(hw, E1000_DMACR, reg);
			return;
		} else
			device_printf(dev, "DMA Coalescing enabled\n");

		/* Set starting threshold */
		E1000_WRITE_REG(hw, E1000_DMCTXTH, 0);

		hwm = 64 * pba - adapter->max_frame_size / 16;
		if (hwm < 64 * (pba - 6))
			hwm = 64 * (pba - 6);
		reg = E1000_READ_REG(hw, E1000_FCRTC);
		reg &= ~E1000_FCRTC_RTH_COAL_MASK;
		reg |= ((hwm << E1000_FCRTC_RTH_COAL_SHIFT)
		    & E1000_FCRTC_RTH_COAL_MASK);
		E1000_WRITE_REG(hw, E1000_FCRTC, reg);


		dmac = pba - adapter->max_frame_size / 512;
		if (dmac < pba - 10)
			dmac = pba - 10;
		reg = E1000_READ_REG(hw, E1000_DMACR);
		reg &= ~E1000_DMACR_DMACTHR_MASK;
		reg = ((dmac << E1000_DMACR_DMACTHR_SHIFT)
		    & E1000_DMACR_DMACTHR_MASK);

		/* transition to L0x or L1 if available..*/
		reg |= (E1000_DMACR_DMAC_EN | E1000_DMACR_DMAC_LX_MASK);

		/* Check if status is 2.5Gb backplane connection
		* before configuration of watchdog timer, which is
		* in msec values in 12.8usec intervals
		* watchdog timer= msec values in 32usec intervals
		* for non 2.5Gb connection
		*/
		if (hw->mac.type == e1000_i354) {
			int status = E1000_READ_REG(hw, E1000_STATUS);
			if ((status & E1000_STATUS_2P5_SKU) &&
			    (!(status & E1000_STATUS_2P5_SKU_OVER)))
				reg |= ((adapter->dmac * 5) >> 6);
			else
				reg |= (adapter->dmac >> 5);
		} else {
			reg |= (adapter->dmac >> 5);
		}

		E1000_WRITE_REG(hw, E1000_DMACR, reg);

		E1000_WRITE_REG(hw, E1000_DMCRTRH, 0);

		/* Set the interval before transition */
		reg = E1000_READ_REG(hw, E1000_DMCTLX);
		if (hw->mac.type == e1000_i350)
			reg |= IGB_DMCTLX_DCFLUSH_DIS;
		/*
		** in 2.5Gb connection, TTLX unit is 0.4 usec
		** which is 0x4*2 = 0xA. But delay is still 4 usec
		*/
		if (hw->mac.type == e1000_i354) {
			int status = E1000_READ_REG(hw, E1000_STATUS);
			if ((status & E1000_STATUS_2P5_SKU) &&
			    (!(status & E1000_STATUS_2P5_SKU_OVER)))
				reg |= 0xA;
			else
				reg |= 0x4;
		} else {
			reg |= 0x4;
		}

		E1000_WRITE_REG(hw, E1000_DMCTLX, reg);

		/* free space in tx packet buffer to wake from DMA coal */
		E1000_WRITE_REG(hw, E1000_DMCTXTH, (IGB_TXPBSIZE -
		    (2 * adapter->max_frame_size)) >> 6);

		/* make low power state decision controlled by DMA coal */
		reg = E1000_READ_REG(hw, E1000_PCIEMISC);
		reg &= ~E1000_PCIEMISC_LX_DECISION;
		E1000_WRITE_REG(hw, E1000_PCIEMISC, reg);

	} else if (hw->mac.type == e1000_82580) {
		u32 reg = E1000_READ_REG(hw, E1000_PCIEMISC);
		E1000_WRITE_REG(hw, E1000_PCIEMISC,
		    reg & ~E1000_PCIEMISC_LX_DECISION);
		E1000_WRITE_REG(hw, E1000_DMACR, 0);
	}
}


/*********************************************************************
 *
 *  Set up an fresh starting state
 *
 **********************************************************************/
static void
igb_reset(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	device_t	dev = iflib_get_dev(ctx); 
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_fc_info *fc = &hw->fc;
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	u32		pba = 0;
	u16		hwm;

	INIT_DEBUGOUT("igb_reset: begin");

	/* Let the firmware know the OS is in control */
	igb_get_hw_control(adapter);

	/*
	 * Packet Buffer Allocation (PBA)
	 * Writing PBA sets the receive portion of the buffer
	 * the remainder is used for the transmit buffer.
	 */
	switch (hw->mac.type) {
	case e1000_82575:
		pba = E1000_PBA_32K;
		break;
	case e1000_82576:
	case e1000_vfadapt:
		pba = E1000_READ_REG(hw, E1000_RXPBS);
		pba &= E1000_RXPBS_SIZE_MASK_82576;
		break;
	case e1000_82580:
	case e1000_i350:
	case e1000_i354:
	case e1000_vfadapt_i350:
		pba = E1000_READ_REG(hw, E1000_RXPBS);
		pba = e1000_rxpbs_adjust_82580(pba);
		break;
	case e1000_i210:
	case e1000_i211:
		pba = E1000_PBA_34K;
	default:
		break;
	}

	/* Special needs in case of Jumbo frames */
	if ((hw->mac.type == e1000_82575) && (ifp->if_mtu > ETHERMTU)) {
		u32 tx_space, min_tx, min_rx;
		pba = E1000_READ_REG(hw, E1000_PBA);
		tx_space = pba >> 16;
		pba &= 0xffff;
		min_tx = (adapter->max_frame_size +
		    sizeof(struct e1000_tx_desc) - ETHERNET_FCS_SIZE) * 2;
		min_tx = roundup2(min_tx, 1024);
		min_tx >>= 10;
                min_rx = adapter->max_frame_size;
                min_rx = roundup2(min_rx, 1024);
                min_rx >>= 10;
		if (tx_space < min_tx &&
		    ((min_tx - tx_space) < pba)) {
			pba = pba - (min_tx - tx_space);
			/*
                         * if short on rx space, rx wins
                         * and must trump tx adjustment
			 */
                        if (pba < min_rx)
                                pba = min_rx;
		}
		E1000_WRITE_REG(hw, E1000_PBA, pba);
	}

	INIT_DEBUGOUT1("igb_init: pba=%dK",pba);

	/*
	 * These parameters control the automatic generation (Tx) and
	 * response (Rx) to Ethernet PAUSE frames.
	 * - High water mark should allow for at least two frames to be
	 *   received after sending an XOFF.
	 * - Low water mark works best when it is very near the high water mark.
	 *   This allows the receiver to restart by sending XON when it has
	 *   drained a bit.
	 */
	hwm = min(((pba << 10) * 9 / 10),
	    ((pba << 10) - 2 * adapter->max_frame_size));

	if (hw->mac.type < e1000_82576) {
		fc->high_water = hwm & 0xFFF8;  /* 8-byte granularity */
		fc->low_water = fc->high_water - 8;
	} else {
		fc->high_water = hwm & 0xFFF0;  /* 16-byte granularity */
		fc->low_water = fc->high_water - 16;
	}

	fc->pause_time = IGB_FC_PAUSE_TIME;
	fc->send_xon = TRUE;
	if (adapter->fc)
		fc->requested_mode = adapter->fc;
	else
		fc->requested_mode = e1000_fc_default;

	/* Issue a global reset */
	e1000_reset_hw(hw);
	E1000_WRITE_REG(hw, E1000_WUC, 0);

	/* Reset for AutoMediaDetect */
	if (adapter->flags & IGB_MEDIA_RESET) {
		e1000_setup_init_funcs(hw, TRUE);
		e1000_get_bus_info(hw);
		adapter->flags &= ~IGB_MEDIA_RESET;
	}

	if (e1000_init_hw(hw) < 0)
		device_printf(dev, "Hardware Initialization Failed\n");

	/* Setup DMA Coalescing */
	igb_init_dmac(ctx, pba);

	E1000_WRITE_REG(&adapter->hw, E1000_VET, ETHERTYPE_VLAN);
	e1000_get_phy_info(hw);
	e1000_check_for_link(hw);
	return;
}

/*********************************************************************
 *
 *  Setup networking device structure and register an interface.
 *
 **********************************************************************/
static int
igb_setup_interface(if_ctx_t ctx)
{
	struct ifnet   *ifp = iflib_get_ifp(ctx);
	struct adapter *adapter = iflib_get_softc(ctx); 
    uint64_t cap = 0; 
	
	INIT_DEBUGOUT("igb_setup_interface: begin");

#ifdef IGB_LEGACY_TX	
	ifp->if_start = igb_start;
	IFQ_SET_MAXLEN(&ifp->if_snd, igb_sctx->isc_ntxd - 1);
	ifp->if_snd.ifq_drv_maxlen = igb_sctx->isc_ntxd - 1;
	IFQ_SET_READY(&ifp->if_snd);
#endif

	cap = IFCAP_HWCSUM | IFCAP_VLAN_HWCSUM;
	cap |= IFCAP_TSO;
	cap |= IFCAP_JUMBO_MTU;
	ifp->if_capenable = ifp->if_capabilities;

	/* Don't enable LRO by default */
	cap |= IFCAP_LRO;

#ifdef DEVICE_POLLING
	cap |= IFCAP_POLLING;
#endif

	/*
	 * Tell the upper layer(s) we
	 * support full VLAN capability.
	 */
	cap |= IFCAP_VLAN_HWTAGGING
			     |  IFCAP_VLAN_HWTSO
			     |  IFCAP_VLAN_MTU;
	cap |= IFCAP_VLAN_HWTAGGING
			  |  IFCAP_VLAN_HWTSO
			  |  IFCAP_VLAN_MTU;

	if_setifheaderlen(ifp, sizeof(struct ether_vlan_header));
	if_setcapabilitiesbit(ifp, cap, 0);
	if_setcapenable(ifp, if_getcapabilities(ifp));
	/*
	** Don't turn this on by default, if vlans are
	** created on another pseudo device (eg. lagg)
	** then vlan events are not passed thru, breaking
	** operation, but with HW FILTER off it works. If
	** using vlans directly on the igb driver you can
	** enable this and get full hardware tag filtering.
	*/
	ifp->if_capabilities |= IFCAP_VLAN_HWFILTER;

	/*
	 * Specify the media types supported by this adapter and register
	 * callbacks to update media and link information
	 */
	if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
	    (adapter->hw.phy.media_type == e1000_media_type_internal_serdes)) {
		ifmedia_add(adapter->media, IFM_ETHER | IFM_1000_SX | IFM_FDX, 
			    0, NULL);
		ifmedia_add(adapter->media, IFM_ETHER | IFM_1000_SX, 0, NULL);
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
 *  Enable transmit unit.
 *
 **********************************************************************/
static void
igb_initialize_transmit_units(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	struct igb_queue *que;
	struct e1000_hw *hw = &adapter->hw;
    u32 txdctl, tctl;
	int i; 
	
	INIT_DEBUGOUT("igb_initialize_transmit_units: begin");

	/* Setup the Tx Descriptor Rings */
	for (i = 0, que = adapter->queues; i < adapter->num_queues; i++, que++) {
		struct tx_ring *txr = &que->txr; 
		u64 bus_addr = txr->tx_paddr;
		txdctl = tctl = 0;
		int j = txr->me; 

		E1000_WRITE_REG(hw, E1000_TDLEN(j),
		    igb_sctx->isc_ntxd * sizeof(struct e1000_tx_desc));
		E1000_WRITE_REG(hw, E1000_TDBAH(j),
		    (uint32_t)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_TDBAL(j),
		    (uint32_t)bus_addr);

		/* Setup the HW Tx Head and Tail descriptor pointers */
		E1000_WRITE_REG(hw, E1000_TDT(j), 0);
		E1000_WRITE_REG(hw, E1000_TDH(j), 0);

		HW_DEBUGOUT2("Base = %x, Length = %x\n",
		    E1000_READ_REG(hw, E1000_TDBAL(j)),
		    E1000_READ_REG(hw, E1000_TDLEN(j)));

		txdctl |= IGB_TX_PTHRESH;
		txdctl |= IGB_TX_HTHRESH << 8;
		/** NEED TO FIX in iflib.c */
		/*	txdctl |= IGB_TX_WTHRESH << 16; */
		txdctl |= E1000_TXDCTL_QUEUE_ENABLE;
		E1000_WRITE_REG(hw, E1000_TXDCTL(j), txdctl);
	}
	if (adapter->vf_ifp) {
		return;
	}	

	e1000_config_collision_dist(hw);

	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
			 (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));
	
	/* This write will effectively turn on the transmit unit. */
			E1000_WRITE_REG(hw, E1000_TCTL, tctl);
}

/*
 * Initialize the RSS mapping for NICs that support multiple transmit/
 * receive rings.
 */
static void
igb_initialize_rss_mapping(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int i;
	int queue_id;
	u32 reta;
	u32 rss_key[10], mrqc, shift = 0;

	/* XXX? */
	if (adapter->hw.mac.type == e1000_82575)
		shift = 6;

	/*
	 * The redirection table controls which destination
	 * queue each bucket redirects traffic to.
	 * Each DWORD represents four queues, with the LSB
	 * being the first queue in the DWORD.
	 *
	 * This just allocates buckets to queues using round-robin
	 * allocation.
	 *
	 * NOTE: It Just Happens to line up with the default
	 * RSS allocation method.
	 */

	/* Warning FM follows */
	reta = 0;
	for (i = 0; i < 128; i++) {
#ifdef	RSS
		queue_id = rss_get_indirection_to_bucket(i);
		/*
		 * If we have more queues than buckets, we'll
		 * end up mapping buckets to a subset of the
		 * queues.
		 *
		 * If we have more buckets than queues, we'll
		 * end up instead assigning multiple buckets
		 * to queues.
		 *
		 * Both are suboptimal, but we need to handle
		 * the case so we don't go out of bounds
		 * indexing arrays and such.
		 */
		queue_id = queue_id % adapter->num_queues;
#else
		queue_id = (i % adapter->num_queues);
#endif
		/* Adjust if required */
		queue_id = queue_id << shift;

		/*
		 * The low 8 bits are for hash value (n+0);
		 * The next 8 bits are for hash value (n+1), etc.
		 */
		reta = reta >> 8;
		reta = reta | ( ((uint32_t) queue_id) << 24);
		if ((i & 3) == 3) {
			E1000_WRITE_REG(hw, E1000_RETA(i >> 2), reta);
			reta = 0;
		}
	}

	/* Now fill in hash table */

	/*
	 * MRQC: Multiple Receive Queues Command
	 * Set queuing to RSS control, number depends on the device.
	 */
	mrqc = E1000_MRQC_ENABLE_RSS_8Q;

#ifdef	RSS
	/* XXX ew typecasting */
	rss_getkey((uint8_t *) &rss_key);
#else
	arc4rand(&rss_key, sizeof(rss_key), 0);
#endif
	for (i = 0; i < 10; i++)
		E1000_WRITE_REG_ARRAY(hw,
		    E1000_RSSRK(0), i, rss_key[i]);

	/*
	 * Configure the RSS fields to hash upon.
	 */
	mrqc |= (E1000_MRQC_RSS_FIELD_IPV4 |
	    E1000_MRQC_RSS_FIELD_IPV4_TCP);
	mrqc |= (E1000_MRQC_RSS_FIELD_IPV6 |
	    E1000_MRQC_RSS_FIELD_IPV6_TCP);
	mrqc |=( E1000_MRQC_RSS_FIELD_IPV4_UDP |
	    E1000_MRQC_RSS_FIELD_IPV6_UDP);
	mrqc |=( E1000_MRQC_RSS_FIELD_IPV6_UDP_EX |
	    E1000_MRQC_RSS_FIELD_IPV6_TCP_EX);

	E1000_WRITE_REG(hw, E1000_MRQC, mrqc);
}

/*********************************************************************
 *
 *  Setup receive registers and features
 *
 **********************************************************************/
static void
igb_initialize_receive_units(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	struct e1000_hw *hw = &adapter->hw;
	struct igb_queue *que; 
	u32	rctl, rxcsum, psize, srrctl = 0;
    int i; 
	
	INIT_DEBUGOUT("igb_initialize_receive_unit: begin");

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring
	 */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	
	/*
	** Set up for header split
	*/
	if (igb_header_split) {
		/* Use a standard mbuf for the header */
		srrctl |= IGB_HDR_BUF << E1000_SRRCTL_BSIZEHDRSIZE_SHIFT;
		srrctl |= E1000_SRRCTL_DESCTYPE_HDR_SPLIT_ALWAYS;
	} else
		srrctl |= E1000_SRRCTL_DESCTYPE_ADV_ONEBUF;
	
	/*
	** Set up for jumbo frames
	*/
	if (ifp->if_mtu > ETHERMTU) {
		rctl |= E1000_RCTL_LPE;
		if (adapter->rx_mbuf_sz == MJUMPAGESIZE) {
			srrctl |= 4096 >> E1000_SRRCTL_BSIZEPKT_SHIFT;
			rctl |= E1000_RCTL_SZ_4096 | E1000_RCTL_BSEX;
		} else if (adapter->rx_mbuf_sz > MJUMPAGESIZE) {
			srrctl |= 8192 >> E1000_SRRCTL_BSIZEPKT_SHIFT;
			rctl |= E1000_RCTL_SZ_8192 | E1000_RCTL_BSEX;
		}
		/* Set maximum packet len */
		psize = adapter->max_frame_size;
		/* are we on a vlan? */
		if (adapter->ifp->if_vlantrunk != NULL)
			psize += VLAN_TAG_SIZE;
		E1000_WRITE_REG(&adapter->hw, E1000_RLPML, psize);
	} else {
		rctl &= ~E1000_RCTL_LPE;
		srrctl |= 2048 >> E1000_SRRCTL_BSIZEPKT_SHIFT;
		rctl |= E1000_RCTL_SZ_2048;
	}
	
	/*
	 * If TX flow control is disabled and there's >1 queue defined,
	 * enable DROP.
	 *
	 * This drops frames rather than hanging the RX MAC for all queues.
	 */
	if ((adapter->num_queues > 1) &&
	    (adapter->fc == e1000_fc_none ||
	     adapter->fc == e1000_fc_rx_pause)) {
		srrctl |= E1000_SRRCTL_DROP_EN;
	}
	
	/* Setup the Base and Length of the Rx Descriptor Rings */
	for (i = 0, que = adapter->queues; i < adapter->num_queues; i++, que++) {
		struct rx_ring *rxr = &que->rxr;
		u64 bus_addr = rxr->rx_paddr;
		u32 rxdctl;

		E1000_WRITE_REG(hw, E1000_RDLEN(i),
		    igb_sctx->isc_nrxd  * sizeof(struct e1000_rx_desc));
		E1000_WRITE_REG(hw, E1000_RDBAH(i),
		    (uint32_t)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_RDBAL(i),
		    (uint32_t)bus_addr);
		E1000_WRITE_REG(hw, E1000_SRRCTL(i), srrctl);
		/* Enable this Queue */
		rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(i));
		rxdctl |= E1000_RXDCTL_QUEUE_ENABLE;
		rxdctl &= 0xFFF00000;
		rxdctl |= IGB_RX_PTHRESH;
		rxdctl |= IGB_RX_HTHRESH << 8;
		/* NEED TO FIX iflib.c */
		/*	rxdctl |= IGB_RX_WTHRESH << 16; */
		E1000_WRITE_REG(hw, E1000_RXDCTL(i), rxdctl);
	}

	/*
	** Setup for RX MultiQueue
	*/
	rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
	if (adapter->num_queues >1) {

		/* rss setup */
		igb_initialize_rss_mapping(adapter);

		/*
		** NOTE: Receive Full-Packet Checksum Offload 
		** is mutually exclusive with Multiqueue. However
		** this is not the same as TCP/IP checksums which
		** still work.
		*/
		rxcsum |= E1000_RXCSUM_PCSD;
#if __FreeBSD_version >= 800000
		/* For SCTP Offload */
		if (((hw->mac.type == e1000_82576) ||
		     (hw->mac.type == e1000_82580)) &&
		    (ifp->if_capenable & IFCAP_RXCSUM))
			rxcsum |= E1000_RXCSUM_CRCOFL;
#endif
	} else {
		/* Non RSS setup */
		if (ifp->if_capenable & IFCAP_RXCSUM) {
			rxcsum |= E1000_RXCSUM_IPPCSE;
#if __FreeBSD_version >= 800000
			if ((adapter->hw.mac.type == e1000_82576) ||
			    (adapter->hw.mac.type == e1000_82580))
				rxcsum |= E1000_RXCSUM_CRCOFL;
#endif
		} else
			rxcsum &= ~E1000_RXCSUM_TUOFL;
	}
	E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);

	/* Setup the Receive Control Register */
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO |
		   E1000_RCTL_RDMTS_HALF |
		   (hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);
	/* Strip CRC bytes. */
	rctl |= E1000_RCTL_SECRC;
	/* Make sure VLAN Filters are off */
	rctl &= ~E1000_RCTL_VFE;
	/* Don't store bad packets */
	rctl &= ~E1000_RCTL_SBP;

	/* Enable Receives */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers
	 *   - needs to be after enable
	 */
	for (i = 0, que = adapter->queues; i < adapter->num_queues; i++, que++) {
        struct rx_ring *rxr = &que->rxr; 
		E1000_WRITE_REG(hw, E1000_RDH(i), rxr->next_to_check);
#ifdef DEV_NETMAP
		/*
		 * an init() while a netmap client is active must
		 * preserve the rx buffers passed to userspace.
		 * In this driver it means we adjust RDT to
		 * something different from next_to_refresh
		 * (which is not used in netmap mode).
		 */
		if (ifp->if_capenable & IFCAP_NETMAP) {
			struct netmap_adapter *na = NA(adapter->ifp);
			struct netmap_kring *kring = &na->rx_rings[i];
			int t = rxr->next_to_refresh - nm_kr_rxspace(kring);

			if (t >= adapter->num_rx_desc)
				t -= adapter->num_rx_desc;
			else if (t < 0)
				t += adapter->num_rx_desc;
			E1000_WRITE_REG(hw, E1000_RDT(i), t);
		} else
#endif /* DEV_NETMAP */
		E1000_WRITE_REG(hw, E1000_RDT(i), rxr->next_to_refresh);
	}
	return;
}


static void
igb_if_vlan_register(if_ctx_t ctx, u16 vtag)
{
	struct adapter	*adapter = iflib_get_softc(ctx);
	struct ifnet *ifp = iflib_get_ifp(ctx); 
	u32		index, bit;

	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	adapter->shadow_vfta[index] |= (1 << bit);
	++adapter->num_vlans;
	/* Change hw filter setting */
	if (ifp->if_capenable & IFCAP_VLAN_HWFILTER)
		igb_setup_vlan_hw_support(ctx);
}

/*
 * This routine is run via an vlan
 * unconfig EVENT
 */
static void
igb_if_vlan_unregister(if_ctx_t ctx, u16 vtag)
{
	struct adapter	*adapter = iflib_get_softc(ctx);
	struct ifnet *ifp = iflib_get_ifp(ctx); 
	u32		index, bit;

	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	adapter->shadow_vfta[index] &= ~(1 << bit);
	--adapter->num_vlans;
	/* Change hw filter setting */
	if (ifp->if_capenable & IFCAP_VLAN_HWFILTER)
		igb_setup_vlan_hw_support(ctx);
}

static void
igb_setup_vlan_hw_support(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	struct e1000_hw *hw = &adapter->hw;
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	u32             reg;
	int             i;

	if (adapter->vf_ifp) {
		e1000_rlpml_set_vf(hw,
		    adapter->max_frame_size + VLAN_TAG_SIZE);
		return;
	}

	reg = E1000_READ_REG(hw, E1000_CTRL);
	reg |= E1000_CTRL_VME;
	E1000_WRITE_REG(hw, E1000_CTRL, reg);

	/* Enable the Filter Table */
	if (ifp->if_capenable & IFCAP_VLAN_HWFILTER) {
		reg = E1000_READ_REG(hw, E1000_RCTL);
		reg &= ~E1000_RCTL_CFIEN;
		reg |= E1000_RCTL_VFE;
		E1000_WRITE_REG(hw, E1000_RCTL, reg);
	}

	/* Update the frame size */
	E1000_WRITE_REG(&adapter->hw, E1000_RLPML,
	    adapter->max_frame_size + VLAN_TAG_SIZE);

	/* Don't bother with table if no vlans */
	if ((adapter->num_vlans == 0) ||
	    ((ifp->if_capenable & IFCAP_VLAN_HWFILTER) == 0))
                return;
	/*
	** A soft reset zero's out the VFTA, so
	** we need to repopulate it now.
	*/
	for (i = 0; i < IGB_VFTA_SIZE; i++)
                if (adapter->shadow_vfta[i] != 0) {
			if (adapter->vf_ifp)
				e1000_vfta_set_vf(hw,
				    adapter->shadow_vfta[i], TRUE);
			else
				e1000_write_vfta(hw,
				    i, adapter->shadow_vfta[i]);
		}
}

static void
igb_if_enable_intr(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	/* With RSS set up what to auto clear */
	if (adapter->intr_type == IFLIB_INTR_MSIX) {
		u32 mask = (adapter->que_mask | adapter->link_mask);
		E1000_WRITE_REG(&adapter->hw, E1000_EIAC, mask);
		E1000_WRITE_REG(&adapter->hw, E1000_EIAM, mask);
		E1000_WRITE_REG(&adapter->hw, E1000_EIMS, mask);
		E1000_WRITE_REG(&adapter->hw, E1000_IMS,
		    E1000_IMS_LSC);
	} else {
		E1000_WRITE_REG(&adapter->hw, E1000_IMS,
		    IMS_ENABLE_MASK);
	}
	E1000_WRITE_FLUSH(&adapter->hw);

	return;
}

static void
igb_if_disable_intr(if_ctx_t ctx)
{
	struct adapter *adapter = iflib_get_softc(ctx); 
	if (adapter->intr_type == IFLIB_INTR_MSIX) {
		E1000_WRITE_REG(&adapter->hw, E1000_EIMC, ~0);
		E1000_WRITE_REG(&adapter->hw, E1000_EIAC, 0);
	} 
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, ~0);
	E1000_WRITE_FLUSH(&adapter->hw);
	return;
}

/*
 * Bit of a misnomer, what this really means is
 * to enable OS management of the system... aka
 * to disable special hardware management features 
 */
static void
igb_init_manageability(struct adapter *adapter)
{
	if (adapter->has_manage) {
		int manc2h = E1000_READ_REG(&adapter->hw, E1000_MANC2H);
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);

		/* disable hardware interception of ARP */
		manc &= ~(E1000_MANC_ARP_EN);

                /* enable receiving management packets to the host */
		manc |= E1000_MANC_EN_MNG2HOST;
		manc2h |= 1 << 5;  /* Mng Port 623 */
		manc2h |= 1 << 6;  /* Mng Port 664 */
		E1000_WRITE_REG(&adapter->hw, E1000_MANC2H, manc2h);
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*
 * Give control back to hardware management
 * controller if there is one.
 */
static void
igb_release_manageability(struct adapter *adapter)
{
	if (adapter->has_manage) {
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);

		/* re-enable hardware interception of ARP */
		manc |= E1000_MANC_ARP_EN;
		manc &= ~E1000_MANC_EN_MNG2HOST;

		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*
 * igb_get_hw_control sets CTRL_EXT:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is loaded. 
 *
 */
static void
igb_get_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext;

	if (adapter->vf_ifp)
		return;

	/* Let firmware know the driver has taken over */
	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
}

/*
 * igb_release_hw_control resets CTRL_EXT:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that the
 * driver is no longer loaded.
 *
 */
static void
igb_release_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext;

	if (adapter->vf_ifp)
		return;

	/* Let firmware taken over control of h/w */
	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
}

static int
igb_is_valid_ether_addr(uint8_t *addr)
{
	char zero_addr[6] = { 0, 0, 0, 0, 0, 0 };

	if ((addr[0] & 1) || (!bcmp(addr, zero_addr, ETHER_ADDR_LEN))) {
		return (FALSE);
	}

	return (TRUE);
}


/*
 * Enable PCI Wake On Lan capability
 */
static void
igb_enable_wakeup(device_t dev)
{
	u16     cap, status;
	u8      id;

	/* First find the capabilities pointer*/
	cap = pci_read_config(dev, PCIR_CAP_PTR, 2);
	/* Read the PM Capabilities */
	id = pci_read_config(dev, cap, 1);
	if (id != PCIY_PMG)     /* Something wrong */
		return;
	/* OK, we have the power capabilities, so
	   now get the status register */
	cap += PCIR_POWER_STATUS;
	status = pci_read_config(dev, cap, 2);
	status |= PCIM_PSTAT_PME | PCIM_PSTAT_PMEENABLE;
	pci_write_config(dev, cap, status, 2);
	return;
}

static void
igb_led_func(void *arg, int onoff)
{
	struct adapter	*adapter = arg;

	if (onoff) {
		e1000_setup_led(&adapter->hw);
		e1000_led_on(&adapter->hw);
	} else {
		e1000_led_off(&adapter->hw);
		e1000_cleanup_led(&adapter->hw);
	}
}

static uint64_t
igb_get_vf_counter(if_ctx_t ctx, ift_counter cnt)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	struct e1000_vf_stats *stats;

	stats = (struct e1000_vf_stats *)adapter->stats;

	switch (cnt) {
	case IFCOUNTER_IPACKETS:
		return (stats->gprc);
	case IFCOUNTER_OPACKETS:
		return (stats->gptc);
	case IFCOUNTER_IBYTES:
		return (stats->gorc);
	case IFCOUNTER_OBYTES:
		return (stats->gotc);
	case IFCOUNTER_IMCASTS:
		return (stats->mprc);
	case IFCOUNTER_IERRORS:
		return (adapter->dropped_pkts);
	case IFCOUNTER_OERRORS:
		return (adapter->watchdog_events);
	default:
		return (if_get_counter_default(ifp, cnt));
	}
}

static uint64_t
igb_if_get_counter(if_ctx_t ctx, ift_counter cnt)
{
	struct adapter *adapter = iflib_get_softc(ctx);
	struct ifnet	*ifp = iflib_get_ifp(ctx); 
	struct e1000_hw_stats *stats;

	if (adapter->vf_ifp)
		return (igb_get_vf_counter(ctx, cnt));

	stats = (struct e1000_hw_stats *)adapter->stats;

	switch (cnt) {
	case IFCOUNTER_IPACKETS:
		return (stats->gprc);
	case IFCOUNTER_OPACKETS:
		return (stats->gptc);
	case IFCOUNTER_IBYTES:
		return (stats->gorc);
	case IFCOUNTER_OBYTES:
		return (stats->gotc);
	case IFCOUNTER_IMCASTS:
		return (stats->mprc);
	case IFCOUNTER_OMCASTS:
		return (stats->mptc);
	case IFCOUNTER_IERRORS:
		return (adapter->dropped_pkts + stats->rxerrc +
		    stats->crcerrs + stats->algnerrc +
		    stats->ruc + stats->roc + stats->cexterr);
	case IFCOUNTER_OERRORS:
		return (stats->ecol + stats->latecol +
		    adapter->watchdog_events);
	case IFCOUNTER_COLLISIONS:
		return (stats->colc);
	case IFCOUNTER_IQDROPS:
		return (stats->mpc);
	default:
		return (if_get_counter_default(ifp, cnt));
	}
}

/**********************************************************************
 *
 *  Update the board statistics counters.
 *
 **********************************************************************/
static void
igb_update_stats_counters(struct adapter *adapter)
{
	struct e1000_hw		*hw = &adapter->hw;
	struct e1000_hw_stats	*stats;

	/* 
	** The virtual function adapter has only a
	** small controlled set of stats, do only 
	** those and return.
	*/
	if (adapter->vf_ifp) {
		igb_update_vf_stats_counters(adapter);
		return;
	}

	stats = (struct e1000_hw_stats	*)adapter->stats;

	if (adapter->hw.phy.media_type == e1000_media_type_copper ||
	   (E1000_READ_REG(hw, E1000_STATUS) & E1000_STATUS_LU)) {
		stats->symerrs +=
		    E1000_READ_REG(hw,E1000_SYMERRS);
		stats->sec += E1000_READ_REG(hw, E1000_SEC);
	}

	stats->crcerrs += E1000_READ_REG(hw, E1000_CRCERRS);
	stats->mpc += E1000_READ_REG(hw, E1000_MPC);
	stats->scc += E1000_READ_REG(hw, E1000_SCC);
	stats->ecol += E1000_READ_REG(hw, E1000_ECOL);

	stats->mcc += E1000_READ_REG(hw, E1000_MCC);
	stats->latecol += E1000_READ_REG(hw, E1000_LATECOL);
	stats->colc += E1000_READ_REG(hw, E1000_COLC);
	stats->dc += E1000_READ_REG(hw, E1000_DC);
	stats->rlec += E1000_READ_REG(hw, E1000_RLEC);
	stats->xonrxc += E1000_READ_REG(hw, E1000_XONRXC);
	stats->xontxc += E1000_READ_REG(hw, E1000_XONTXC);
	/*
	** For watchdog management we need to know if we have been
	** paused during the last interval, so capture that here.
	*/ 
	adapter->pause_frames = E1000_READ_REG(&adapter->hw, E1000_XOFFRXC);
	stats->xoffrxc += adapter->pause_frames;
	stats->xofftxc += E1000_READ_REG(hw, E1000_XOFFTXC);
	stats->fcruc += E1000_READ_REG(hw, E1000_FCRUC);
	stats->prc64 += E1000_READ_REG(hw, E1000_PRC64);
	stats->prc127 += E1000_READ_REG(hw, E1000_PRC127);
	stats->prc255 += E1000_READ_REG(hw, E1000_PRC255);
	stats->prc511 += E1000_READ_REG(hw, E1000_PRC511);
	stats->prc1023 += E1000_READ_REG(hw, E1000_PRC1023);
	stats->prc1522 += E1000_READ_REG(hw, E1000_PRC1522);
	stats->gprc += E1000_READ_REG(hw, E1000_GPRC);
	stats->bprc += E1000_READ_REG(hw, E1000_BPRC);
	stats->mprc += E1000_READ_REG(hw, E1000_MPRC);
	stats->gptc += E1000_READ_REG(hw, E1000_GPTC);

	/* For the 64-bit byte counters the low dword must be read first. */
	/* Both registers clear on the read of the high dword */

	stats->gorc += E1000_READ_REG(hw, E1000_GORCL) +
	    ((u64)E1000_READ_REG(hw, E1000_GORCH) << 32);
	stats->gotc += E1000_READ_REG(hw, E1000_GOTCL) +
	    ((u64)E1000_READ_REG(hw, E1000_GOTCH) << 32);

	stats->rnbc += E1000_READ_REG(hw, E1000_RNBC);
	stats->ruc += E1000_READ_REG(hw, E1000_RUC);
	stats->rfc += E1000_READ_REG(hw, E1000_RFC);
	stats->roc += E1000_READ_REG(hw, E1000_ROC);
	stats->rjc += E1000_READ_REG(hw, E1000_RJC);

	stats->mgprc += E1000_READ_REG(hw, E1000_MGTPRC);
	stats->mgpdc += E1000_READ_REG(hw, E1000_MGTPDC);
	stats->mgptc += E1000_READ_REG(hw, E1000_MGTPTC);

	stats->tor += E1000_READ_REG(hw, E1000_TORL) +
	    ((u64)E1000_READ_REG(hw, E1000_TORH) << 32);
	stats->tot += E1000_READ_REG(hw, E1000_TOTL) +
	    ((u64)E1000_READ_REG(hw, E1000_TOTH) << 32);

	stats->tpr += E1000_READ_REG(hw, E1000_TPR);
	stats->tpt += E1000_READ_REG(hw, E1000_TPT);
	stats->ptc64 += E1000_READ_REG(hw, E1000_PTC64);
	stats->ptc127 += E1000_READ_REG(hw, E1000_PTC127);
	stats->ptc255 += E1000_READ_REG(hw, E1000_PTC255);
	stats->ptc511 += E1000_READ_REG(hw, E1000_PTC511);
	stats->ptc1023 += E1000_READ_REG(hw, E1000_PTC1023);
	stats->ptc1522 += E1000_READ_REG(hw, E1000_PTC1522);
	stats->mptc += E1000_READ_REG(hw, E1000_MPTC);
	stats->bptc += E1000_READ_REG(hw, E1000_BPTC);

	/* Interrupt Counts */

	stats->iac += E1000_READ_REG(hw, E1000_IAC);
	stats->icrxptc += E1000_READ_REG(hw, E1000_ICRXPTC);
	stats->icrxatc += E1000_READ_REG(hw, E1000_ICRXATC);
	stats->ictxptc += E1000_READ_REG(hw, E1000_ICTXPTC);
	stats->ictxatc += E1000_READ_REG(hw, E1000_ICTXATC);
	stats->ictxqec += E1000_READ_REG(hw, E1000_ICTXQEC);
	stats->ictxqmtc += E1000_READ_REG(hw, E1000_ICTXQMTC);
	stats->icrxdmtc += E1000_READ_REG(hw, E1000_ICRXDMTC);
	stats->icrxoc += E1000_READ_REG(hw, E1000_ICRXOC);

	/* Host to Card Statistics */

	stats->cbtmpc += E1000_READ_REG(hw, E1000_CBTMPC);
	stats->htdpmc += E1000_READ_REG(hw, E1000_HTDPMC);
	stats->cbrdpc += E1000_READ_REG(hw, E1000_CBRDPC);
	stats->cbrmpc += E1000_READ_REG(hw, E1000_CBRMPC);
	stats->rpthc += E1000_READ_REG(hw, E1000_RPTHC);
	stats->hgptc += E1000_READ_REG(hw, E1000_HGPTC);
	stats->htcbdpc += E1000_READ_REG(hw, E1000_HTCBDPC);
	stats->hgorc += (E1000_READ_REG(hw, E1000_HGORCL) +
	    ((u64)E1000_READ_REG(hw, E1000_HGORCH) << 32));
	stats->hgotc += (E1000_READ_REG(hw, E1000_HGOTCL) +
	    ((u64)E1000_READ_REG(hw, E1000_HGOTCH) << 32));
	stats->lenerrs += E1000_READ_REG(hw, E1000_LENERRS);
	stats->scvpc += E1000_READ_REG(hw, E1000_SCVPC);
	stats->hrmpc += E1000_READ_REG(hw, E1000_HRMPC);

	stats->algnerrc += E1000_READ_REG(hw, E1000_ALGNERRC);
	stats->rxerrc += E1000_READ_REG(hw, E1000_RXERRC);
	stats->tncrs += E1000_READ_REG(hw, E1000_TNCRS);
	stats->cexterr += E1000_READ_REG(hw, E1000_CEXTERR);
	stats->tsctc += E1000_READ_REG(hw, E1000_TSCTC);
	stats->tsctfc += E1000_READ_REG(hw, E1000_TSCTFC);

	/* Driver specific counters */
	adapter->device_control = E1000_READ_REG(hw, E1000_CTRL);
	adapter->rx_control = E1000_READ_REG(hw, E1000_RCTL);
	adapter->int_mask = E1000_READ_REG(hw, E1000_IMS);
	adapter->eint_mask = E1000_READ_REG(hw, E1000_EIMS);
	adapter->packet_buf_alloc_tx =
	    ((E1000_READ_REG(hw, E1000_PBA) & 0xffff0000) >> 16);
	adapter->packet_buf_alloc_rx =
	    (E1000_READ_REG(hw, E1000_PBA) & 0xffff);
}


/**********************************************************************
 *
 *  Initialize the VF board statistics counters.
 *
 **********************************************************************/
static void
igb_vf_init_stats(struct adapter *adapter)
{
        struct e1000_hw *hw = &adapter->hw;
	struct e1000_vf_stats	*stats;

	stats = (struct e1000_vf_stats	*)adapter->stats;
	if (stats == NULL)
		return;
        stats->last_gprc = E1000_READ_REG(hw, E1000_VFGPRC);
        stats->last_gorc = E1000_READ_REG(hw, E1000_VFGORC);
        stats->last_gptc = E1000_READ_REG(hw, E1000_VFGPTC);
        stats->last_gotc = E1000_READ_REG(hw, E1000_VFGOTC);
        stats->last_mprc = E1000_READ_REG(hw, E1000_VFMPRC);
}
 
/**********************************************************************
 *
 *  Update the VF board statistics counters.
 *
 **********************************************************************/
static void
igb_update_vf_stats_counters(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_vf_stats	*stats;

	if (adapter->link_speed == 0)
		return;

	stats = (struct e1000_vf_stats	*)adapter->stats;

	UPDATE_VF_REG(E1000_VFGPRC,
	    stats->last_gprc, stats->gprc);
	UPDATE_VF_REG(E1000_VFGORC,
	    stats->last_gorc, stats->gorc);
	UPDATE_VF_REG(E1000_VFGPTC,
	    stats->last_gptc, stats->gptc);
	UPDATE_VF_REG(E1000_VFGOTC,
	    stats->last_gotc, stats->gotc);
	UPDATE_VF_REG(E1000_VFMPRC,
	    stats->last_mprc, stats->mprc);
}

/* Export a single 32-bit register via a read-only sysctl. */
static int
igb_sysctl_reg_handler(SYSCTL_HANDLER_ARGS)
{
	struct adapter *adapter;
	u_int val;

	adapter = oidp->oid_arg1;
	val = E1000_READ_REG(&adapter->hw, oidp->oid_arg2);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

/*
**  Tuneable interrupt rate handler
*/
static int
igb_sysctl_interrupt_rate_handler(SYSCTL_HANDLER_ARGS)
{
	struct igb_queue	*que = ((struct igb_queue *)oidp->oid_arg1);
	int			error;
	u32			reg, usec, rate;
                        
	reg = E1000_READ_REG(&que->adapter->hw, E1000_EITR(que->msix));
	usec = ((reg & 0x7FFC) >> 2);
	if (usec > 0)
		rate = 1000000 / usec;
	else
		rate = 0;
	error = sysctl_handle_int(oidp, &rate, 0, req);
	if (error || !req->newptr)
		return error;
	return 0;
}

/*
 * Add sysctl variables, one per statistic, to the system.
 */
static void
igb_add_hw_stats(struct adapter *adapter)
{
	device_t dev = adapter->dev;
	struct igb_queue *que; 
    int i; 
	
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(dev);
	struct sysctl_oid_list *child = SYSCTL_CHILDREN(tree);
	struct e1000_hw_stats *stats = adapter->stats;

	struct sysctl_oid *stat_node, *queue_node, *int_node, *host_node;
	struct sysctl_oid_list *stat_list, *queue_list, *int_list, *host_list;

#define QUEUE_NAME_LEN 32
	char namebuf[QUEUE_NAME_LEN];

	/* Driver Statistics */
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "dropped", 
			CTLFLAG_RD, &adapter->dropped_pkts,
			"Driver dropped packets");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "link_irq", 
			CTLFLAG_RD, &adapter->link_irq,
			"Link MSIX IRQ Handled");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "mbuf_defrag_fail",
			CTLFLAG_RD, &adapter->mbuf_defrag_failed,
			"Defragmenting mbuf chain failed");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "tx_dma_fail", 
			CTLFLAG_RD, &adapter->no_tx_dma_setup,
			"Driver tx dma failure in xmit");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "rx_overruns",
			CTLFLAG_RD, &adapter->rx_overruns,
			"RX overruns");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "watchdog_timeouts",
			CTLFLAG_RD, &adapter->watchdog_events,
			"Watchdog timeouts");

	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "device_control", 
			CTLFLAG_RD, &adapter->device_control,
			"Device Control Register");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "rx_control", 
			CTLFLAG_RD, &adapter->rx_control,
			"Receiver Control Register");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "interrupt_mask", 
			CTLFLAG_RD, &adapter->int_mask,
			"Interrupt Mask");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "extended_int_mask", 
			CTLFLAG_RD, &adapter->eint_mask,
			"Extended Interrupt Mask");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "tx_buf_alloc", 
			CTLFLAG_RD, &adapter->packet_buf_alloc_tx,
			"Transmit Buffer Packet Allocation");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "rx_buf_alloc", 
			CTLFLAG_RD, &adapter->packet_buf_alloc_rx,
			"Receive Buffer Packet Allocation");
	SYSCTL_ADD_UINT(ctx, child, OID_AUTO, "fc_high_water",
			CTLFLAG_RD, &adapter->hw.fc.high_water, 0,
			"Flow Control High Watermark");
	SYSCTL_ADD_UINT(ctx, child, OID_AUTO, "fc_low_water", 
			CTLFLAG_RD, &adapter->hw.fc.low_water, 0,
			"Flow Control Low Watermark");

	for (i = 0, que = adapter->queues; i < adapter->num_queues; i++, que++) {
		struct tx_ring *txr = &que->txr;
		struct rx_ring *rxr = &que->rxr; 
		struct lro_ctrl *lro = &rxr->lro;

		snprintf(namebuf, QUEUE_NAME_LEN, "queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf,
					    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "interrupt_rate", 
				CTLTYPE_UINT | CTLFLAG_RD, &adapter->queues[i],
				sizeof(&adapter->queues[i]),
				igb_sysctl_interrupt_rate_handler,
				"IU", "Interrupt Rate");

		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "txd_head", 
				CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_TDH(txr->me),
				igb_sysctl_reg_handler, "IU",
 				"Transmit Descriptor Head");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "txd_tail", 
				CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_TDT(txr->me),
				igb_sysctl_reg_handler, "IU",
 				"Transmit Descriptor Tail");
		SYSCTL_ADD_QUAD(ctx, queue_list, OID_AUTO, "no_desc_avail", 
				CTLFLAG_RD, &txr->no_desc_avail,
				"Queue Descriptors Unavailable");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "tx_packets",
				CTLFLAG_RD, &txr->total_packets,
				"Queue Packets Transmitted");

		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "rxd_head", 
				CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_RDH(rxr->me),
				igb_sysctl_reg_handler, "IU",
				"Receive Descriptor Head");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "rxd_tail", 
				CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_RDT(rxr->me),
				igb_sysctl_reg_handler, "IU",
				"Receive Descriptor Tail");
		SYSCTL_ADD_QUAD(ctx, queue_list, OID_AUTO, "rx_packets",
				CTLFLAG_RD, &rxr->rx_packets,
				"Queue Packets Received");
		SYSCTL_ADD_QUAD(ctx, queue_list, OID_AUTO, "rx_bytes",
				CTLFLAG_RD, &rxr->rx_bytes,
				"Queue Bytes Received");
		SYSCTL_ADD_U64(ctx, queue_list, OID_AUTO, "lro_queued",
				CTLFLAG_RD, &lro->lro_queued, 0,
				"LRO Queued");
		SYSCTL_ADD_U64(ctx, queue_list, OID_AUTO, "lro_flushed",
				CTLFLAG_RD, &lro->lro_flushed, 0,
				"LRO Flushed");
	}

	/* MAC stats get their own sub node */

	stat_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "mac_stats", 
				    CTLFLAG_RD, NULL, "MAC Statistics");
	stat_list = SYSCTL_CHILDREN(stat_node);

	/*
	** VF adapter has a very limited set of stats
	** since its not managing the metal, so to speak.
	*/
	if (adapter->vf_ifp) {
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "good_pkts_recvd",
			CTLFLAG_RD, &stats->gprc,
			"Good Packets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "good_pkts_txd",
			CTLFLAG_RD, &stats->gptc,
			"Good Packets Transmitted");
 	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "good_octets_recvd", 
 			CTLFLAG_RD, &stats->gorc, 
 			"Good Octets Received"); 
 	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "good_octets_txd", 
 			CTLFLAG_RD, &stats->gotc, 
 			"Good Octets Transmitted"); 
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_recvd",
			CTLFLAG_RD, &stats->mprc,
			"Multicast Packets Received");
		return;
	}

	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "excess_coll", 
			CTLFLAG_RD, &stats->ecol,
			"Excessive collisions");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "single_coll", 
			CTLFLAG_RD, &stats->scc,
			"Single collisions");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "multiple_coll", 
			CTLFLAG_RD, &stats->mcc,
			"Multiple collisions");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "late_coll", 
			CTLFLAG_RD, &stats->latecol,
			"Late collisions");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "collision_count", 
			CTLFLAG_RD, &stats->colc,
			"Collision Count");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "symbol_errors",
			CTLFLAG_RD, &stats->symerrs,
			"Symbol Errors");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "sequence_errors",
			CTLFLAG_RD, &stats->sec,
			"Sequence Errors");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "defer_count",
			CTLFLAG_RD, &stats->dc,
			"Defer Count");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "missed_packets",
			CTLFLAG_RD, &stats->mpc,
			"Missed Packets");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "recv_length_errors",
			CTLFLAG_RD, &stats->rlec,
			"Receive Length Errors");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "recv_no_buff",
			CTLFLAG_RD, &stats->rnbc,
			"Receive No Buffers");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "recv_undersize",
			CTLFLAG_RD, &stats->ruc,
			"Receive Undersize");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "recv_fragmented",
			CTLFLAG_RD, &stats->rfc,
			"Fragmented Packets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "recv_oversize",
			CTLFLAG_RD, &stats->roc,
			"Oversized Packets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "recv_jabber",
			CTLFLAG_RD, &stats->rjc,
			"Recevied Jabber");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "recv_errs",
			CTLFLAG_RD, &stats->rxerrc,
			"Receive Errors");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "crc_errs",
			CTLFLAG_RD, &stats->crcerrs,
			"CRC errors");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "alignment_errs",
			CTLFLAG_RD, &stats->algnerrc,
			"Alignment Errors");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tx_no_crs",
			CTLFLAG_RD, &stats->tncrs,
			"Transmit with No CRS");
	/* On 82575 these are collision counts */
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "coll_ext_errs",
			CTLFLAG_RD, &stats->cexterr,
			"Collision/Carrier extension errors");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "xon_recvd",
			CTLFLAG_RD, &stats->xonrxc,
			"XON Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "xon_txd",
			CTLFLAG_RD, &stats->xontxc,
			"XON Transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "xoff_recvd",
			CTLFLAG_RD, &stats->xoffrxc,
			"XOFF Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "xoff_txd",
			CTLFLAG_RD, &stats->xofftxc,
			"XOFF Transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "unsupported_fc_recvd",
			CTLFLAG_RD, &stats->fcruc,
			"Unsupported Flow Control Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "mgmt_pkts_recvd",
			CTLFLAG_RD, &stats->mgprc,
			"Management Packets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "mgmt_pkts_drop",
			CTLFLAG_RD, &stats->mgpdc,
			"Management Packets Dropped");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "mgmt_pkts_txd",
			CTLFLAG_RD, &stats->mgptc,
			"Management Packets Transmitted");
	/* Packet Reception Stats */
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "total_pkts_recvd",
			CTLFLAG_RD, &stats->tpr,
			"Total Packets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "good_pkts_recvd",
			CTLFLAG_RD, &stats->gprc,
			"Good Packets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "bcast_pkts_recvd",
			CTLFLAG_RD, &stats->bprc,
			"Broadcast Packets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_recvd",
			CTLFLAG_RD, &stats->mprc,
			"Multicast Packets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "rx_frames_64",
			CTLFLAG_RD, &stats->prc64,
			"64 byte frames received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "rx_frames_65_127",
			CTLFLAG_RD, &stats->prc127,
			"65-127 byte frames received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "rx_frames_128_255",
			CTLFLAG_RD, &stats->prc255,
			"128-255 byte frames received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "rx_frames_256_511",
			CTLFLAG_RD, &stats->prc511,
			"256-511 byte frames received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "rx_frames_512_1023",
			CTLFLAG_RD, &stats->prc1023,
			"512-1023 byte frames received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "rx_frames_1024_1522",
			CTLFLAG_RD, &stats->prc1522,
			"1023-1522 byte frames received");
 	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "good_octets_recvd", 
 			CTLFLAG_RD, &stats->gorc, 
			"Good Octets Received");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "total_octets_recvd", 
			CTLFLAG_RD, &stats->tor, 
			"Total Octets Received");

	/* Packet Transmission Stats */
 	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "good_octets_txd", 
 			CTLFLAG_RD, &stats->gotc, 
 			"Good Octets Transmitted"); 
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "total_octets_txd", 
			CTLFLAG_RD, &stats->tot, 
			"Total Octets Transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "total_pkts_txd",
			CTLFLAG_RD, &stats->tpt,
			"Total Packets Transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "good_pkts_txd",
			CTLFLAG_RD, &stats->gptc,
			"Good Packets Transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "bcast_pkts_txd",
			CTLFLAG_RD, &stats->bptc,
			"Broadcast Packets Transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_txd",
			CTLFLAG_RD, &stats->mptc,
			"Multicast Packets Transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tx_frames_64",
			CTLFLAG_RD, &stats->ptc64,
			"64 byte frames transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tx_frames_65_127",
			CTLFLAG_RD, &stats->ptc127,
			"65-127 byte frames transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tx_frames_128_255",
			CTLFLAG_RD, &stats->ptc255,
			"128-255 byte frames transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tx_frames_256_511",
			CTLFLAG_RD, &stats->ptc511,
			"256-511 byte frames transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tx_frames_512_1023",
			CTLFLAG_RD, &stats->ptc1023,
			"512-1023 byte frames transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tx_frames_1024_1522",
			CTLFLAG_RD, &stats->ptc1522,
			"1024-1522 byte frames transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tso_txd",
			CTLFLAG_RD, &stats->tsctc,
			"TSO Contexts Transmitted");
	SYSCTL_ADD_QUAD(ctx, stat_list, OID_AUTO, "tso_ctx_fail",
			CTLFLAG_RD, &stats->tsctfc,
			"TSO Contexts Failed");


	/* Interrupt Stats */

	int_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "interrupts", 
				    CTLFLAG_RD, NULL, "Interrupt Statistics");
	int_list = SYSCTL_CHILDREN(int_node);

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "asserts",
			CTLFLAG_RD, &stats->iac,
			"Interrupt Assertion Count");

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "rx_pkt_timer",
			CTLFLAG_RD, &stats->icrxptc,
			"Interrupt Cause Rx Pkt Timer Expire Count");

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "rx_abs_timer",
			CTLFLAG_RD, &stats->icrxatc,
			"Interrupt Cause Rx Abs Timer Expire Count");

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "tx_pkt_timer",
			CTLFLAG_RD, &stats->ictxptc,
			"Interrupt Cause Tx Pkt Timer Expire Count");

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "tx_abs_timer",
			CTLFLAG_RD, &stats->ictxatc,
			"Interrupt Cause Tx Abs Timer Expire Count");

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "tx_queue_empty",
			CTLFLAG_RD, &stats->ictxqec,
			"Interrupt Cause Tx Queue Empty Count");

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "tx_queue_min_thresh",
			CTLFLAG_RD, &stats->ictxqmtc,
			"Interrupt Cause Tx Queue Min Thresh Count");

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "rx_desc_min_thresh",
			CTLFLAG_RD, &stats->icrxdmtc,
			"Interrupt Cause Rx Desc Min Thresh Count");

	SYSCTL_ADD_QUAD(ctx, int_list, OID_AUTO, "rx_overrun",
			CTLFLAG_RD, &stats->icrxoc,
			"Interrupt Cause Receiver Overrun Count");

	/* Host to Card Stats */

	host_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "host", 
				    CTLFLAG_RD, NULL, 
				    "Host to Card Statistics");

	host_list = SYSCTL_CHILDREN(host_node);

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "breaker_tx_pkt",
			CTLFLAG_RD, &stats->cbtmpc,
			"Circuit Breaker Tx Packet Count");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "host_tx_pkt_discard",
			CTLFLAG_RD, &stats->htdpmc,
			"Host Transmit Discarded Packets");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "rx_pkt",
			CTLFLAG_RD, &stats->rpthc,
			"Rx Packets To Host");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "breaker_rx_pkts",
			CTLFLAG_RD, &stats->cbrmpc,
			"Circuit Breaker Rx Packet Count");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "breaker_rx_pkt_drop",
			CTLFLAG_RD, &stats->cbrdpc,
			"Circuit Breaker Rx Dropped Count");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "tx_good_pkt",
			CTLFLAG_RD, &stats->hgptc,
			"Host Good Packets Tx Count");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "breaker_tx_pkt_drop",
			CTLFLAG_RD, &stats->htcbdpc,
			"Host Tx Circuit Breaker Dropped Count");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "rx_good_bytes",
			CTLFLAG_RD, &stats->hgorc,
			"Host Good Octets Received Count");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "tx_good_bytes",
			CTLFLAG_RD, &stats->hgotc,
			"Host Good Octets Transmit Count");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "length_errors",
			CTLFLAG_RD, &stats->lenerrs,
			"Length Errors");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "serdes_violation_pkt",
			CTLFLAG_RD, &stats->scvpc,
			"SerDes/SGMII Code Violation Pkt Count");

	SYSCTL_ADD_QUAD(ctx, host_list, OID_AUTO, "header_redir_missed",
			CTLFLAG_RD, &stats->hrmpc,
			"Header Redirection Missed Packet Count");
}


/**********************************************************************
 *
 *  This routine provides a way to dump out the adapter eeprom,
 *  often a useful debug/service tool. This only dumps the first
 *  32 words, stuff that matters is in that extent.
 *
 **********************************************************************/
static int
igb_sysctl_nvm_info(SYSCTL_HANDLER_ARGS)
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
		igb_print_nvm_info(adapter);
        }

	return (error);
}

static void
igb_print_nvm_info(struct adapter *adapter)
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

static void
igb_set_sysctl_value(struct adapter *adapter, const char *name,
	const char *description, int *limit, int value)
{
	*limit = value;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(adapter->dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(adapter->dev)),
	    OID_AUTO, name, CTLFLAG_RW, limit, value, description);
}

/*
** Set flow control using sysctl:
** Flow control values:
** 	0 - off
**	1 - rx pause
**	2 - tx pause
**	3 - full
*/
static int
igb_set_flowcntl(SYSCTL_HANDLER_ARGS)
{
	int		error;
	static int	input = 3; /* default is full */
	struct adapter	*adapter = (struct adapter *) arg1;

	error = sysctl_handle_int(oidp, &input, 0, req);

	if ((error) || (req->newptr == NULL))
		return (error);

	switch (input) {
		case e1000_fc_rx_pause:
		case e1000_fc_tx_pause:
		case e1000_fc_full:
		case e1000_fc_none:
			adapter->hw.fc.requested_mode = input;
			adapter->fc = input;
			break;
		default:
			/* Do nothing */
			return (error);
	}

	adapter->hw.fc.current_mode = adapter->hw.fc.requested_mode;
	e1000_force_mac_fc(&adapter->hw);
	/* XXX TODO: update DROP_EN on each RX queue if appropriate */
	return (error);
}

/*
** Manage DMA Coalesce:
** Control values:
** 	0/1 - off/on
**	Legal timer values are:
**	250,500,1000-10000 in thousands
*/
static int
igb_sysctl_dmac(SYSCTL_HANDLER_ARGS)
{
	struct adapter *adapter = (struct adapter *) arg1;
	int		error;

	error = sysctl_handle_int(oidp, &adapter->dmac, 0, req);

	if ((error) || (req->newptr == NULL))
		return (error);

	switch (adapter->dmac) {
		case 0:
			/* Disabling */
			break;
		case 1: /* Just enable and use default */
			adapter->dmac = 1000;
			break;
		case 250:
		case 500:
		case 1000:
		case 2000:
		case 3000:
		case 4000:
		case 5000:
		case 6000:
		case 7000:
		case 8000:
		case 9000:
		case 10000:
			/* Legal values - allow */
			break;
		default:
			/* Do nothing, illegal value */
			adapter->dmac = 0;
			return (EINVAL);
	}
	/* Reinit the interface */
	igb_if_init(adapter->ctx);
	return (error);
}

/*
** Manage Energy Efficient Ethernet:
** Control values:
**     0/1 - enabled/disabled
*/
static int
igb_sysctl_eee(SYSCTL_HANDLER_ARGS)
{
	struct adapter	*adapter = (struct adapter *) arg1;
	int		error, value;

	value = adapter->hw.dev_spec._82575.eee_disable;
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || req->newptr == NULL)
		return (error);
	adapter->hw.dev_spec._82575.eee_disable = (value != 0);
	igb_if_init(adapter->ctx);
	return (0);
}
