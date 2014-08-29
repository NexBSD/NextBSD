/******************************************************************************

  Copyright (c) 2001-2014, Intel Corporation 
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

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_device_polling.h"
#endif

#include <sys/param.h>
#include <sys/systm.h>
#if __FreeBSD_version >= 800000
#include <sys/buf_ring.h>
#endif
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
#include "e1000_82571.h"
#include "if_em.h"

/*********************************************************************
 *  Set this to one to display debug statistics
 *********************************************************************/
int	em_display_debug_stats = 0;

/*********************************************************************
 *  Driver version:
 *********************************************************************/
char em_driver_version[] = "7.4.2";

/*********************************************************************
 *  PCI Device ID Table
 *
 *  Used by probe to select devices to load on
 *  Last field stores an index into e1000_strings
 *  Last entry must be all 0s
 *
 *  { Vendor ID, Device ID, SubVendor ID, SubDevice ID, String Index }
 *********************************************************************/

static em_vendor_info_t em_vendor_info_array[] =
{
	/* Intel(R) PRO/1000 Network Connection */
	{ 0x8086, E1000_DEV_ID_82571EB_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82571EB_FIBER,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82571EB_SERDES,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82571EB_SERDES_DUAL,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82571EB_SERDES_QUAD,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82571EB_QUAD_COPPER,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82571EB_QUAD_COPPER_LP,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82571EB_QUAD_FIBER,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82571PT_QUAD_COPPER,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82572EI_COPPER,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82572EI_FIBER,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82572EI_SERDES,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82572EI,		PCI_ANY_ID, PCI_ANY_ID, 0},

	{ 0x8086, E1000_DEV_ID_82573E,		PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82573E_IAMT,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82573L,		PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82583V,		PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_80003ES2LAN_COPPER_SPT,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_80003ES2LAN_SERDES_SPT,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_80003ES2LAN_COPPER_DPT,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_80003ES2LAN_SERDES_DPT,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH8_IGP_M_AMT,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH8_IGP_AMT,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH8_IGP_C,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH8_IFE,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH8_IFE_GT,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH8_IFE_G,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH8_IGP_M,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH8_82567V_3,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_M_AMT,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_AMT,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_C,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_M,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_IGP_M_V,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_IFE,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_IFE_GT,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_IFE_G,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH9_BM,		PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82574L,		PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_82574LA,		PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH10_R_BM_LM,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH10_R_BM_LF,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH10_R_BM_V,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH10_D_BM_LM,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH10_D_BM_LF,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_ICH10_D_BM_V,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_M_HV_LM,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_M_HV_LC,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_D_HV_DM,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_D_HV_DC,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH2_LV_LM,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH2_LV_V,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_LPT_I217_LM,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_LPT_I217_V,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_LPTLP_I218_LM,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_LPTLP_I218_V,
						PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_I218_LM2,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_I218_V2,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_I218_LM3,	PCI_ANY_ID, PCI_ANY_ID, 0},
	{ 0x8086, E1000_DEV_ID_PCH_I218_V3,	PCI_ANY_ID, PCI_ANY_ID, 0},
	/* required last entry */
	{ 0, 0, 0, 0, 0}
};

/*********************************************************************
 *  Table of branding strings for all supported NICs.
 *********************************************************************/

static char *em_strings[] = {
	"Intel(R) PRO/1000 Network Connection"
};

/*********************************************************************
 *  Function prototypes
 *********************************************************************/
static int	em_probe(device_t);
static int	em_if_attach(iflib_ctx_t);
static int	em_if_detach(iflib_ctx_t);
static int	em_if_suspend(iflib_ctx_t);
static int	em_if_resume(iflib_ctx_t);
static void	em_stop(void *);
static void	em_media_status(iflib_ctx_t, struct ifmediareq *);
static int	em_media_change(iflib_ctx_t);
static void	em_identify_hardware(struct adapter *);
static int	em_allocate_pci_resources(struct adapter *);
static int	em_allocate_legacy(struct adapter *);
static int	em_allocate_msix(struct adapter *);
static int	em_if_queues_alloc(iflib_ctx_t);
static int	em_setup_msix(struct adapter *);
static void	em_free_pci_resources(struct adapter *);
static void	em_reset(struct adapter *);
static int	em_setup_interface(device_t, struct adapter *);
static void	em_initialize_transmit_unit(struct adapter *);

static int	em_setup_receive_structures(struct adapter *);
static int	em_allocate_receive_buffers(struct rx_ring *);
static void	em_initialize_receive_unit(struct adapter *);
static void	em_free_receive_structures(struct adapter *);
static void	em_free_receive_buffers(struct rx_ring *);

static void	em_if_intr_enable(iflib_ctx_t);
static void	em_if_intr_disable(iflib_ctx_t);
static void	em_update_stats_counters(struct adapter *);
static void	em_add_hw_stats(struct adapter *adapter);
static void	em_if_txeof(iflib_tx_ring_t);
static bool em_if_packet_get(void *, int, rx_info_t);
#ifndef __NO_STRICT_ALIGNMENT
static int	em_fixup_rx(struct rx_ring *);
#endif
static void	em_receive_checksum(struct e1000_rx_desc *, int *, int *);
static void	em_transmit_checksum_setup(struct tx_ring *, int, int,
		    struct ip *, u32 *, u32 *);
static void	em_tso_setup(struct tx_ring *, pkt_info_t, u32 *, u32 *);
static void	em_if_promisc_config(iflib_ctx_t, int);
static void	em_if_promisc_disable(iflib_ctx_t, int);
static void	em_if_multi_set(iflib_ctx_t);
static void	em_if_update_link_status(iflib_ctx_t);
static void	em_refresh_mbufs(struct rx_ring *, int);
static void	em_if_vlan_register(iflib_ctx_t, u16);
static void	em_if_vlan_unregister(iflib_ctx_t, u16);
static void	em_setup_vlan_hw_support(struct adapter *);
static int	em_if_tx(iflib_tx_ring_t, pkt_info_t);
static int	em_sysctl_nvm_info(SYSCTL_HANDLER_ARGS);
static void	em_print_nvm_info(struct adapter *);
static int	em_sysctl_debug_info(SYSCTL_HANDLER_ARGS);
static void	em_print_debug_info(struct adapter *);
static int 	em_is_valid_ether_addr(u8 *);
static int	em_sysctl_int_delay(SYSCTL_HANDLER_ARGS);
static void	em_add_int_delay_sysctl(struct adapter *, const char *,
		    const char *, struct em_int_delay_info *, int, int);
/* Management and WOL Support */
static void	em_init_manageability(struct adapter *);
static void	em_release_manageability(struct adapter *);
static void     em_get_hw_control(struct adapter *);
static void     em_release_hw_control(struct adapter *);
static void	em_get_wakeup(iflib_ctx_t);
static void     em_enable_wakeup(iflib_ctx_t);
static int	em_enable_phy_wakeup(struct adapter *);
static void	em_led_func(void *, int);
static void	em_disable_aspm(struct adapter *);

static int	em_irq_fast(void *);

/* MSIX handlers */
static void	em_set_sysctl_value(struct adapter *, const char *,
		    const char *, int *, int);
static int	em_set_flowcntl(SYSCTL_HANDLER_ARGS);
static int	em_sysctl_eee(SYSCTL_HANDLER_ARGS);

#ifdef DEVICE_POLLING
static poll_handler_drv_t em_poll;
#endif /* POLLING */

/*********************************************************************
 *  FreeBSD Device Interface Entry Points
 *********************************************************************/

static device_method_t em_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, em_probe),
	DEVMETHOD(device_attach, iflib_attach),
	DEVMETHOD(device_detach, iflib_detach),
	DEVMETHOD(device_shutdown, iflib_suspend),
	DEVMETHOD(device_suspend, iflib_suspend),
	DEVMETHOD(device_resume, iflib_resume),
	DEVMETHOD_END
};

static driver_t em_driver = {
	"em", em_methods, sizeof(struct adapter),
};

devclass_t em_devclass;
DRIVER_MODULE(em, pci, em_driver, em_devclass, 0, 0);
MODULE_DEPEND(em, pci, 1, 1, 1);
MODULE_DEPEND(em, ether, 1, 1, 1);

/*********************************************************************
 *  Tunable default values.
 *********************************************************************/

#define EM_TICKS_TO_USECS(ticks)	((1024 * (ticks) + 500) / 1000)
#define EM_USECS_TO_TICKS(usecs)	((1000 * (usecs) + 512) / 1024)
#define M_TSO_LEN			66

#define MAX_INTS_PER_SEC	8000
#define DEFAULT_ITR		(1000000000/(MAX_INTS_PER_SEC * 256))

/* Allow common code without TSO */
#ifndef CSUM_TSO
#define CSUM_TSO	0
#endif

static SYSCTL_NODE(_hw, OID_AUTO, em, CTLFLAG_RD, 0, "EM driver parameters");

static int em_tx_int_delay_dflt = EM_TICKS_TO_USECS(EM_TIDV);
static int em_rx_int_delay_dflt = EM_TICKS_TO_USECS(EM_RDTR);
SYSCTL_INT(_hw_em, OID_AUTO, tx_int_delay, CTLFLAG_RDTUN, &em_tx_int_delay_dflt,
    0, "Default transmit interrupt delay in usecs");
SYSCTL_INT(_hw_em, OID_AUTO, rx_int_delay, CTLFLAG_RDTUN, &em_rx_int_delay_dflt,
    0, "Default receive interrupt delay in usecs");

static int em_tx_abs_int_delay_dflt = EM_TICKS_TO_USECS(EM_TADV);
static int em_rx_abs_int_delay_dflt = EM_TICKS_TO_USECS(EM_RADV);
SYSCTL_INT(_hw_em, OID_AUTO, tx_abs_int_delay, CTLFLAG_RDTUN,
    &em_tx_abs_int_delay_dflt, 0,
    "Default transmit interrupt delay limit in usecs");
SYSCTL_INT(_hw_em, OID_AUTO, rx_abs_int_delay, CTLFLAG_RDTUN,
    &em_rx_abs_int_delay_dflt, 0,
    "Default receive interrupt delay limit in usecs");

static int em_rxd = EM_DEFAULT_RXD;
static int em_txd = EM_DEFAULT_TXD;
SYSCTL_INT(_hw_em, OID_AUTO, rxd, CTLFLAG_RDTUN, &em_rxd, 0,
    "Number of receive descriptors per queue");
SYSCTL_INT(_hw_em, OID_AUTO, txd, CTLFLAG_RDTUN, &em_txd, 0,
    "Number of transmit descriptors per queue");

static int em_smart_pwr_down = FALSE;
SYSCTL_INT(_hw_em, OID_AUTO, smart_pwr_down, CTLFLAG_RDTUN, &em_smart_pwr_down,
    0, "Set to true to leave smart power down enabled on newer adapters");

/* Controls whether promiscuous also shows bad packets */
static int em_debug_sbp = FALSE;
SYSCTL_INT(_hw_em, OID_AUTO, sbp, CTLFLAG_RDTUN, &em_debug_sbp, 0,
    "Show bad packets in promiscuous mode");

static int em_enable_msix = TRUE;
SYSCTL_INT(_hw_em, OID_AUTO, enable_msix, CTLFLAG_RDTUN, &em_enable_msix, 0,
    "Enable MSI-X interrupts");

/* How many packets rxeof tries to clean at a time */
static int em_rx_process_limit = 100;
SYSCTL_INT(_hw_em, OID_AUTO, rx_process_limit, CTLFLAG_RDTUN,
    &em_rx_process_limit, 0,
    "Maximum number of received packets to process "
    "at a time, -1 means unlimited");

/* Energy efficient ethernet - default to OFF */
static int eee_setting = 1;
SYSCTL_INT(_hw_em, OID_AUTO, eee_setting, CTLFLAG_RDTUN, &eee_setting, 0,
    "Enable Energy Efficient Ethernet");

/* Global used in WOL setup with multiport cards */
static int global_quad_port_a = 0;

#ifdef DEV_NETMAP	/* see ixgbe.c for details */
#include <dev/netmap/if_em_netmap.h>
#endif /* DEV_NETMAP */

/*********************************************************************
 *  Device identification routine
 *
 *  em_probe determines if the driver should be loaded on
 *  adapter based on PCI vendor/device id of the adapter.
 *
 *  return BUS_PROBE_DEFAULT on success, positive on failure
 *********************************************************************/

static int
em_probe(device_t dev)
{
	char		adapter_name[60];
	u16		pci_vendor_id = 0;
	u16		pci_device_id = 0;
	u16		pci_subvendor_id = 0;
	u16		pci_subdevice_id = 0;
	em_vendor_info_t *ent;

	INIT_DEBUGOUT("em_probe: begin");

	pci_vendor_id = pci_get_vendor(dev);
	if (pci_vendor_id != EM_VENDOR_ID)
		return (ENXIO);

	pci_device_id = pci_get_device(dev);
	pci_subvendor_id = pci_get_subvendor(dev);
	pci_subdevice_id = pci_get_subdevice(dev);

	ent = em_vendor_info_array;
	while (ent->vendor_id != 0) {
		if ((pci_vendor_id == ent->vendor_id) &&
		    (pci_device_id == ent->device_id) &&

		    ((pci_subvendor_id == ent->subvendor_id) ||
		    (ent->subvendor_id == PCI_ANY_ID)) &&

		    ((pci_subdevice_id == ent->subdevice_id) ||
		    (ent->subdevice_id == PCI_ANY_ID))) {
			sprintf(adapter_name, "%s %s",
				em_strings[ent->index],
				em_driver_version);
			device_set_desc_copy(dev, adapter_name);
			return (BUS_PROBE_DEFAULT);
		}
		ent++;
	}

	return (ENXIO);
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
em_if_attach(iflib_ctx_t ctx)
{
	device_t dev = iflib_device_get(ctx);
	struct adapter	*adapter = iflib_softc_get(ctx);
	struct e1000_hw	*hw;
	int		error = 0;
	
	INIT_DEBUGOUT("em_if_attach: begin");

	if (resource_disabled("em", device_get_unit(dev))) {
		device_printf(dev, "Disabled by device hint\n");
		return (ENXIO);
	}

	hw = &adapter->hw;
	
	/* SYSCTL stuff */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "nvm", CTLTYPE_INT|CTLFLAG_RW, adapter, 0,
	    em_sysctl_nvm_info, "I", "NVM Information");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "debug", CTLTYPE_INT|CTLFLAG_RW, adapter, 0,
	    em_sysctl_debug_info, "I", "Debug Information");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "fc", CTLTYPE_INT|CTLFLAG_RW, adapter, 0,
	    em_set_flowcntl, "I", "Flow Control");

	/* Determine hardware and mac info */
	em_identify_hardware(adapter);

	/* Setup PCI resources */
	if (em_allocate_pci_resources(adapter)) {
		device_printf(dev, "Allocation of PCI resources failed\n");
		error = ENXIO;
		goto err_pci;
	}

	/*
	** For ICH8 and family we need to
	** map the flash memory, and this
	** must happen after the MAC is 
	** identified
	*/
	if ((hw->mac.type == e1000_ich8lan) ||
	    (hw->mac.type == e1000_ich9lan) ||
	    (hw->mac.type == e1000_ich10lan) ||
	    (hw->mac.type == e1000_pchlan) ||
	    (hw->mac.type == e1000_pch2lan) ||
	    (hw->mac.type == e1000_pch_lpt)) {
		int rid = EM_BAR_TYPE_FLASH;
		adapter->flash = bus_alloc_resource_any(dev,
		    SYS_RES_MEMORY, &rid, RF_ACTIVE);
		if (adapter->flash == NULL) {
			device_printf(dev, "Mapping of Flash failed\n");
			error = ENXIO;
			goto err_pci;
		}
		/* This is used in the shared code */
		hw->flash_address = (u8 *)adapter->flash;
		adapter->osdep.flash_bus_space_tag =
		    rman_get_bustag(adapter->flash);
		adapter->osdep.flash_bus_space_handle =
		    rman_get_bushandle(adapter->flash);
	}

	/* Do Shared Code initialization */
	if (e1000_setup_init_funcs(hw, TRUE)) {
		device_printf(dev, "Setup of Shared code failed\n");
		error = ENXIO;
		goto err_pci;
	}

	e1000_get_bus_info(hw);

	/* Set up some sysctls for the tunable interrupt delays */
	em_add_int_delay_sysctl(adapter, "rx_int_delay",
	    "receive interrupt delay in usecs", &adapter->rx_int_delay,
	    E1000_REGISTER(hw, E1000_RDTR), em_rx_int_delay_dflt);
	em_add_int_delay_sysctl(adapter, "tx_int_delay",
	    "transmit interrupt delay in usecs", &adapter->tx_int_delay,
	    E1000_REGISTER(hw, E1000_TIDV), em_tx_int_delay_dflt);
	em_add_int_delay_sysctl(adapter, "rx_abs_int_delay",
	    "receive interrupt delay limit in usecs",
	    &adapter->rx_abs_int_delay,
	    E1000_REGISTER(hw, E1000_RADV),
	    em_rx_abs_int_delay_dflt);
	em_add_int_delay_sysctl(adapter, "tx_abs_int_delay",
	    "transmit interrupt delay limit in usecs",
	    &adapter->tx_abs_int_delay,
	    E1000_REGISTER(hw, E1000_TADV),
	    em_tx_abs_int_delay_dflt);
	em_add_int_delay_sysctl(adapter, "itr",
	    "interrupt delay limit in usecs/4",
	    &adapter->tx_itr,
	    E1000_REGISTER(hw, E1000_ITR),
	    DEFAULT_ITR);

	/* Sysctl for limiting the amount of work done in the taskqueue */
	em_set_sysctl_value(adapter, "rx_processing_limit",
	    "max number of rx packets to process", &adapter->rx_process_limit,
	    em_rx_process_limit);

	/*
	 * Validate number of transmit and receive descriptors. It
	 * must not exceed hardware maximum, and must be multiple
	 * of E1000_DBA_ALIGN.
	 */
	if (((em_txd * sizeof(struct e1000_tx_desc)) % EM_DBA_ALIGN) != 0 ||
	    (em_txd > EM_MAX_TXD) || (em_txd < EM_MIN_TXD)) {
		device_printf(dev, "Using %d TX descriptors instead of %d!\n",
		    EM_DEFAULT_TXD, em_txd);
		adapter->num_tx_desc = EM_DEFAULT_TXD;
	} else
		adapter->num_tx_desc = em_txd;

	if (((em_rxd * sizeof(struct e1000_rx_desc)) % EM_DBA_ALIGN) != 0 ||
	    (em_rxd > EM_MAX_RXD) || (em_rxd < EM_MIN_RXD)) {
		device_printf(dev, "Using %d RX descriptors instead of %d!\n",
		    EM_DEFAULT_RXD, em_rxd);
		adapter->num_rx_desc = EM_DEFAULT_RXD;
	} else
		adapter->num_rx_desc = em_rxd;

	hw->mac.autoneg = DO_AUTO_NEG;
	hw->phy.autoneg_wait_to_complete = FALSE;
	hw->phy.autoneg_advertised = AUTONEG_ADV_DEFAULT;

	/* Copper options */
	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = FALSE;
		hw->phy.ms_type = EM_MASTER_SLAVE;
	}

	/*
	 * Set the frame limits assuming
	 * standard ethernet sized frames.
	 */
	adapter->hw.mac.max_frame_size =
	    ETHERMTU + ETHER_HDR_LEN + ETHERNET_FCS_SIZE;

	/*
	 * This controls when hardware reports transmit completion
	 * status.
	 */
	hw->mac.report_tx_early = 1;

	/* 
	** Get queue/ring memory
	*/
	tsize = roundup2(adapter->num_tx_desc *
		sizeof(struct e1000_tx_desc), EM_DBA_ALIGN);
	rsize = roundup2(adapter->num_rx_desc *
	    sizeof(struct e1000_rx_desc), EM_DBA_ALIGN);

	if (iflib_queues_alloc(ctx, tsize, rsize)) {
		error = ENOMEM;
		goto err_pci;
	}

	/* Allocate multicast array memory. */
	adapter->mta = malloc(sizeof(u8) * ETH_ADDR_LEN *
	    MAX_NUM_MULTICAST_ADDRESSES, M_DEVBUF, M_NOWAIT);
	if (adapter->mta == NULL) {
		device_printf(dev, "Can not allocate multicast setup array\n");
		error = ENOMEM;
		goto err_late;
	}

	/* Check SOL/IDER usage */
	if (e1000_check_reset_block(hw))
		device_printf(dev, "PHY reset is blocked"
		    " due to SOL/IDER session.\n");

	/* Sysctl for setting Energy Efficient Ethernet */
	hw->dev_spec.ich8lan.eee_disable = eee_setting;
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "eee_control", CTLTYPE_INT|CTLFLAG_RW,
	    adapter, 0, em_sysctl_eee, "I",
	    "Disable Energy Efficient Ethernet");

	/*
	** Start from a known state, this is
	** important in reading the nvm and
	** mac from that.
	*/
	e1000_reset_hw(hw);


	/* Make sure we have a good EEPROM before we read from it */
	if (e1000_validate_nvm_checksum(hw) < 0) {
		/*
		** Some PCI-E parts fail the first check due to
		** the link being in sleep state, call it again,
		** if it fails a second time its a real issue.
		*/
		if (e1000_validate_nvm_checksum(hw) < 0) {
			device_printf(dev,
			    "The EEPROM Checksum Is Not Valid\n");
			error = EIO;
			goto err_late;
		}
	}

	/* Copy the permanent MAC address out of the EEPROM */
	if (e1000_read_mac_addr(hw) < 0) {
		device_printf(dev, "EEPROM read error while reading MAC"
		    " address\n");
		error = EIO;
		goto err_late;
	}

	if (!em_is_valid_ether_addr(hw->mac.addr)) {
		device_printf(dev, "Invalid MAC address\n");
		error = EIO;
		goto err_late;
	}

	/* Disable ULP support */
	e1000_disable_ulp_lpt_lp(hw, TRUE);

	/*
	**  Do interrupt configuration
	*/
	if (adapter->msix > 1) /* Do MSIX */
		error = em_allocate_msix(adapter);
	else  /* MSI or Legacy */
		error = em_allocate_legacy(adapter);
	if (error)
		goto err_late;

	/*
	 * Get Wake-on-Lan and Management info for later use
	 */
	em_get_wakeup(ctx);

	/* Setup OS specific network interface */
	if (em_setup_interface(dev, adapter) != 0)
		goto err_late;

	em_reset(adapter);

	/* Initialize statistics */
	em_update_stats_counters(adapter);

	hw->mac.get_link_status = 1;
	em_if_update_link_status(ctx);

	/* Register for VLAN events */
	iflib_vlan_register(ctx, em_register_vlan, em_unregister_vlan);
	
	em_add_hw_stats(adapter);

	/* Non-AMT based hardware can now take control from firmware */
	if (adapter->has_manage && !adapter->has_amt)
		em_get_hw_control(adapter);

	iflib_clearactive(ctx);

	adapter->led_dev = led_create(em_led_func, adapter,
	    device_get_nameunit(dev));
#ifdef DEV_NETMAP
	em_netmap_attach(adapter);
#endif /* DEV_NETMAP */

	INIT_DEBUGOUT("em_if_attach: end");

	return (0);

err_late:
	iflib_transmit_structures_free(ctx);
	em_free_receive_structures(adapter);
	em_release_hw_control(adapter);
	if (adapter->ctx != (void *)NULL)
		iflib_free_ctx(adapter->ctx);
err_pci:
	em_free_pci_resources(adapter);
	free(adapter->mta, M_DEVBUF);
	EM_CORE_LOCK_DESTROY(adapter);

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
em_if_detach(iflib_ctx_t ctx)
{
	struct adapter	*adapter = iflib_softc_get(ctx);

	INIT_DEBUGOUT("em_if_detach: begin");

	if (adapter->led_dev != NULL)
		led_destroy(adapter->led_dev);

	e1000_phy_hw_reset(&adapter->hw);

	em_release_manageability(adapter);
	em_release_hw_control(adapter);
	em_free_pci_resources(adapter);
	em_release_hw_control(adapter);
	free(adapter->mta, M_DEVBUF);
	return (0);
}

/*********************************************************************
 *
 *  Shutdown entry point
 *
 **********************************************************************/

/*
 * Suspend/resume device methods.
 */
static int
em_if_suspend(iflib_ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);

	em_release_manageability(adapter);
	em_release_hw_control(adapter);
	em_enable_wakeup(ctx);
}

static int
em_if_resume(iflib_ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);

	if (adapter->hw.mac.type == e1000_pch2lan)
		e1000_resume_workarounds_pchlan(&adapter->hw);
	em_if_init(ctx);
	em_init_manageability(adapter);
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
em_if_mtu_set(iflib_ctx_t ctx, uint32_t mtu)
{
		int max_frame_size;
		struct adapter *adapter = iflib_softc_get(ctx);
		
		IOCTL_DEBUGOUT("ioctl rcv'd: SIOCSIFMTU (Set Interface MTU)");

		EM_CORE_LOCK(adapter);
		switch (adapter->hw.mac.type) {
		case e1000_82571:
		case e1000_82572:
		case e1000_ich9lan:
		case e1000_ich10lan:
		case e1000_pch2lan:
		case e1000_pch_lpt:
		case e1000_82574:
		case e1000_82583:
		case e1000_80003es2lan:	/* 9K Jumbo Frame size */
			max_frame_size = 9234;
			break;
		case e1000_pchlan:
			max_frame_size = 4096;
			break;
			/* Adapters that do not support jumbo frames */
		case e1000_ich8lan:
			max_frame_size = ETHER_MAX_LEN;
			break;
		default:
			max_frame_size = MAX_JUMBO_FRAME_SIZE;
		}
		if (mtu > max_frame_size - ETHER_HDR_LEN -
		    ETHER_CRC_LEN) {
			EM_CORE_UNLOCK(adapter);
			error = EINVAL;
			break;
		}
		adapter->hw.mac.max_frame_size =
		    mtu + ETHER_HDR_LEN + ETHER_CRC_LEN;
		em_init_locked(adapter);
		EM_CORE_UNLOCK(adapter);
}

static void
em_if_set_media(iflib_ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);

	/* Check SOL/IDER usage */
	if (e1000_check_reset_block(&adapter->hw)) {
		iflib_printf(ctx, "Media change is"
					  " blocked due to SOL/IDER session.\n");
		break;
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
em_if_init(ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);

	INIT_DEBUGOUT("em_init: begin");

	/* Get the latest mac address, User can use a LAA */
	bcopy(iflib_getlladdr(ctx), adapter->hw.mac.addr,
		  ETHER_ADDR_LEN);

	/* Put the address into the Receive Address Array */
	e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	/*
	 * With the 82571 adapter, RAR[0] may be overwritten
	 * when the other port is reset, we make a duplicate
	 * in RAR[14] for that eventuality, this assures
	 * the interface continues to function.
	 */
	if (adapter->hw.mac.type == e1000_82571) {
		e1000_set_laa_state_82571(&adapter->hw, TRUE);
		e1000_rar_set(&adapter->hw, adapter->hw.mac.addr,
		    E1000_RAR_ENTRIES - 1);
	}

	/* Initialize the hardware */
	em_reset(adapter);
	em_if_update_link_status(ctx);

	/* Setup VLAN support, basic and offload if available */
	E1000_WRITE_REG(&adapter->hw, E1000_VET, ETHERTYPE_VLAN);

	/* Set hardware offload abilities */
	iflib_sethwassist(ctx);

	/* Configure for OS presence */
	em_init_manageability(adapter);

	/* Prepare transmit descriptors and buffers */
	iflib_transmit_structures_setup(ctx);
	em_initialize_transmit_unit(adapter);

	/* Setup Multicast table */
	em_if_multi_set(ctx);

	/*
	** Figure out the desired mbuf
	** pool for doing jumbos
	*/
	if (adapter->hw.mac.max_frame_size <= 2048)
		adapter->rx_mbuf_sz = MCLBYTES;
	else if (adapter->hw.mac.max_frame_size <= 4096)
		adapter->rx_mbuf_sz = MJUMPAGESIZE;
	else
		adapter->rx_mbuf_sz = MJUM9BYTES;

	/* Prepare receive descriptors and buffers */
	if (em_setup_receive_structures(adapter)) {
		device_printf(dev, "Could not setup receive structures\n");
		em_stop(adapter);
		return;
	}
	em_initialize_receive_unit(adapter);

#ifdef notyet	
	/* Use real VLAN Filter support? */
	if (if_getcapenable(ifp) & IFCAP_VLAN_HWTAGGING) {
		if (if_getcapenable(ifp) & IFCAP_VLAN_HWFILTER)
			/* Use real VLAN Filter support */
			em_setup_vlan_hw_support(adapter);
		else {
			u32 ctrl;
			ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
			ctrl |= E1000_CTRL_VME;
			E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
		}
	}
#endif
	/* Don't lose promiscuous settings */
	iflib_promisc_config(ctx);

	e1000_clear_hw_cntrs_base_generic(&adapter->hw);

	/* MSI/X configuration for 82574 */
	if (adapter->hw.mac.type == e1000_82574) {
		int tmp;
		tmp = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
		tmp |= E1000_CTRL_EXT_PBA_CLR;
		E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT, tmp);
		/* Set the IVAR - interrupt vector routing. */
		E1000_WRITE_REG(&adapter->hw, E1000_IVAR, adapter->ivars);
	}

	em_if_intr_enable(adapter);

	/* AMT based hardware can now take control from firmware */
	if (adapter->has_manage && adapter->has_amt)
		em_get_hw_control(adapter);
}

/*********************************************************************
 *
 *  Fast Legacy/MSI Combined Interrupt Service routine  
 *
 *********************************************************************/
static int
em_irq_fast(void *arg)
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

	/*
	 * Starting with the 82571 chip, bit 31 should be used to
	 * determine whether the interrupt belongs to us.
	 */
	if (adapter->hw.mac.type >= e1000_82571 &&
	    (reg_icr & E1000_ICR_INT_ASSERTED) == 0)
		return FILTER_STRAY;

	em_if_intr_disable(adapter->ctx);
	iflib_legacy_intr_deferred(adapter->ctx);

	/* Link status change */
	if (reg_icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		adapter->hw.mac.get_link_status = 1;
		iflib_link_intr_deferred(adapter->ctx);
	}

	if (reg_icr & E1000_ICR_RXO)
		adapter->rx_overruns++;
	return FILTER_HANDLED;
}

/*********************************************************************
 *
 *  MSIX Interrupt Service Routines
 *
 **********************************************************************/
static void
em_if_tx_intr_enable(void *arg)
{
	struct tx_ring *txr = arg;
	struct adapter *adapter = txr->adapter;
	
	/* Reenable this interrupt */
	E1000_WRITE_REG(&adapter->hw, E1000_IMS, txr->ims);
}

/*********************************************************************
 *
 *  MSIX RX Interrupt Service routine
 *
 **********************************************************************/

static void
em_if_rx_intr_enable(struct rx_ring *rxr)
{
	struct adapter	*adapter = rxr->adapter;

	/* Reenable this interrupt */
	E1000_WRITE_REG(&adapter->hw, E1000_IMS, rxr->ims);
}

/*********************************************************************
 *
 *  MSIX Link Fast Interrupt Service routine
 *
 **********************************************************************/
static void
em_msix_link(void *arg)
{
	struct adapter	*adapter = arg;
	u32		reg_icr;

	++adapter->link_irq;
	reg_icr = E1000_READ_REG(&adapter->hw, E1000_ICR);

	if (reg_icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		adapter->hw.mac.get_link_status = 1;
		iflib_link_intr_deferred(adapter->ctx);
	} else
		E1000_WRITE_REG(&adapter->hw, E1000_IMS,
		    EM_MSIX_LINK | E1000_IMS_LSC);
}

static void
em_if_link_intr_enable(void *arg)
{
	struct adapter	*adapter = arg;

	E1000_WRITE_REG(&adapter->hw, E1000_IMS,
					EM_MSIX_LINK | E1000_IMS_LSC);

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
em_if_media_status(iflib_ctx_t ctx, struct ifmediareq *ifmr)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	u_char fiber_type = IFM_1000_SX;

	INIT_DEBUGOUT("em_media_status: begin");

	em_if_update_link_status(ctx);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (!adapter->link_active) {
		return;
	}

	ifmr->ifm_status |= IFM_ACTIVE;

	if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
	    (adapter->hw.phy.media_type == e1000_media_type_internal_serdes)) {
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
em_if_media_change(iflib_ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	struct ifmedia  *ifm = &adapter->media;

	INIT_DEBUGOUT("em_media_change: begin");

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
		iflib_printf(ctx, "Unsupported media type\n");
	}

	iflib_init(ctx);
	return (0);
}

/*********************************************************************
 *
 *  This routine maps the mbufs to tx descriptors.
 *
 *  return 0 on success, positive on failure
 **********************************************************************/

static int
em_if_tx(void *hw_txr, pkt_info_t pi)
{
	struct tx_ring *txr = hw_txr;
	struct e1000_tx_desc	*ctxd = NULL;
	u32 txd_upper, txd_lower, txd_used, txd_saved;
	bus_dma_segment_t *segs = pi->pi_segs;
	int nsegs = pi->pi_nsegs

	txd_upper = txd_lower = txd_used = txd_saved = 0;
	/* Do hardware assists */
	if (pi->pi_csum_flags & CSUM_TSO) {
		em_tso_setup(txr, pi, &txd_upper, &txd_lower);
		/* we need to make a final sentinel transmit desc */
		txr->tx_tso = tso_desc = TRUE;
	} else if (pi->pi_csum_flags & CSUM_OFFLOAD)
		em_transmit_checksum_setup(txr, pi->pi_csum_flags,
		    ip_off, ip, &txd_upper, &txd_lower);

	if (pi->pi_flags & M_VLANTAG) {
		/* Set the vlan id. */
		txd_upper |= htole16(pi->pi_vtag) << 16;
		/* Tell hardware to add tag */
		txd_lower |= htole32(E1000_TXD_CMD_VLE);
	}

	/*
	 * TSO Hardware workaround, if this packet is not
	 * TSO, and is only a single descriptor long, and
	 * it follows a TSO burst, then we need to add a
	 * sentinel descriptor to prevent premature writeback.
	 */
	if ((tso_desc == FALSE) && (txr->tx_tso == TRUE)) {
		if (nsegs == 1)
			tso_desc = TRUE;
		txr->tx_tso = FALSE;
	}
	i = first;

	/* Set up our transmit descriptors */
	for (j = 0; j < nsegs; j++) {
		bus_size_t seg_len;
		bus_addr_t seg_addr;

		ctxd = &txr->tx_base[i];
		seg_addr = segs[j].ds_addr;
		seg_len  = segs[j].ds_len;
		/*
		** TSO Workaround:
		** If this is the last descriptor, we want to
		** split it so we have a small final sentinel
		*/
		if (tso_desc && (j == (nsegs -1)) && (seg_len > 8)) {
			seg_len -= 4;
			ctxd->buffer_addr = htole64(seg_addr);
			ctxd->lower.data = htole32(
			adapter->txd_cmd | txd_lower | seg_len);
			ctxd->upper.data =
			    htole32(txd_upper);
			if (++i == adapter->num_tx_desc)
				i = 0;
			/* Now make the sentinel */	
			++txd_used; /* using an extra txd */
			ctxd = &txr->tx_base[i];
			ctxd->buffer_addr =
			    htole64(seg_addr + seg_len);
			ctxd->lower.data = htole32(
			adapter->txd_cmd | txd_lower | 4);
			ctxd->upper.data =
			    htole32(txd_upper);
			last = i;
			if (++i == adapter->num_tx_desc)
				i = 0;
		} else {
			ctxd->buffer_addr = htole64(seg_addr);
			ctxd->lower.data = htole32(
			adapter->txd_cmd | txd_lower | seg_len);
			ctxd->upper.data =
			    htole32(txd_upper);
			last = i;
			if (++i == adapter->num_tx_desc)
				i = 0;
		}
#if 0
		tx_buffer->m_head = NULL;
		tx_buffer->next_eop = -1;
#endif
	}

	txr->next_avail_desc = i;
	txr->tx_avail -= nsegs;
	if (tso_desc) /* TSO used an extra for sentinel */
		txr->tx_avail -= txd_used;

#if 0	
	/*
	** Here we swap the map so the last descriptor,
	** which gets the completion interrupt has the
	** real map, and the first descriptor gets the
	** unused map from this descriptor.
	*/
	tx_buffer = &txr->tx_buffers[first];
	tx_buffer_mapped->map = tx_buffer->map;
	tx_buffer->map = map;
	bus_dmamap_sync(txr->txtag, map, BUS_DMASYNC_PREWRITE);
#endif
	/*
	 * Last Descriptor of Packet
	 * needs End Of Packet (EOP)
	 * and Report Status (RS)
	 */
	ctxd->lower.data |=
	    htole32(E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);
	/*
	 * Keep track in the first buffer which
	 * descriptor will be written back
	 */
	tx_buffer = &txr->tx_buffers[first];
	pi->pi_last = last;
	/* Update the watchdog time early and often */
	txr->watchdog_time = ticks;

	/*
	 * Advance the Transmit Descriptor Tail (TDT), this tells the E1000
	 * that this frame is available to transmit.
	 */
	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
					BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	/* maybe this can be batched - should be moved to em_tx_flush */
	E1000_WRITE_REG(&adapter->hw, E1000_TDT(txr->me), i);

	return (0);
}

static void
em_if_promisc_config(iflib_ctx_t ctx, int flags)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	u32		reg_rctl;

	reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);

	if (flags & IFF_PROMISC) {
		reg_rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		/* Turn this on if you want to see bad packets */
		if (em_debug_sbp)
			reg_rctl |= E1000_RCTL_SBP;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
	} else if (flags & IFF_ALLMULTI) {
		reg_rctl |= E1000_RCTL_MPE;
		reg_rctl &= ~E1000_RCTL_UPE;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
	}
}

static void
em_if_promisc_disable(iflib_ctx_t ctx, int flags)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	u32		reg_rctl;
	int		mcnt = 0;

	reg_rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
	reg_rctl &=  (~E1000_RCTL_UPE);
	mcnt = MAX_NUM_MULTICAST_ADDRESSES;

	if ((flags & IFF_ALLMULTI) == 0)
		mcnt = iflib_multiaddr_count(adapter->ctx, MAX_NUM_MULTICAST_ADDRESSES);
	/* Don't disable if in MAX groups */
	if (mcnt < MAX_NUM_MULTICAST_ADDRESSES)
		reg_rctl &=  (~E1000_RCTL_MPE);
	reg_rctl &=  (~E1000_RCTL_SBP);
	E1000_WRITE_REG(&adapter->hw, E1000_RCTL, reg_rctl);
}


/*********************************************************************
 *  Multicast Update
 *
 *  This routine is called whenever multicast address list is updated.
 *
 **********************************************************************/

static void
em_if_multi_set(iflib_ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	u32 reg_rctl = 0;
	u8  *mta; /* Multicast array memory */
	int mcnt = 0;

	IOCTL_DEBUGOUT("em_if_multi_set: begin");

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

	iflib_multiaddr_array(ctx, mta, &mcnt, MAX_NUM_MULTICAST_ADDRESSES);

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
em_if_timer(iflib_ctx_t ctx)
{
	struct adapter	*adapter = iflib_softc_get(ctx);
	struct tx_ring	*txr = adapter->tx_rings;
	struct rx_ring	*rxr = adapter->rx_rings;
	u32		trigger;

	em_if_update_link_status(ctx);
	em_update_stats_counters(adapter);

	/* Reset LAA into RAR[0] on 82571 */
	if ((adapter->hw.mac.type == e1000_82571) &&
	    e1000_get_laa_state_82571(&adapter->hw))
		e1000_rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	/* Mask to use in the irq trigger */
	if (adapter->msix_mem)
		trigger = rxr->ims;
	else
		trigger = E1000_ICS_RXDMT0;

	/* Trigger an RX interrupt to guarantee mbuf refresh */
	E1000_WRITE_REG(&adapter->hw, E1000_ICS, trigger);
}

static void
em_if_watchdog_reset(iflib_ctx_t ctx)
{
	struct adapter	*adapter = iflib_softc_get(ctx);
	struct tx_ring	*txr = adapter->tx_rings;

	iflib_printf(ctx, "Watchdog timeout -- resetting\n");
	iflib_printf(ctx,
	    "Queue(%d) tdh = %d, hw tdt = %d\n", txr->me,
	    E1000_READ_REG(&adapter->hw, E1000_TDH(txr->me)),
	    E1000_READ_REG(&adapter->hw, E1000_TDT(txr->me)));
	iflib_printf(ctx,
				  "TX(%d) desc avail = %d,"
	    "Next TX to Clean = %d\n",
	    txr->me, txr->tx_avail, txr->next_to_clean);
}

static void
em_if_update_link_status(iflib_ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	struct e1000_hw *hw = &adapter->hw;
	device_t dev = adapter->dev;
	struct tx_ring *txr = adapter->tx_rings;
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
		/* Check if we must disable SPEED_MODE bit on PCI-E */
		if ((adapter->link_speed != SPEED_1000) &&
		    ((hw->mac.type == e1000_82571) ||
		    (hw->mac.type == e1000_82572))) {
			int tarc0;
			tarc0 = E1000_READ_REG(hw, E1000_TARC(0));
			tarc0 &= ~SPEED_MODE_BIT;
			E1000_WRITE_REG(hw, E1000_TARC(0), tarc0);
		}
		if (bootverbose)
			iflib_printf(ctx, "Link is up %d Mbps %s\n",
			    adapter->link_speed,
			    ((adapter->link_duplex == FULL_DUPLEX) ?
			    "Full Duplex" : "Half Duplex"));
		adapter->link_active = 1;
		adapter->smartspeed = 0;
		iflib_linkstate_change(ctx, adapter->link_speed * 1000000,
							   LINK_STATE_UP);
	} else if (!link_check && (adapter->link_active == 1)) {
		adapter->link_speed = 0;
		adapter->link_duplex = 0;
		if (bootverbose)
			iflib_printf(ctx, "Link is Down\n");
		adapter->link_active = 0;
		iflib_linkstate_change(ctx, 0, LINK_STATE_DOWN);
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
em_if_stop(iflib_ctx_t ctx)
{
	struct adapter	*adapter = iflib_softc_get(ctx);;

	INIT_DEBUGOUT("em_stop: begin");

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
em_identify_hardware(struct adapter *adapter)
{
	device_t dev = iflib_device_get(adapter->ctx);

	/* Make sure our PCI config space has the necessary stuff set */
	pci_enable_busmaster(dev);
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
em_allocate_pci_resources(struct adapter *adapter)
{
	device_t	dev = iflib_device_get(adapter->ctx);
	int		rid;

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

	/* Default to a single queue */
	adapter->num_queues = 1;

	/*
	 * Setup MSI/X or MSI if PCI Express
	 */
	adapter->msix = em_setup_msix(adapter);

	adapter->hw.back = &adapter->osdep;

	return (0);
}

/*********************************************************************
 *
 *  Setup the Legacy or MSI Interrupt handler
 *
 **********************************************************************/
int
em_allocate_legacy(struct adapter *adapter)
{
	int rid = 0;

	/* Manually turn off all interrupts */
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, 0xffffffff);

	if (adapter->msix == 1) /* using MSI */
		rid = 1;

	/*
	 * Allocate a fast interrupt and the associated
	 * deferred processing contexts.
	 */
	return (iflib_legacy_setup(ctx, em_irq_fast, &rid));
}

/*********************************************************************
 *
 *  Setup the MSIX Interrupt handlers
 *   This is not really Multiqueue, rather
 *   its just seperate interrupt vectors
 *   for TX, RX, and Link.
 *
 **********************************************************************/
int
em_allocate_msix(struct adapter *adapter)
{
	struct		tx_ring *txr = adapter->tx_rings;
	struct		rx_ring *rxr = adapter->rx_rings;
	int		error, rid, vector = 0;
	char buf[16];

	/* Make sure all interrupts are disabled */
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, 0xffffffff);

	/* First set up ring resources */
	for (int i = 0; i < adapter->num_queues; i++, txr++, rxr++) {

		/* RX ring */
		rid = vector + 1;
		snprintf(buf, sizeof(buf), "rx %d", i);
		error = iflib_irq_alloc(adapter->ctx, &rxr->irq, rid,
								IFLIB_INTR_RX, rxr->hi_rxr, buf);

		rxr->msix = vector++; /* NOTE increment vector for TX */
		/*
		** Set the bit to enable interrupt
		** in E1000_IMS -- bits 20 and 21
		** are for RX0 and RX1, note this has
		** NOTHING to do with the MSIX vector
		*/
		rxr->ims = 1 << (20 + i);
		adapter->ivars |= (8 | rxr->msix) << (i * 4);

		/* TX ring */
		rid = vector + 1;
		snprintf(buf, sizeof(buf), "tx %d", i);
		error = iflib_irq_alloc(adapter->ctx, &txr->irq, rid,
								IFLIB_INTR_TX, txr->hi_txr, buf);

		txr->msix = vector++; /* Increment vector for next pass */
		/*
		** Set the bit to enable interrupt
		** in E1000_IMS -- bits 22 and 23
		** are for TX0 and TX1, note this has
		** NOTHING to do with the MSIX vector
		*/
		txr->ims = 1 << (22 + i);
		adapter->ivars |= (8 | txr->msix) << (8 + (i * 4));
	}

	/* Link interrupt */
	++rid;
	error = iflib_irq_alloc(adapter->ctx, &rxr->irq, rid,
							IFLIB_INTR_LINK, adapter->ctx, buf);
	adapter->linkvec = vector;
	adapter->ivars |=  (8 | vector) << 16;
	adapter->ivars |= 0x80000000;

	return (0);
}


static void
em_free_pci_resources(struct adapter *adapter)
{
	device_t	dev = adapter->dev;
	struct tx_ring	*txr;
	struct rx_ring	*rxr;
	int		rid;


	/*
	** Release all the queue interrupt resources:
	*/
	for (int i = 0; i < adapter->num_queues; i++) {
		txr = &adapter->tx_rings[i];
		rxr = &adapter->rx_rings[i];
		/* an early abort? */
		if ((txr == NULL) || (rxr == NULL))
			break;
		rid = txr->msix +1;
		if (txr->tag != NULL) {
			bus_teardown_intr(dev, txr->res, txr->tag);
			txr->tag = NULL;
		}
		if (txr->res != NULL)
			bus_release_resource(dev, SYS_RES_IRQ,
			    rid, txr->res);
		rid = rxr->msix +1;
		if (rxr->tag != NULL) {
			bus_teardown_intr(dev, rxr->res, rxr->tag);
			rxr->tag = NULL;
		}
		if (rxr->res != NULL)
			bus_release_resource(dev, SYS_RES_IRQ,
			    rid, rxr->res);
	}

        if (adapter->linkvec) /* we are doing MSIX */
                rid = adapter->linkvec + 1;
        else
                (adapter->msix != 0) ? (rid = 1):(rid = 0);

	if (adapter->tag != NULL) {
		bus_teardown_intr(dev, adapter->res, adapter->tag);
		adapter->tag = NULL;
	}

	if (adapter->res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, rid, adapter->res);


	if (adapter->msix)
		pci_release_msi(dev);

	if (adapter->msix_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(EM_MSIX_BAR), adapter->msix_mem);

	if (adapter->memory != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(0), adapter->memory);

	if (adapter->flash != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    EM_FLASH, adapter->flash);
}

/*
 * Setup MSI or MSI/X
 */
static int
em_setup_msix(struct adapter *adapter)
{
	device_t dev = adapter->dev;
	int val;

	/*
	** Setup MSI/X for Hartwell: tests have shown
	** use of two queues to be unstable, and to
	** provide no great gain anyway, so we simply
	** seperate the interrupts and use a single queue.
	*/
	if ((adapter->hw.mac.type == e1000_82574) &&
	    (em_enable_msix == TRUE)) {
		/* Map the MSIX BAR */
		int rid = PCIR_BAR(EM_MSIX_BAR);
		adapter->msix_mem = bus_alloc_resource_any(dev,
		    SYS_RES_MEMORY, &rid, RF_ACTIVE);
       		if (adapter->msix_mem == NULL) {
			/* May not be enabled */
               		device_printf(adapter->dev,
			    "Unable to map MSIX table \n");
			goto msi;
       		}
		val = pci_msix_count(dev); 
		/* We only need/want 3 vectors */
		if (val >= 3)
			val = 3;
		else {
               		device_printf(adapter->dev,
			    "MSIX: insufficient vectors, using MSI\n");
			goto msi;
		}

		if ((pci_alloc_msix(dev, &val) == 0) && (val == 3)) {
			device_printf(adapter->dev,
			    "Using MSIX interrupts "
			    "with %d vectors\n", val);
			return (val);
		}

		/*
		** If MSIX alloc failed or provided us with
		** less than needed, free and fall through to MSI
		*/
		pci_release_msi(dev);
	}
msi:
	if (adapter->msix_mem != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(EM_MSIX_BAR), adapter->msix_mem);
		adapter->msix_mem = NULL;
	}
       	val = 1;
       	if (pci_alloc_msi(dev, &val) == 0) {
               	device_printf(adapter->dev,"Using an MSI interrupt\n");
		return (val);
	} 
	/* Should only happen due to manual configuration */
	device_printf(adapter->dev,"No MSI/MSIX using a Legacy IRQ\n");
	return (0);
}


/*********************************************************************
 *
 *  Initialize the hardware to a configuration
 *  as specified by the adapter structure.
 *
 **********************************************************************/
static void
em_reset(struct adapter *adapter)
{
	struct e1000_hw	*hw = &adapter->hw;
	u16		rx_buffer_size;
	u32		pba;

	INIT_DEBUGOUT("em_reset: begin");

	/* Set up smart power down as default off on newer adapters. */
	if (!em_smart_pwr_down && (hw->mac.type == e1000_82571 ||
	    hw->mac.type == e1000_82572)) {
		u16 phy_tmp = 0;

		/* Speed up time to link by disabling smart power down. */
		e1000_read_phy_reg(hw, IGP02E1000_PHY_POWER_MGMT, &phy_tmp);
		phy_tmp &= ~IGP02E1000_PM_SPD;
		e1000_write_phy_reg(hw, IGP02E1000_PHY_POWER_MGMT, phy_tmp);
	}

	/*
	 * Packet Buffer Allocation (PBA)
	 * Writing PBA sets the receive portion of the buffer
	 * the remainder is used for the transmit buffer.
	 */
	switch (hw->mac.type) {
	/* Total Packet Buffer on these is 48K */
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
			pba = E1000_PBA_32K; /* 32K for Rx, 16K for Tx */
		break;
	case e1000_82573: /* 82573: Total Packet Buffer is 32K */
			pba = E1000_PBA_12K; /* 12K for Rx, 20K for Tx */
		break;
	case e1000_82574:
	case e1000_82583:
			pba = E1000_PBA_20K; /* 20K for Rx, 20K for Tx */
		break;
	case e1000_ich8lan:
		pba = E1000_PBA_8K;
		break;
	case e1000_ich9lan:
	case e1000_ich10lan:
		/* Boost Receive side for jumbo frames */
		if (adapter->hw.mac.max_frame_size > 4096)
			pba = E1000_PBA_14K;
		else
			pba = E1000_PBA_10K;
		break;
	case e1000_pchlan:
	case e1000_pch2lan:
	case e1000_pch_lpt:
		pba = E1000_PBA_26K;
		break;
	default:
		if (adapter->hw.mac.max_frame_size > 8192)
			pba = E1000_PBA_40K; /* 40K for Rx, 24K for Tx */
		else
			pba = E1000_PBA_48K; /* 48K for Rx, 16K for Tx */
	}
	E1000_WRITE_REG(&adapter->hw, E1000_PBA, pba);

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
	rx_buffer_size = ((E1000_READ_REG(hw, E1000_PBA) & 0xffff) << 10 );
	hw->fc.high_water = rx_buffer_size -
	    roundup2(adapter->hw.mac.max_frame_size, 1024);
	hw->fc.low_water = hw->fc.high_water - 1500;

	if (adapter->fc) /* locally set flow control value? */
		hw->fc.requested_mode = adapter->fc;
	else
		hw->fc.requested_mode = e1000_fc_full;

	if (hw->mac.type == e1000_80003es2lan)
		hw->fc.pause_time = 0xFFFF;
	else
		hw->fc.pause_time = EM_FC_PAUSE_TIME;

	hw->fc.send_xon = TRUE;

	/* Device specific overrides/settings */
	switch (hw->mac.type) {
	case e1000_pchlan:
		/* Workaround: no TX flow ctrl for PCH */
                hw->fc.requested_mode = e1000_fc_rx_pause;
		hw->fc.pause_time = 0xFFFF; /* override */
		if (iflib_mtu_get(ctx) > ETHERMTU) {
			hw->fc.high_water = 0x3500;
			hw->fc.low_water = 0x1500;
		} else {
			hw->fc.high_water = 0x5000;
			hw->fc.low_water = 0x3000;
		}
		hw->fc.refresh_time = 0x1000;
		break;
	case e1000_pch2lan:
	case e1000_pch_lpt:
		hw->fc.high_water = 0x5C20;
		hw->fc.low_water = 0x5048;
		hw->fc.pause_time = 0x0650;
		hw->fc.refresh_time = 0x0400;
		/* Jumbos need adjusted PBA */
		if (iflib_mtu_get(ctx) > ETHERMTU)
			E1000_WRITE_REG(hw, E1000_PBA, 12);
		else
			E1000_WRITE_REG(hw, E1000_PBA, 26);
		break;
        case e1000_ich9lan:
        case e1000_ich10lan:
		if (iflib_mtu_get(ctx) > ETHERMTU) {
			hw->fc.high_water = 0x2800;
			hw->fc.low_water = hw->fc.high_water - 8;
			break;
		} 
		/* else fall thru */
	default:
		if (hw->mac.type == e1000_80003es2lan)
			hw->fc.pause_time = 0xFFFF;
		break;
	}

	/* Issue a global reset */
	e1000_reset_hw(hw);
	E1000_WRITE_REG(hw, E1000_WUC, 0);
	em_disable_aspm(adapter);
	/* and a re-init */
	if (e1000_init_hw(hw) < 0) {
		iflib_printf(ctx, "Hardware Initialization Failed\n");
		return;
	}

	E1000_WRITE_REG(hw, E1000_VET, ETHERTYPE_VLAN);
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
em_setup_interface(device_t dev, struct adapter *adapter)
{
	iflib_ctx_t ctx;
	
	INIT_DEBUGOUT("em_setup_interface: begin");

	ctx = iflib_drv_init(device_get_name(dev), device_get_unit(dev), adapter);
	/*
	 *..., maxsize, nsegments, maxsegsize
	 */
	iflib_tx_tag_prop_set(ctx, EM_TSO_SIZE, EM_MAX_SCATTER, PAGE_SIZE);
	/*
	 *..., maxsize, nsegments, maxsegsize
	 */
	iflib_rx_tag_prop_set(ctx, MJUM9BYTES, 1, MJUM9BYTES);
	/*
	 *..., alignment
	 */
	iflib_queue_tag_prop_set(ctx, EM_DBA_ALIGN);

	
	iflib_settx_fn(ctx, ...);
	iflib_setcapabilitiesbit(ctx, IFCAP_HWCSUM | IFCAP_VLAN_HWCSUM |
	    IFCAP_TSO4);
	
	/*
	 * Tell the upper layer(s) we
	 * support full VLAN capability
	 *  XXX is this a more generic operation common to all VLAN
	 * setting?
	 */
	iflib_setifheaderlen(ctx, sizeof(struct ether_vlan_header));
	if_setcapabilitiesbit(ctx, IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_HWTSO |
						  IFCAP_VLAN_MTU);

	/*
	** Don't turn this on by default, if vlans are
	** created on another pseudo device (eg. lagg)
	** then vlan events are not passed thru, breaking
	** operation, but with HW FILTER off it works. If
	** using vlans directly on the em driver you can
	** enable this and get full hardware tag filtering.
	*/
	if_setcapabilitiesbit(ctx, IFCAP_VLAN_HWFILTER);

#ifdef DEVICE_POLLING
	iflib_setcapabilitiesbit(ctx, IFCAP_POLLING);
#endif

	/* Enable only WOL MAGIC by default */
	if (adapter->wol)
		iflib_setcapabilitiesbit(ctx, IFCAP_WOL);

 	iflib_setcapenable(ctx);
	
	/*
	 * Specify the media types supported by this adapter and register
	 * callbacks to update media and link information
	 */
	ifmedia_init_drv(&adapter->media, IFM_IMASK,
	    em_media_change, em_media_status);
	if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
	    (adapter->hw.phy.media_type == e1000_media_type_internal_serdes)) {
		u_char fiber_type = IFM_1000_SX;	/* default type */

		ifmedia_add(&adapter->media, IFM_ETHER | fiber_type | IFM_FDX, 
			    0, NULL);
		ifmedia_add(&adapter->media, IFM_ETHER | fiber_type, 0, NULL);
	} else {
		ifmedia_add(&adapter->media, IFM_ETHER | IFM_10_T, 0, NULL);
		ifmedia_add(&adapter->media, IFM_ETHER | IFM_10_T | IFM_FDX,
			    0, NULL);
		ifmedia_add(&adapter->media, IFM_ETHER | IFM_100_TX,
			    0, NULL);
		ifmedia_add(&adapter->media, IFM_ETHER | IFM_100_TX | IFM_FDX,
			    0, NULL);
		if (adapter->hw.phy.type != e1000_phy_ife) {
			ifmedia_add(&adapter->media,
				IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
			ifmedia_add(&adapter->media,
				IFM_ETHER | IFM_1000_T, 0, NULL);
		}
	}
	ifmedia_add(&adapter->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&adapter->media, IFM_ETHER | IFM_AUTO);
	return (0);
}


/*
 * Manage DMA'able memory.
 */
static void
em_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error)
		return;
	*(bus_addr_t *) arg = segs[0].ds_addr;
}

/*********************************************************************
 *
 *  Allocate memory for the transmit and receive rings, and then
 *  the descriptors associated with each, called only once at attach.
 *
 **********************************************************************/
static int
em_if_queues_alloc(iflib_ctx_t ctx)
{
	struct adapter      *adapter = iflib_softc_get(ctx);
	struct tx_ring		*txr = NULL;
	struct rx_ring		*rxr = NULL;
	int error = E1000_SUCCESS;

	/* Allocate the TX ring struct memory */
	if (!(adapter->tx_rings =
	    (struct tx_ring *) malloc(sizeof(struct tx_ring) *
	    adapter->num_queues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate TX ring memory\n");
		error = ENOMEM;
		goto fail;
	}
	
	/* Now allocate the RX */
	if (!(adapter->rx_rings =
	    (struct rx_ring *) malloc(sizeof(struct rx_ring) *
	    adapter->num_queues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate RX ring memory\n");
		error = ENOMEM;
		goto rx_fail;
	}
	for (int i = 0, txr = adapter->tx_rings, rxr = adapter->rx_rings;
		 i < adapter->num_queues; i++, rxr++, txr++) {
		rxr->adapter = txr->adapter = adapter;
		rxr->me = txr->me = i;

		/* set the hardware independent queues hardware queue ptr */
		iflib_tx_softc_set(ctx, txr, i);
		iflib_rx_softc_set(ctx, rxr, i);

		/* get the virtual address of the hardware queues */
		txr->tx_base = (struct e1000_tx_desc *)iflib_txq_vaddr_get(ctx, i);
		txr->rx_base = (struct e1000_rx_desc *)iflib_rxq_vaddr_get(ctx, i);
	}

	return (0);

rx_fail:
	free(adapter->tx_rings, M_DEVBUF);
fail:
	return (error);
}


/*********************************************************************
 *
 *  Initialize a transmit ring.
 *
 **********************************************************************/
static void
em_if_transmit_ring_setup(void *arg)
{
	struct tx_ring *txr = arg;
	struct adapter *adapter = txr->adapter;

	/* Clear the old descriptor contents */
	bzero((void *)txr->tx_base,
	      (sizeof(struct e1000_tx_desc)) * adapter->num_tx_desc);

	/* Clear checksum offload context. */
	txr->last_hw_offload = 0;
	txr->last_hw_ipcss = 0;
	txr->last_hw_ipcso = 0;
	txr->last_hw_tucss = 0;
	txr->last_hw_tucso = 0;
}

/*********************************************************************
 *
 *  Enable transmit unit.
 *
 **********************************************************************/
static void
em_initialize_transmit_unit(struct adapter *adapter)
{
	struct tx_ring	*txr = adapter->tx_rings;
	struct e1000_hw	*hw = &adapter->hw;
	u32	tctl, tarc, tipg = 0;

	 INIT_DEBUGOUT("em_initialize_transmit_unit: begin");

	for (int i = 0; i < adapter->num_queues; i++, txr++) {
		u64 bus_addr = txr->txdma.dma_paddr;
		/* Base and Len of TX Ring */
		E1000_WRITE_REG(hw, E1000_TDLEN(i),
	    	    adapter->num_tx_desc * sizeof(struct e1000_tx_desc));
		E1000_WRITE_REG(hw, E1000_TDBAH(i),
	    	    (u32)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_TDBAL(i),
	    	    (u32)bus_addr);
		/* Init the HEAD/TAIL indices */
		E1000_WRITE_REG(hw, E1000_TDT(i), 0);
		E1000_WRITE_REG(hw, E1000_TDH(i), 0);

		HW_DEBUGOUT2("Base = %x, Length = %x\n",
		    E1000_READ_REG(&adapter->hw, E1000_TDBAL(i)),
		    E1000_READ_REG(&adapter->hw, E1000_TDLEN(i)));
	}

	/* Set the default values for the Tx Inter Packet Gap timer */
	switch (adapter->hw.mac.type) {
	case e1000_80003es2lan:
		tipg = DEFAULT_82543_TIPG_IPGR1;
		tipg |= DEFAULT_80003ES2LAN_TIPG_IPGR2 <<
		    E1000_TIPG_IPGR2_SHIFT;
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
	E1000_WRITE_REG(&adapter->hw, E1000_TIDV, adapter->tx_int_delay.value);

	if(adapter->hw.mac.type >= e1000_82540)
		E1000_WRITE_REG(&adapter->hw, E1000_TADV,
		    adapter->tx_abs_int_delay.value);

	if ((adapter->hw.mac.type == e1000_82571) ||
	    (adapter->hw.mac.type == e1000_82572)) {
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(0));
		tarc |= SPEED_MODE_BIT;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc);
	} else if (adapter->hw.mac.type == e1000_80003es2lan) {
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(0));
		tarc |= 1;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(0), tarc);
		tarc = E1000_READ_REG(&adapter->hw, E1000_TARC(1));
		tarc |= 1;
		E1000_WRITE_REG(&adapter->hw, E1000_TARC(1), tarc);
	}

	adapter->txd_cmd = E1000_TXD_CMD_IFCS;
	if (adapter->tx_int_delay.value > 0)
		adapter->txd_cmd |= E1000_TXD_CMD_IDE;

	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		   (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));

	if (adapter->hw.mac.type >= e1000_82571)
		tctl |= E1000_TCTL_MULR;

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);

}

/*********************************************************************
 *  The offload context is protocol specific (TCP/UDP) and thus
 *  only needs to be set when the protocol changes. The occasion
 *  of a context change can be a performance detriment, and
 *  might be better just disabled. The reason arises in the way
 *  in which the controller supports pipelined requests from the
 *  Tx data DMA. Up to four requests can be pipelined, and they may
 *  belong to the same packet or to multiple packets. However all
 *  requests for one packet are issued before a request is issued
 *  for a subsequent packet and if a request for the next packet
 *  requires a context change, that request will be stalled
 *  until the previous request completes. This means setting up
 *  a new context effectively disables pipelined Tx data DMA which
 *  in turn greatly slow down performance to send small sized
 *  frames. 
 **********************************************************************/
static void
em_transmit_checksum_setup(struct tx_ring *txr, int csum_flags,	int ip_off,
    struct ip *ip, u32 *txd_upper, u32 *txd_lower)
{
	struct adapter			*adapter = txr->adapter;
	struct e1000_context_desc	*TXD = NULL;
	struct em_buffer		*tx_buffer;
	int				cur, hdr_len;
	u32				cmd = 0;
	u16				offload = 0;
	u8				ipcso, ipcss, tucso, tucss;

	ipcss = ipcso = tucss = tucso = 0;
	hdr_len = ip_off + (ip->ip_hl << 2);
	cur = txr->next_avail_desc;

	/* Setup of IP header checksum. */
	if (csum_flags & CSUM_IP) {
		*txd_upper |= E1000_TXD_POPTS_IXSM << 8;
		offload |= CSUM_IP;
		ipcss = ip_off;
		ipcso = ip_off + offsetof(struct ip, ip_sum);
		/*
		 * Start offset for header checksum calculation.
		 * End offset for header checksum calculation.
		 * Offset of place to put the checksum.
		 */
		TXD = (struct e1000_context_desc *)&txr->tx_base[cur];
		TXD->lower_setup.ip_fields.ipcss = ipcss;
		TXD->lower_setup.ip_fields.ipcse = htole16(hdr_len);
		TXD->lower_setup.ip_fields.ipcso = ipcso;
		cmd |= E1000_TXD_CMD_IP;
	}

	if (csum_flags & CSUM_TCP) {
 		*txd_lower = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
 		*txd_upper |= E1000_TXD_POPTS_TXSM << 8;
 		offload |= CSUM_TCP;
 		tucss = hdr_len;
 		tucso = hdr_len + offsetof(struct tcphdr, th_sum);
 		/*
 		 * Setting up new checksum offload context for every frames
 		 * takes a lot of processing time for hardware. This also
 		 * reduces performance a lot for small sized frames so avoid
 		 * it if driver can use previously configured checksum
 		 * offload context.
 		 */
 		if (txr->last_hw_offload == offload) {
 			if (offload & CSUM_IP) {
 				if (txr->last_hw_ipcss == ipcss &&
 				    txr->last_hw_ipcso == ipcso &&
 				    txr->last_hw_tucss == tucss &&
 				    txr->last_hw_tucso == tucso)
 					return;
 			} else {
 				if (txr->last_hw_tucss == tucss &&
 				    txr->last_hw_tucso == tucso)
 					return;
 			}
  		}
 		txr->last_hw_offload = offload;
 		txr->last_hw_tucss = tucss;
 		txr->last_hw_tucso = tucso;
 		/*
 		 * Start offset for payload checksum calculation.
 		 * End offset for payload checksum calculation.
 		 * Offset of place to put the checksum.
 		 */
		TXD = (struct e1000_context_desc *)&txr->tx_base[cur];
 		TXD->upper_setup.tcp_fields.tucss = hdr_len;
 		TXD->upper_setup.tcp_fields.tucse = htole16(0);
 		TXD->upper_setup.tcp_fields.tucso = tucso;
 		cmd |= E1000_TXD_CMD_TCP;
 	} else if (csum_flags & CSUM_UDP) {
 		*txd_lower = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
 		*txd_upper |= E1000_TXD_POPTS_TXSM << 8;
 		tucss = hdr_len;
 		tucso = hdr_len + offsetof(struct udphdr, uh_sum);
 		/*
 		 * Setting up new checksum offload context for every frames
 		 * takes a lot of processing time for hardware. This also
 		 * reduces performance a lot for small sized frames so avoid
 		 * it if driver can use previously configured checksum
 		 * offload context.
 		 */
 		if (txr->last_hw_offload == offload) {
 			if (offload & CSUM_IP) {
 				if (txr->last_hw_ipcss == ipcss &&
 				    txr->last_hw_ipcso == ipcso &&
 				    txr->last_hw_tucss == tucss &&
 				    txr->last_hw_tucso == tucso)
 					return;
 			} else {
 				if (txr->last_hw_tucss == tucss &&
 				    txr->last_hw_tucso == tucso)
 					return;
 			}
 		}
 		txr->last_hw_offload = offload;
 		txr->last_hw_tucss = tucss;
 		txr->last_hw_tucso = tucso;
 		/*
 		 * Start offset for header checksum calculation.
 		 * End offset for header checksum calculation.
 		 * Offset of place to put the checksum.
 		 */
		TXD = (struct e1000_context_desc *)&txr->tx_base[cur];
 		TXD->upper_setup.tcp_fields.tucss = tucss;
 		TXD->upper_setup.tcp_fields.tucse = htole16(0);
 		TXD->upper_setup.tcp_fields.tucso = tucso;
  	}
  
 	if (offload & CSUM_IP) {
 		txr->last_hw_ipcss = ipcss;
 		txr->last_hw_ipcso = ipcso;
  	}

	TXD->tcp_seg_setup.data = htole32(0);
	TXD->cmd_and_length =
	    htole32(adapter->txd_cmd | E1000_TXD_CMD_DEXT | cmd);
	tx_buffer = &txr->tx_buffers[cur];
	tx_buffer->m_head = NULL;
	tx_buffer->next_eop = -1;

	if (++cur == adapter->num_tx_desc)
		cur = 0;

	txr->tx_avail--;
	txr->next_avail_desc = cur;
}


/**********************************************************************
 *
 *  Setup work for hardware segmentation offload (TSO)
 *
 **********************************************************************/
static void
em_tso_setup(struct tx_ring *txr, pkt_info_t pi, u32 *txd_upper, u32 *txd_lower)
{
	struct adapter			*adapter = txr->adapter;
	struct e1000_context_desc	*TXD;
	struct em_buffer		*tx_buffer;
	int ip_off = pi->ip_off;
	struct ip *ip = pi->ip;
	struct tcphdr *tp = pi->tp;
	int cur, hdr_len;

	/*
	 * In theory we can use the same TSO context if and only if
	 * frame is the same type(IP/TCP) and the same MSS. However
	 * checking whether a frame has the same IP/TCP structure is
	 * hard thing so just ignore that and always restablish a
	 * new TSO context.
	 */
	hdr_len = ip_off + (ip->ip_hl << 2) + (tp->th_off << 2);
	*txd_lower = (E1000_TXD_CMD_DEXT |	/* Extended descr type */
		      E1000_TXD_DTYP_D |	/* Data descr type */
		      E1000_TXD_CMD_TSE);	/* Do TSE on this packet */

	/* IP and/or TCP header checksum calculation and insertion. */
	*txd_upper = (E1000_TXD_POPTS_IXSM | E1000_TXD_POPTS_TXSM) << 8;

	cur = txr->next_avail_desc;
	tx_buffer = &txr->tx_buffers[cur];
	TXD = (struct e1000_context_desc *) &txr->tx_base[cur];

	/*
	 * Start offset for header checksum calculation.
	 * End offset for header checksum calculation.
	 * Offset of place put the checksum.
	 */
	TXD->lower_setup.ip_fields.ipcss = ip_off;
	TXD->lower_setup.ip_fields.ipcse =
	    htole16(ip_off + (ip->ip_hl << 2) - 1);
	TXD->lower_setup.ip_fields.ipcso = ip_off + offsetof(struct ip, ip_sum);
	/*
	 * Start offset for payload checksum calculation.
	 * End offset for payload checksum calculation.
	 * Offset of place to put the checksum.
	 */
	TXD->upper_setup.tcp_fields.tucss = ip_off + (ip->ip_hl << 2);
	TXD->upper_setup.tcp_fields.tucse = 0;
	TXD->upper_setup.tcp_fields.tucso =
	    ip_off + (ip->ip_hl << 2) + offsetof(struct tcphdr, th_sum);
	/*
	 * Payload size per packet w/o any headers.
	 * Length of all headers up to payload.
	 */
	TXD->tcp_seg_setup.fields.mss = htole16(pi->tso_segsz);
	TXD->tcp_seg_setup.fields.hdr_len = hdr_len;

	TXD->cmd_and_length = htole32(adapter->txd_cmd |
				E1000_TXD_CMD_DEXT |	/* Extended descr */
				E1000_TXD_CMD_TSE |	/* TSE context */
				E1000_TXD_CMD_IP |	/* Do IP csum */
				E1000_TXD_CMD_TCP |	/* Do TCP checksum */
				(pi->pktlen - (hdr_len))); /* Total len */

	tx_buffer->m_head = NULL;
	tx_buffer->next_eop = -1;

	if (++cur == adapter->num_tx_desc)
		cur = 0;

	txr->tx_avail--;
	txr->next_avail_desc = cur;
	txr->tx_tso = TRUE;
}


/**********************************************************************
 *
 *  Examine each tx_buffer in the used queue. If the hardware is done
 *  processing the packet then free associated resources. The
 *  tx_buffer is put back on the free queue.
 *
 **********************************************************************/
static void
em_if_txeof(struct tx_ring *txr)
{
	struct adapter	*adapter = txr->adapter;
	int first, last, done, processed;
	struct e1000_tx_desc   *tx_desc, *eop_desc;
	
	/* No work, make sure watchdog is off */
	if (txr->tx_avail == adapter->num_tx_desc) {
		txr->queue_status = EM_QUEUE_IDLE;
                return;
	}

	processed = 0;
	first = txr->next_to_clean;
	tx_buffer = &txr->tx_buffers[first];
	last = tx_buffer->next_eop;
	
	tx_desc = &txr->tx_base[first];
	eop_desc = &txr->tx_base[last];

	/*
	 * What this does is get the index of the
	 * first descriptor AFTER the EOP of the 
	 * first packet, that way we can do the
	 * simple comparison on the inner while loop.
	 */
	if (++last == adapter->num_tx_desc)
 		last = 0;
	done = last;
	
	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
					BUS_DMASYNC_POSTREAD);
	
	while (eop_desc->upper.fields.status & E1000_TXD_STAT_DD) {
		/* We clean the range of the packet */
		while (first != done) {
			tx_desc->upper.data = 0;
			tx_desc->lower.data = 0;
			tx_desc->buffer_addr = 0;
			++txr->tx_avail;
			++processed;
			
			if (tx_buffer->m_head)
				iflib_tx_free(ctx, txr, &tx_buffer);
			tx_buffer->next_eop = -1;
			txr->watchdog_time = ticks;
			
			if (++first == adapter->num_tx_desc)
				first = 0;
			
			tx_buffer = &txr->tx_buffers[first];
			tx_desc = &txr->tx_base[first];
		}
		iflib_incopackets(ctx, 1);
		/* See if we can continue to the next packet */
		last = tx_buffer->next_eop;
		if (last != -1) {
			eop_desc = &txr->tx_base[last];
			/* Get new done point */
			if (++last == adapter->num_tx_desc) last = 0;
			done = last;
		} else
			break;
	}
	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
					BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	
	txr->next_to_clean = first;	
	/*
	** Watchdog calculation, we know there's
	** work outstanding or the first return
	** would have been taken, so none processed
	** for too long indicates a hang. local timer
	** will examine this and do a reset if needed.
	*/
	if ((!processed) && ((ticks - txr->watchdog_time) > EM_WATCHDOG))
		txr->queue_status = EM_QUEUE_HUNG;

	/* Disable watchdog if all clean */
	if (txr->tx_avail == adapter->num_tx_desc) {
		txr->queue_status = EM_QUEUE_IDLE;
	} 
}


/*********************************************************************
 *
 *  Refresh RX descriptor mbufs from system mbuf buffer pool.
 *
 **********************************************************************/
static void
em_refresh_mbufs(struct rx_ring *rxr, int limit)
{
	struct adapter		*adapter = rxr->adapter;
	struct mbuf		*m;
	bus_dma_segment_t	segs[1];
	struct em_buffer	*rxbuf;
	int			i, j, error, nsegs;
	bool			cleaned = FALSE;

	i = j = rxr->next_to_refresh;
	/*
	** Get one descriptor beyond
	** our work mark to control
	** the loop.
	*/
	if (++j == adapter->num_rx_desc)
		j = 0;

	while (j != limit) {
		rxbuf = &rxr->rx_buffers[i];
		if (rxbuf->m_head == NULL) {
			m = m_getjcl(M_NOWAIT, MT_DATA,
			    M_PKTHDR, adapter->rx_mbuf_sz);
			/*
			** If we have a temporary resource shortage
			** that causes a failure, just abort refresh
			** for now, we will return to this point when
			** reinvoked from em_rxeof.
			*/
			if (m == NULL)
				goto update;
		} else
			m = rxbuf->m_head;

		m->m_len = m->m_pkthdr.len = adapter->rx_mbuf_sz;
		m->m_flags |= M_PKTHDR;
		m->m_data = m->m_ext.ext_buf;

		/* Use bus_dma machinery to setup the memory mapping  */
		error = bus_dmamap_load_virtual_buffer(rxr->rxtag, rxbuf->map,
					m->m_data, adapter->rx_mbuf_sz, segs, &nsegs,
					BUS_DMA_NOWAIT|BUS_DMA_LOAD_MBUF);
		if (error != 0) {
			printf("Refresh mbufs: hdr dmamap load"
			    " failure - %d\n", error);
			m_free(m);
			rxbuf->m_head = NULL;
			goto update;
		}
		rxbuf->m_head = m;
		bus_dmamap_sync(rxr->rxtag,
		    rxbuf->map, BUS_DMASYNC_PREREAD);
		cleaned = TRUE;

		i = j; /* Next is precalulated for us */
		rxr->next_to_refresh = i;
		/* Calculate next controlling index */
		if (++j == adapter->num_rx_desc)
			j = 0;
	}
update:
	/*
	** Update the tail pointer only if,
	** and as far as we have refreshed.
	*/
	if (cleaned)


	return;
}

/*********************************************************************
 *
 *  Enable receive unit.
 *
 **********************************************************************/

static void
em_initialize_receive_unit(struct adapter *adapter)
{
	struct rx_ring	*rxr = adapter->rx_rings;
	struct e1000_hw	*hw = &adapter->hw;
	u64	bus_addr;
	u32	rctl, rxcsum;

	INIT_DEBUGOUT("em_initialize_receive_units: begin");

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring
	 */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	/* Do not disable if ever enabled on this hardware */
	if ((hw->mac.type != e1000_82574) && (hw->mac.type != e1000_82583))
		E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);

	E1000_WRITE_REG(&adapter->hw, E1000_RADV,
	    adapter->rx_abs_int_delay.value);
	/*
	 * Set the interrupt throttling rate. Value is calculated
	 * as DEFAULT_ITR = 1/(MAX_INTS_PER_SEC * 256ns)
	 */
	E1000_WRITE_REG(hw, E1000_ITR, DEFAULT_ITR);

	/*
	** When using MSIX interrupts we need to throttle
	** using the EITR register (82574 only)
	*/
	if (hw->mac.type == e1000_82574) {
		for (int i = 0; i < 4; i++)
			E1000_WRITE_REG(hw, E1000_EITR_82574(i),
			    DEFAULT_ITR);
		/* Disable accelerated acknowledge */
		E1000_WRITE_REG(hw, E1000_RFCTL, E1000_RFCTL_ACK_DIS);
	}

	rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);
	if (iflib_getcapenable(ctx) & IFCAP_RXCSUM)
		rxcsum |= E1000_RXCSUM_TUOFL;
	else
		rxcsum &= ~E1000_RXCSUM_TUOFL;
	E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);

	/*
	** XXX TEMPORARY WORKAROUND: on some systems with 82573
	** long latencies are observed, like Lenovo X60. This
	** change eliminates the problem, but since having positive
	** values in RDTR is a known source of problems on other
	** platforms another solution is being sought.
	*/
	if (hw->mac.type == e1000_82573)
		E1000_WRITE_REG(hw, E1000_RDTR, 0x20);

	for (int i = 0; i < adapter->num_queues; i++, rxr++) {
		/* Setup the Base and Length of the Rx Descriptor Ring */
		u32 rdt = adapter->num_rx_desc - 1; /* default */

		bus_addr = rxr->rxdma.dma_paddr;
		E1000_WRITE_REG(hw, E1000_RDLEN(i),
		    adapter->num_rx_desc * sizeof(struct e1000_rx_desc));
		E1000_WRITE_REG(hw, E1000_RDBAH(i), (u32)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_RDBAL(i), (u32)bus_addr);
		/* Setup the Head and Tail Descriptor Pointers */
		E1000_WRITE_REG(hw, E1000_RDH(i), 0);
#ifdef DEV_NETMAP
		/*
		 * an init() while a netmap client is active must
		 * preserve the rx buffers passed to userspace.
		 */
		if (iflib_getcapenable(ctx) & IFCAP_NETMAP) {
			struct netmap_adapter *na = netmap_getna(adapter->ifp);
			rdt -= nm_kr_rxspace(&na->rx_rings[i]);
		}
#endif /* DEV_NETMAP */
		E1000_WRITE_REG(hw, E1000_RDT(i), rdt);
	}

	/* Set PTHRESH for improved jumbo performance */
	if (((adapter->hw.mac.type == e1000_ich9lan) ||
	    (adapter->hw.mac.type == e1000_pch2lan) ||
	    (adapter->hw.mac.type == e1000_ich10lan)) &&
	    (iflib_mtu_get(ctx) > ETHERMTU)) {
		u32 rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(0));
		E1000_WRITE_REG(hw, E1000_RXDCTL(0), rxdctl | 3);
	}
		
	if (adapter->hw.mac.type >= e1000_pch2lan) {
		if (iflib_mtu_get(ctx) > ETHERMTU)
			e1000_lv_jumbo_workaround_ich8lan(hw, TRUE);
		else
			e1000_lv_jumbo_workaround_ich8lan(hw, FALSE);
	}

	/* Setup the Receive Control Register */
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM |
	    E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF |
	    (hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

        /* Strip the CRC */
        rctl |= E1000_RCTL_SECRC;

        /* Make sure VLAN Filters are off */
        rctl &= ~E1000_RCTL_VFE;
	rctl &= ~E1000_RCTL_SBP;

	if (adapter->rx_mbuf_sz == MCLBYTES)
		rctl |= E1000_RCTL_SZ_2048;
	else if (adapter->rx_mbuf_sz == MJUMPAGESIZE)
		rctl |= E1000_RCTL_SZ_4096 | E1000_RCTL_BSEX;
	else if (adapter->rx_mbuf_sz > MJUMPAGESIZE)
		rctl |= E1000_RCTL_SZ_8192 | E1000_RCTL_BSEX;

	if (iflib_mtu_get(ctx) > ETHERMTU)
		rctl |= E1000_RCTL_LPE;
	else
		rctl &= ~E1000_RCTL_LPE;

	/* Write out the settings */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	return;
}

static void
em_if_refill_rxd(iflib_cxt_t ctx, int rxqid, int pidx, uint64_t paddr)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	struct rx_ring *rxr = adapter->rx_rings[rxqid];

	rxr->rx_base[pidx].buffer_addr = htole64(paddr);
}

static void
em_if_refill_flush(iflib_ctx_t ctx, int rxqid, int pidx)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	E1000_WRITE_REG(&adapter->hw,
		    E1000_RDT(rxqid), pidx);
}

static int
em_if_is_new(void *_rxr, int idx)
{
	struct rx_ring *rxr = _rxr;
	struct e1000_rx_desc	*cur;

	cur = &rxr->rx_base[idx];
	return ((cur->status & E1000_RXD_STAT_DD) != 0);
}
	
/*********************************************************************
 *
 *  This routine executes in interrupt context. It replenishes
 *  the mbufs in the descriptor and sends data which has been
 *  dma'ed into host memory to upper layer.
 *
 *  We loop at most count times if count is > 0, or until done if
 *  count < 0.
 *  
 *  For polling we also now return the number of cleaned packets
 *********************************************************************/
static bool
em_if_packet_get(void *_rxr, int i, rx_info_t ri)
{
	struct rx_ring *rxr = _rxr;
	struct adapter		*adapter = rxr->adapter;
	struct mbuf		*mp, *sendmp;
	u8			status;
	u16 			len;
	int			processed, rxdone = 0;
	bool			eop;
	struct e1000_rx_desc	*cur;
	uint64_t csum_flags, csum_data;
	
	cur = &rxr->rx_base[i];
	status = cur->status;
	mp = sendmp = NULL;

	len = le16toh(cur->length);
	eop = (status & E1000_RXD_STAT_EOP) != 0;

	if ((cur->errors & E1000_RXD_ERR_FRAME_ERR_MASK) ||
		(rxr->discard == TRUE)) {
		adapter->dropped_pkts++;
		++rxr->rx_discarded;
		if (!eop) /* Catch subsequent segs */
			rxr->discard = TRUE;
		else
			rxr->discard = FALSE;
		/* XXX recycle buffer if there was an error */
		em_rx_discard(rxr, i);
		goto next_desc;
	}
	
	/* Assign correct length to the current fragment */
	ri->ri_flags = RXD_SOP_EOP;
	ri->ri_qidx = rxr->me;
	ri->ri_cidx = i;
	ri->ri_len = len;

	if (eop) {
		em_receive_checksum(cur, &ri->csum_flags,
							&ri->csum_data);
#ifndef __NO_STRICT_ALIGNMENT
		if (adapter->hw.mac.max_frame_size >
			(MCLBYTES - ETHER_ALIGN) &&
			em_fixup_rx(rxr) != 0)
			goto skip;
#endif
		if (status & E1000_RXD_STAT_VP) {
			ri->ri_flags |= RXD_VLAN;
			ri->ri_vtag = le16toh(cur->special);
		}
#ifndef __NO_STRICT_ALIGNMENT
skip:
#endif
		}
next_desc:
		/* Zero out the receive descriptors status. */
		cur->status = 0;
	}

	return ((status & E1000_RXD_STAT_DD) ? TRUE : FALSE);
}

#ifndef __NO_STRICT_ALIGNMENT
/*
 * When jumbo frames are enabled we should realign entire payload on
 * architecures with strict alignment. This is serious design mistake of 8254x
 * as it nullifies DMA operations. 8254x just allows RX buffer size to be
 * 2048/4096/8192/16384. What we really want is 2048 - ETHER_ALIGN to align its
 * payload. On architecures without strict alignment restrictions 8254x still
 * performs unaligned memory access which would reduce the performance too.
 * To avoid copying over an entire frame to align, we allocate a new mbuf and
 * copy ethernet header to the new mbuf. The new mbuf is prepended into the
 * existing mbuf chain.
 *
 * Be aware, best performance of the 8254x is achived only when jumbo frame is
 * not used at all on architectures with strict alignment.
 */
static int
em_fixup_rx(struct rx_ring *rxr)
{
	struct adapter *adapter = rxr->adapter;
	struct mbuf *m, *n;
	int error;

	error = 0;
	m = rxr->fmp;
	if (m->m_len <= (MCLBYTES - ETHER_HDR_LEN)) {
		bcopy(m->m_data, m->m_data + ETHER_HDR_LEN, m->m_len);
		m->m_data += ETHER_HDR_LEN;
	} else {
		MGETHDR(n, M_NOWAIT, MT_DATA);
		if (n != NULL) {
			bcopy(m->m_data, n->m_data, ETHER_HDR_LEN);
			m->m_data += ETHER_HDR_LEN;
			m->m_len -= ETHER_HDR_LEN;
			n->m_len = ETHER_HDR_LEN;
			M_MOVE_PKTHDR(n, m);
			n->m_next = m;
			rxr->fmp = n;
		} else {
			adapter->dropped_pkts++;
			m_freem(rxr->fmp);
			rxr->fmp = NULL;
			error = ENOMEM;
		}
	}

	return (error);
}
#endif

/*********************************************************************
 *
 *  Verify that the hardware indicated that the checksum is valid.
 *  Inform the stack about the status of checksum so that stack
 *  doesn't spend time verifying the checksum.
 *
 *********************************************************************/
static void
em_receive_checksum(struct e1000_rx_desc *rx_desc, int *csum_flags, int *csum_data)
{
	*csum_flags = 0;

	/* Ignore Checksum bit is set */
	if (rx_desc->status & E1000_RXD_STAT_IXSM)
		return;

	if (rx_desc->errors & (E1000_RXD_ERR_TCPE | E1000_RXD_ERR_IPE))
		return;

	/* IP Checksum Good? */
	if (rx_desc->status & E1000_RXD_STAT_IPCS)
		*csum_flags = (CSUM_IP_CHECKED | CSUM_IP_VALID);

	/* TCP or UDP checksum */
	if (rx_desc->status & (E1000_RXD_STAT_TCPCS | E1000_RXD_STAT_UDPCS)) {
		*csum_flags |= (CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
		*csum_data = htons(0xffff);
	}
}

/*
 * This routine is run via an vlan
 * config EVENT
 */
static void
em_if_vlan_register(iflib_ctx_t ctx, u16 vtag)
{
	struct adapter	*adapter = iflib_softc_get(ctx);
	u32		index, bit;

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
em_if_vlan_unregister(iflib_ctx_t ctx, u16 vtag)
{
	struct adapter	*adapter = iflib_softc_get(ctx);
	u32		index, bit;

	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	adapter->shadow_vfta[index] &= ~(1 << bit);
	--adapter->num_vlans;
}

static void
em_setup_vlan_hw_support(struct adapter *adapter)
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
em_if_intr_enable(iflib_ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	struct e1000_hw *hw = &adapter->hw;
	u32 ims_mask = IMS_ENABLE_MASK;

	if (hw->mac.type == e1000_82574) {
		E1000_WRITE_REG(hw, EM_EIAC, EM_MSIX_MASK);
		ims_mask |= EM_MSIX_MASK;
	} 
	E1000_WRITE_REG(hw, E1000_IMS, ims_mask);
}

static void
em_if_intr_disable(iflib_ctx_t ctx)
{
	struct adapter *adapter = iflib_softc_get(ctx);
	struct e1000_hw *hw = &adapter->hw;

	if (hw->mac.type == e1000_82574)
		E1000_WRITE_REG(hw, EM_EIAC, 0);
	E1000_WRITE_REG(&adapter->hw, E1000_IMC, 0xffffffff);
}

/*
 * Bit of a misnomer, what this really means is
 * to enable OS management of the system... aka
 * to disable special hardware management features 
 */
static void
em_init_manageability(struct adapter *adapter)
{
	/* A shared code workaround */
#define E1000_82542_MANC2H E1000_MANC2H
	if (adapter->has_manage) {
		int manc2h = E1000_READ_REG(&adapter->hw, E1000_MANC2H);
		int manc = E1000_READ_REG(&adapter->hw, E1000_MANC);

		/* disable hardware interception of ARP */
		manc &= ~(E1000_MANC_ARP_EN);

                /* enable receiving management packets to the host */
		manc |= E1000_MANC_EN_MNG2HOST;
#define E1000_MNG2HOST_PORT_623 (1 << 5)
#define E1000_MNG2HOST_PORT_664 (1 << 6)
		manc2h |= E1000_MNG2HOST_PORT_623;
		manc2h |= E1000_MNG2HOST_PORT_664;
		E1000_WRITE_REG(&adapter->hw, E1000_MANC2H, manc2h);
		E1000_WRITE_REG(&adapter->hw, E1000_MANC, manc);
	}
}

/*
 * Give control back to hardware management
 * controller if there is one.
 */
static void
em_release_manageability(struct adapter *adapter)
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
 * em_get_hw_control sets the {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means
 * that the driver is loaded. For AMT version type f/w
 * this means that the network i/f is open.
 */
static void
em_get_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext, swsm;

	if (adapter->hw.mac.type == e1000_82573) {
		swsm = E1000_READ_REG(&adapter->hw, E1000_SWSM);
		E1000_WRITE_REG(&adapter->hw, E1000_SWSM,
		    swsm | E1000_SWSM_DRV_LOAD);
		return;
	}
	/* else */
	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
	return;
}

/*
 * em_release_hw_control resets {CTRL_EXT|FWSM}:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is no longer loaded. For AMT versions of the
 * f/w this means that the network i/f is closed.
 */
static void
em_release_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext, swsm;

	if (!adapter->has_manage)
		return;

	if (adapter->hw.mac.type == e1000_82573) {
		swsm = E1000_READ_REG(&adapter->hw, E1000_SWSM);
		E1000_WRITE_REG(&adapter->hw, E1000_SWSM,
		    swsm & ~E1000_SWSM_DRV_LOAD);
		return;
	}
	/* else */
	ctrl_ext = E1000_READ_REG(&adapter->hw, E1000_CTRL_EXT);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL_EXT,
	    ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
	return;
}

static int
em_is_valid_ether_addr(u8 *addr)
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
em_get_wakeup(iflib_ctx_t ctx)
{
	struct adapter	*adapter = iflib_softc_get(ctx);
	u16		eeprom_data = 0, device_id, apme_mask;

	adapter->has_manage = e1000_enable_mng_pass_thru(&adapter->hw);
	apme_mask = EM_EEPROM_APME;

	switch (adapter->hw.mac.type) {
	case e1000_82573:
	case e1000_82583:
		adapter->has_amt = TRUE;
		/* Falls thru */
	case e1000_82571:
	case e1000_82572:
	case e1000_80003es2lan:
		if (adapter->hw.bus.func == 1) {
			e1000_read_nvm(&adapter->hw,
			    NVM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
			break;
		} else
			e1000_read_nvm(&adapter->hw,
			    NVM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	case e1000_ich8lan:
	case e1000_ich9lan:
	case e1000_ich10lan:
	case e1000_pchlan:
	case e1000_pch2lan:
		apme_mask = E1000_WUC_APME;
		adapter->has_amt = TRUE;
		eeprom_data = E1000_READ_REG(&adapter->hw, E1000_WUC);
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
	device_id = adapter->hw.device_id; 
        switch (device_id) {
	case E1000_DEV_ID_82571EB_FIBER:
		/* Wake events only supported on port A for dual fiber
		 * regardless of eeprom setting */
		if (E1000_READ_REG(&adapter->hw, E1000_STATUS) &
		    E1000_STATUS_FUNC_1)
			adapter->wol = 0;
		break;
	case E1000_DEV_ID_82571EB_QUAD_COPPER:
	case E1000_DEV_ID_82571EB_QUAD_FIBER:
	case E1000_DEV_ID_82571EB_QUAD_COPPER_LP:
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
em_enable_wakeup(iflib_ctx_t ctx)
{
	device_t dev = iflib_device_get(ctx);
	struct adapter	*adapter = iflib_softc_get(ctx);
	u32		pmc, ctrl, ctrl_ext, rctl;
	u16     	status;

	if ((pci_find_cap(dev, PCIY_PMG, &pmc) != 0))
		return;

	/* Advertise the wakeup capability */
	ctrl = E1000_READ_REG(&adapter->hw, E1000_CTRL);
	ctrl |= (E1000_CTRL_SWDPIN2 | E1000_CTRL_SWDPIN3);
	E1000_WRITE_REG(&adapter->hw, E1000_CTRL, ctrl);
	E1000_WRITE_REG(&adapter->hw, E1000_WUC, E1000_WUC_PME_EN);

	if ((adapter->hw.mac.type == e1000_ich8lan) ||
	    (adapter->hw.mac.type == e1000_pchlan) ||
	    (adapter->hw.mac.type == e1000_ich9lan) ||
	    (adapter->hw.mac.type == e1000_ich10lan))
		e1000_suspend_workarounds_ich8lan(&adapter->hw);

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
	if ((iflib_getcapenable(ctx) & IFCAP_WOL_MAGIC) == 0)
		adapter->wol &= ~E1000_WUFC_MAG;

	if ((iflib_getcapenable(ctx) & IFCAP_WOL_MCAST) == 0)
		adapter->wol &= ~E1000_WUFC_MC;
	else {
		rctl = E1000_READ_REG(&adapter->hw, E1000_RCTL);
		rctl |= E1000_RCTL_MPE;
		E1000_WRITE_REG(&adapter->hw, E1000_RCTL, rctl);
	}

	if ((adapter->hw.mac.type == e1000_pchlan) ||
	    (adapter->hw.mac.type == e1000_pch2lan)) {
		if (em_enable_phy_wakeup(adapter))
			return;
	} else {
		E1000_WRITE_REG(&adapter->hw, E1000_WUC, E1000_WUC_PME_EN);
		E1000_WRITE_REG(&adapter->hw, E1000_WUFC, adapter->wol);
	}

	if (adapter->hw.phy.type == e1000_phy_igp_3)
		e1000_igp3_phy_powerdown_workaround_ich8lan(&adapter->hw);

        /* Request PME */
        status = pci_read_config(dev, pmc + PCIR_POWER_STATUS, 2);
	status &= ~(PCIM_PSTAT_PME | PCIM_PSTAT_PMEENABLE);
	if (iflib_getcapenable(ctx) & IFCAP_WOL)
		status |= PCIM_PSTAT_PME | PCIM_PSTAT_PMEENABLE;
        pci_write_config(dev, pmc + PCIR_POWER_STATUS, status, 2);

	return;
}

/*
** WOL in the newer chipset interfaces (pchlan)
** require thing to be copied into the phy
*/
static int
em_enable_phy_wakeup(struct adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 mreg, ret = 0;
	u16 preg;

	/* copy MAC RARs to PHY RARs */
	e1000_copy_rx_addrs_to_phy_ich8lan(hw);

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

static void
em_led_func(void *arg, int onoff)
{
	struct adapter	*adapter = arg;
 
	EM_CORE_LOCK(adapter);
	if (onoff) {
		e1000_setup_led(&adapter->hw);
		e1000_led_on(&adapter->hw);
	} else {
		e1000_led_off(&adapter->hw);
		e1000_cleanup_led(&adapter->hw);
	}
	EM_CORE_UNLOCK(adapter);
}

/*
** Disable the L0S and L1 LINK states
*/
static void
em_disable_aspm(struct adapter *adapter)
{
	int		base, reg;
	u16		link_cap,link_ctrl;
	device_t	dev = adapter->dev;

	switch (adapter->hw.mac.type) {
		case e1000_82573:
		case e1000_82574:
		case e1000_82583:
			break;
		default:
			return;
	}
	if (pci_find_cap(dev, PCIY_EXPRESS, &base) != 0)
		return;
	reg = base + PCIER_LINK_CAP;
	link_cap = pci_read_config(dev, reg, 2);
	if ((link_cap & PCIEM_LINK_CAP_ASPM) == 0)
		return;
	reg = base + PCIER_LINK_CTL;
	link_ctrl = pci_read_config(dev, reg, 2);
	link_ctrl &= ~PCIEM_LINK_CTL_ASPMC;
	pci_write_config(dev, reg, link_ctrl, 2);
	return;
}

/**********************************************************************
 *
 *  Update the board statistics counters.
 *
 **********************************************************************/
static void
em_update_stats_counters(struct adapter *adapter)
{
	if_t ifp;

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
	/*
	** For watchdog management we need to know if we have been
	** paused during the last interval, so capture that here.
	*/
	adapter->pause_frames = E1000_READ_REG(&adapter->hw, E1000_XOFFRXC);
	adapter->stats.xoffrxc += adapter->pause_frames;
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

	/* Interrupt Counts */

	adapter->stats.iac += E1000_READ_REG(&adapter->hw, E1000_IAC);
	adapter->stats.icrxptc += E1000_READ_REG(&adapter->hw, E1000_ICRXPTC);
	adapter->stats.icrxatc += E1000_READ_REG(&adapter->hw, E1000_ICRXATC);
	adapter->stats.ictxptc += E1000_READ_REG(&adapter->hw, E1000_ICTXPTC);
	adapter->stats.ictxatc += E1000_READ_REG(&adapter->hw, E1000_ICTXATC);
	adapter->stats.ictxqec += E1000_READ_REG(&adapter->hw, E1000_ICTXQEC);
	adapter->stats.ictxqmtc += E1000_READ_REG(&adapter->hw, E1000_ICTXQMTC);
	adapter->stats.icrxdmtc += E1000_READ_REG(&adapter->hw, E1000_ICRXDMTC);
	adapter->stats.icrxoc += E1000_READ_REG(&adapter->hw, E1000_ICRXOC);

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
	ifp = adapter->ifp;

	if_setcollisions(ifp, adapter->stats.colc);

	/* Rx Errors */
	if_setierrors(ifp, adapter->dropped_pkts + adapter->stats.rxerrc +
	    adapter->stats.crcerrs + adapter->stats.algnerrc +
	    adapter->stats.ruc + adapter->stats.roc +
	    adapter->stats.mpc + adapter->stats.cexterr);

	/* Tx Errors */
	if_setoerrors(ifp, adapter->stats.ecol + adapter->stats.latecol +
	    adapter->watchdog_events);
}

/* Export a single 32-bit register via a read-only sysctl. */
static int
em_sysctl_reg_handler(SYSCTL_HANDLER_ARGS)
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
em_add_hw_stats(struct adapter *adapter)
{
	device_t dev = adapter->dev;

	struct tx_ring *txr = adapter->tx_rings;
	struct rx_ring *rxr = adapter->rx_rings;

	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(dev);
	struct sysctl_oid_list *child = SYSCTL_CHILDREN(tree);
	struct e1000_hw_stats *stats = &adapter->stats;

	struct sysctl_oid *stat_node, *queue_node, *int_node;
	struct sysctl_oid_list *stat_list, *queue_list, *int_list;

#define QUEUE_NAME_LEN 32
	char namebuf[QUEUE_NAME_LEN];
	
	/* Driver Statistics */
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "link_irq",
			CTLFLAG_RD, &adapter->link_irq,
			"Link MSIX IRQ Handled");
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
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "rx_overruns",
			CTLFLAG_RD, &adapter->rx_overruns,
			"RX overruns");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "watchdog_timeouts",
			CTLFLAG_RD, &adapter->watchdog_events,
			"Watchdog timeouts");
	
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "device_control",
			CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_CTRL,
			em_sysctl_reg_handler, "IU",
			"Device Control Register");
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "rx_control",
			CTLTYPE_UINT | CTLFLAG_RD, adapter, E1000_RCTL,
			em_sysctl_reg_handler, "IU",
			"Receiver Control Register");
	SYSCTL_ADD_UINT(ctx, child, OID_AUTO, "fc_high_water",
			CTLFLAG_RD, &adapter->hw.fc.high_water, 0,
			"Flow Control High Watermark");
	SYSCTL_ADD_UINT(ctx, child, OID_AUTO, "fc_low_water", 
			CTLFLAG_RD, &adapter->hw.fc.low_water, 0,
			"Flow Control Low Watermark");

	for (int i = 0; i < adapter->num_queues; i++, rxr++, txr++) {
		snprintf(namebuf, QUEUE_NAME_LEN, "queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf,
					    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "txd_head", 
				CTLTYPE_UINT | CTLFLAG_RD, adapter,
				E1000_TDH(txr->me),
				em_sysctl_reg_handler, "IU",
 				"Transmit Descriptor Head");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "txd_tail", 
				CTLTYPE_UINT | CTLFLAG_RD, adapter,
				E1000_TDT(txr->me),
				em_sysctl_reg_handler, "IU",
 				"Transmit Descriptor Tail");
		SYSCTL_ADD_ULONG(ctx, queue_list, OID_AUTO, "tx_irq",
				CTLFLAG_RD, &txr->tx_irq,
				"Queue MSI-X Transmit Interrupts");
		SYSCTL_ADD_ULONG(ctx, queue_list, OID_AUTO, "no_desc_avail", 
				CTLFLAG_RD, &txr->no_desc_avail,
				"Queue No Descriptor Available");
		
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "rxd_head", 
				CTLTYPE_UINT | CTLFLAG_RD, adapter,
				E1000_RDH(rxr->me),
				em_sysctl_reg_handler, "IU",
				"Receive Descriptor Head");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "rxd_tail", 
				CTLTYPE_UINT | CTLFLAG_RD, adapter,
				E1000_RDT(rxr->me),
				em_sysctl_reg_handler, "IU",
				"Receive Descriptor Tail");
		SYSCTL_ADD_ULONG(ctx, queue_list, OID_AUTO, "rx_irq",
				CTLFLAG_RD, &rxr->rx_irq,
				"Queue MSI-X Receive Interrupts");
	}

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
	/* On 82575 these are collision counts */
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


	/* Interrupt Stats */

	int_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "interrupts", 
				    CTLFLAG_RD, NULL, "Interrupt Statistics");
	int_list = SYSCTL_CHILDREN(int_node);

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "asserts",
			CTLFLAG_RD, &adapter->stats.iac,
			"Interrupt Assertion Count");

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "rx_pkt_timer",
			CTLFLAG_RD, &adapter->stats.icrxptc,
			"Interrupt Cause Rx Pkt Timer Expire Count");

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "rx_abs_timer",
			CTLFLAG_RD, &adapter->stats.icrxatc,
			"Interrupt Cause Rx Abs Timer Expire Count");

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "tx_pkt_timer",
			CTLFLAG_RD, &adapter->stats.ictxptc,
			"Interrupt Cause Tx Pkt Timer Expire Count");

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "tx_abs_timer",
			CTLFLAG_RD, &adapter->stats.ictxatc,
			"Interrupt Cause Tx Abs Timer Expire Count");

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "tx_queue_empty",
			CTLFLAG_RD, &adapter->stats.ictxqec,
			"Interrupt Cause Tx Queue Empty Count");

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "tx_queue_min_thresh",
			CTLFLAG_RD, &adapter->stats.ictxqmtc,
			"Interrupt Cause Tx Queue Min Thresh Count");

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "rx_desc_min_thresh",
			CTLFLAG_RD, &adapter->stats.icrxdmtc,
			"Interrupt Cause Rx Desc Min Thresh Count");

	SYSCTL_ADD_UQUAD(ctx, int_list, OID_AUTO, "rx_overrun",
			CTLFLAG_RD, &adapter->stats.icrxoc,
			"Interrupt Cause Receiver Overrun Count");
}

/**********************************************************************
 *
 *  This routine provides a way to dump out the adapter eeprom,
 *  often a useful debug/service tool. This only dumps the first
 *  32 words, stuff that matters is in that extent.
 *
 **********************************************************************/
static int
em_sysctl_nvm_info(SYSCTL_HANDLER_ARGS)
{
	struct adapter *adapter = (struct adapter *)arg1;
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
	if (result == 1)
		em_print_nvm_info(adapter);

	return (error);
}

static void
em_print_nvm_info(struct adapter *adapter)
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
em_sysctl_int_delay(SYSCTL_HANDLER_ARGS)
{
	struct em_int_delay_info *info;
	struct adapter *adapter;
	u32 regval;
	int error, usecs, ticks;

	info = (struct em_int_delay_info *)arg1;
	usecs = info->value;
	error = sysctl_handle_int(oidp, &usecs, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (usecs < 0 || usecs > EM_TICKS_TO_USECS(65535))
		return (EINVAL);
	info->value = usecs;
	ticks = EM_USECS_TO_TICKS(usecs);
	if (info->offset == E1000_ITR)	/* units are 256ns here */
		ticks *= 4;

	adapter = info->adapter;
	
	EM_CORE_LOCK(adapter);
	regval = E1000_READ_OFFSET(&adapter->hw, info->offset);
	regval = (regval & ~0xffff) | (ticks & 0xffff);
	/* Handle a few special cases. */
	switch (info->offset) {
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
	E1000_WRITE_OFFSET(&adapter->hw, info->offset, regval);
	EM_CORE_UNLOCK(adapter);
	return (0);
}

static void
em_add_int_delay_sysctl(struct adapter *adapter, const char *name,
	const char *description, struct em_int_delay_info *info,
	int offset, int value)
{
	info->adapter = adapter;
	info->offset = offset;
	info->value = value;
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(adapter->dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(adapter->dev)),
	    OID_AUTO, name, CTLTYPE_INT|CTLFLAG_RW,
	    info, 0, em_sysctl_int_delay, "I", description);
}

static void
em_set_sysctl_value(struct adapter *adapter, const char *name,
	const char *description, int *limit, int value)
{
	*limit = value;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(adapter->dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(adapter->dev)),
	    OID_AUTO, name, CTLTYPE_INT|CTLFLAG_RW, limit, value, description);
}


/*
** Set flow control using sysctl:
** Flow control values:
**      0 - off
**      1 - rx pause
**      2 - tx pause
**      3 - full
*/
static int
em_set_flowcntl(SYSCTL_HANDLER_ARGS)
{       
        int		error;
	static int	input = 3; /* default is full */
        struct adapter	*adapter = (struct adapter *) arg1;
                    
        error = sysctl_handle_int(oidp, &input, 0, req);
    
        if ((error) || (req->newptr == NULL))
                return (error);
                
	if (input == adapter->fc) /* no change? */
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
        return (error);
}

/*
** Manage Energy Efficient Ethernet:
** Control values:
**     0/1 - enabled/disabled
*/
static int
em_sysctl_eee(SYSCTL_HANDLER_ARGS)
{
       struct adapter *adapter = (struct adapter *) arg1;
       int             error, value;

       value = adapter->hw.dev_spec.ich8lan.eee_disable;
       error = sysctl_handle_int(oidp, &value, 0, req);
       if (error || req->newptr == NULL)
               return (error);
       adapter->hw.dev_spec.ich8lan.eee_disable = (value != 0);
	   iflib_init(adapter->ctx);

       return (0);
}

static int
em_sysctl_debug_info(SYSCTL_HANDLER_ARGS)
{
	struct adapter *adapter;
	int error;
	int result;

	result = -1;
	error = sysctl_handle_int(oidp, &result, 0, req);

	if (error || !req->newptr)
		return (error);

	if (result == 1) {
		adapter = (struct adapter *)arg1;
		em_print_debug_info(adapter);
	}
	return (error);
}

/*
** This routine is meant to be fluid, add whatever is
** needed for debugging a problem.  -jfv
*/
static void
em_print_debug_info(struct adapter *adapter)
{
	device_t dev = adapter->dev;
	struct tx_ring *txr = adapter->tx_rings;
	struct rx_ring *rxr = adapter->rx_rings;

	if (iflib_getactive(adapter->ctx))
		printf("Interface is RUNNING ");
	else
		printf("Interface is NOT RUNNING\n");

	device_printf(dev, "hw tdh = %d, hw tdt = %d\n",
	    E1000_READ_REG(&adapter->hw, E1000_TDH(0)),
	    E1000_READ_REG(&adapter->hw, E1000_TDT(0)));
	device_printf(dev, "hw rdh = %d, hw rdt = %d\n",
	    E1000_READ_REG(&adapter->hw, E1000_RDH(0)),
	    E1000_READ_REG(&adapter->hw, E1000_RDT(0)));
	device_printf(dev, "Tx Queue Status = %d\n", txr->queue_status);
	device_printf(dev, "TX descriptors avail = %d\n",
	    txr->tx_avail);
	device_printf(dev, "Tx Descriptors avail failure = %ld\n",
	    txr->no_desc_avail);
	device_printf(dev, "RX discarded packets = %ld\n",
	    rxr->rx_discarded);
	device_printf(dev, "RX Next to Check = %d\n", rxr->next_to_check);
	device_printf(dev, "RX Next to Refresh = %d\n", rxr->next_to_refresh);
}
