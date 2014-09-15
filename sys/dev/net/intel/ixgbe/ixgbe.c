/******************************************************************************

  Copyright (c) 2001-2013, Intel Corporation 
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
#include "ixgbe.h"

#ifdef	RSS
#include <netinet/in_rss.h>
#endif

/*********************************************************************
 *  Set this to one to display debug statistics
 *********************************************************************/
int             ixgbe_display_debug_stats = 0;

/*********************************************************************
 *  Driver version
 *********************************************************************/
char ixgbe_driver_version[] = "2.5.15";

/*********************************************************************
 *  PCI Device ID Table
 *
 *  Used by probe to select devices to load on
 *  Last field stores an index into ixgbe_strings
 *  Last entry must be all 0s
 *
 *  { Vendor ID, Device ID, SubVendor ID, SubDevice ID, String Index }
 *********************************************************************/

static ixgbe_vendor_info_t ixgbe_vendor_info_array[] =
{
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598AF_DUAL_PORT, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598AF_SINGLE_PORT, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598EB_CX4, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598AT, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598AT2, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598_DA_DUAL_PORT, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598_CX4_DUAL_PORT, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598EB_XF_LR, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598_SR_DUAL_PORT_EM, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82598EB_SFP_LOM, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_KX4, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_KX4_MEZZ, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_SFP, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_XAUI_LOM, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_CX4, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_T3_LOM, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_COMBO_BACKPLANE, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_BACKPLANE_FCOE, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_SFP_SF2, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_SFP_FCOE, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599EN_SFP, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_82599_SFP_SF_QP, 0, 0, 0},
	{IXGBE_INTEL_VENDOR_ID, IXGBE_DEV_ID_X540T, 0, 0, 0},
	/* required last entry */
	{0, 0, 0, 0, 0}
};

/*********************************************************************
 *  Table of branding strings
 *********************************************************************/

static char    *ixgbe_strings[] = {
	"Intel(R) PRO/10GbE PCI-Express Network Driver"
};

/*********************************************************************
 *  Function prototypes
 *********************************************************************/
static int      ixgbe_probe(device_t);
static int      ixgbe_attach(device_t);
static int		ixgbe_if_detach(if_shared_ctx_t);
static void	ixgbe_if_init(if_shared_ctx_t);
static void     ixgbe_if_stop(if_shared_ctx_t);
static void     ixgbe_if_media_status(if_shared_ctx_t, struct ifmediareq *);
static int      ixgbe_if_media_change(if_shared_ctx_t);
static void     ixgbe_identify_hardware(struct adapter *);
static int      ixgbe_allocate_pci_resources(struct adapter *);
static void	ixgbe_get_slot_info(struct ixgbe_hw *);
static int      ixgbe_allocate_msix(struct adapter *);
static int      ixgbe_allocate_legacy(struct adapter *);
static int	ixgbe_if_queues_alloc(if_shared_ctx_t);
static int	ixgbe_setup_msix(struct adapter *);
static void	ixgbe_free_pci_resources(struct adapter *);
static void	ixgbe_if_timer(if_shared_ctx_t);
static void	ixgbe_if_watchdog_reset(if_shared_ctx_t);
static void	ixgbe_if_vlan_register(if_shared_ctx_t, uint16_t);
static void	ixgbe_if_vlan_unregister(if_shared_ctx_t, uint16_t);
static int	ixgbe_if_txq_setup(if_shared_ctx_t, uint32_t);
static int	ixgbe_if_rxq_setup(if_shared_ctx_t, uint32_t);
static int	ixgbe_setup_interface(device_t, struct adapter *);
static void	ixgbe_config_link(struct adapter *);

static int      ixgbe_allocate_transmit_buffers(struct tx_ring *);
static void     ixgbe_initialize_transmit_units(struct adapter *);
static void     ixgbe_if_tx_structures_free(if_shared_ctx_t);
static void     ixgbe_free_transmit_buffers(struct tx_ring *);

static void     ixgbe_initialize_receive_units(struct adapter *);
static void     ixgbe_if_rx_structures_free(if_shared_ctx_t);
static void	ixgbe_setup_hw_rsc(struct rx_ring *);

static void     ixgbe_if_rx_intr_enable(if_shared_ctx_t, uint32_t);
static void     ixgbe_if_intr_enable(if_shared_ctx_t);
static void     ixgbe_if_intr_disable(if_shared_ctx_t);
static void     ixgbe_update_stats_counters(struct adapter *);
static int	ixgbe_isc_txd_credits_update(if_shared_ctx_t, uint32_t, uint32_t);
static int	ixgbe_isc_rxd_pkt_get(if_shared_ctx_t, uint32_t, uint32_t, if_rxd_info_t);
static void	ixgbe_rx_checksum(u32, if_rxd_info_t, u32);
static void     ixgbe_if_promisc_set(if_shared_ctx_t, int);
static int      ixgbe_if_mtu_set(if_shared_ctx_t, uint32_t);
static void     ixgbe_if_multi_set(if_shared_ctx_t);
static int		ixgbe_if_i2c_req(if_shared_ctx_t, struct ifi2creq *);
static void     ixgbe_if_update_link_status(if_shared_ctx_t);
static int      ixgbe_isc_txd_encap(if_shared_ctx_t, uint32_t, if_pkt_info_t);
static void ixgbe_isc_txd_flush(if_shared_ctx_t, uint32_t, uint32_t);
static void ixgbe_isc_rxd_refill(if_shared_ctx_t, uint32_t, uint32_t, uint64_t*, uint32_t);
static void ixgbe_isc_rxd_flush(if_shared_ctx_t, uint32_t, uint32_t);
static int ixgbe_isc_rxd_available(if_shared_ctx_t, uint32_t, uint32_t);


static int	ixgbe_set_flowcntl(SYSCTL_HANDLER_ARGS);
static int	ixgbe_set_advertise(SYSCTL_HANDLER_ARGS);
static int	ixgbe_set_thermal_test(SYSCTL_HANDLER_ARGS);
static int	ixgbe_tx_ctx_setup(struct tx_ring *,
		    struct mbuf *, u32 *, u32 *);
static int	ixgbe_tso_setup(struct tx_ring *,
		    struct mbuf *, u32 *, u32 *);
static void	ixgbe_set_ivar(struct adapter *, u8, u8, s8);
static void	ixgbe_configure_ivars(struct adapter *);
static u8 *	ixgbe_mc_array_itr(struct ixgbe_hw *, u8 **, u32 *);

static void	ixgbe_setup_vlan_hw_support(struct adapter *);
static void	ixgbe_if_vlan_register(if_shared_ctx_t, u16);
static void	ixgbe_if_vlan_unregister(if_shared_ctx_t, u16);

static void     ixgbe_add_hw_stats(struct adapter *adapter);
#if 0
static __inline void ixgbe_rx_input(struct rx_ring *, struct ifnet *,
		    struct mbuf *, u32);
#endif
static void	ixgbe_enable_rx_drop(struct adapter *);
static void	ixgbe_disable_rx_drop(struct adapter *);

/* Support for pluggable optic modules */
static bool	ixgbe_sfp_probe(struct adapter *);
static void	ixgbe_setup_optics(struct adapter *);

/* Legacy (single vector interrupt handler */
static int	ixgbe_legacy_irq(void *);

/* The MSI/X Interrupt handlers */
static int	ixgbe_msix_que(void *);
static int	ixgbe_msix_link(void *);

/* Deferred interrupt tasklets */
static void	ixgbe_handle_link(void *, int);
static void	ixgbe_handle_msf(void *, int);
static void	ixgbe_handle_mod(void *, int);

#ifdef IXGBE_FDIR
static void	ixgbe_atr(struct tx_ring *, struct mbuf *);
static void	ixgbe_reinit_fdir(void *, int);
#endif

/* Missing shared code prototype */
extern void ixgbe_stop_mac_link_on_d3_82599(struct ixgbe_hw *hw);

/*********************************************************************
 *  FreeBSD Device Interface Entry Points
 *********************************************************************/

static device_method_t ixgbe_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, ixgbe_probe),
	DEVMETHOD(device_attach, ixgbe_attach),
	DEVMETHOD(device_detach, iflib_device_detach),
	DEVMETHOD(device_shutdown, iflib_device_suspend),
	DEVMETHOD_END
};

static driver_t ixgbe_driver = {
	"ix", ixgbe_methods, sizeof(struct adapter),
};

devclass_t ixgbe_devclass;
DRIVER_MODULE(ixgbe, pci, ixgbe_driver, ixgbe_devclass, 0, 0);
MODULE_DEPEND(ixgbe, pci, 1, 1, 1);
MODULE_DEPEND(ixgbe, iflib, 1, 1, 1);
MODULE_DEPEND(ixgbe, ether, 1, 1, 1);

static device_method_t ixgbe_if_methods[] = {
	DEVMETHOD(ifdi_detach, ixgbe_if_detach),
	DEVMETHOD(ifdi_init, ixgbe_if_init),
	DEVMETHOD(ifdi_stop, ixgbe_if_stop),
	DEVMETHOD(ifdi_intr_disable, ixgbe_if_intr_disable),
	DEVMETHOD(ifdi_intr_enable, ixgbe_if_intr_enable),
	DEVMETHOD(ifdi_rx_intr_enable, ixgbe_if_rx_intr_enable),
	DEVMETHOD(ifdi_multi_set, ixgbe_if_multi_set),
	DEVMETHOD(ifdi_queues_alloc, ixgbe_if_queues_alloc),
	DEVMETHOD(ifdi_update_link_status, ixgbe_if_update_link_status),
	DEVMETHOD(ifdi_mtu_set, ixgbe_if_mtu_set),
	DEVMETHOD(ifdi_i2c_req, ixgbe_if_i2c_req),
	DEVMETHOD(ifdi_media_status, ixgbe_if_media_status),
	DEVMETHOD(ifdi_media_change, ixgbe_if_media_change),
	DEVMETHOD(ifdi_timer, ixgbe_if_timer),
	DEVMETHOD(ifdi_watchdog_reset, ixgbe_if_watchdog_reset),
	DEVMETHOD(ifdi_promisc_set, ixgbe_if_promisc_set),
	DEVMETHOD(ifdi_vlan_register, ixgbe_if_vlan_register),
	DEVMETHOD(ifdi_vlan_unregister, ixgbe_if_vlan_unregister),
	DEVMETHOD(ifdi_txq_setup, ixgbe_if_txq_setup),
	DEVMETHOD(ifdi_rxq_setup, ixgbe_if_rxq_setup),
	DEVMETHOD(ifdi_rx_structures_free, ixgbe_if_rx_structures_free),
	DEVMETHOD(ifdi_tx_structures_free, ixgbe_if_tx_structures_free),
	DEVMETHOD_END
};

static driver_t ixgbe_if_driver = {
	"ixgbe_if", ixgbe_if_methods, sizeof(struct if_shared_ctx),
};

#define DOWNCAST(sctx) ((struct adapter *)(sctx))

/*
** TUNEABLE PARAMETERS:
*/

static SYSCTL_NODE(_hw, OID_AUTO, ix, CTLFLAG_RD, 0,
		   "IXGBE driver parameters");

/*
** AIM: Adaptive Interrupt Moderation
** which means that the interrupt rate
** is varied over time based on the
** traffic for that interrupt vector
*/
static int ixgbe_enable_aim = TRUE;
SYSCTL_INT(_hw_ix, OID_AUTO, enable_aim, CTLFLAG_RWTUN, &ixgbe_enable_aim, 0,
    "Enable adaptive interrupt moderation");

static int ixgbe_max_interrupt_rate = (4000000 / IXGBE_LOW_LATENCY);
SYSCTL_INT(_hw_ix, OID_AUTO, max_interrupt_rate, CTLFLAG_RDTUN,
    &ixgbe_max_interrupt_rate, 0, "Maximum interrupts per second");

/* How many packets rxeof tries to clean at a time */
static int ixgbe_rx_process_limit = 256;
SYSCTL_INT(_hw_ix, OID_AUTO, rx_process_limit, CTLFLAG_RDTUN,
    &ixgbe_rx_process_limit, 0,
    "Maximum number of received packets to process at a time,"
    "-1 means unlimited");

/* How many packets txeof tries to clean at a time */
static int ixgbe_tx_process_limit = 256;
SYSCTL_INT(_hw_ix, OID_AUTO, tx_process_limit, CTLFLAG_RDTUN,
    &ixgbe_tx_process_limit, 0,
    "Maximum number of sent packets to process at a time,"
    "-1 means unlimited");

/*
** Smart speed setting, default to on
** this only works as a compile option
** right now as its during attach, set
** this to 'ixgbe_smart_speed_off' to
** disable.
*/
static int ixgbe_smart_speed = ixgbe_smart_speed_on;

/*
 * MSIX should be the default for best performance,
 * but this allows it to be forced off for testing.
 */
static int ixgbe_enable_msix = 1;
SYSCTL_INT(_hw_ix, OID_AUTO, enable_msix, CTLFLAG_RDTUN, &ixgbe_enable_msix, 0,
    "Enable MSI-X interrupts");

/*
 * Number of Queues, can be set to 0,
 * it then autoconfigures based on the
 * number of cpus with a max of 8. This
 * can be overriden manually here.
 */
static int ixgbe_num_queues = 0;
SYSCTL_INT(_hw_ix, OID_AUTO, num_queues, CTLFLAG_RDTUN, &ixgbe_num_queues, 0,
    "Number of queues to configure, 0 indicates autoconfigure");

/*
** Number of TX descriptors per ring,
** setting higher than RX as this seems
** the better performing choice.
*/
static int ixgbe_txd = PERFORM_TXD;
SYSCTL_INT(_hw_ix, OID_AUTO, txd, CTLFLAG_RDTUN, &ixgbe_txd, 0,
    "Number of transmit descriptors per queue");

/* Number of RX descriptors per ring */
static int ixgbe_rxd = PERFORM_RXD;
SYSCTL_INT(_hw_ix, OID_AUTO, rxd, CTLFLAG_RDTUN, &ixgbe_rxd, 0,
    "Number of receive descriptors per queue");

/*
** Defining this on will allow the use
** of unsupported SFP+ modules, note that
** doing so you are on your own :)
*/
static int allow_unsupported_sfp = FALSE;
TUNABLE_INT("hw.ix.unsupported_sfp", &allow_unsupported_sfp);

/*
** HW RSC control: 
**  this feature only works with
**  IPv4, and only on 82599 and later.
**  Also this will cause IP forwarding to
**  fail and that can't be controlled by
**  the stack as LRO can. For all these
**  reasons I've deemed it best to leave
**  this off and not bother with a tuneable
**  interface, this would need to be compiled
**  to enable.
*/
static bool ixgbe_rsc_enable = FALSE;

/* Keep running tab on them for sanity check */
static int ixgbe_total_ports;

#ifdef IXGBE_FDIR
/*
** For Flow Director: this is the
** number of TX packets we sample
** for the filter pool, this means
** every 20th packet will be probed.
**
** This feature can be disabled by 
** setting this to 0.
*/
static int atr_sample_rate = 20;
/* 
** Flow Director actually 'steals'
** part of the packet buffer as its
** filter pool, this variable controls
** how much it uses:
**  0 = 64K, 1 = 128K, 2 = 256K
*/
static int fdir_pballoc = 1;
#endif

#ifdef DEV_NETMAP
/*
 * The #ifdef DEV_NETMAP / #endif blocks in this file are meant to
 * be a reference on how to implement netmap support in a driver.
 * Additional comments are in ixgbe_netmap.h .
 *
 * <dev/netmap/ixgbe_netmap.h> contains functions for netmap support
 * that extend the standard driver.
 */
#include <dev/netmap/ixgbe_netmap.h>
#endif /* DEV_NETMAP */

/*********************************************************************
 *  Device identification routine
 *
 *  ixgbe_probe determines if the driver should be loaded on
 *  adapter based on PCI vendor/device id of the adapter.
 *
 *  return BUS_PROBE_DEFAULT on success, positive on failure
 *********************************************************************/

static int
ixgbe_probe(device_t dev)
{
	ixgbe_vendor_info_t *ent;

	u16	pci_vendor_id = 0;
	u16	pci_device_id = 0;
	u16	pci_subvendor_id = 0;
	u16	pci_subdevice_id = 0;
	char	adapter_name[256];

	INIT_DEBUGOUT("ixgbe_probe: begin");

	pci_vendor_id = pci_get_vendor(dev);
	if (pci_vendor_id != IXGBE_INTEL_VENDOR_ID)
		return (ENXIO);

	pci_device_id = pci_get_device(dev);
	pci_subvendor_id = pci_get_subvendor(dev);
	pci_subdevice_id = pci_get_subdevice(dev);

	ent = ixgbe_vendor_info_array;
	while (ent->vendor_id != 0) {
		if ((pci_vendor_id == ent->vendor_id) &&
		    (pci_device_id == ent->device_id) &&

		    ((pci_subvendor_id == ent->subvendor_id) ||
		     (ent->subvendor_id == 0)) &&

		    ((pci_subdevice_id == ent->subdevice_id) ||
		     (ent->subdevice_id == 0))) {
			sprintf(adapter_name, "%s, Version - %s",
				ixgbe_strings[ent->index],
				ixgbe_driver_version);
			device_set_desc_copy(dev, adapter_name);
			++ixgbe_total_ports;
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
ixgbe_attach(device_t dev)
{
	struct adapter *adapter;
	struct ixgbe_hw *hw;
	if_shared_ctx_t sctx;
	int             tsize, rsize, error = 0;
	u16		csum;
	u32		ctrl_ext;

	INIT_DEBUGOUT("ixgbe_attach: begin");

	/* Allocate, clear, and link in our adapter structure */
	adapter = device_get_softc(dev);
	sctx = UPCAST(adapter);
	adapter->hwdev = adapter->osdep.dev = dev;
	hw = &adapter->hw;

	/* SYSCTL APIs */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
			SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
			OID_AUTO, "fc", CTLTYPE_INT | CTLFLAG_RW,
			adapter, 0, ixgbe_set_flowcntl, "I", "Flow Control");

        SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
			SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
			OID_AUTO, "enable_aim", CTLTYPE_INT|CTLFLAG_RW,
			&ixgbe_enable_aim, 1, "Interrupt Moderation");

	/*
	** Allow a kind of speed control by forcing the autoneg
	** advertised speed list to only a certain value, this
	** supports 1G on 82599 devices, and 100Mb on x540.
	*/
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
			SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
			OID_AUTO, "advertise_speed", CTLTYPE_INT | CTLFLAG_RW,
			adapter, 0, ixgbe_set_advertise, "I", "Link Speed");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
			SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
			OID_AUTO, "ts", CTLTYPE_INT | CTLFLAG_RW, adapter,
			0, ixgbe_set_thermal_test, "I", "Thermal Test");

	/* Determine hardware revision */
	ixgbe_identify_hardware(adapter);

	if ((error = iflib_register(dev, &ixgbe_if_driver, hw->mac.addr)) != 0)
		return (error);

	/* Do base PCI setup - map BAR0 */
	if (ixgbe_allocate_pci_resources(adapter)) {
		device_printf(dev, "Allocation of PCI resources failed\n");
		error = ENXIO;
		goto err_out;
	}

	/* Do descriptor calc and sanity checks */
	if (((ixgbe_txd * sizeof(union ixgbe_adv_tx_desc)) % DBA_ALIGN) != 0 ||
	    ixgbe_txd < MIN_TXD || ixgbe_txd > MAX_TXD) {
		device_printf(dev, "TXD config issue, using default!\n");
		adapter->num_tx_desc = DEFAULT_TXD;
	} else
		adapter->num_tx_desc = ixgbe_txd;

	/*
	** With many RX rings it is easy to exceed the
	** system mbuf allocation. Tuning nmbclusters
	** can alleviate this.
	*/
	if (nmbclusters > 0 ) {
		int s;
		s = (ixgbe_rxd * adapter->num_queues) * ixgbe_total_ports;
		if (s > nmbclusters) {
			device_printf(dev, "RX Descriptors exceed "
			    "system mbuf max, using default instead!\n");
			ixgbe_rxd = DEFAULT_RXD;
		}
	}

	if (((ixgbe_rxd * sizeof(union ixgbe_adv_rx_desc)) % DBA_ALIGN) != 0 ||
	    ixgbe_rxd < MIN_RXD || ixgbe_rxd > MAX_RXD) {
		device_printf(dev, "RXD config issue, using default!\n");
		adapter->num_rx_desc = DEFAULT_RXD;
	} else
		adapter->num_rx_desc = ixgbe_rxd;

	/* Allocate our TX/RX Queues */
	tsize = roundup2(adapter->num_tx_desc *
					 sizeof(union ixgbe_adv_tx_desc), DBA_ALIGN);
	rsize = roundup2(adapter->num_rx_desc *
					 sizeof(union ixgbe_adv_rx_desc), DBA_ALIGN);
	if (iflib_queues_alloc(sctx, tsize, rsize)) {
		error = ENOMEM;
		goto err_out;
	}

	/* Allocate multicast array memory. */
	adapter->mta = malloc(sizeof(u8) * IXGBE_ETH_LENGTH_OF_ADDRESS *
	    MAX_NUM_MULTICAST_ADDRESSES, M_DEVBUF, M_NOWAIT);
	if (adapter->mta == NULL) {
		device_printf(dev, "Can not allocate multicast setup array\n");
		error = ENOMEM;
		goto err_late;
	}

	/* Initialize the shared code */
	hw->allow_unsupported_sfp = allow_unsupported_sfp;
	error = ixgbe_init_shared_code(hw);
	if (error == IXGBE_ERR_SFP_NOT_PRESENT) {
		/*
		** No optics in this port, set up
		** so the timer routine will probe 
		** for later insertion.
		*/
		adapter->sfp_probe = TRUE;
		error = 0;
	} else if (error == IXGBE_ERR_SFP_NOT_SUPPORTED) {
		device_printf(dev,"Unsupported SFP+ module detected!\n");
		error = EIO;
		goto err_late;
	} else if (error) {
		device_printf(dev,"Unable to initialize the shared code\n");
		error = EIO;
		goto err_late;
	}

	/* Make sure we have a good EEPROM before we read from it */
	if (ixgbe_validate_eeprom_checksum(&adapter->hw, &csum) < 0) {
		device_printf(dev,"The EEPROM Checksum Is Not Valid\n");
		error = EIO;
		goto err_late;
	}

	error = ixgbe_init_hw(hw);
	switch (error) {
	case IXGBE_ERR_EEPROM_VERSION:
		device_printf(dev, "This device is a pre-production adapter/"
		    "LOM.  Please be aware there may be issues associated "
		    "with your hardware.\n If you are experiencing problems "
		    "please contact your Intel or hardware representative "
		    "who provided you with this hardware.\n");
		break;
	case IXGBE_ERR_SFP_NOT_SUPPORTED:
		device_printf(dev,"Unsupported SFP+ Module\n");
		error = EIO;
		goto err_late;
	case IXGBE_ERR_SFP_NOT_PRESENT:
		device_printf(dev,"No SFP+ Module found\n");
		/* falls thru */
	default:
		break;
	}

	/* Detect and set physical type */
	ixgbe_setup_optics(adapter);

	if ((adapter->msix > 1) && (ixgbe_enable_msix))
		error = ixgbe_allocate_msix(adapter); 
	else
		error = ixgbe_allocate_legacy(adapter); 
	if (error) 
		goto err_late;

	/* Setup OS specific network interface */
	if (ixgbe_setup_interface(dev, adapter) != 0)
		goto err_late;

	/* Initialize statistics */
	ixgbe_update_stats_counters(adapter);

	/*
	** Check PCIE slot type/speed/width
	*/
	ixgbe_get_slot_info(hw);

	/* Set an initial default flow control value */
	adapter->fc =  ixgbe_fc_full;

	/* let hardware know driver is loaded */
	ctrl_ext = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
	ctrl_ext |= IXGBE_CTRL_EXT_DRV_LOAD;
	IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT, ctrl_ext);

	ixgbe_add_hw_stats(adapter);

#ifdef DEV_NETMAP
	ixgbe_netmap_attach(adapter);
#endif /* DEV_NETMAP */
	INIT_DEBUGOUT("ixgbe_attach: end");
	return (0);
err_late:
	iflib_txrx_structures_free(sctx);
err_out:
	if (adapter->hwifp != NULL)
		if_free(adapter->hwifp);
	ixgbe_free_pci_resources(adapter);
	free(adapter->mta, M_DEVBUF);
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
ixgbe_if_detach(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	u32	ctrl_ext;

	INIT_DEBUGOUT("ixgbe_detach: begin");

	/* let hardware know driver is unloading */
	ctrl_ext = IXGBE_READ_REG(&adapter->hw, IXGBE_CTRL_EXT);
	ctrl_ext &= ~IXGBE_CTRL_EXT_DRV_LOAD;
	IXGBE_WRITE_REG(&adapter->hw, IXGBE_CTRL_EXT, ctrl_ext);

	ixgbe_free_pci_resources(adapter);
	free(adapter->mta, M_DEVBUF);

	return (0);
}

/*********************************************************************
 *  Ioctl entry point
 *
 *  ixgbe_ioctl is called when the user wants to configure the
 *  interface.
 *
 *  return 0 on success, positive on failure
 **********************************************************************/

static int
ixgbe_if_mtu_set(if_shared_ctx_t sctx, uint32_t mtu)
{
	struct adapter *adapter = DOWNCAST(sctx);

	if (mtu > IXGBE_MAX_FRAME_SIZE - ETHER_HDR_LEN)
		return (EINVAL);

	adapter->max_frame_size = mtu + ETHER_HDR_LEN + ETHER_CRC_LEN;
	return (0);
}

static int
ixgbe_if_i2c_req(if_shared_ctx_t sctx, struct ifi2creq *i2c)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct ixgbe_hw *hw = &adapter->hw;
	int i;

	for (i = 0; i < i2c->len; i++)
		hw->phy.ops.read_i2c_byte(hw, i2c->offset + i,
								  i2c->dev_addr, &i2c->data[i]);
	return (0);
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
#define IXGBE_MHADD_MFS_SHIFT 16

static void
ixgbe_if_init(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct ifnet   *ifp = adapter->hwifp;
	device_t 	dev = adapter->hwdev;
	struct ixgbe_hw *hw = &adapter->hw;
	u32		k, txdctl, mhadd, gpie;
	u32		rxdctl, rxctrl;

	INIT_DEBUGOUT("ixgbe_init_locked: begin");
	hw->adapter_stopped = FALSE;
	ixgbe_stop_adapter(hw);

        /* reprogram the RAR[0] in case user changed it. */
        ixgbe_set_rar(hw, 0, adapter->hw.mac.addr, 0, IXGBE_RAH_AV);

	/* Get the latest mac address, User can use a LAA */
	bcopy(IF_LLADDR(adapter->hwifp), hw->mac.addr,
	      IXGBE_ETH_LENGTH_OF_ADDRESS);
	ixgbe_set_rar(hw, 0, hw->mac.addr, 0, 1);
	hw->addr_ctrl.rar_used_count = 1;

	ifp->if_hwassist = 0;
	if (ifp->if_capenable & IFCAP_TSO)
		ifp->if_hwassist |= CSUM_TSO;
	if (ifp->if_capenable & IFCAP_TXCSUM) {
		ifp->if_hwassist |= (CSUM_TCP | CSUM_UDP);
#if __FreeBSD_version >= 800000
		if (hw->mac.type != ixgbe_mac_82598EB)
			ifp->if_hwassist |= CSUM_SCTP;
#endif
	}

	/* Prepare transmit descriptors and buffers */
	if (iflib_txrx_structures_setup(sctx)) {
		device_printf(dev,"Could not setup transmit/receive structures\n");
		ixgbe_if_stop(UPCAST(adapter));
		return;
	}

	ixgbe_init_hw(hw);
	ixgbe_initialize_transmit_units(adapter);

	/* Setup Multicast table */
	ixgbe_if_multi_set(sctx);

	/* Configure RX settings */
	ixgbe_initialize_receive_units(adapter);

	gpie = IXGBE_READ_REG(&adapter->hw, IXGBE_GPIE);

	/* Enable Fan Failure Interrupt */
	gpie |= IXGBE_SDP1_GPIEN;

	/* Add for Module detection */
	if (hw->mac.type == ixgbe_mac_82599EB)
		gpie |= IXGBE_SDP2_GPIEN;

	/* Thermal Failure Detection */
	if (hw->mac.type == ixgbe_mac_X540)
		gpie |= IXGBE_SDP0_GPIEN;

	if (adapter->msix > 1) {
		/* Enable Enhanced MSIX mode */
		gpie |= IXGBE_GPIE_MSIX_MODE;
		gpie |= IXGBE_GPIE_EIAME | IXGBE_GPIE_PBA_SUPPORT |
		    IXGBE_GPIE_OCD;
	}
	IXGBE_WRITE_REG(hw, IXGBE_GPIE, gpie);

	/* Set MTU size */
	if (ifp->if_mtu > ETHERMTU) {
		mhadd = IXGBE_READ_REG(hw, IXGBE_MHADD);
		mhadd &= ~IXGBE_MHADD_MFS_MASK;
		mhadd |= adapter->max_frame_size << IXGBE_MHADD_MFS_SHIFT;
		IXGBE_WRITE_REG(hw, IXGBE_MHADD, mhadd);
	}
	
	/* Now enable all the queues */

	for (int i = 0; i < adapter->num_queues; i++) {
		txdctl = IXGBE_READ_REG(hw, IXGBE_TXDCTL(i));
		txdctl |= IXGBE_TXDCTL_ENABLE;
		/* Set WTHRESH to 8, burst writeback */
		txdctl |= (8 << 16);
		/*
		 * When the internal queue falls below PTHRESH (32),
		 * start prefetching as long as there are at least
		 * HTHRESH (1) buffers ready. The values are taken
		 * from the Intel linux driver 3.8.21.
		 * Prefetching enables tx line rate even with 1 queue.
		 */
		txdctl |= (32 << 0) | (1 << 8);
		IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(i), txdctl);
	}

	for (int i = 0; i < adapter->num_queues; i++) {
		rxdctl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
		if (hw->mac.type == ixgbe_mac_82598EB) {
			/*
			** PTHRESH = 21
			** HTHRESH = 4
			** WTHRESH = 8
			*/
			rxdctl &= ~0x3FFFFF;
			rxdctl |= 0x080420;
		}
		rxdctl |= IXGBE_RXDCTL_ENABLE;
		IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(i), rxdctl);
		for (k = 0; k < 10; k++) {
			if (IXGBE_READ_REG(hw, IXGBE_RXDCTL(i)) &
			    IXGBE_RXDCTL_ENABLE)
				break;
			else
				msec_delay(1);
		}
		wmb();
#ifdef DEV_NETMAP
		/*
		 * In netmap mode, we must preserve the buffers made
		 * available to userspace before the if_init()
		 * (this is true by default on the TX side, because
		 * init makes all buffers available to userspace).
		 *
		 * netmap_reset() and the device specific routines
		 * (e.g. ixgbe_setup_receive_rings()) map these
		 * buffers at the end of the NIC ring, so here we
		 * must set the RDT (tail) register to make sure
		 * they are not overwritten.
		 *
		 * In this driver the NIC ring starts at RDH = 0,
		 * RDT points to the last slot available for reception (?),
		 * so RDT = num_rx_desc - 1 means the whole ring is available.
		 */
		if (ifp->if_capenable & IFCAP_NETMAP) {
			struct netmap_adapter *na = NA(adapter->hwifp);
			struct netmap_kring *kring = &na->rx_rings[i];
			int t = na->num_rx_desc - 1 - nm_kr_rxspace(kring);

			IXGBE_WRITE_REG(hw, IXGBE_RDT(i), t);
		} else
#endif /* DEV_NETMAP */
		IXGBE_WRITE_REG(hw, IXGBE_RDT(i), adapter->num_rx_desc - 1);
	}

	/* Enable Receive engine */
	rxctrl = IXGBE_READ_REG(hw, IXGBE_RXCTRL);
	if (hw->mac.type == ixgbe_mac_82598EB)
		rxctrl |= IXGBE_RXCTRL_DMBYPS;
	rxctrl |= IXGBE_RXCTRL_RXEN;
	ixgbe_enable_rx_dma(hw, rxctrl);

	/* Set up MSI/X routing */
	if (ixgbe_enable_msix)  {
		ixgbe_configure_ivars(adapter);
		/* Set up auto-mask */
		if (hw->mac.type == ixgbe_mac_82598EB)
			IXGBE_WRITE_REG(hw, IXGBE_EIAM, IXGBE_EICS_RTX_QUEUE);
		else {
			IXGBE_WRITE_REG(hw, IXGBE_EIAM_EX(0), 0xFFFFFFFF);
			IXGBE_WRITE_REG(hw, IXGBE_EIAM_EX(1), 0xFFFFFFFF);
		}
	} else {  /* Simple settings for Legacy/MSI */
                ixgbe_set_ivar(adapter, 0, 0, 0);
                ixgbe_set_ivar(adapter, 0, 0, 1);
		IXGBE_WRITE_REG(hw, IXGBE_EIAM, IXGBE_EICS_RTX_QUEUE);
	}

#ifdef IXGBE_FDIR
	/* Init Flow director */
	if (hw->mac.type != ixgbe_mac_82598EB) {
		u32 hdrm = 32 << fdir_pballoc;

		hw->mac.ops.setup_rxpba(hw, 0, hdrm, PBA_STRATEGY_EQUAL);
		ixgbe_init_fdir_signature_82599(&adapter->hw, fdir_pballoc);
	}
#endif

	/*
	** Check on any SFP devices that
	** need to be kick-started
	*/
	if (hw->phy.type == ixgbe_phy_none) {
		int err = hw->phy.ops.identify(hw);
		if (err == IXGBE_ERR_SFP_NOT_SUPPORTED) {
                	device_printf(dev,
			    "Unsupported SFP+ module type was detected.\n");
			return;
        	}
	}

	/* Set moderation on the Link interrupt */
	IXGBE_WRITE_REG(hw, IXGBE_EITR(adapter->linkvec), IXGBE_LINK_ITR);

	/* Config/Enable Link */
	ixgbe_config_link(adapter);

	/* Hardware Packet Buffer & Flow Control setup */
	{
		u32 rxpb, frame, size, tmp;

		frame = adapter->max_frame_size;

		/* Calculate High Water */
		if (hw->mac.type == ixgbe_mac_X540)
			tmp = IXGBE_DV_X540(frame, frame);
		else
			tmp = IXGBE_DV(frame, frame);
		size = IXGBE_BT2KB(tmp);
		rxpb = IXGBE_READ_REG(hw, IXGBE_RXPBSIZE(0)) >> 10;
		hw->fc.high_water[0] = rxpb - size;

		/* Now calculate Low Water */
		if (hw->mac.type == ixgbe_mac_X540)
			tmp = IXGBE_LOW_DV_X540(frame);
		else
			tmp = IXGBE_LOW_DV(frame);
		hw->fc.low_water[0] = IXGBE_BT2KB(tmp);
		
		hw->fc.requested_mode = adapter->fc;
		hw->fc.pause_time = IXGBE_FC_PAUSE;
		hw->fc.send_xon = TRUE;
	}
	/* Initialize the FC settings */
	ixgbe_start_hw(hw);

	/* Set up VLAN support and filter */
	ixgbe_setup_vlan_hw_support(adapter);
}

/*
**
** MSIX Interrupt Handlers and Tasklets
**
*/

static void
ixgbe_if_rx_intr_enable(if_shared_ctx_t sctx, uint32_t qid)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct ixgbe_hw *hw = &adapter->hw;
	uint32_t vector = adapter->queues[qid].msix;
	u64	queue = (u64)(1 << vector);
	u32	mask;

	if (hw->mac.type == ixgbe_mac_82598EB) {
		mask = (IXGBE_EIMS_RTX_QUEUE & queue);
		IXGBE_WRITE_REG(hw, IXGBE_EIMS, mask);
	} else {
		mask = (queue & 0xFFFFFFFF);
		if (mask)
			IXGBE_WRITE_REG(hw, IXGBE_EIMS_EX(0), mask);
		mask = (queue >> 32);
		if (mask)
			IXGBE_WRITE_REG(hw, IXGBE_EIMS_EX(1), mask);
	}
}

static inline void
ixgbe_disable_queue(struct adapter *adapter, u32 vector)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u64	queue = (u64)(1 << vector);
	u32	mask;

	if (hw->mac.type == ixgbe_mac_82598EB) {
                mask = (IXGBE_EIMS_RTX_QUEUE & queue);
                IXGBE_WRITE_REG(hw, IXGBE_EIMC, mask);
	} else {
                mask = (queue & 0xFFFFFFFF);
                if (mask)
                        IXGBE_WRITE_REG(hw, IXGBE_EIMC_EX(0), mask);
                mask = (queue >> 32);
                if (mask)
                        IXGBE_WRITE_REG(hw, IXGBE_EIMC_EX(1), mask);
	}
}

/*********************************************************************
 *
 *  Legacy Interrupt Service routine
 *
 **********************************************************************/

static int
ixgbe_legacy_irq(void *arg)
{
	struct ix_queue *que = arg;
	struct adapter	*adapter = que->adapter;
	struct ixgbe_hw	*hw = &adapter->hw;
	u32       	reg_eicr;

	reg_eicr = IXGBE_READ_REG(hw, IXGBE_EICR);

	++que->irqs;
	if (reg_eicr == 0) {
		ixgbe_if_intr_enable(UPCAST(adapter));
		return (FILTER_STRAY);
	}
	/*
	 * Process newly arrived packets
	 */
	iflib_rx_intr_deferred(UPCAST(adapter), 0);

	/*
	 * Free completed packets and send enqueued ones
	 */
	iflib_tx_intr_deferred(UPCAST(adapter), 0);

	/* Check for fan failure */
	if ((hw->phy.media_type == ixgbe_media_type_copper) &&
	    (reg_eicr & IXGBE_EICR_GPI_SDP1)) {
                device_printf(adapter->hwdev, "\nCRITICAL: FAN FAILURE!! "
		    "REPLACE IMMEDIATELY!!\n");
		IXGBE_WRITE_REG(hw, IXGBE_EIMS, IXGBE_EICR_GPI_SDP1);
	}

	/* Link status change */
	if (reg_eicr & IXGBE_EICR_LSC)
		iflib_link_intr_deferred(UPCAST(adapter));

	return (FILTER_HANDLED);
}

/*********************************************************************
 *
 *  MSIX Queue Interrupt Service routine
 *
 **********************************************************************/
static int
ixgbe_msix_que(void *arg)
{
	struct ix_queue	*que = arg;
	struct adapter  *adapter = que->adapter;
	struct tx_ring	*txr = que->txr;
	struct rx_ring	*rxr = que->rxr;
	u32		newitr = 0;

	ixgbe_disable_queue(adapter, que->msix);
	++que->irqs;

	/* Do AIM now? */

	if (ixgbe_enable_aim == FALSE)
		goto no_calc;
	/*
	** Do Adaptive Interrupt Moderation:
	**  - Write out last calculated setting
	**  - Calculate based on average size over
	**    the last interval.
	*/
	if (que->eitr_setting)
		IXGBE_WRITE_REG(&adapter->hw,
						IXGBE_EITR(que->msix), que->eitr_setting);

	que->eitr_setting = 0;

	/* Idle, do nothing */
	if ((txr->bytes == 0) && (rxr->bytes == 0))
		goto no_calc;
                                
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

	if (adapter->hw.mac.type == ixgbe_mac_82598EB)
		newitr |= newitr << 16;
	else
		newitr |= IXGBE_EITR_CNT_WDIS;

	/* save for next interrupt */
	que->eitr_setting = newitr;

	/* Reset state */
	txr->bytes = 0;
	txr->packets = 0;
	rxr->bytes = 0;
	rxr->packets = 0;

no_calc:
	return (FILTER_SCHEDULE_THREAD);
}

static int
ixgbe_msix_link(void *arg)
{
	struct adapter	*adapter = arg;
	struct ixgbe_hw *hw = &adapter->hw;
	u32		reg_eicr;

	++adapter->link_irq;

	/* First get the cause */
	reg_eicr = IXGBE_READ_REG(hw, IXGBE_EICS);
	/* Be sure the queue bits are not cleared */
	reg_eicr &= ~IXGBE_EICR_RTX_QUEUE;
	/* Clear interrupt with write */
	IXGBE_WRITE_REG(hw, IXGBE_EICR, reg_eicr);

	/* Link status change */
	if (reg_eicr & IXGBE_EICR_LSC)
		iflib_link_intr_deferred(UPCAST(adapter));

	if (adapter->hw.mac.type != ixgbe_mac_82598EB) {
#ifdef IXGBE_FDIR
		if (reg_eicr & IXGBE_EICR_FLOW_DIR) {
			/* This is probably overkill :) */
			if (!atomic_cmpset_int(&adapter->fdir_reinit, 0, 1))
				return (FILTER_HANDLED);
                	/* Disable the interrupt */
			IXGBE_WRITE_REG(hw, IXGBE_EIMC, IXGBE_EICR_FLOW_DIR);
			GROUPTASK_ENQUEUE(&adapter->fdir_task);
		} else
#endif
		if (reg_eicr & IXGBE_EICR_ECC) {
			device_printf(adapter->hwdev, "\nCRITICAL: ECC ERROR!! "
			    "Please Reboot!!\n");
			IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_ECC);
		} else

		if (reg_eicr & IXGBE_EICR_GPI_SDP1) {
                	/* Clear the interrupt */
			IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_GPI_SDP1);
			GROUPTASK_ENQUEUE(&adapter->msf_task);
        	} else if (reg_eicr & IXGBE_EICR_GPI_SDP2) {
                	/* Clear the interrupt */
                	IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_GPI_SDP2);
					GROUPTASK_ENQUEUE(&adapter->mod_task);
		}
        } 

	/* Check for fan failure */
	if ((hw->device_id == IXGBE_DEV_ID_82598AT) &&
	    (reg_eicr & IXGBE_EICR_GPI_SDP1)) {
                device_printf(adapter->hwdev, "\nCRITICAL: FAN FAILURE!! "
		    "REPLACE IMMEDIATELY!!\n");
		IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_GPI_SDP1);
	}

	/* Check for over temp condition */
	if ((hw->mac.type == ixgbe_mac_X540) &&
	    (reg_eicr & IXGBE_EICR_TS)) {
                device_printf(adapter->hwdev, "\nCRITICAL: OVER TEMP!! "
		    "PHY IS SHUT DOWN!!\n");
                device_printf(adapter->hwdev, "System shutdown required\n");
		IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_TS);
	}

	IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMS, IXGBE_EIMS_OTHER);
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
ixgbe_if_media_status(if_shared_ctx_t sctx, struct ifmediareq * ifmr)
{
	struct adapter *adapter = DOWNCAST(sctx);

	INIT_DEBUGOUT("ixgbe_media_status: begin");
	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (!adapter->link_active)
		return;

	ifmr->ifm_status |= IFM_ACTIVE;

	switch (adapter->link_speed) {
		case IXGBE_LINK_SPEED_100_FULL:
			ifmr->ifm_active |= IFM_100_TX | IFM_FDX;
			break;
		case IXGBE_LINK_SPEED_1GB_FULL:
			ifmr->ifm_active |= IFM_1000_SX | IFM_FDX;
			break;
		case IXGBE_LINK_SPEED_10GB_FULL:
			ifmr->ifm_active |= adapter->optics | IFM_FDX;
			break;
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
ixgbe_if_media_change(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct ifmedia *ifm = &adapter->media;

	INIT_DEBUGOUT("ixgbe_media_change: begin");

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

        switch (IFM_SUBTYPE(ifm->ifm_media)) {
        case IFM_AUTO:
                adapter->hw.phy.autoneg_advertised =
		    IXGBE_LINK_SPEED_100_FULL |
		    IXGBE_LINK_SPEED_1GB_FULL |
		    IXGBE_LINK_SPEED_10GB_FULL;
                break;
        default:
                device_printf(adapter->hwdev, "Only auto media type\n");
		return (EINVAL);
        }

	return (0);
}

/*********************************************************************
 *
 *  This routine maps the mbufs to tx descriptors, allowing the
 *  TX engine to transmit the packets. 
 *  	- return 0 on success, positive on failure
 *
 **********************************************************************/

static int
ixgbe_isc_txd_encap(if_shared_ctx_t sctx, uint32_t txqid, if_pkt_info_t pi)
{
	struct adapter  *adapter = DOWNCAST(sctx);
	struct tx_ring *txr = &adapter->tx_rings[txqid];
	u32		olinfo_status = 0, cmd_type_len;
	int             i, j, error, nsegs;
	int		first;
	bus_dma_segment_t *segs = pi->ipi_segs;
	struct mbuf *m_head = pi->ipi_m;
	struct ixgbe_tx_buf *txbuf;
	union ixgbe_adv_tx_desc *txd = NULL;

	nsegs = pi->ipi_nsegs;
	/* Basic descriptor defines */
        cmd_type_len = (IXGBE_ADVTXD_DTYP_DATA |
	    IXGBE_ADVTXD_DCMD_IFCS | IXGBE_ADVTXD_DCMD_DEXT);

	if (m_head->m_flags & M_VLANTAG)
        	cmd_type_len |= IXGBE_ADVTXD_DCMD_VLE;

        /*
         * Important to capture the first descriptor
         * used because it will contain the index of
         * the one we tell the hardware to report back
         */
	first = pi->ipi_pidx;
	txbuf = &txr->tx_buffers[first];

	/*
	** Set up the appropriate offload context
	** this will consume the first descriptor
	*/
	if ((error = ixgbe_tx_ctx_setup(txr, m_head, &cmd_type_len, &olinfo_status)) != 0)
		return (error);

#ifdef IXGBE_FDIR
	/* Do the flow director magic */
	if ((txr->atr_sample) && (!adapter->fdir_reinit)) {
		++txr->atr_count;
		if (txr->atr_count >= atr_sample_rate) {
			ixgbe_atr(txr, m_head);
			txr->atr_count = 0;
		}
	}
#endif

	i = first;
	for (j = 0; j < nsegs; j++) {
		bus_size_t seglen;
		bus_addr_t segaddr;

		txd = &txr->tx_base[i];
		seglen = segs[j].ds_len;
		segaddr = htole64(segs[j].ds_addr);

		txd->read.buffer_addr = segaddr;
		txd->read.cmd_type_len = htole32(txr->txd_cmd |
		    cmd_type_len |seglen);
		txd->read.olinfo_status = htole32(olinfo_status);

		if (++i == txr->num_desc)
			i = 0;
	}
	txr->bytes += pi->ipi_m->m_pkthdr.len;
	txr->packets++;
	txd->read.cmd_type_len |=
	    htole32(IXGBE_TXD_CMD_EOP | IXGBE_TXD_CMD_RS);
	pi->ipi_new_pidx = i;

	/* Set the EOP descriptor that will be marked done */
	txbuf = &txr->tx_buffers[first];
	txbuf->eop = txd;
	++txr->total_packets;

	return (0);
}

static void
ixgbe_isc_txd_flush(if_shared_ctx_t sctx, uint32_t txqid, uint32_t pidx)
{
	/*
	 * Advance the Transmit Descriptor Tail (Tdt), this tells the
	 * hardware that this frame is available to transmit.
	 */
	IXGBE_WRITE_REG(&DOWNCAST(sctx)->hw, IXGBE_TDT(txqid), pidx);
}

static void
ixgbe_if_promisc_set(if_shared_ctx_t sctx, int flags)
{
	struct adapter *adapter = DOWNCAST(sctx);
	u_int32_t       reg_rctl;
	int mcnt = MAX_NUM_MULTICAST_ADDRESSES;

	reg_rctl = IXGBE_READ_REG(&adapter->hw, IXGBE_FCTRL);
	reg_rctl &= (~IXGBE_FCTRL_UPE);
	if ((flags & IFF_ALLMULTI) == 0)
		mcnt = if_multiaddr_count(sctx->isc_ifp, MAX_NUM_MULTICAST_ADDRESSES);
	if (mcnt < MAX_NUM_MULTICAST_ADDRESSES)
		reg_rctl &= (~IXGBE_FCTRL_MPE);
	IXGBE_WRITE_REG(&adapter->hw, IXGBE_FCTRL, reg_rctl);

	if (flags & IFF_PROMISC) {
		reg_rctl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_FCTRL, reg_rctl);
	} else if (flags & IFF_ALLMULTI) {
		reg_rctl |= IXGBE_FCTRL_MPE;
		reg_rctl &= ~IXGBE_FCTRL_UPE;
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_FCTRL, reg_rctl);
	}
}


/*********************************************************************
 *  Multicast Update
 *
 *  This routine is called whenever multicast address list is updated.
 *
 **********************************************************************/
#define IXGBE_RAR_ENTRIES 16

static void
ixgbe_if_multi_set(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	u32	fctrl;
	u8	*mta;
	u8	*update_ptr;
	struct ifnet *ifp = sctx->isc_ifp;
	int	mcnt = 0;

	IOCTL_DEBUGOUT("ixgbe_set_multi: begin");

	mta = adapter->mta;
	bzero(mta, sizeof(u8) * IXGBE_ETH_LENGTH_OF_ADDRESS *
	    MAX_NUM_MULTICAST_ADDRESSES);
	mcnt = if_multiaddr_count(sctx->isc_ifp, MAX_NUM_MULTICAST_ADDRESSES);

	fctrl = IXGBE_READ_REG(&adapter->hw, IXGBE_FCTRL);
	fctrl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
	if (ifp->if_flags & IFF_PROMISC)
		fctrl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
	else if (mcnt >= MAX_NUM_MULTICAST_ADDRESSES ||
	    ifp->if_flags & IFF_ALLMULTI) {
		fctrl |= IXGBE_FCTRL_MPE;
		fctrl &= ~IXGBE_FCTRL_UPE;
	} else
		fctrl &= ~(IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
	
	IXGBE_WRITE_REG(&adapter->hw, IXGBE_FCTRL, fctrl);

	if (mcnt < MAX_NUM_MULTICAST_ADDRESSES) {
		update_ptr = mta;
		ixgbe_update_mc_addr_list(&adapter->hw,
		    update_ptr, mcnt, ixgbe_mc_array_itr, TRUE);
	}

	return;
}

/*
 * This is an iterator function now needed by the multicast
 * shared code. It simply feeds the shared code routine the
 * addresses in the array of ixgbe_set_multi() one by one.
 */
static u8 *
ixgbe_mc_array_itr(struct ixgbe_hw *hw, u8 **update_ptr, u32 *vmdq)
{
	u8 *addr = *update_ptr;
	u8 *newptr;
	*vmdq = 0;

	newptr = addr + IXGBE_ETH_LENGTH_OF_ADDRESS;
	*update_ptr = newptr;
	return addr;
}


/*********************************************************************
 *  Timer routine
 *
 *  This routine checks for link status,updates statistics,
 *  and runs the watchdog check.
 *
 **********************************************************************/

static void
ixgbe_if_timer(if_shared_ctx_t sctx)
{
	struct adapter	*adapter = DOWNCAST(sctx);

	/* Check for pluggable optics */
	if (adapter->sfp_probe)
		if (!ixgbe_sfp_probe(adapter))
			return;
	ixgbe_if_update_link_status(sctx);
	ixgbe_update_stats_counters(adapter);

}

static void
ixgbe_if_watchdog_reset(if_shared_ctx_t sctx)
{
	device_t dev = sctx->isc_dev;

	device_printf(dev, "Watchdog timeout -- resetting\n");
#if 0
	device_printf(dev,"Queue(%d) tdh = %d, hw tdt = %d\n", txr->me,
	    IXGBE_READ_REG(&adapter->hw, IXGBE_TDH(txr->me)),
	    IXGBE_READ_REG(&adapter->hw, IXGBE_TDT(txr->me)));
	device_printf(dev,"TX(%d) desc avail = %d,"
	    "Next TX to Clean = %d\n",
				  txr->me, txr->tx_avail, txr->next_to_clean);
#endif
}

/*
** Note: this routine updates the OS on the link state
**	the real check of the hardware only happens with
**	a link interrupt.
*/
static void
ixgbe_if_update_link_status(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct ifnet	*ifp = adapter->hwifp;
	device_t dev = adapter->hwdev;

	if (adapter->link_up){ 
		if (adapter->link_active == FALSE) {
			if (bootverbose)
				device_printf(dev,"Link is up %d Gbps %s \n",
				    ((adapter->link_speed == 128)? 10:1),
				    "Full Duplex");
			adapter->link_active = TRUE;
			/* Update any Flow Control changes */
			ixgbe_fc_enable(&adapter->hw);
			if_link_state_change(ifp, LINK_STATE_UP);
		}
	} else { /* Link down */
		if (adapter->link_active == TRUE) {
			if (bootverbose)
				device_printf(dev,"Link is Down\n");
			if_link_state_change(ifp, LINK_STATE_DOWN);
			adapter->link_active = FALSE;
		}
	}
}

/*********************************************************************
 *
 *  This routine disables all traffic on the adapter by issuing a
 *  global reset on the MAC and deallocates TX/RX buffers.
 *
 **********************************************************************/

static void
ixgbe_if_stop(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct ixgbe_hw *hw = &adapter->hw;

	INIT_DEBUGOUT("ixgbe_stop: begin\n");

	ixgbe_reset_hw(hw);
	hw->adapter_stopped = FALSE;
	ixgbe_stop_adapter(hw);
	if (hw->mac.type == ixgbe_mac_82599EB)
		ixgbe_stop_mac_link_on_d3_82599(hw);
	/* Turn off the laser - noop with no optics */
	ixgbe_disable_tx_laser(hw);

	/* Update the stack */
	adapter->link_up = FALSE;
	ixgbe_if_update_link_status(UPCAST(adapter));

	/* reprogram the RAR[0] in case user changed it. */
	ixgbe_set_rar(&adapter->hw, 0, adapter->hw.mac.addr, 0, IXGBE_RAH_AV);

	return;
}


/*********************************************************************
 *
 *  Determine hardware revision.
 *
 **********************************************************************/
static void
ixgbe_identify_hardware(struct adapter *adapter)
{
	device_t        dev = adapter->hwdev;
	struct ixgbe_hw *hw = &adapter->hw;

	/* Save off the information about this board */
	hw->vendor_id = pci_get_vendor(dev);
	hw->device_id = pci_get_device(dev);
	hw->revision_id = pci_read_config(dev, PCIR_REVID, 1);
	hw->subsystem_vendor_id =
	    pci_read_config(dev, PCIR_SUBVEND_0, 2);
	hw->subsystem_device_id =
	    pci_read_config(dev, PCIR_SUBDEV_0, 2);

	/* We need this here to set the num_segs below */
	ixgbe_set_mac_type(hw);

	/* Pick up the 82599 and VF settings */
	if (hw->mac.type != ixgbe_mac_82598EB) {
		hw->phy.smart_speed = ixgbe_smart_speed;
		adapter->num_segs = IXGBE_82599_SCATTER;
	} else
		adapter->num_segs = IXGBE_82598_SCATTER;

	return;
}

/*********************************************************************
 *
 *  Determine optic type
 *
 **********************************************************************/
static void
ixgbe_setup_optics(struct adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int		layer;

	layer = ixgbe_get_supported_physical_layer(hw);

	if (layer & IXGBE_PHYSICAL_LAYER_10GBASE_T) {
		adapter->optics = IFM_10G_T;
		return;
	}

	if (layer & IXGBE_PHYSICAL_LAYER_1000BASE_T) {
		adapter->optics = IFM_1000_T;
		return;
	}

	if (layer & IXGBE_PHYSICAL_LAYER_1000BASE_SX) {
		adapter->optics = IFM_1000_SX;
		return;
	}

	if (layer & (IXGBE_PHYSICAL_LAYER_10GBASE_LR |
	    IXGBE_PHYSICAL_LAYER_10GBASE_LRM)) {
		adapter->optics = IFM_10G_LR;
		return;
	}

	if (layer & IXGBE_PHYSICAL_LAYER_10GBASE_SR) {
		adapter->optics = IFM_10G_SR;
		return;
	}

	if (layer & IXGBE_PHYSICAL_LAYER_SFP_PLUS_CU) {
		adapter->optics = IFM_10G_TWINAX;
		return;
	}

	if (layer & (IXGBE_PHYSICAL_LAYER_10GBASE_KX4 |
	    IXGBE_PHYSICAL_LAYER_10GBASE_CX4)) {
		adapter->optics = IFM_10G_CX4;
		return;
	}

	/* If we get here just set the default */
	adapter->optics = IFM_ETHER | IFM_AUTO;
	return;
}

/*********************************************************************
 *
 *  Setup the Legacy or MSI Interrupt handler
 *
 **********************************************************************/
static int
ixgbe_allocate_legacy(struct adapter *adapter)
{
	int		rid = 0;

	/* MSI RID at 1 */
	if (adapter->msix == 1)
		rid = 1;

	/* Tasklets for Link, SFP and Multispeed Fiber */
	GROUPTASK_INIT(&adapter->link_task, 0, ixgbe_handle_link, adapter);
	iflib_taskqgroup_attach(&adapter->link_task, adapter, "link");
	GROUPTASK_INIT(&adapter->mod_task, 0, ixgbe_handle_mod, adapter);
	iflib_taskqgroup_attach(&adapter->mod_task, adapter, "mod");
	GROUPTASK_INIT(&adapter->msf_task, 0, ixgbe_handle_msf, adapter);
	iflib_taskqgroup_attach(&adapter->msf_task, adapter, "msf");

#ifdef IXGBE_FDIR
	GROUPTASK_INIT(&adapter->fdir_task, 0, ixgbe_reinit_fdir, adapter);
	iflib_taskqgroup_attach(&adapter->fdir_task, adapter, "fdir");
#endif
	/* For simplicity in the handlers */
	adapter->que_mask = IXGBE_EIMS_ENABLE_MASK;

	return (iflib_legacy_setup(UPCAST(adapter), ixgbe_legacy_irq, &rid));
}


/*********************************************************************
 *
 *  Setup MSIX Interrupt resources and handlers 
 *
 **********************************************************************/
static int
ixgbe_allocate_msix(struct adapter *adapter)
{
	device_t        dev = adapter->hwdev;
	struct 		ix_queue *que = adapter->queues;
	struct  	tx_ring *txr = adapter->tx_rings;
	int 		error, rid, vector = 0;
	char buf[16];

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

	for (int i = 0; i < adapter->num_queues; i++, vector++, que++, txr++) {
		rid = vector + 1;

		snprintf(buf, sizeof(buf), "rx %d", i);
		error = iflib_irq_alloc_generic(UPCAST(adapter), &que->rxr->rx_irq, rid,
									  IFLIB_INTR_RX, ixgbe_msix_que, que,
										que->rxr->me, buf);
		if (error) {
			device_printf(dev,"Unable to allocate"
		    	    " bus resource: que interrupt [%d]\n", vector);
			return (error);
		}
		que->msix = vector;
		adapter->que_mask |= (u64)(1 << que->msix);
	}

	/* and Link */
	rid = vector + 1;
	error = iflib_irq_alloc_generic(UPCAST(adapter), &que->rxr->rx_irq, rid,
								  IFLIB_INTR_LINK, ixgbe_msix_link,
								  adapter, que->rxr->me, buf);

	/* XXX check err */
	adapter->linkvec = vector;
	/* Tasklets for Link, SFP and Multispeed Fiber */
	/* Tasklets for Link, SFP and Multispeed Fiber */
	GROUPTASK_INIT(&adapter->link_task, 0, ixgbe_handle_link, adapter);
	iflib_taskqgroup_attach(&adapter->link_task, adapter, "link");
	GROUPTASK_INIT(&adapter->mod_task, 0, ixgbe_handle_mod, adapter);
	iflib_taskqgroup_attach(&adapter->mod_task, adapter, "mod");
	GROUPTASK_INIT(&adapter->msf_task, 0, ixgbe_handle_msf, adapter);
	iflib_taskqgroup_attach(&adapter->msf_task, adapter, "msf");
#ifdef IXGBE_FDIR
	GROUPTASK_INIT(&adapter->fdir_task, 0, ixgbe_reinit_fdir, adapter);
	iflib_taskqgroup_attach(&adapter->fdir_task, adapter, "fdir");
#endif
	return (0);
}

/*
 * Setup Either MSI/X or MSI
 */
static int
ixgbe_setup_msix(struct adapter *adapter)
{
	device_t dev = adapter->hwdev;
	int rid, want, queues, msgs;

	/* Override by tuneable */
	if (ixgbe_enable_msix == 0)
		goto msi;

	/* First try MSI/X */
	msgs = pci_msix_count(dev); 
	if (msgs == 0)
		goto msi;
	rid = PCIR_BAR(MSIX_82598_BAR);
	adapter->msix_mem = bus_alloc_resource_any(dev,
	    SYS_RES_MEMORY, &rid, RF_ACTIVE);
       	if (adapter->msix_mem == NULL) {
		rid += 4;	/* 82599 maps in higher BAR */
		adapter->msix_mem = bus_alloc_resource_any(dev,
		    SYS_RES_MEMORY, &rid, RF_ACTIVE);
	}
       	if (adapter->msix_mem == NULL) {
			/* May not be enabled */
			device_printf(adapter->hwdev,
		    "Unable to map MSIX table \n");
		goto msi;
	}

	/* Figure out a reasonable auto config value */
	queues = (mp_ncpus > (msgs-1)) ? (msgs-1) : mp_ncpus;
#ifdef	RSS
	/* If we're doing RSS, clamp at the number of RSS buckets */
	if (queues > rss_getnumbuckets())
		queues = rss_getnumbuckets();
#endif

	if (ixgbe_num_queues != 0)
		queues = ixgbe_num_queues;
	/* Set max queues to 8 when autoconfiguring */
	else if ((ixgbe_num_queues == 0) && (queues > 8))
		queues = 8;

	/* reflect correct sysctl value */
	ixgbe_num_queues = queues;

	/*
	** Want one vector (RX/TX pair) per queue
	** plus an additional for Link.
	*/
	want = queues + 1;
	if (msgs >= want)
		msgs = want;
	else {
		device_printf(adapter->hwdev,
		    "MSIX Configuration Problem, "
		    "%d vectors but %d queues wanted!\n",
		    msgs, want);
		goto msi;
	}
	if ((pci_alloc_msix(dev, &msgs) == 0) && (msgs == want)) {
		device_printf(adapter->hwdev,
		    "Using MSIX interrupts with %d vectors\n", msgs);
		adapter->num_queues = queues;
		return (msgs);
	}
	/*
	** If MSIX alloc failed or provided us with
	** less than needed, free and fall through to MSI
	*/
	pci_release_msi(dev);

msi:
       	if (adapter->msix_mem != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rid, adapter->msix_mem);
		adapter->msix_mem = NULL;
	}
       	msgs = 1;
       	if (pci_alloc_msi(dev, &msgs) == 0) {
			device_printf(adapter->hwdev,"Using an MSI interrupt\n");
		return (msgs);
	}
	device_printf(adapter->hwdev,"Using a Legacy interrupt\n");
	return (0);
}


static int
ixgbe_allocate_pci_resources(struct adapter *adapter)
{
	int             rid;
	device_t        dev = adapter->hwdev;

	rid = PCIR_BAR(0);
	adapter->pci_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);

	if (!(adapter->pci_mem)) {
		device_printf(dev,"Unable to allocate bus resource: memory\n");
		return (ENXIO);
	}

	adapter->osdep.mem_bus_space_tag =
		rman_get_bustag(adapter->pci_mem);
	adapter->osdep.mem_bus_space_handle =
		rman_get_bushandle(adapter->pci_mem);
	adapter->hw.hw_addr = (u8 *) &adapter->osdep.mem_bus_space_handle;

	/* Legacy defaults */
	adapter->num_queues = 1;
	adapter->hw.back = &adapter->osdep;

	/*
	** Now setup MSI or MSI/X, should
	** return us the number of supported
	** vectors. (Will be 1 for MSI)
	*/
	adapter->msix = ixgbe_setup_msix(adapter);
	return (0);
}

static void
ixgbe_free_pci_resources(struct adapter * adapter)
{
	struct 		ix_queue *que = adapter->queues;
	device_t	dev = adapter->hwdev;
	int		rid, memrid;

	if (adapter->hw.mac.type == ixgbe_mac_82598EB)
		memrid = PCIR_BAR(MSIX_82598_BAR);
	else
		memrid = PCIR_BAR(MSIX_82599_BAR);

	/*
	** There is a slight possibility of a failure mode
	** in attach that will result in entering this function
	** before interrupt resources have been initialized, and
	** in that case we do not want to execute the loops below
	** We can detect this reliably by the state of the adapter
	** res pointer.
	*/
	if (adapter->res == NULL)
		goto mem;

	/*
	**  Release all msix queue resources:
	*/
	for (int i = 0; i < adapter->num_queues; i++, que++) {
		rid = que->msix + 1;
		if (que->tag != NULL) {
			bus_teardown_intr(dev, que->res, que->tag);
			que->tag = NULL;
		}
		if (que->res != NULL)
			bus_release_resource(dev, SYS_RES_IRQ, rid, que->res);
	}


	/* Clean the Legacy or Link interrupt last */
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

mem:
	if (adapter->msix)
		pci_release_msi(dev);

	if (adapter->msix_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    memrid, adapter->msix_mem);

	if (adapter->pci_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(0), adapter->pci_mem);

	return;
}

/*********************************************************************
 *
 *  Setup networking device structure and register an interface.
 *
 **********************************************************************/
static int
ixgbe_setup_interface(device_t dev, struct adapter *adapter)
{
	if_shared_ctx_t sctx = UPCAST(adapter);
	struct ixgbe_hw *hw = &adapter->hw;
	struct ifnet   *ifp = adapter->hwifp;

	INIT_DEBUGOUT("ixgbe_setup_interface: begin");

	sctx->isc_q_align = DBA_ALIGN;
	sctx->isc_tx_maxsize = IXGBE_TSO_SIZE;
	sctx->isc_tx_nsegments = adapter->num_segs;
	sctx->isc_tx_maxsegsize = PAGE_SIZE;
	sctx->isc_rx_maxsize = MJUM16BYTES;
	sctx->isc_rx_nsegments = 1;
	sctx->isc_rx_maxsegsize = MJUM16BYTES;

	sctx->isc_txd_encap = ixgbe_isc_txd_encap;
	sctx->isc_txd_flush = ixgbe_isc_txd_flush;
	sctx->isc_txd_credits_update = ixgbe_isc_txd_credits_update;

	sctx->isc_rxd_available = ixgbe_isc_rxd_available;
	sctx->isc_rxd_pkt_get = ixgbe_isc_rxd_pkt_get;
	sctx->isc_rxd_refill = ixgbe_isc_rxd_refill;
	sctx->isc_rxd_flush = ixgbe_isc_rxd_flush;

	/*
	 * Tell the upper layer(s) we support long frames.
	 */
	ifp->if_hdrlen = sizeof(struct ether_vlan_header);

	ifp->if_capabilities |= IFCAP_HWCSUM | IFCAP_TSO | IFCAP_VLAN_HWCSUM;
	ifp->if_capabilities |= IFCAP_JUMBO_MTU;
	ifp->if_capabilities |= IFCAP_LRO;
	ifp->if_capabilities |= IFCAP_VLAN_HWTAGGING
			     |  IFCAP_VLAN_HWTSO
			     |  IFCAP_VLAN_MTU
			     |  IFCAP_HWSTATS;
	ifp->if_capenable = ifp->if_capabilities;

	/*
	** Don't turn this on by default, if vlans are
	** created on another pseudo device (eg. lagg)
	** then vlan events are not passed thru, breaking
	** operation, but with HW FILTER off it works. If
	** using vlans directly on the ixgbe driver you can
	** enable this and get full hardware tag filtering.
	*/
	ifp->if_capabilities |= IFCAP_VLAN_HWFILTER;

	/*
	 * Specify the media types supported by this adapter
	 */
	ifmedia_add(&adapter->media, IFM_ETHER | adapter->optics, 0, NULL);
	ifmedia_set(&adapter->media, IFM_ETHER | adapter->optics);
	if (hw->device_id == IXGBE_DEV_ID_82598AT) {
		ifmedia_add(&adapter->media,
		    IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
		ifmedia_add(&adapter->media,
		    IFM_ETHER | IFM_1000_T, 0, NULL);
	}
	ifmedia_add(&adapter->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&adapter->media, IFM_ETHER | IFM_AUTO);

	return (0);
}

static void
ixgbe_config_link(struct adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32	autoneg, err = 0;
	bool	sfp, negotiate;

	sfp = ixgbe_is_sfp(hw);

	if (sfp) { 
		if (hw->phy.multispeed_fiber) {
			hw->mac.ops.setup_sfp(hw);
			ixgbe_enable_tx_laser(hw);
			GROUPTASK_ENQUEUE(&adapter->msf_task);
		} else
			GROUPTASK_ENQUEUE(&adapter->mod_task);
	} else {
		if (hw->mac.ops.check_link)
			err = ixgbe_check_link(hw, &adapter->link_speed,
			    &adapter->link_up, FALSE);
		if (err)
			goto out;
		autoneg = hw->phy.autoneg_advertised;
		if ((!autoneg) && (hw->mac.ops.get_link_capabilities))
                	err  = hw->mac.ops.get_link_capabilities(hw,
			    &autoneg, &negotiate);
		if (err)
			goto out;
		if (hw->mac.ops.setup_link)
                	err = hw->mac.ops.setup_link(hw,
			    autoneg, adapter->link_up);
	}
out:
	return;
}

/*********************************************************************
 *
 *  Allocate memory for the transmit and receive rings, and then
 *  the descriptors associated with each, called only once at attach.
 *
 **********************************************************************/
static int
ixgbe_if_queues_alloc(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	device_t	dev = adapter->hwdev;
	struct ix_queue	*que;
	struct tx_ring	*txr;
	struct rx_ring	*rxr;
	uint64_t addrs[2];
	int error = IXGBE_SUCCESS;
	int txconf = 0, rxconf = 0;

	/* First allocate the top level queue structs */
	if (!(adapter->queues =
		  (struct ix_queue *) malloc(sizeof(struct ix_queue) *
									 adapter->num_queues,
									 M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate queue memory\n");
		error = ENOMEM;
		goto fail;
	}

	/* First allocate the TX ring struct memory */
	if (!(adapter->tx_rings =
	    (struct tx_ring *) malloc(sizeof(struct tx_ring) *
	    adapter->num_queues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate TX ring memory\n");
		error = ENOMEM;
		goto tx_fail;
	}

	/* Next allocate the RX */
	if (!(adapter->rx_rings =
	    (struct rx_ring *) malloc(sizeof(struct rx_ring) *
	    adapter->num_queues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate RX ring memory\n");
		error = ENOMEM;
		goto rx_fail;
	}

	/*
	 * Now set up the TX queues, txconf is needed to handle the
	 * possibility that things fail midcourse and we need to
	 * undo memory gracefully
	 */ 
	for (int i = 0; i < adapter->num_queues; i++, txconf++) {
		/* Set up some basics */
		txr = &adapter->tx_rings[i];
		txr->adapter = adapter;
		txr->me = i;
		txr->num_desc = adapter->num_tx_desc;

		iflib_txq_addr_get(sctx, i, addrs);
		txr->tx_base = (union ixgbe_adv_tx_desc *)addrs[0];
		txr->tx_paddr = addrs[1];
		/* Now allocate transmit buffers for the ring */
		if (ixgbe_allocate_transmit_buffers(txr)) {
			device_printf(dev,"Failure setting up transmit buffers\n");
			error = ENOMEM;
			goto err_tx_desc;
		}
	}

	for (int i = 0; i < adapter->num_queues; i++, rxconf++) {
		rxr = &adapter->rx_rings[i];
		/* Set up some basics */
		rxr->adapter = adapter;
		rxr->me = i;
		rxr->num_desc = adapter->num_rx_desc;

		iflib_rxq_addr_get(sctx, i, addrs);
		rxr->rx_base = (union ixgbe_adv_rx_desc *)addrs[0];
		rxr->rx_paddr = addrs[1];
	}

	/*
	** Finally set up the queue holding structs
	*/
	for (int i = 0; i < adapter->num_queues; i++) {
		que = &adapter->queues[i];
		que->adapter = adapter;
		que->txr = &adapter->tx_rings[i];
		que->rxr = &adapter->rx_rings[i];
	}

	return (0);

err_tx_desc:
	free(adapter->rx_rings, M_DEVBUF);
rx_fail:
	free(adapter->tx_rings, M_DEVBUF);
tx_fail:
	free(adapter->queues, M_DEVBUF);
fail:
	iflib_txrx_structures_free(sctx);
	return (error);
}

/*********************************************************************
 *
 *  Allocate memory for tx_buffer structures. The tx_buffer stores all
 *  the information needed to transmit a packet on the wire. This is
 *  called only once at attach, setup is done every reset.
 *
 **********************************************************************/
static int
ixgbe_if_txq_setup(if_shared_ctx_t sctx, uint32_t txqid)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct tx_ring *txr = &adapter->tx_rings[txqid];
	device_t dev = adapter->hwdev;

	if (!(txr->tx_buffers =
	    (struct ixgbe_tx_buf *) malloc(sizeof(struct ixgbe_tx_buf) *
	    adapter->num_tx_desc, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate tx_buffer memory\n");
		return (ENOMEM);
	}

	return (0);
}

static int
ixgbe_allocate_transmit_buffers(struct tx_ring *txr)
{
	struct adapter *adapter = txr->adapter;
	device_t dev = adapter->hwdev;

	if (!(txr->tx_buffers =
	    (struct ixgbe_tx_buf *) malloc(sizeof(struct ixgbe_tx_buf) *
	    adapter->num_tx_desc, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate tx_buffer memory\n");
		return (ENOMEM);
	}
	return (0);
}

/*********************************************************************
 *
 *  Enable transmit unit.
 *
 **********************************************************************/
static void
ixgbe_initialize_transmit_units(struct adapter *adapter)
{
	struct tx_ring	*txr = adapter->tx_rings;
	struct ixgbe_hw	*hw = &adapter->hw;

	/* Setup the Base and Length of the Tx Descriptor Ring */

	for (int i = 0; i < adapter->num_queues; i++, txr++) {
		u64	tdba = txr->tx_paddr;
		u32	txctrl;

		IXGBE_WRITE_REG(hw, IXGBE_TDBAL(i),
		       (tdba & 0x00000000ffffffffULL));
		IXGBE_WRITE_REG(hw, IXGBE_TDBAH(i), (tdba >> 32));
		IXGBE_WRITE_REG(hw, IXGBE_TDLEN(i),
		    adapter->num_tx_desc * sizeof(union ixgbe_adv_tx_desc));

		/* Setup the HW Tx Head and Tail descriptor pointers */
		IXGBE_WRITE_REG(hw, IXGBE_TDH(i), 0);
		IXGBE_WRITE_REG(hw, IXGBE_TDT(i), 0);

		/* Setup Transmit Descriptor Cmd Settings */
		txr->txd_cmd = IXGBE_TXD_CMD_IFCS;
		txr->queue_status = IXGBE_QUEUE_IDLE;

		/* Set the processing limit */
		txr->process_limit = ixgbe_tx_process_limit;

		/* Disable Head Writeback */
		switch (hw->mac.type) {
		case ixgbe_mac_82598EB:
			txctrl = IXGBE_READ_REG(hw, IXGBE_DCA_TXCTRL(i));
			break;
		case ixgbe_mac_82599EB:
		case ixgbe_mac_X540:
		default:
			txctrl = IXGBE_READ_REG(hw, IXGBE_DCA_TXCTRL_82599(i));
			break;
                }
		txctrl &= ~IXGBE_DCA_TXCTRL_DESC_WRO_EN;
		switch (hw->mac.type) {
		case ixgbe_mac_82598EB:
			IXGBE_WRITE_REG(hw, IXGBE_DCA_TXCTRL(i), txctrl);
			break;
		case ixgbe_mac_82599EB:
		case ixgbe_mac_X540:
		default:
			IXGBE_WRITE_REG(hw, IXGBE_DCA_TXCTRL_82599(i), txctrl);
			break;
		}

	}

	if (hw->mac.type != ixgbe_mac_82598EB) {
		u32 dmatxctl, rttdcs;
		dmatxctl = IXGBE_READ_REG(hw, IXGBE_DMATXCTL);
		dmatxctl |= IXGBE_DMATXCTL_TE;
		IXGBE_WRITE_REG(hw, IXGBE_DMATXCTL, dmatxctl);
		/* Disable arbiter to set MTQC */
		rttdcs = IXGBE_READ_REG(hw, IXGBE_RTTDCS);
		rttdcs |= IXGBE_RTTDCS_ARBDIS;
		IXGBE_WRITE_REG(hw, IXGBE_RTTDCS, rttdcs);
		IXGBE_WRITE_REG(hw, IXGBE_MTQC, IXGBE_MTQC_64Q_1PB);
		rttdcs &= ~IXGBE_RTTDCS_ARBDIS;
		IXGBE_WRITE_REG(hw, IXGBE_RTTDCS, rttdcs);
	}

	return;
}

/*********************************************************************
 *
 *  Free all transmit rings.
 *
 **********************************************************************/
static void
ixgbe_if_tx_structures_free(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct tx_ring *txr = adapter->tx_rings;

	for (int i = 0; i < adapter->num_queues; i++, txr++)
		ixgbe_free_transmit_buffers(txr);

	free(adapter->tx_rings, M_DEVBUF);
}

/*********************************************************************
 *
 *  Free transmit ring related data structures.
 *
 **********************************************************************/
static void
ixgbe_free_transmit_buffers(struct tx_ring *txr)
{

	INIT_DEBUGOUT("ixgbe_free_transmit_ring: begin");
	if (txr->tx_buffers != NULL) {
		free(txr->tx_buffers, M_DEVBUF);
		txr->tx_buffers = NULL;
	}
}

/*********************************************************************
 *
 *  Advanced Context Descriptor setup for VLAN, CSUM or TSO
 *
 **********************************************************************/

static int
ixgbe_tx_ctx_setup(struct tx_ring *txr, struct mbuf *mp,
    u32 *cmd_type_len, u32 *olinfo_status)
{
	struct ixgbe_adv_tx_context_desc *TXD;
	struct ether_vlan_header *eh;
	struct ip *ip;
	struct ip6_hdr *ip6;
	u32 vlan_macip_lens = 0, type_tucmd_mlhl = 0;
	int	ehdrlen, ip_hlen = 0;
	u16	etype;
	u8	ipproto = 0;
	int	offload = TRUE;
	int	ctxd = txr->next_avail_desc;
	u16	vtag = 0;

	/* First check if TSO is to be used */
	if (mp->m_pkthdr.csum_flags & CSUM_TSO)
		return (ixgbe_tso_setup(txr, mp, cmd_type_len, olinfo_status));

	if ((mp->m_pkthdr.csum_flags & CSUM_OFFLOAD) == 0)
		offload = FALSE;

	/* Indicate the whole packet as payload when not doing TSO */
       	*olinfo_status |= mp->m_pkthdr.len << IXGBE_ADVTXD_PAYLEN_SHIFT;

	/* Now ready a context descriptor */
	TXD = (struct ixgbe_adv_tx_context_desc *) &txr->tx_base[ctxd];

	/*
	** In advanced descriptors the vlan tag must 
	** be placed into the context descriptor. Hence
	** we need to make one even if not doing offloads.
	*/
	if (mp->m_flags & M_VLANTAG) {
		vtag = htole16(mp->m_pkthdr.ether_vtag);
		vlan_macip_lens |= (vtag << IXGBE_ADVTXD_VLAN_SHIFT);
	} else if (offload == FALSE) /* ... no offload to do */
		return (0);

	/*
	 * Determine where frame payload starts.
	 * Jump over vlan headers if already present,
	 * helpful for QinQ too.
	 */
	eh = mtod(mp, struct ether_vlan_header *);
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN)) {
		etype = ntohs(eh->evl_proto);
		ehdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
	} else {
		etype = ntohs(eh->evl_encap_proto);
		ehdrlen = ETHER_HDR_LEN;
	}

	/* Set the ether header length */
	vlan_macip_lens |= ehdrlen << IXGBE_ADVTXD_MACLEN_SHIFT;

	switch (etype) {
		case ETHERTYPE_IP:
			ip = (struct ip *)(mp->m_data + ehdrlen);
			ip_hlen = ip->ip_hl << 2;
			ipproto = ip->ip_p;
			type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV4;
			break;
		case ETHERTYPE_IPV6:
			ip6 = (struct ip6_hdr *)(mp->m_data + ehdrlen);
			ip_hlen = sizeof(struct ip6_hdr);
			/* XXX-BZ this will go badly in case of ext hdrs. */
			ipproto = ip6->ip6_nxt;
			type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV6;
			break;
		default:
			offload = FALSE;
			break;
	}

	vlan_macip_lens |= ip_hlen;
	type_tucmd_mlhl |= IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT;

	switch (ipproto) {
		case IPPROTO_TCP:
			if (mp->m_pkthdr.csum_flags & CSUM_TCP)
				type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_TCP;
			break;

		case IPPROTO_UDP:
			if (mp->m_pkthdr.csum_flags & CSUM_UDP)
				type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_UDP;
			break;

#if __FreeBSD_version >= 800000
		case IPPROTO_SCTP:
			if (mp->m_pkthdr.csum_flags & CSUM_SCTP)
				type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_SCTP;
			break;
#endif
		default:
			offload = FALSE;
			break;
	}

	if (offload) /* For the TX descriptor setup */
		*olinfo_status |= IXGBE_TXD_POPTS_TXSM << 8;

	/* Now copy bits into descriptor */
	TXD->vlan_macip_lens = htole32(vlan_macip_lens);
	TXD->type_tucmd_mlhl = htole32(type_tucmd_mlhl);
	TXD->seqnum_seed = htole32(0);
	TXD->mss_l4len_idx = htole32(0);

	/* We've consumed the first desc, adjust counters */
	if (++ctxd == txr->num_desc)
		ctxd = 0;
	txr->next_avail_desc = ctxd;
	--txr->tx_avail;

        return (0);
}

/**********************************************************************
 *
 *  Setup work for hardware segmentation offload (TSO) on
 *  adapters using advanced tx descriptors
 *
 **********************************************************************/
static int
ixgbe_tso_setup(struct tx_ring *txr, struct mbuf *mp,
    u32 *cmd_type_len, u32 *olinfo_status)
{
	struct ixgbe_adv_tx_context_desc *TXD;
	u32 vlan_macip_lens = 0, type_tucmd_mlhl = 0;
	u32 mss_l4len_idx = 0, paylen;
	u16 vtag = 0, eh_type;
	int ctxd, ehdrlen, ip_hlen, tcp_hlen;
	struct ether_vlan_header *eh;
#ifdef INET6
	struct ip6_hdr *ip6;
#endif
#ifdef INET
	struct ip *ip;
#endif
	struct tcphdr *th;


	/*
	 * Determine where frame payload starts.
	 * Jump over vlan headers if already present
	 */
	eh = mtod(mp, struct ether_vlan_header *);
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN)) {
		ehdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
		eh_type = eh->evl_proto;
	} else {
		ehdrlen = ETHER_HDR_LEN;
		eh_type = eh->evl_encap_proto;
	}

	switch (ntohs(eh_type)) {
#ifdef INET6
	case ETHERTYPE_IPV6:
		ip6 = (struct ip6_hdr *)(mp->m_data + ehdrlen);
		/* XXX-BZ For now we do not pretend to support ext. hdrs. */
		if (ip6->ip6_nxt != IPPROTO_TCP)
			return (ENXIO);
		ip_hlen = sizeof(struct ip6_hdr);
		ip6 = (struct ip6_hdr *)(mp->m_data + ehdrlen);
		th = (struct tcphdr *)((caddr_t)ip6 + ip_hlen);
		th->th_sum = in6_cksum_pseudo(ip6, 0, IPPROTO_TCP, 0);
		type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV6;
		break;
#endif
#ifdef INET
	case ETHERTYPE_IP:
		ip = (struct ip *)(mp->m_data + ehdrlen);
		if (ip->ip_p != IPPROTO_TCP)
			return (ENXIO);
		ip->ip_sum = 0;
		ip_hlen = ip->ip_hl << 2;
		th = (struct tcphdr *)((caddr_t)ip + ip_hlen);
		th->th_sum = in_pseudo(ip->ip_src.s_addr,
		    ip->ip_dst.s_addr, htons(IPPROTO_TCP));
		type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_IPV4;
		/* Tell transmit desc to also do IPv4 checksum. */
		*olinfo_status |= IXGBE_TXD_POPTS_IXSM << 8;
		break;
#endif
	default:
		panic("%s: CSUM_TSO but no supported IP version (0x%04x)",
		    __func__, ntohs(eh_type));
		break;
	}

	ctxd = txr->next_avail_desc;
	TXD = (struct ixgbe_adv_tx_context_desc *) &txr->tx_base[ctxd];

	tcp_hlen = th->th_off << 2;

	/* This is used in the transmit desc in encap */
	paylen = mp->m_pkthdr.len - ehdrlen - ip_hlen - tcp_hlen;

	/* VLAN MACLEN IPLEN */
	if (mp->m_flags & M_VLANTAG) {
		vtag = htole16(mp->m_pkthdr.ether_vtag);
                vlan_macip_lens |= (vtag << IXGBE_ADVTXD_VLAN_SHIFT);
	}

	vlan_macip_lens |= ehdrlen << IXGBE_ADVTXD_MACLEN_SHIFT;
	vlan_macip_lens |= ip_hlen;
	TXD->vlan_macip_lens = htole32(vlan_macip_lens);

	/* ADV DTYPE TUCMD */
	type_tucmd_mlhl |= IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT;
	type_tucmd_mlhl |= IXGBE_ADVTXD_TUCMD_L4T_TCP;
	TXD->type_tucmd_mlhl = htole32(type_tucmd_mlhl);

	/* MSS L4LEN IDX */
	mss_l4len_idx |= (mp->m_pkthdr.tso_segsz << IXGBE_ADVTXD_MSS_SHIFT);
	mss_l4len_idx |= (tcp_hlen << IXGBE_ADVTXD_L4LEN_SHIFT);
	TXD->mss_l4len_idx = htole32(mss_l4len_idx);

	TXD->seqnum_seed = htole32(0);

	if (++ctxd == txr->num_desc)
		ctxd = 0;

	txr->tx_avail--;
	txr->next_avail_desc = ctxd;
	*cmd_type_len |= IXGBE_ADVTXD_DCMD_TSE;
	*olinfo_status |= IXGBE_TXD_POPTS_TXSM << 8;
	*olinfo_status |= paylen << IXGBE_ADVTXD_PAYLEN_SHIFT;
	++txr->tso_tx;
	return (0);
}

#ifdef IXGBE_FDIR
/*
** This routine parses packet headers so that Flow
** Director can make a hashed filter table entry 
** allowing traffic flows to be identified and kept
** on the same cpu.  This would be a performance
** hit, but we only do it at IXGBE_FDIR_RATE of
** packets.
*/
static void
ixgbe_atr(struct tx_ring *txr, struct mbuf *mp)
{
	struct adapter			*adapter = txr->adapter;
	struct ix_queue			*que;
	struct ip			*ip;
	struct tcphdr			*th;
	struct udphdr			*uh;
	struct ether_vlan_header	*eh;
	union ixgbe_atr_hash_dword	input = {.dword = 0}; 
	union ixgbe_atr_hash_dword	common = {.dword = 0}; 
	int  				ehdrlen, ip_hlen;
	u16				etype;

	eh = mtod(mp, struct ether_vlan_header *);
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN)) {
		ehdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
		etype = eh->evl_proto;
	} else {
		ehdrlen = ETHER_HDR_LEN;
		etype = eh->evl_encap_proto;
	}

	/* Only handling IPv4 */
	if (etype != htons(ETHERTYPE_IP))
		return;

	ip = (struct ip *)(mp->m_data + ehdrlen);
	ip_hlen = ip->ip_hl << 2;

	/* check if we're UDP or TCP */
	switch (ip->ip_p) {
	case IPPROTO_TCP:
		th = (struct tcphdr *)((caddr_t)ip + ip_hlen);
		/* src and dst are inverted */
		common.port.dst ^= th->th_sport;
		common.port.src ^= th->th_dport;
		input.formatted.flow_type ^= IXGBE_ATR_FLOW_TYPE_TCPV4;
		break;
	case IPPROTO_UDP:
		uh = (struct udphdr *)((caddr_t)ip + ip_hlen);
		/* src and dst are inverted */
		common.port.dst ^= uh->uh_sport;
		common.port.src ^= uh->uh_dport;
		input.formatted.flow_type ^= IXGBE_ATR_FLOW_TYPE_UDPV4;
		break;
	default:
		return;
	}

	input.formatted.vlan_id = htobe16(mp->m_pkthdr.ether_vtag);
	if (mp->m_pkthdr.ether_vtag)
		common.flex_bytes ^= htons(ETHERTYPE_VLAN);
	else
		common.flex_bytes ^= etype;
	common.ip ^= ip->ip_src.s_addr ^ ip->ip_dst.s_addr;

	que = &adapter->queues[txr->me];
	/*
	** This assumes the Rx queue and Tx
	** queue are bound to the same CPU
	*/
	ixgbe_fdir_add_signature_filter_82599(&adapter->hw,
	    input, common, que->msix);
}
#endif /* IXGBE_FDIR */

/**********************************************************************
 *
 *  Examine each tx_buffer in the used queue. If the hardware is done
 *  processing the packet then free associated resources. The
 *  tx_buffer is put back on the free queue.
 *
 **********************************************************************/
static int
ixgbe_isc_txd_credits_update(if_shared_ctx_t sctx, uint32_t qid, uint32_t cidx)
{
	u32	work, processed = 0;
	struct adapter *adapter = DOWNCAST(sctx);
	struct tx_ring *txr = &adapter->tx_rings[qid];
	struct ixgbe_tx_buf	*buf;
	union ixgbe_adv_tx_desc *txd, *eop;

	/* Get work starting point */
	work = cidx;
	buf = &txr->tx_buffers[work];
	txd = &txr->tx_base[work];
	work -= txr->num_desc; /* The distance to ring end */

	do {
		eop = buf->eop;
		if (eop == NULL) /* No work */
			break;
		if ((eop->wb.status & IXGBE_TXD_STAT_DD) == 0)
			break;	/* I/O not complete */

		/*
		 * Avoid a race between multiple threads doing
		 * tx_credit updates - this will break on 32-bit XXX
		 * XXX - make sure this is safe
		 */
		if (atomic_cmpset_ptr((unsigned long *)&buf->eop, (unsigned long)eop, (unsigned long)NULL) == 0)
			return (0);

		while (txd != eop) {
			++txd;
			++buf;
			++work;
			/* wrap the ring? */
			if (__predict_false(!work)) {
				work -= txr->num_desc;
				txd = txr->tx_base;
			}
			buf->eop = NULL;
			++processed;
		}

		/* Try the next packet */
		++txd;
		++buf;
		++work;
		++processed;

		/* reset with a wrap */
		if (__predict_false(!work)) {
			work -= txr->num_desc;
			buf = txr->tx_buffers;
			txd = txr->tx_base;
		}
		prefetch(txd);
	} while (1);

	if (processed)
		iflib_tx_credits_update(sctx, qid, processed);
	return (processed);
}

static void
ixgbe_isc_rxd_refill(if_shared_ctx_t sctx, uint32_t rxqid, uint32_t pidx, uint64_t *paddrs, uint32_t count)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct rx_ring *rxr = &adapter->rx_rings[rxqid];
	int i;
	uint32_t next_pidx;

	for (i = 0, next_pidx = pidx; i < count; i++) {
		rxr->rx_base[next_pidx].read.pkt_addr = htole64(paddrs[i]);
		if (++next_pidx == sctx->isc_nrxd)
			next_pidx = 0;
	}
}

static void
ixgbe_isc_rxd_flush(if_shared_ctx_t sctx, uint32_t rxqid, uint32_t pidx)
{
	struct adapter *adapter = DOWNCAST(sctx);

	IXGBE_WRITE_REG(&adapter->hw, IXGBE_RDT(rxqid), pidx);
}

static int
ixgbe_isc_rxd_available(if_shared_ctx_t sctx, uint32_t rxqid, uint32_t idx)
{
	struct rx_ring *rxr = &DOWNCAST(sctx)->rx_rings[rxqid];
	union ixgbe_adv_rx_desc	*cur;
	uint32_t staterr;
	int cnt, i;

	cnt = 0;
	i = idx;
	do {
		cur = &rxr->rx_base[i];
		staterr = le32toh(cur->wb.upper.status_error);
		if ((staterr & IXGBE_RXD_STAT_DD) == 0)
			break;
		cnt++;
		if (++i == sctx->isc_nrxd)
			i = 0;
	} while (1);

	return (cnt);
}

/*
** Used to detect a descriptor that has
** been merged by Hardware RSC.
*/
static inline u32
ixgbe_rsc_count(union ixgbe_adv_rx_desc *rx)
{
	return (le32toh(rx->wb.lower.lo_dword.data) &
	    IXGBE_RXDADV_RSCCNT_MASK) >> IXGBE_RXDADV_RSCCNT_SHIFT;
}

/*********************************************************************
 *
 *  Initialize Hardware RSC (LRO) feature on 82599
 *  for an RX ring, this is toggled by the LRO capability
 *  even though it is transparent to the stack.
 *
 *  NOTE: since this HW feature only works with IPV4 and 
 *        our testing has shown soft LRO to be as effective
 *        I have decided to disable this by default.
 *
 **********************************************************************/
static void
ixgbe_setup_hw_rsc(struct rx_ring *rxr)
{
	struct	adapter 	*adapter = rxr->adapter;
	struct	ixgbe_hw	*hw = &adapter->hw;
	u32			rscctrl, rdrxctl;

	/* If turning LRO/RSC off we need to disable it */
	if ((adapter->hwifp->if_capenable & IFCAP_LRO) == 0) {
		rscctrl = IXGBE_READ_REG(hw, IXGBE_RSCCTL(rxr->me));
		rscctrl &= ~IXGBE_RSCCTL_RSCEN;
		return;
	}

	rdrxctl = IXGBE_READ_REG(hw, IXGBE_RDRXCTL);
	rdrxctl &= ~IXGBE_RDRXCTL_RSCFRSTSIZE;
#ifdef DEV_NETMAP /* crcstrip is optional in netmap */
	if (adapter->hwifp->if_capenable & IFCAP_NETMAP && !ix_crcstrip)
#endif /* DEV_NETMAP */
	rdrxctl |= IXGBE_RDRXCTL_CRCSTRIP;
	rdrxctl |= IXGBE_RDRXCTL_RSCACKC;
	IXGBE_WRITE_REG(hw, IXGBE_RDRXCTL, rdrxctl);

	rscctrl = IXGBE_READ_REG(hw, IXGBE_RSCCTL(rxr->me));
	rscctrl |= IXGBE_RSCCTL_RSCEN;
	/*
	** Limit the total number of descriptors that
	** can be combined, so it does not exceed 64K
	*/
	if (rxr->mbuf_sz == MCLBYTES)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_16;
	else if (rxr->mbuf_sz == MJUMPAGESIZE)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_8;
	else if (rxr->mbuf_sz == MJUM9BYTES)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_4;
	else  /* Using 16K cluster */
		rscctrl |= IXGBE_RSCCTL_MAXDESC_1;

	IXGBE_WRITE_REG(hw, IXGBE_RSCCTL(rxr->me), rscctrl);

	/* Enable TCP header recognition */
	IXGBE_WRITE_REG(hw, IXGBE_PSRTYPE(0),
	    (IXGBE_READ_REG(hw, IXGBE_PSRTYPE(0)) |
	    IXGBE_PSRTYPE_TCPHDR));

	/* Disable RSC for ACK packets */
	IXGBE_WRITE_REG(hw, IXGBE_RSCDBU,
	    (IXGBE_RSCDBU_RSCACKDIS | IXGBE_READ_REG(hw, IXGBE_RSCDBU)));

	rxr->hw_rsc = TRUE;
}

/*********************************************************************
 *
 *  Initialize a receive ring and its buffers.
 *
 **********************************************************************/
static int
ixgbe_if_rxq_setup(if_shared_ctx_t sctx, uint32_t rxqid)
{
	struct	adapter 	*adapter = DOWNCAST(sctx);
	struct rx_ring *rxr = &adapter->rx_rings[rxqid];

	rxr->next_to_check = 0;
	rxr->next_to_refresh = 0;
	rxr->rx_copies = 0;
	rxr->rx_bytes = 0;
	rxr->discard = FALSE;
	rxr->vtag_strip = FALSE;

	/*
	** Now set up the LRO interface:
	*/
	if (ixgbe_rsc_enable)
		ixgbe_setup_hw_rsc(rxr);
	return (0);
}

static void
ixgbe_initialise_rss_mapping(struct adapter *adapter)
{
	struct ixgbe_hw	*hw = &adapter->hw;
	uint32_t reta;
	int i, j, queue_id;
	uint32_t rss_key[10];
	uint32_t mrqc;
#ifdef	RSS
	uint32_t rss_hash_config;
#endif

	/* Setup RSS */
	reta = 0;

#ifdef	RSS
	/* Fetch the configured RSS key */
	rss_getkey((uint8_t *) &rss_key);
#else
	/* set up random bits */
	arc4rand(&rss_key, sizeof(rss_key), 0);
#endif

	/* Set up the redirection table */
	for (i = 0, j = 0; i < 128; i++, j++) {
		if (j == adapter->num_queues) j = 0;
#ifdef	RSS
		/*
		 * Fetch the RSS bucket id for the given indirection entry.
		 * Cap it at the number of configured buckets (which is
		 * num_queues.)
		 */
		queue_id = rss_get_indirection_to_bucket(i);
		queue_id = queue_id % adapter->num_queues;
#else
		queue_id = (j * 0x11);
#endif
		/*
		 * The low 8 bits are for hash value (n+0);
		 * The next 8 bits are for hash value (n+1), etc.
		 */
		reta = reta >> 8;
		reta = reta | ( ((uint32_t) queue_id) << 24);
		if ((i & 3) == 3) {
			IXGBE_WRITE_REG(hw, IXGBE_RETA(i >> 2), reta);
			reta = 0;
		}
	}

	/* Now fill our hash function seeds */
	for (int i = 0; i < 10; i++)
		IXGBE_WRITE_REG(hw, IXGBE_RSSRK(i), rss_key[i]);

	/* Perform hash on these packet types */
#ifdef	RSS
	mrqc = IXGBE_MRQC_RSSEN;
	rss_hash_config = rss_gethashconfig();
	if (rss_hash_config & RSS_HASHTYPE_RSS_IPV4)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4;
	if (rss_hash_config & RSS_HASHTYPE_RSS_TCP_IPV4)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4_TCP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_IPV6)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6;
	if (rss_hash_config & RSS_HASHTYPE_RSS_TCP_IPV6)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6_TCP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_IPV6_EX)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6_EX;
	if (rss_hash_config & RSS_HASHTYPE_RSS_TCP_IPV6_EX)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6_EX_TCP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_UDP_IPV4)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4_UDP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_UDP_IPV4_EX)
		device_printf(adapter->hwdev,
		    "%s: RSS_HASHTYPE_RSS_UDP_IPV4_EX defined, "
		    "but not supported\n", __func__);
	if (rss_hash_config & RSS_HASHTYPE_RSS_UDP_IPV6)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6_UDP;
	if (rss_hash_config & RSS_HASHTYPE_RSS_UDP_IPV6_EX)
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6_EX_UDP;
#else
	/*
	 * Disable UDP - IP fragments aren't currently being handled
	 * and so we end up with a mix of 2-tuple and 4-tuple
	 * traffic.
	 */
	mrqc = IXGBE_MRQC_RSSEN
	     | IXGBE_MRQC_RSS_FIELD_IPV4
	     | IXGBE_MRQC_RSS_FIELD_IPV4_TCP
#if 0
	     | IXGBE_MRQC_RSS_FIELD_IPV4_UDP
#endif
	     | IXGBE_MRQC_RSS_FIELD_IPV6_EX_TCP
	     | IXGBE_MRQC_RSS_FIELD_IPV6_EX
	     | IXGBE_MRQC_RSS_FIELD_IPV6
	     | IXGBE_MRQC_RSS_FIELD_IPV6_TCP
#if 0
	     | IXGBE_MRQC_RSS_FIELD_IPV6_UDP
	     | IXGBE_MRQC_RSS_FIELD_IPV6_EX_UDP
#endif
	;
#endif /* RSS */
	IXGBE_WRITE_REG(hw, IXGBE_MRQC, mrqc);
}


/*********************************************************************
 *
 *  Setup receive registers and features.
 *
 **********************************************************************/
#define IXGBE_SRRCTL_BSIZEHDRSIZE_SHIFT 2

#define BSIZEPKT_ROUNDUP ((1<<IXGBE_SRRCTL_BSIZEPKT_SHIFT)-1)
	
static void
ixgbe_initialize_receive_units(struct adapter *adapter)
{
	struct	rx_ring	*rxr = adapter->rx_rings;
	struct ixgbe_hw	*hw = &adapter->hw;
	struct ifnet   *ifp = adapter->hwifp;
	u32		bufsz, rxctrl, fctrl, srrctl, rxcsum;
	u32		hlreg;


	/*
	 * Make sure receives are disabled while
	 * setting up the descriptor ring
	 */
	rxctrl = IXGBE_READ_REG(hw, IXGBE_RXCTRL);
	IXGBE_WRITE_REG(hw, IXGBE_RXCTRL,
	    rxctrl & ~IXGBE_RXCTRL_RXEN);

	/* Enable broadcasts */
	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	fctrl |= IXGBE_FCTRL_BAM;
	fctrl |= IXGBE_FCTRL_DPF;
	fctrl |= IXGBE_FCTRL_PMCF;
	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);

	/* Set for Jumbo Frames? */
	hlreg = IXGBE_READ_REG(hw, IXGBE_HLREG0);
	if (ifp->if_mtu > ETHERMTU)
		hlreg |= IXGBE_HLREG0_JUMBOEN;
	else
		hlreg &= ~IXGBE_HLREG0_JUMBOEN;
#ifdef DEV_NETMAP
	/* crcstrip is conditional in netmap (in RDRXCTL too ?) */
	if (ifp->if_capenable & IFCAP_NETMAP && !ix_crcstrip)
		hlreg &= ~IXGBE_HLREG0_RXCRCSTRP;
	else
		hlreg |= IXGBE_HLREG0_RXCRCSTRP;
#endif /* DEV_NETMAP */
	IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg);

	bufsz = (adapter->rx_mbuf_sz +
	    BSIZEPKT_ROUNDUP) >> IXGBE_SRRCTL_BSIZEPKT_SHIFT;

	for (int i = 0; i < adapter->num_queues; i++, rxr++) {
		u64 rdba = rxr->rx_paddr;

		/* Setup the Base and Length of the Rx Descriptor Ring */
		IXGBE_WRITE_REG(hw, IXGBE_RDBAL(i),
			       (rdba & 0x00000000ffffffffULL));
		IXGBE_WRITE_REG(hw, IXGBE_RDBAH(i), (rdba >> 32));
		IXGBE_WRITE_REG(hw, IXGBE_RDLEN(i),
		    adapter->num_rx_desc * sizeof(union ixgbe_adv_rx_desc));

		/* Set up the SRRCTL register */
		srrctl = IXGBE_READ_REG(hw, IXGBE_SRRCTL(i));
		srrctl &= ~IXGBE_SRRCTL_BSIZEHDR_MASK;
		srrctl &= ~IXGBE_SRRCTL_BSIZEPKT_MASK;
		srrctl |= bufsz;
		srrctl |= IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF;
		IXGBE_WRITE_REG(hw, IXGBE_SRRCTL(i), srrctl);

		/* Setup the HW Rx Head and Tail Descriptor Pointers */
		IXGBE_WRITE_REG(hw, IXGBE_RDH(i), 0);
		IXGBE_WRITE_REG(hw, IXGBE_RDT(i), 0);

		/* Set the processing limit */
		rxr->process_limit = ixgbe_rx_process_limit;
	}

	if (adapter->hw.mac.type != ixgbe_mac_82598EB) {
		u32 psrtype = IXGBE_PSRTYPE_TCPHDR |
			      IXGBE_PSRTYPE_UDPHDR |
			      IXGBE_PSRTYPE_IPV4HDR |
			      IXGBE_PSRTYPE_IPV6HDR;
		IXGBE_WRITE_REG(hw, IXGBE_PSRTYPE(0), psrtype);
	}

	rxcsum = IXGBE_READ_REG(hw, IXGBE_RXCSUM);

	ixgbe_initialise_rss_mapping(adapter);

	if (adapter->num_queues > 1) {
		/* RSS and RX IPP Checksum are mutually exclusive */
		rxcsum |= IXGBE_RXCSUM_PCSD;
	}

	if (ifp->if_capenable & IFCAP_RXCSUM)
		rxcsum |= IXGBE_RXCSUM_PCSD;

	if (!(rxcsum & IXGBE_RXCSUM_PCSD))
		rxcsum |= IXGBE_RXCSUM_IPPCSE;

	IXGBE_WRITE_REG(hw, IXGBE_RXCSUM, rxcsum);

	return;
}

/*********************************************************************
 *
 *  Free all receive rings.
 *
 **********************************************************************/
static void
ixgbe_if_rx_structures_free(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);

	INIT_DEBUGOUT("ixgbe_free_receive_structures: begin");
	free(adapter->rx_rings, M_DEVBUF);
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
 *  Return TRUE for more work, FALSE for all clean.
 *********************************************************************/
static int
ixgbe_isc_rxd_pkt_get(if_shared_ctx_t sctx, uint32_t rxqid, uint32_t i, if_rxd_info_t ri)
{
	struct adapter		*adapter = DOWNCAST(sctx);
	struct rx_ring		*rxr = &adapter->rx_rings[rxqid];
	union ixgbe_adv_rx_desc	*cur;
	u16			pkt_info;

	u32		staterr, rsc, ptype, nextp;
	u16		len;
	u16		vtag = 0;
	bool		eop;
 
	cur = &rxr->rx_base[i];
	staterr = le32toh(cur->wb.upper.status_error);
	pkt_info = le16toh(cur->wb.lower.lo_dword.hs_rss.pkt_info);

	rsc = 0;
	cur->wb.upper.status_error = 0;

	len = le16toh(cur->wb.upper.length);
	rxr->bytes += len;
	ptype = le32toh(cur->wb.lower.lo_dword.data) &
		IXGBE_RXDADV_PKTTYPE_MASK;
	eop = ((staterr & IXGBE_RXD_STAT_EOP) != 0);

	/* Make sure bad packets are discarded */
	if (((staterr & IXGBE_RXDADV_ERR_FRAME_ERR_MASK) != 0) ||
		(rxr->discard)) {
		rxr->rx_discarded++;
		if (eop)
			rxr->discard = FALSE;
		else
			rxr->discard = TRUE;
		return (EBADMSG);
	}

	/*
	 ** On 82599 which supports a hardware
	 ** LRO (called HW RSC), packets need
	 ** not be fragmented across sequential
	 ** descriptors, rather the next descriptor
	 ** is indicated in bits of the descriptor.
	 ** We pass the offset to iflib.
	 */

	if (!eop) {
			/*
			** Figure out the next descriptor
			** of this frame.
			*/
		if (rxr->hw_rsc == TRUE)
			rsc = ixgbe_rsc_count(cur);
		if (rsc) { /* Get hardware index */
			nextp = ((staterr &
					  IXGBE_RXDADV_NEXTP_MASK) >>
					 IXGBE_RXDADV_NEXTP_SHIFT);
			if (nextp < i)
				nextp = (adapter->num_rx_desc - i) + nextp;
			else
				nextp = nextp - i;
		} else
			nextp = 1;
	} else {
		nextp = 0;

		/*
		** Rather than using the fmp/lmp global pointers
		** we now keep the head of a packet chain in the
		** buffer struct and pass this along from one
		** descriptor to the next, until we get EOP.
		*/
		/*
		** See if there is a stored head
		** that determines what we are
		*/

		/* Process vlan info */
		if ((rxr->vtag_strip) &&
			(staterr & IXGBE_RXD_STAT_VP))
			vtag = le16toh(cur->wb.upper.vlan);
		if (vtag) {
			ri->iri_vtag = vtag;
			ri->iri_flags |= M_VLANTAG;
		}
		if ((sctx->isc_ifp->if_capenable & IFCAP_RXCSUM) != 0)
			ixgbe_rx_checksum(staterr, ri, ptype);
#if __FreeBSD_version >= 800000
#ifdef RSS
		ri->iri_flowid =
			le32toh(cur->wb.lower.hi_dword.rss);
		ri->iri_flags |= M_FLOWID;
		switch (pkt_info & IXGBE_RXDADV_RSSTYPE_MASK) {
		case IXGBE_RXDADV_RSSTYPE_IPV4_TCP:
			hash_type = M_HASHTYPE_RSS_TCP_IPV4;
		break;
		case IXGBE_RXDADV_RSSTYPE_IPV4:
			hash_type = M_HASHTYPE_RSS_IPV4;
			break;
		case IXGBE_RXDADV_RSSTYPE_IPV6_TCP:
			hash_type = M_HASHTYPE_RSS_TCP_IPV6;
			break;
		case IXGBE_RXDADV_RSSTYPE_IPV6_EX:
			hash_type = M_HASHTYPE_RSS_IPV6_EX;
			break;
		case IXGBE_RXDADV_RSSTYPE_IPV6:
			hash_type = M_HASHTYPE_RSS_IPV6;
			break;
		case IXGBE_RXDADV_RSSTYPE_IPV6_TCP_EX:
			hash_type = M_HASHTYPE_RSS_TCP_IPV6_EX;
			break;
		case IXGBE_RXDADV_RSSTYPE_IPV4_UDP:
			hash_type = M_HASHTYPE_RSS_UDP_IPV4;
			break;
		case IXGBE_RXDADV_RSSTYPE_IPV6_UDP:
			hash_type = M_HASHTYPE_RSS_UDP_IPV6;
			break;
		case IXGBE_RXDADV_RSSTYPE_IPV6_UDP_EX:
			hash_type = M_HASHTYPE_RSS_UDP_IPV6_EX;
			break;
		default:
			/* XXX fallthrough */
			hash_type = M_HASHTYPE_NONE;
		}
#else /* RSS */
		ri->iri_flowid = adapter->queues[rxqid].msix;
		ri->iri_flags |= IF_RXD_FLOWID;
#endif /* RSS */
#endif /* FreeBSD_version */
		rxr->packets++;
	}
	ri->iri_next_offset = nextp;

	return (0);
}

/*********************************************************************
 *
 *  Verify that the hardware indicated that the checksum is valid.
 *  Inform the stack about the status of checksum so that stack
 *  doesn't spend time verifying the checksum.
 *
 *********************************************************************/
static void
ixgbe_rx_checksum(u32 staterr, if_rxd_info_t ri, u32 ptype)
{
	u16	status = (u16) staterr;
	u8	errors = (u8) (staterr >> 24);
	bool	sctp = FALSE;

	if ((ptype & IXGBE_RXDADV_PKTTYPE_ETQF) == 0 &&
	    (ptype & IXGBE_RXDADV_PKTTYPE_SCTP) != 0)
		sctp = TRUE;

	if (status & IXGBE_RXD_STAT_IPCS) {
		if (!(errors & IXGBE_RXD_ERR_IPE)) {
			/* IP Checksum Good */
			ri->iri_csum_flags = CSUM_IP_CHECKED|CSUM_IP_VALID;

		} else
			ri->iri_csum_flags = 0;
	}
	if (status & IXGBE_RXD_STAT_L4CS) {
		u64 type = (CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
#if __FreeBSD_version >= 800000
		if (sctp)
			type = CSUM_SCTP_VALID;
#endif
		if (!(errors & IXGBE_RXD_ERR_TCPE)) {
			ri->iri_csum_flags |= type;
			if (!sctp)
				ri->iri_csum_data = htons(0xffff);
		} 
	}
}


/*
** This routine is run via an vlan config EVENT,
** it enables us to use the HW Filter table since
** we can get the vlan id. This just creates the
** entry in the soft version of the VFTA, init will
** repopulate the real table.
*/
static void
ixgbe_if_vlan_register(if_shared_ctx_t sctx, u16 vtag)
{
	struct adapter	*adapter = DOWNCAST(sctx);
	u16		index, bit;

	if ((vtag == 0) || (vtag > 4095))	/* Invalid */
		return;

	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	adapter->shadow_vfta[index] |= (1 << bit);
	++adapter->num_vlans;
	ixgbe_setup_vlan_hw_support(adapter);
}

/*
** This routine is run via an vlan
** unconfig EVENT, remove our entry
** in the soft vfta.
*/
static void
ixgbe_if_vlan_unregister(if_shared_ctx_t sctx, u16 vtag)
{
	struct adapter	*adapter = DOWNCAST(sctx);
	u16		index, bit;

	if ((vtag == 0) || (vtag > 4095))	/* Invalid */
		return;

	index = (vtag >> 5) & 0x7F;
	bit = vtag & 0x1F;
	adapter->shadow_vfta[index] &= ~(1 << bit);
	--adapter->num_vlans;
	/* Re-init to load the changes */
	ixgbe_setup_vlan_hw_support(adapter);
}

static void
ixgbe_setup_vlan_hw_support(struct adapter *adapter)
{
	struct ifnet 	*ifp = adapter->hwifp;
	struct ixgbe_hw *hw = &adapter->hw;
	struct rx_ring	*rxr;
	u32		ctrl;


	/*
	** We get here thru init_locked, meaning
	** a soft reset, this has already cleared
	** the VFTA and other state, so if there
	** have been no vlan's registered do nothing.
	*/
	if (adapter->num_vlans == 0)
		return;

	/* Setup the queues for vlans */
	for (int i = 0; i < adapter->num_queues; i++) {
		rxr = &adapter->rx_rings[i];
		/* On 82599 the VLAN enable is per/queue in RXDCTL */
		if (hw->mac.type != ixgbe_mac_82598EB) {
			ctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
			ctrl |= IXGBE_RXDCTL_VME;
			IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(i), ctrl);
		}
		rxr->vtag_strip = TRUE;
	}

	if ((ifp->if_capenable & IFCAP_VLAN_HWFILTER) == 0)
		return;
	/*
	** A soft reset zero's out the VFTA, so
	** we need to repopulate it now.
	*/
	for (int i = 0; i < IXGBE_VFTA_SIZE; i++)
		if (adapter->shadow_vfta[i] != 0)
			IXGBE_WRITE_REG(hw, IXGBE_VFTA(i),
			    adapter->shadow_vfta[i]);

	ctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
	/* Enable the Filter Table if enabled */
	if (ifp->if_capenable & IFCAP_VLAN_HWFILTER) {
		ctrl &= ~IXGBE_VLNCTRL_CFIEN;
		ctrl |= IXGBE_VLNCTRL_VFE;
	}
	if (hw->mac.type == ixgbe_mac_82598EB)
		ctrl |= IXGBE_VLNCTRL_VME;
	IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, ctrl);
}

static void
ixgbe_if_intr_enable(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct ixgbe_hw	*hw = &adapter->hw;
	u32		mask, fwsm;

	mask = (IXGBE_EIMS_ENABLE_MASK & ~IXGBE_EIMS_RTX_QUEUE);
	/* Enable Fan Failure detection */
	if (hw->device_id == IXGBE_DEV_ID_82598AT)
		    mask |= IXGBE_EIMS_GPI_SDP1;

	switch (adapter->hw.mac.type) {
		case ixgbe_mac_82599EB:
			mask |= IXGBE_EIMS_ECC;
			mask |= IXGBE_EIMS_GPI_SDP0;
			mask |= IXGBE_EIMS_GPI_SDP1;
			mask |= IXGBE_EIMS_GPI_SDP2;
#ifdef IXGBE_FDIR
			mask |= IXGBE_EIMS_FLOW_DIR;
#endif
			break;
		case ixgbe_mac_X540:
			mask |= IXGBE_EIMS_ECC;
			/* Detect if Thermal Sensor is enabled */
			fwsm = IXGBE_READ_REG(hw, IXGBE_FWSM);
			if (fwsm & IXGBE_FWSM_TS_ENABLED)
				mask |= IXGBE_EIMS_TS;
#ifdef IXGBE_FDIR
			mask |= IXGBE_EIMS_FLOW_DIR;
#endif
		/* falls through */
		default:
			break;
	}

	IXGBE_WRITE_REG(hw, IXGBE_EIMS, mask);

	/* With RSS we use auto clear */
	if (adapter->msix_mem) {
		mask = IXGBE_EIMS_ENABLE_MASK;
		/* Don't autoclear Link */
		mask &= ~IXGBE_EIMS_OTHER;
		mask &= ~IXGBE_EIMS_LSC;
		IXGBE_WRITE_REG(hw, IXGBE_EIAC, mask);
	}

	/*
	** Now enable all queues, this is done separately to
	** allow for handling the extended (beyond 32) MSIX
	** vectors that can be used by 82599
	*/
        for (int i = 0; i < adapter->num_queues; i++)
			ixgbe_if_rx_intr_enable(UPCAST(adapter), i);

	IXGBE_WRITE_FLUSH(hw);

	return;
}

static void
ixgbe_if_intr_disable(if_shared_ctx_t sctx)
{
	struct adapter *adapter = DOWNCAST(sctx);

	if (adapter->msix_mem)
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIAC, 0);
	if (adapter->hw.mac.type == ixgbe_mac_82598EB) {
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC, ~0);
	} else {
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC, 0xFFFF0000);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC_EX(0), ~0);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC_EX(1), ~0);
	}
	IXGBE_WRITE_FLUSH(&adapter->hw);
	return;
}

u16
ixgbe_read_pci_cfg(struct ixgbe_hw *hw, u32 reg)
{
	u16 value;

	value = pci_read_config(((struct ixgbe_osdep *)hw->back)->dev,
	    reg, 2);

	return (value);
}

void
ixgbe_write_pci_cfg(struct ixgbe_hw *hw, u32 reg, u16 value)
{
	pci_write_config(((struct ixgbe_osdep *)hw->back)->dev,
	    reg, value, 2);

	return;
}

/*
** Get the width and transaction speed of
** the slot this adapter is plugged into.
*/
static void
ixgbe_get_slot_info(struct ixgbe_hw *hw)
{
	device_t		dev = ((struct ixgbe_osdep *)hw->back)->dev;
	struct ixgbe_mac_info	*mac = &hw->mac;
	u16			link;
	u32			offset;

	/* For most devices simply call the shared code routine */
	if (hw->device_id != IXGBE_DEV_ID_82599_SFP_SF_QP) {
		ixgbe_get_bus_info(hw);
		goto display;
	}

	/*
	** For the Quad port adapter we need to parse back
	** up the PCI tree to find the speed of the expansion
	** slot into which this adapter is plugged. A bit more work.
	*/
	dev = device_get_parent(device_get_parent(dev));
#ifdef IXGBE_DEBUG
	device_printf(dev, "parent pcib = %x,%x,%x\n",
	    pci_get_bus(dev), pci_get_slot(dev), pci_get_function(dev));
#endif
	dev = device_get_parent(device_get_parent(dev));
#ifdef IXGBE_DEBUG
	device_printf(dev, "slot pcib = %x,%x,%x\n",
	    pci_get_bus(dev), pci_get_slot(dev), pci_get_function(dev));
#endif
	/* Now get the PCI Express Capabilities offset */
	pci_find_cap(dev, PCIY_EXPRESS, &offset);
	/* ...and read the Link Status Register */
	link = pci_read_config(dev, offset + PCIER_LINK_STA, 2);
	switch (link & IXGBE_PCI_LINK_WIDTH) {
	case IXGBE_PCI_LINK_WIDTH_1:
		hw->bus.width = ixgbe_bus_width_pcie_x1;
		break;
	case IXGBE_PCI_LINK_WIDTH_2:
		hw->bus.width = ixgbe_bus_width_pcie_x2;
		break;
	case IXGBE_PCI_LINK_WIDTH_4:
		hw->bus.width = ixgbe_bus_width_pcie_x4;
		break;
	case IXGBE_PCI_LINK_WIDTH_8:
		hw->bus.width = ixgbe_bus_width_pcie_x8;
		break;
	default:
		hw->bus.width = ixgbe_bus_width_unknown;
		break;
	}

	switch (link & IXGBE_PCI_LINK_SPEED) {
	case IXGBE_PCI_LINK_SPEED_2500:
		hw->bus.speed = ixgbe_bus_speed_2500;
		break;
	case IXGBE_PCI_LINK_SPEED_5000:
		hw->bus.speed = ixgbe_bus_speed_5000;
		break;
	case IXGBE_PCI_LINK_SPEED_8000:
		hw->bus.speed = ixgbe_bus_speed_8000;
		break;
	default:
		hw->bus.speed = ixgbe_bus_speed_unknown;
		break;
	}

	mac->ops.set_lan_id(hw);

display:
	device_printf(dev,"PCI Express Bus: Speed %s %s\n",
	    ((hw->bus.speed == ixgbe_bus_speed_8000) ? "8.0GT/s":
	    (hw->bus.speed == ixgbe_bus_speed_5000) ? "5.0GT/s":
	    (hw->bus.speed == ixgbe_bus_speed_2500) ? "2.5GT/s":"Unknown"),
	    (hw->bus.width == ixgbe_bus_width_pcie_x8) ? "Width x8" :
	    (hw->bus.width == ixgbe_bus_width_pcie_x4) ? "Width x4" :
	    (hw->bus.width == ixgbe_bus_width_pcie_x1) ? "Width x1" :
	    ("Unknown"));

	if ((hw->device_id != IXGBE_DEV_ID_82599_SFP_SF_QP) &&
	    ((hw->bus.width <= ixgbe_bus_width_pcie_x4) &&
	    (hw->bus.speed == ixgbe_bus_speed_2500))) {
		device_printf(dev, "PCI-Express bandwidth available"
		    " for this card\n     is not sufficient for"
		    " optimal performance.\n");
		device_printf(dev, "For optimal performance a x8 "
		    "PCIE, or x4 PCIE Gen2 slot is required.\n");
        }
	if ((hw->device_id == IXGBE_DEV_ID_82599_SFP_SF_QP) &&
	    ((hw->bus.width <= ixgbe_bus_width_pcie_x8) &&
	    (hw->bus.speed < ixgbe_bus_speed_8000))) {
		device_printf(dev, "PCI-Express bandwidth available"
		    " for this card\n     is not sufficient for"
		    " optimal performance.\n");
		device_printf(dev, "For optimal performance a x8 "
		    "PCIE Gen3 slot is required.\n");
        }

	return;
}

/*
** Setup the correct IVAR register for a particular MSIX interrupt
**   (yes this is all very magic and confusing :)
**  - entry is the register array entry
**  - vector is the MSIX vector for this queue
**  - type is RX/TX/MISC
*/
static void
ixgbe_set_ivar(struct adapter *adapter, u8 entry, u8 vector, s8 type)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 ivar, index;

	vector |= IXGBE_IVAR_ALLOC_VAL;

	switch (hw->mac.type) {

	case ixgbe_mac_82598EB:
		if (type == -1)
			entry = IXGBE_IVAR_OTHER_CAUSES_INDEX;
		else
			entry += (type * 64);
		index = (entry >> 2) & 0x1F;
		ivar = IXGBE_READ_REG(hw, IXGBE_IVAR(index));
		ivar &= ~(0xFF << (8 * (entry & 0x3)));
		ivar |= (vector << (8 * (entry & 0x3)));
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_IVAR(index), ivar);
		break;

	case ixgbe_mac_82599EB:
	case ixgbe_mac_X540:
		if (type == -1) { /* MISC IVAR */
			index = (entry & 1) * 8;
			ivar = IXGBE_READ_REG(hw, IXGBE_IVAR_MISC);
			ivar &= ~(0xFF << index);
			ivar |= (vector << index);
			IXGBE_WRITE_REG(hw, IXGBE_IVAR_MISC, ivar);
		} else {	/* RX/TX IVARS */
			index = (16 * (entry & 1)) + (8 * type);
			ivar = IXGBE_READ_REG(hw, IXGBE_IVAR(entry >> 1));
			ivar &= ~(0xFF << index);
			ivar |= (vector << index);
			IXGBE_WRITE_REG(hw, IXGBE_IVAR(entry >> 1), ivar);
		}

	default:
		break;
	}
}

static void
ixgbe_configure_ivars(struct adapter *adapter)
{
	struct  ix_queue *que = adapter->queues;
	u32 newitr;

	if (ixgbe_max_interrupt_rate > 0)
		newitr = (4000000 / ixgbe_max_interrupt_rate) & 0x0FF8;
	else
		newitr = 0;

        for (int i = 0; i < adapter->num_queues; i++, que++) {
		/* First the RX queue entry */
                ixgbe_set_ivar(adapter, i, que->msix, 0);
		/* ... and the TX */
		ixgbe_set_ivar(adapter, i, que->msix, 1);
		/* Set an Initial EITR value */
                IXGBE_WRITE_REG(&adapter->hw,
                    IXGBE_EITR(que->msix), newitr);
	}

	/* For the Link interrupt */
        ixgbe_set_ivar(adapter, 1, adapter->linkvec, -1);
}

/*
** ixgbe_sfp_probe - called in the local timer to
** determine if a port had optics inserted.
*/  
static bool ixgbe_sfp_probe(struct adapter *adapter)
{
	struct ixgbe_hw	*hw = &adapter->hw;
	device_t	dev = adapter->hwdev;
	bool		result = FALSE;

	if ((hw->phy.type == ixgbe_phy_nl) &&
	    (hw->phy.sfp_type == ixgbe_sfp_type_not_present)) {
		s32 ret = hw->phy.ops.identify_sfp(hw);
		if (ret)
                        goto out;
		ret = hw->phy.ops.reset(hw);
		if (ret == IXGBE_ERR_SFP_NOT_SUPPORTED) {
			device_printf(dev,"Unsupported SFP+ module detected!");
			printf(" Reload driver with supported module.\n");
			adapter->sfp_probe = FALSE;
                        goto out;
		} else
			device_printf(dev,"SFP+ module detected!\n");
		/* We now have supported optics */
		adapter->sfp_probe = FALSE;
		/* Set the optics type so system reports correctly */
		ixgbe_setup_optics(adapter);
		result = TRUE;
	}
out:
	return (result);
}

/*
** Tasklet handler for MSIX Link interrupts
**  - do outside interrupt since it might sleep
*/
static void
ixgbe_handle_link(void *context, int pending)
{
	struct adapter  *adapter = context;

	ixgbe_check_link(&adapter->hw,
	    &adapter->link_speed, &adapter->link_up, 0);
	ixgbe_if_update_link_status(UPCAST(adapter));
}

/*
** Tasklet for handling SFP module interrupts
*/
static void
ixgbe_handle_mod(void *context, int pending)
{
	struct adapter  *adapter = context;
	struct ixgbe_hw *hw = &adapter->hw;
	device_t	dev = adapter->hwdev;
	u32 err;

	err = hw->phy.ops.identify_sfp(hw);
	if (err == IXGBE_ERR_SFP_NOT_SUPPORTED) {
		device_printf(dev,
		    "Unsupported SFP+ module type was detected.\n");
		return;
	}
	err = hw->mac.ops.setup_sfp(hw);
	if (err == IXGBE_ERR_SFP_NOT_SUPPORTED) {
		device_printf(dev,
		    "Setup failure - unsupported SFP+ module type.\n");
		return;
	}
	GROUPTASK_ENQUEUE(&adapter->msf_task);
}


/*
** Tasklet for handling MSF (multispeed fiber) interrupts
*/
static void
ixgbe_handle_msf(void *context, int pending)
{
	struct adapter  *adapter = context;
	struct ixgbe_hw *hw = &adapter->hw;
	u32 autoneg;
	bool negotiate;

	autoneg = hw->phy.autoneg_advertised;
	if ((!autoneg) && (hw->mac.ops.get_link_capabilities))
		hw->mac.ops.get_link_capabilities(hw, &autoneg, &negotiate);
	if (hw->mac.ops.setup_link)
		hw->mac.ops.setup_link(hw, autoneg, TRUE);
	return;
}

#ifdef IXGBE_FDIR
/*
** Tasklet for reinitializing the Flow Director filter table
*/
static void
ixgbe_reinit_fdir(void *context, int pending)
{
	struct adapter  *adapter = context;
	struct ifnet   *ifp = adapter->hwifp;

	if (adapter->fdir_reinit != 1) /* Shouldn't happen */
		return;
	ixgbe_reinit_fdir_tables_82599(&adapter->hw);
	adapter->fdir_reinit = 0;
	/* re-enable flow director interrupts */
	IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMS, IXGBE_EIMS_FLOW_DIR);
	/* Restart the interface */
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	return;
}
#endif

/**********************************************************************
 *
 *  Update the board statistics counters.
 *
 **********************************************************************/
static void
ixgbe_update_stats_counters(struct adapter *adapter)
{
	struct ifnet   *ifp = adapter->hwifp;
	struct ixgbe_hw *hw = &adapter->hw;
	u32  missed_rx = 0, bprc, lxon, lxoff, total;
	u64  total_missed_rx = 0;

	adapter->stats.crcerrs += IXGBE_READ_REG(hw, IXGBE_CRCERRS);
	adapter->stats.illerrc += IXGBE_READ_REG(hw, IXGBE_ILLERRC);
	adapter->stats.errbc += IXGBE_READ_REG(hw, IXGBE_ERRBC);
	adapter->stats.mspdc += IXGBE_READ_REG(hw, IXGBE_MSPDC);

	/*
	** Note: these are for the 8 possible traffic classes,
	**	 which in current implementation is unused,
	**	 therefore only 0 should read real data.
	*/
	for (int i = 0; i < 8; i++) {
		u32 mp;
		mp = IXGBE_READ_REG(hw, IXGBE_MPC(i));
		/* missed_rx tallies misses for the gprc workaround */
		missed_rx += mp;
		/* global total per queue */
        	adapter->stats.mpc[i] += mp;
		/* Running comprehensive total for stats display */
		total_missed_rx += adapter->stats.mpc[i];
		if (hw->mac.type == ixgbe_mac_82598EB) {
			adapter->stats.rnbc[i] +=
			    IXGBE_READ_REG(hw, IXGBE_RNBC(i));
			adapter->stats.qbtc[i] +=
			    IXGBE_READ_REG(hw, IXGBE_QBTC(i));
			adapter->stats.qbrc[i] +=
			    IXGBE_READ_REG(hw, IXGBE_QBRC(i));
			adapter->stats.pxonrxc[i] +=
		    	    IXGBE_READ_REG(hw, IXGBE_PXONRXC(i));
		} else
			adapter->stats.pxonrxc[i] +=
		    	    IXGBE_READ_REG(hw, IXGBE_PXONRXCNT(i));
		adapter->stats.pxontxc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXONTXC(i));
		adapter->stats.pxofftxc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXOFFTXC(i));
		adapter->stats.pxoffrxc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXOFFRXC(i));
		adapter->stats.pxon2offc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXON2OFFCNT(i));
	}
	for (int i = 0; i < 16; i++) {
		adapter->stats.qprc[i] += IXGBE_READ_REG(hw, IXGBE_QPRC(i));
		adapter->stats.qptc[i] += IXGBE_READ_REG(hw, IXGBE_QPTC(i));
		adapter->stats.qprdc[i] += IXGBE_READ_REG(hw, IXGBE_QPRDC(i));
	}
	adapter->stats.mlfc += IXGBE_READ_REG(hw, IXGBE_MLFC);
	adapter->stats.mrfc += IXGBE_READ_REG(hw, IXGBE_MRFC);
	adapter->stats.rlec += IXGBE_READ_REG(hw, IXGBE_RLEC);

	/* Hardware workaround, gprc counts missed packets */
	adapter->stats.gprc += IXGBE_READ_REG(hw, IXGBE_GPRC);
	adapter->stats.gprc -= missed_rx;

	if (hw->mac.type != ixgbe_mac_82598EB) {
		adapter->stats.gorc += IXGBE_READ_REG(hw, IXGBE_GORCL) +
		    ((u64)IXGBE_READ_REG(hw, IXGBE_GORCH) << 32);
		adapter->stats.gotc += IXGBE_READ_REG(hw, IXGBE_GOTCL) +
		    ((u64)IXGBE_READ_REG(hw, IXGBE_GOTCH) << 32);
		adapter->stats.tor += IXGBE_READ_REG(hw, IXGBE_TORL) +
		    ((u64)IXGBE_READ_REG(hw, IXGBE_TORH) << 32);
		adapter->stats.lxonrxc += IXGBE_READ_REG(hw, IXGBE_LXONRXCNT);
		adapter->stats.lxoffrxc += IXGBE_READ_REG(hw, IXGBE_LXOFFRXCNT);
	} else {
		adapter->stats.lxonrxc += IXGBE_READ_REG(hw, IXGBE_LXONRXC);
		adapter->stats.lxoffrxc += IXGBE_READ_REG(hw, IXGBE_LXOFFRXC);
		/* 82598 only has a counter in the high register */
		adapter->stats.gorc += IXGBE_READ_REG(hw, IXGBE_GORCH);
		adapter->stats.gotc += IXGBE_READ_REG(hw, IXGBE_GOTCH);
		adapter->stats.tor += IXGBE_READ_REG(hw, IXGBE_TORH);
	}

	/*
	 * Workaround: mprc hardware is incorrectly counting
	 * broadcasts, so for now we subtract those.
	 */
	bprc = IXGBE_READ_REG(hw, IXGBE_BPRC);
	adapter->stats.bprc += bprc;
	adapter->stats.mprc += IXGBE_READ_REG(hw, IXGBE_MPRC);
	if (hw->mac.type == ixgbe_mac_82598EB)
		adapter->stats.mprc -= bprc;

	adapter->stats.prc64 += IXGBE_READ_REG(hw, IXGBE_PRC64);
	adapter->stats.prc127 += IXGBE_READ_REG(hw, IXGBE_PRC127);
	adapter->stats.prc255 += IXGBE_READ_REG(hw, IXGBE_PRC255);
	adapter->stats.prc511 += IXGBE_READ_REG(hw, IXGBE_PRC511);
	adapter->stats.prc1023 += IXGBE_READ_REG(hw, IXGBE_PRC1023);
	adapter->stats.prc1522 += IXGBE_READ_REG(hw, IXGBE_PRC1522);

	lxon = IXGBE_READ_REG(hw, IXGBE_LXONTXC);
	adapter->stats.lxontxc += lxon;
	lxoff = IXGBE_READ_REG(hw, IXGBE_LXOFFTXC);
	adapter->stats.lxofftxc += lxoff;
	total = lxon + lxoff;

	adapter->stats.gptc += IXGBE_READ_REG(hw, IXGBE_GPTC);
	adapter->stats.mptc += IXGBE_READ_REG(hw, IXGBE_MPTC);
	adapter->stats.ptc64 += IXGBE_READ_REG(hw, IXGBE_PTC64);
	adapter->stats.gptc -= total;
	adapter->stats.mptc -= total;
	adapter->stats.ptc64 -= total;
	adapter->stats.gotc -= total * ETHER_MIN_LEN;

	adapter->stats.ruc += IXGBE_READ_REG(hw, IXGBE_RUC);
	adapter->stats.rfc += IXGBE_READ_REG(hw, IXGBE_RFC);
	adapter->stats.roc += IXGBE_READ_REG(hw, IXGBE_ROC);
	adapter->stats.rjc += IXGBE_READ_REG(hw, IXGBE_RJC);
	adapter->stats.mngprc += IXGBE_READ_REG(hw, IXGBE_MNGPRC);
	adapter->stats.mngpdc += IXGBE_READ_REG(hw, IXGBE_MNGPDC);
	adapter->stats.mngptc += IXGBE_READ_REG(hw, IXGBE_MNGPTC);
	adapter->stats.tpr += IXGBE_READ_REG(hw, IXGBE_TPR);
	adapter->stats.tpt += IXGBE_READ_REG(hw, IXGBE_TPT);
	adapter->stats.ptc127 += IXGBE_READ_REG(hw, IXGBE_PTC127);
	adapter->stats.ptc255 += IXGBE_READ_REG(hw, IXGBE_PTC255);
	adapter->stats.ptc511 += IXGBE_READ_REG(hw, IXGBE_PTC511);
	adapter->stats.ptc1023 += IXGBE_READ_REG(hw, IXGBE_PTC1023);
	adapter->stats.ptc1522 += IXGBE_READ_REG(hw, IXGBE_PTC1522);
	adapter->stats.bptc += IXGBE_READ_REG(hw, IXGBE_BPTC);
	adapter->stats.xec += IXGBE_READ_REG(hw, IXGBE_XEC);
	adapter->stats.fccrc += IXGBE_READ_REG(hw, IXGBE_FCCRC);
	adapter->stats.fclast += IXGBE_READ_REG(hw, IXGBE_FCLAST);
	/* Only read FCOE on 82599 */
	if (hw->mac.type != ixgbe_mac_82598EB) {
		adapter->stats.fcoerpdc += IXGBE_READ_REG(hw, IXGBE_FCOERPDC);
		adapter->stats.fcoeprc += IXGBE_READ_REG(hw, IXGBE_FCOEPRC);
		adapter->stats.fcoeptc += IXGBE_READ_REG(hw, IXGBE_FCOEPTC);
		adapter->stats.fcoedwrc += IXGBE_READ_REG(hw, IXGBE_FCOEDWRC);
		adapter->stats.fcoedwtc += IXGBE_READ_REG(hw, IXGBE_FCOEDWTC);
	}

	/* Fill out the OS statistics structure */
	ifp->if_ipackets = adapter->stats.gprc;
	ifp->if_opackets = adapter->stats.gptc;
	ifp->if_ibytes = adapter->stats.gorc;
	ifp->if_obytes = adapter->stats.gotc;
	ifp->if_imcasts = adapter->stats.mprc;
	ifp->if_omcasts = adapter->stats.mptc;
	ifp->if_collisions = 0;

	/* Rx Errors */
	ifp->if_iqdrops = total_missed_rx;
	ifp->if_ierrors = adapter->stats.crcerrs + adapter->stats.rlec;
}

/** ixgbe_sysctl_tdh_handler - Handler function
 *  Retrieves the TDH value from the hardware
 */
static int 
ixgbe_sysctl_tdh_handler(SYSCTL_HANDLER_ARGS)
{
	int error;

	struct tx_ring *txr = ((struct tx_ring *)oidp->oid_arg1);
	if (!txr) return 0;

	unsigned val = IXGBE_READ_REG(&txr->adapter->hw, IXGBE_TDH(txr->me));
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || !req->newptr)
		return error;
	return 0;
}

/** ixgbe_sysctl_tdt_handler - Handler function
 *  Retrieves the TDT value from the hardware
 */
static int 
ixgbe_sysctl_tdt_handler(SYSCTL_HANDLER_ARGS)
{
	int error;

	struct tx_ring *txr = ((struct tx_ring *)oidp->oid_arg1);
	if (!txr) return 0;

	unsigned val = IXGBE_READ_REG(&txr->adapter->hw, IXGBE_TDT(txr->me));
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || !req->newptr)
		return error;
	return 0;
}

/** ixgbe_sysctl_rdh_handler - Handler function
 *  Retrieves the RDH value from the hardware
 */
static int 
ixgbe_sysctl_rdh_handler(SYSCTL_HANDLER_ARGS)
{
	int error;

	struct rx_ring *rxr = ((struct rx_ring *)oidp->oid_arg1);
	if (!rxr) return 0;

	unsigned val = IXGBE_READ_REG(&rxr->adapter->hw, IXGBE_RDH(rxr->me));
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || !req->newptr)
		return error;
	return 0;
}

/** ixgbe_sysctl_rdt_handler - Handler function
 *  Retrieves the RDT value from the hardware
 */
static int 
ixgbe_sysctl_rdt_handler(SYSCTL_HANDLER_ARGS)
{
	int error;

	struct rx_ring *rxr = ((struct rx_ring *)oidp->oid_arg1);
	if (!rxr) return 0;

	unsigned val = IXGBE_READ_REG(&rxr->adapter->hw, IXGBE_RDT(rxr->me));
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || !req->newptr)
		return error;
	return 0;
}

static int
ixgbe_sysctl_interrupt_rate_handler(SYSCTL_HANDLER_ARGS)
{
	int error;
	struct ix_queue *que = ((struct ix_queue *)oidp->oid_arg1);
	unsigned int reg, usec, rate;

	reg = IXGBE_READ_REG(&que->adapter->hw, IXGBE_EITR(que->msix));
	usec = ((reg & 0x0FF8) >> 3);
	if (usec > 0)
		rate = 500000 / usec;
	else
		rate = 0;
	error = sysctl_handle_int(oidp, &rate, 0, req);
	if (error || !req->newptr)
		return error;
	reg &= ~0xfff; /* default, no limitation */
	ixgbe_max_interrupt_rate = 0;
	if (rate > 0 && rate < 500000) {
		if (rate < 1000)
			rate = 1000;
		ixgbe_max_interrupt_rate = rate;
		reg |= ((4000000/rate) & 0xff8 );
	}
	IXGBE_WRITE_REG(&que->adapter->hw, IXGBE_EITR(que->msix), reg);
	return 0;
}

/*
 * Add sysctl variables, one per statistic, to the system.
 */
static void
ixgbe_add_hw_stats(struct adapter *adapter)
{
	device_t dev = adapter->hwdev;
	struct tx_ring *txr = adapter->tx_rings;
	struct rx_ring *rxr = adapter->rx_rings;

	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(dev);
	struct sysctl_oid_list *child = SYSCTL_CHILDREN(tree);
	struct ixgbe_hw_stats *stats = &adapter->stats;

	struct sysctl_oid *stat_node, *queue_node;
	struct sysctl_oid_list *stat_list, *queue_list;

#define QUEUE_NAME_LEN 32
	char namebuf[QUEUE_NAME_LEN];

	/* Driver Statistics */
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "dropped",
			CTLFLAG_RD, &adapter->dropped_pkts,
			"Driver dropped packets");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "mbuf_defrag_failed",
			CTLFLAG_RD, &adapter->mbuf_defrag_failed,
			"m_defrag() failed");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "watchdog_events",
			CTLFLAG_RD, &adapter->watchdog_events,
			"Watchdog timeouts");
	SYSCTL_ADD_ULONG(ctx, child, OID_AUTO, "link_irq",
			CTLFLAG_RD, &adapter->link_irq,
			"Link MSIX IRQ Handled");

	for (int i = 0; i < adapter->num_queues; i++, txr++) {
		snprintf(namebuf, QUEUE_NAME_LEN, "queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf,
					    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "interrupt_rate",
				CTLTYPE_UINT | CTLFLAG_RW, &adapter->queues[i],
				sizeof(&adapter->queues[i]),
				ixgbe_sysctl_interrupt_rate_handler, "IU",
				"Interrupt Rate");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "irqs",
				CTLFLAG_RD, &(adapter->queues[i].irqs),
				"irqs on this queue");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "txd_head", 
				CTLTYPE_UINT | CTLFLAG_RD, txr, sizeof(txr),
				ixgbe_sysctl_tdh_handler, "IU",
				"Transmit Descriptor Head");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "txd_tail", 
				CTLTYPE_UINT | CTLFLAG_RD, txr, sizeof(txr),
				ixgbe_sysctl_tdt_handler, "IU",
				"Transmit Descriptor Tail");
		SYSCTL_ADD_ULONG(ctx, queue_list, OID_AUTO, "tso_tx",
				CTLFLAG_RD, &txr->tso_tx,
				"TSO");
		SYSCTL_ADD_ULONG(ctx, queue_list, OID_AUTO, "no_tx_dma_setup",
				CTLFLAG_RD, &txr->no_tx_dma_setup,
				"Driver tx dma failure in xmit");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "no_desc_avail",
				CTLFLAG_RD, &txr->no_desc_avail,
				"Queue No Descriptor Available");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "tx_packets",
				CTLFLAG_RD, &txr->total_packets,
				"Queue Packets Transmitted");
	}

	for (int i = 0; i < adapter->num_queues; i++, rxr++) {
		snprintf(namebuf, QUEUE_NAME_LEN, "queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf, 
					    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

#if 0
		struct lro_ctrl *lro = &rxr->lro;
#endif
		snprintf(namebuf, QUEUE_NAME_LEN, "queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf, 
					    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "rxd_head", 
				CTLTYPE_UINT | CTLFLAG_RD, rxr, sizeof(rxr),
				ixgbe_sysctl_rdh_handler, "IU",
				"Receive Descriptor Head");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "rxd_tail", 
				CTLTYPE_UINT | CTLFLAG_RD, rxr, sizeof(rxr),
				ixgbe_sysctl_rdt_handler, "IU",
				"Receive Descriptor Tail");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_packets",
				CTLFLAG_RD, &rxr->rx_packets,
				"Queue Packets Received");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_bytes",
				CTLFLAG_RD, &rxr->rx_bytes,
				"Queue Bytes Received");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_copies",
				CTLFLAG_RD, &rxr->rx_copies,
						 "Copied RX Frames");
#if 0		
		SYSCTL_ADD_INT(ctx, queue_list, OID_AUTO, "lro_queued",
				CTLFLAG_RD, &lro->lro_queued, 0,
				"LRO Queued");
		SYSCTL_ADD_INT(ctx, queue_list, OID_AUTO, "lro_flushed",
				CTLFLAG_RD, &lro->lro_flushed, 0,
					   "LRO Flushed");
#endif		
	}

	/* MAC stats get the own sub node */

	stat_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "mac_stats", 
				    CTLFLAG_RD, NULL, "MAC Statistics");
	stat_list = SYSCTL_CHILDREN(stat_node);

	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "crc_errs",
			CTLFLAG_RD, &stats->crcerrs,
			"CRC Errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "ill_errs",
			CTLFLAG_RD, &stats->illerrc,
			"Illegal Byte Errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "byte_errs",
			CTLFLAG_RD, &stats->errbc,
			"Byte Errors");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "short_discards",
			CTLFLAG_RD, &stats->mspdc,
			"MAC Short Packets Discarded");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "local_faults",
			CTLFLAG_RD, &stats->mlfc,
			"MAC Local Faults");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "remote_faults",
			CTLFLAG_RD, &stats->mrfc,
			"MAC Remote Faults");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rec_len_errs",
			CTLFLAG_RD, &stats->rlec,
			"Receive Length Errors");

	/* Flow Control stats */
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "xon_txd",
			CTLFLAG_RD, &stats->lxontxc,
			"Link XON Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "xon_recvd",
			CTLFLAG_RD, &stats->lxonrxc,
			"Link XON Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "xoff_txd",
			CTLFLAG_RD, &stats->lxofftxc,
			"Link XOFF Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "xoff_recvd",
			CTLFLAG_RD, &stats->lxoffrxc,
			"Link XOFF Received");

	/* Packet Reception Stats */
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "total_octets_rcvd",
			CTLFLAG_RD, &stats->tor, 
			"Total Octets Received"); 
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_octets_rcvd",
			CTLFLAG_RD, &stats->gorc, 
			"Good Octets Received"); 
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "total_pkts_rcvd",
			CTLFLAG_RD, &stats->tpr,
			"Total Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_pkts_rcvd",
			CTLFLAG_RD, &stats->gprc,
			"Good Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_rcvd",
			CTLFLAG_RD, &stats->mprc,
			"Multicast Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "bcast_pkts_rcvd",
			CTLFLAG_RD, &stats->bprc,
			"Broadcast Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_64",
			CTLFLAG_RD, &stats->prc64,
			"64 byte frames received ");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_65_127",
			CTLFLAG_RD, &stats->prc127,
			"65-127 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_128_255",
			CTLFLAG_RD, &stats->prc255,
			"128-255 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_256_511",
			CTLFLAG_RD, &stats->prc511,
			"256-511 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_512_1023",
			CTLFLAG_RD, &stats->prc1023,
			"512-1023 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rx_frames_1024_1522",
			CTLFLAG_RD, &stats->prc1522,
			"1023-1522 byte frames received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_undersized",
			CTLFLAG_RD, &stats->ruc,
			"Receive Undersized");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_fragmented",
			CTLFLAG_RD, &stats->rfc,
			"Fragmented Packets Received ");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_oversized",
			CTLFLAG_RD, &stats->roc,
			"Oversized Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "recv_jabberd",
			CTLFLAG_RD, &stats->rjc,
			"Received Jabber");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "management_pkts_rcvd",
			CTLFLAG_RD, &stats->mngprc,
			"Management Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "management_pkts_drpd",
			CTLFLAG_RD, &stats->mngptc,
			"Management Packets Dropped");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "checksum_errs",
			CTLFLAG_RD, &stats->xec,
			"Checksum Errors");

	/* Packet Transmission Stats */
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_octets_txd",
			CTLFLAG_RD, &stats->gotc, 
			"Good Octets Transmitted"); 
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "total_pkts_txd",
			CTLFLAG_RD, &stats->tpt,
			"Total Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_pkts_txd",
			CTLFLAG_RD, &stats->gptc,
			"Good Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "bcast_pkts_txd",
			CTLFLAG_RD, &stats->bptc,
			"Broadcast Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_txd",
			CTLFLAG_RD, &stats->mptc,
			"Multicast Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "management_pkts_txd",
			CTLFLAG_RD, &stats->mngptc,
			"Management Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_64",
			CTLFLAG_RD, &stats->ptc64,
			"64 byte frames transmitted ");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_65_127",
			CTLFLAG_RD, &stats->ptc127,
			"65-127 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_128_255",
			CTLFLAG_RD, &stats->ptc255,
			"128-255 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_256_511",
			CTLFLAG_RD, &stats->ptc511,
			"256-511 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_512_1023",
			CTLFLAG_RD, &stats->ptc1023,
			"512-1023 byte frames transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "tx_frames_1024_1522",
			CTLFLAG_RD, &stats->ptc1522,
			"1024-1522 byte frames transmitted");
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
ixgbe_set_flowcntl(SYSCTL_HANDLER_ARGS)
{
	int error, last;
	struct adapter *adapter = (struct adapter *) arg1;

	last = adapter->fc;
	error = sysctl_handle_int(oidp, &adapter->fc, 0, req);
	if ((error) || (req->newptr == NULL))
		return (error);

	/* Don't bother if it's not changed */
	if (adapter->fc == last)
		return (0);

	switch (adapter->fc) {
		case ixgbe_fc_rx_pause:
		case ixgbe_fc_tx_pause:
		case ixgbe_fc_full:
			adapter->hw.fc.requested_mode = adapter->fc;
			if (adapter->num_queues > 1)
				ixgbe_disable_rx_drop(adapter);
			break;
		case ixgbe_fc_none:
			adapter->hw.fc.requested_mode = ixgbe_fc_none;
			if (adapter->num_queues > 1)
				ixgbe_enable_rx_drop(adapter);
			break;
		default:
			adapter->fc = last;
			return (EINVAL);
	}
	/* Don't autoneg if forcing a value */
	adapter->hw.fc.disable_fc_autoneg = TRUE;
	ixgbe_fc_enable(&adapter->hw);
	return error;
}

/*
** Control link advertise speed:
**	1 - advertise only 1G
**	2 - advertise 100Mb
**	3 - advertise normal
*/
static int
ixgbe_set_advertise(SYSCTL_HANDLER_ARGS)
{
	int			error = 0;
	struct adapter		*adapter;
	device_t		dev;
	struct ixgbe_hw		*hw;
	ixgbe_link_speed	speed, last;

	adapter = (struct adapter *) arg1;
	dev = adapter->hwdev;
	hw = &adapter->hw;
	last = adapter->advertise;

	error = sysctl_handle_int(oidp, &adapter->advertise, 0, req);
	if ((error) || (req->newptr == NULL))
		return (error);

	if (adapter->advertise == last) /* no change */
		return (0);

	if (!((hw->phy.media_type == ixgbe_media_type_copper) ||
            (hw->phy.multispeed_fiber)))
		return (EINVAL);

	if ((adapter->advertise == 2) && (hw->mac.type != ixgbe_mac_X540)) {
		device_printf(dev, "Set Advertise: 100Mb on X540 only\n");
		return (EINVAL);
	}

	if (adapter->advertise == 1)
                speed = IXGBE_LINK_SPEED_1GB_FULL;
	else if (adapter->advertise == 2)
                speed = IXGBE_LINK_SPEED_100_FULL;
	else if (adapter->advertise == 3)
                speed = IXGBE_LINK_SPEED_1GB_FULL |
			IXGBE_LINK_SPEED_10GB_FULL;
	else {	/* bogus value */
		adapter->advertise = last;
		return (EINVAL);
	}

	hw->mac.autotry_restart = TRUE;
	hw->mac.ops.setup_link(hw, speed, TRUE);

	return (error);
}

/*
** Thermal Shutdown Trigger
**   - cause a Thermal Overtemp IRQ
**   - this now requires firmware enabling
*/
static int
ixgbe_set_thermal_test(SYSCTL_HANDLER_ARGS)
{
	int		error, fire = 0;
	struct adapter	*adapter = (struct adapter *) arg1;
	struct ixgbe_hw *hw = &adapter->hw;


	if (hw->mac.type != ixgbe_mac_X540)
		return (0);

	error = sysctl_handle_int(oidp, &fire, 0, req);
	if ((error) || (req->newptr == NULL))
		return (error);

	if (fire) {
		u32 reg = IXGBE_READ_REG(hw, IXGBE_EICS);
		reg |= IXGBE_EICR_TS;
		IXGBE_WRITE_REG(hw, IXGBE_EICS, reg);
	}

	return (0);
}

/*
** Enable the hardware to drop packets when the buffer is
** full. This is useful when multiqueue,so that no single
** queue being full stalls the entire RX engine. We only
** enable this when Multiqueue AND when Flow Control is 
** disabled.
*/
static void
ixgbe_enable_rx_drop(struct adapter *adapter)
{
        struct ixgbe_hw *hw = &adapter->hw;

	for (int i = 0; i < adapter->num_queues; i++) {
        	u32 srrctl = IXGBE_READ_REG(hw, IXGBE_SRRCTL(i));
        	srrctl |= IXGBE_SRRCTL_DROP_EN;
        	IXGBE_WRITE_REG(hw, IXGBE_SRRCTL(i), srrctl);
	}
}

static void
ixgbe_disable_rx_drop(struct adapter *adapter)
{
        struct ixgbe_hw *hw = &adapter->hw;

	for (int i = 0; i < adapter->num_queues; i++) {
        	u32 srrctl = IXGBE_READ_REG(hw, IXGBE_SRRCTL(i));
        	srrctl &= ~IXGBE_SRRCTL_DROP_EN;
        	IXGBE_WRITE_REG(hw, IXGBE_SRRCTL(i), srrctl);
	}
}
