#ifndef __IFLIB_H_
#define __IFLIB_H_
#include "iflib_if.h"

struct iflib_ctx;
typedef struct iflib_ctx *iflib_ctx_t;

typedef struct iflib_irq {
	struct resource  *ifi_res;
	int               ifi_rid;
	void             *ifi_tag;
} *iflib_irq_t;

#define RXD_SOP       (1 << 0)
#define RXD_SOP_EOP   (1 << 1)
#define RXD_EOP       (1 << 2)
#define RXD_NSOP_NEOP (1 << 3)
#define RXD_VLAN      (1 << 4)

typedef struct rxd_info {
	uint16_t ri_qidx;
	uint16_t ri_vtag;
	int      ri_flags;
	int      ri_cidx;
	uint32_t ri_len;
	struct mbuf *ri_m;
	uint64_t ri_csum_flags;
	uint64_t ri_csum_data;
} *rxd_info_t;

typedef struct if_pkt_info {
	bus_dma_segment_t *ipi_segs;
	uint16_t ipi_vtag;
	uint16_t ipi_nsegs;
	uint32_t ipi_flags;
	uint64_t ipi_csum_flags;
	uint64_t ipi_csum_data;
	uint32_t ipi_first;
	uint32_t ipi_last;
} if_pkt_info_t;

typedef struct iflib_shared_ctx {
	iflib_ctx_t isc_ctx;
	struct ifmedia	isc_media;

	/*
	 * Shared stats or whatever
	 */
} iflib_shared_ctx_t;


int iflib_attach(device_t dev, driver_t *driver);
int iflib_queues_alloc(iflib_ctx_t ctx, int txq_size, int rxq_size);
void iflib_active_clear(iflib_ctx_t);
void iflib_led_create(iflib_ctx_t);
void iflib_tx_structures_setup(iflib_ctx_t);
void iflib_rx_structures_setup(iflib_ctx_t);
void iflib_tx_structures_free(iflib_ctx_t);
void iflib_rx_structures_free(iflib_ctx_t);
void iflib_ctx_free(iflib_ctx_t);
void *iflib_softc_get(iflib_ctx_t);
device_t iflib_device_get(iflib_ctx_t);
void iflib_init(iflib_ctx_t);
caddr_t iflib_lladdr_get(iflib_ctx_t);
void iflib_hwassist_set(iflib_ctx_t);
int iflib_active_get(iflib_ctx_t ctx);
void iflib_active_clear(iflib_ctx_t ctx);
int iflib_printf(iflib_ctx_t ctx, const char *, ...) __printflike(2, 3);
void iflib_promisc_config(iflib_ctx_t);
void iflib_legacy_intr_deferred(iflib_ctx_t);
void iflib_link_intr_deferred(iflib_ctx_t);
int iflib_multiaddr_count(iflib_ctx_t, int);
void iflib_multiaddr_array(iflib_ctx_t, u8 *, int *, int);
void iflib_linkstate_change(iflib_ctx_t, uint64_t, int);
int iflib_legacy_setup(iflib_ctx_t, driver_filter_t, int *);
int iflib_irq_alloc(iflib_ctx_t, iflib_irq_t, int, driver_intr_t, void *arg, char *name);
int iflib_irq_alloc_generic(iflib_ctx_t ctx, iflib_irq_t *irq, int rid,
							intr_type_t type, void *arg, char *name);
int iflib_mtu_get(iflib_ctx_t);
void iflib_tx_tag_prop_set(iflib_ctx_t, int field_name, uint64_t value);
void iflib_rx_tag_prop_set(iflib_ctx_t, int field_name, uint64_t value);
void iflib_queue_tag_prop_set(iflib_ctx_t, int field_name, uint64_t value);
void iflib_capabilitiesbit_set(iflib_ctx_t, int);
int iflib_capenable_set(iflib_ctx_t);

void iflib_ifheaderlen_set(iflib_ctx_t, int);
void iflib_tx_hwq_set(iflib_ctx_t, void *hwq, int idx);
void iflib_rx_hwq_set(iflib_ctx_t, void *hwq, int idx);
void *iflib_txq_vaddr_get(iflib_ctx_t, int idx);
void *iflib_rxq_vaddr_get(iflib_ctx_t, int idx);

#endif /*  __IFLIB_H_ */
