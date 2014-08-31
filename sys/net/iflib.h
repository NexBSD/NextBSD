#ifndef __IFLIB_H_
#define __IFLIB_H_

struct iflib_ctx;
typedef struct iflib_ctx *iflib_ctx_t;
#include "iflib_if.h"


/*
 * File organization:
 *  - public structures
 *  - iflib accessors
 *  - iflib utility functions
 *  - iflib core functions
 */

typedef struct if_irq {
	struct resource  *ii_res;
	int               ii_rid;
	void             *ii_tag;
} *if_irq_t;

#define RXD_SOP       (1 << 0)
#define RXD_SOP_EOP   (1 << 1)
#define RXD_EOP       (1 << 2)
#define RXD_NSOP_NEOP (1 << 3)
#define RXD_VLAN      (1 << 4)

typedef struct if_rxd_info {
	uint16_t iri_qidx;
	uint16_t iri_vtag;
	int      iri_flags;
	int      iri_cidx;
	uint32_t iri_len;
	struct mbuf *iri_m;
	uint64_t iri_csum_flags;
	uint64_t rii_csum_data;
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

struct if_common_stats {
	uint64_t ics_colls;
	uint64_t ics_ierrs;
	uint64_t ics_oerrs;
};

/*
 * Context shared between the driver and the iflib layer
 * Is treated as a superclass of the driver's softc, so
 * must be the first element
 */
typedef struct if_shared_ctx {
	/*
	 * KOBJ requires that the following be the first field
	 * Do not move
	 */
	KOBJ_FIELDS;
	iflib_ctx_t isc_ctx;
	device_t isc_dev;
	if_t isc_ifp;
	struct ifmedia	isc_media;
	struct if_common_stats isc_common_stats;
} if_shared_ctx_t;

#define UPCAST(sc) ((if_shared_ctx_t)(sc))

int iflib_attach(device_t dev, driver_t *driver);
int iflib_detach(device_t dev);
int iflib_suspend(device_t dev);
int iflib_resume(device_t dev);

int iflib_queues_alloc(if_shared_ctx_t ctx, int txq_size, int rxq_size);

void iflib_tx_tag_prop_set(if_shared_ctx_t, int field_name, uint64_t value);
void iflib_rx_tag_prop_set(if_shared_ctx_t, int field_name, uint64_t value);
void iflib_queue_tag_prop_set(if_shared_ctx_t, int field_name, uint64_t value);

void iflib_tx_structures_setup(if_shared_ctx_t);
void iflib_tx_structures_free(if_shared_ctx_t);
void iflib_rx_structures_setup(if_shared_ctx_t);
void iflib_rx_structures_free(if_shared_ctx_t);
void iflib_ctx_free(if_shared_ctx_t);

void iflib_txq_addr_get(if_shared_ctx_t, int idx, uint64_t addrs[2]);
void iflib_rxq_addr_get(if_shared_ctx_t, int idx, uint64_t addrs[2]);

int iflib_irq_alloc(if_shared_ctx_t, if_irq_t, int, driver_intr_t, void *arg, char *name);
int iflib_irq_alloc_generic(if_shared_ctx_t ctx, if_irq_t irq, int rid,
							intr_type_t type, void *arg, char *name);

int iflib_legacy_setup(if_shared_ctx_t, driver_filter_t, int *);
void iflib_led_create(if_shared_ctx_t);

void iflib_init(if_shared_ctx_t);

void iflib_legacy_intr_deferred(if_shared_ctx_t);
void iflib_link_intr_deferred(if_shared_ctx_t);
void iflib_linkstate_change(if_shared_ctx_t, uint64_t, int);

void iflib_stats_update(if_shared_ctx_t);

#endif /*  __IFLIB_H_ */
