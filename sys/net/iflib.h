/*-
 * Copyright (c) 2014, Matthew Macy (kmacy@freebsd.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Neither the name of Matthew Macy nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __IFLIB_H_
#define __IFLIB_H_

struct iflib_ctx;
typedef struct iflib_ctx *iflib_ctx_t;
struct if_shared_ctx;
typedef struct if_shared_ctx *if_shared_ctx_t;
struct if_int_delay_info;
typedef struct if_int_delay_info  *if_int_delay_info_t;

/*
 * File organization:
 *  - public structures
 *  - iflib accessors
 *  - iflib utility functions
 *  - iflib core functions
 */

#define	IF_RXD_SOP				(1 << 0)
#define	IF_RXD_SOP_EOP		(1 << 1)
#define	IF_RXD_EOP				(1 << 2)
#define	IF_RXD_NSOP_NEOP	(1 << 3)
#define	IF_RXD_VLAN				(1 << 4)

typedef struct if_rxd_info {
	uint16_t iri_qidx;
	uint16_t iri_vtag;
	int      iri_flags;
	int      iri_cidx;
	uint32_t iri_len;
	struct mbuf *iri_m;
	uint32_t iri_csum_flags;
	uint32_t iri_csum_data;
} *if_rxd_info_t;

typedef struct if_pkt_info {
	bus_dma_segment_t *ipi_segs;
	caddr_t	ipi_pkt_hdr;
	uint16_t ipi_nsegs;
	uint16_t ipi_vtag;
	uint32_t ipi_pidx;
	uint32_t ipi_csum_flags;
	uint16_t ipi_flags;
	uint16_t ipi_tso_segsz;
	uint32_t ipi_new_pidx;
	uint16_t ipi_pktlen;
} *if_pkt_info_t;

struct if_common_stats {
	uint64_t ics_colls;
	uint64_t ics_ierrs;
	uint64_t ics_oerrs;
};

typedef struct if_irq {
	struct resource  *ii_res;
	int               ii_rid;
	void             *ii_tag;
} *if_irq_t;

struct if_int_delay_info {
	if_shared_ctx_t iidi_sctx;	/* Back-pointer to the shared ctx (softc) */
	int iidi_offset;			/* Register offset to read/write */
	int iidi_value;			/* Current value in usecs */
	struct sysctl_oid *iidi_oidp;
	struct sysctl_req *iidi_req;
};

/*
 * Context shared between the driver and the iflib layer
 * Is treated as a superclass of the driver's softc, so
 * must be the first element
 */
struct if_shared_ctx {
	/*
	 * KOBJ requires that the following be the first field
	 * Do not move
	 */
	KOBJ_FIELDS;
	int (*isc_txd_encap) (if_shared_ctx_t, int, if_pkt_info_t);
	void (*isc_txd_flush) (if_shared_ctx_t, int, int);

	int (*isc_rxd_is_new) (if_shared_ctx_t, int, int);
	int (*isc_rxd_pkt_get) (if_shared_ctx_t, int, int, if_rxd_info_t);
	void (*isc_rxd_refill) (if_shared_ctx_t, int, int, uint64_t);
	void (*isc_rxd_flush) (if_shared_ctx_t, int, int);

	iflib_ctx_t isc_ctx;
	device_t isc_dev;
	if_t isc_ifp;
	int isc_nqsets;
	int isc_nrxq;
	int isc_ntxd;
	int isc_nrxd;
	bus_size_t isc_q_align;
	bus_size_t isc_tx_maxsize;
	bus_size_t isc_tx_maxsegsize;
	int isc_tx_nsegments;
	bus_size_t isc_rx_maxsize;
	bus_size_t isc_rx_maxsegsize;
	int isc_rx_nsegments;
	int isc_rx_process_limit;
	int isc_pause_frames;
	int isc_watchdog_events;
	int isc_tx_reclaim_thresh;

	struct ifmedia	isc_media;
	struct if_common_stats isc_common_stats;
};

typedef enum {
	IFLIB_INTR_TX,
	IFLIB_INTR_RX,
	IFLIB_INTR_LINK,
} intr_type_t;

#define UPCAST(sc) ((if_shared_ctx_t)(sc))
#ifndef ETH_ADDR_LEN
#define ETH_ADDR_LEN 6
#endif

int iflib_device_detach(device_t);
int iflib_device_suspend(device_t);
int iflib_device_resume(device_t);

int iflib_register(device_t dev, driver_t *driver, uint8_t addr[ETH_ADDR_LEN]);

int iflib_queues_alloc(if_shared_ctx_t ctx, int txq_size, int rxq_size);

int iflib_tx_structures_setup(if_shared_ctx_t);
void iflib_tx_structures_free(if_shared_ctx_t);
int iflib_rx_structures_setup(if_shared_ctx_t);
void iflib_rx_structures_free(if_shared_ctx_t);

void iflib_txq_addr_get(if_shared_ctx_t, int idx, uint64_t addrs[2]);
void iflib_rxq_addr_get(if_shared_ctx_t, int idx, uint64_t addrs[2]);

int iflib_irq_alloc(if_shared_ctx_t, if_irq_t, int, driver_filter_t, driver_intr_t, void *arg, char *name);
int iflib_irq_alloc_generic(if_shared_ctx_t ctx, if_irq_t irq, int rid,
							intr_type_t type, driver_filter_t *filter, int qid, char *name);

int iflib_legacy_setup(if_shared_ctx_t, driver_filter_t, int *);
void iflib_led_create(if_shared_ctx_t);

void iflib_init(if_shared_ctx_t);

void iflib_legacy_intr_deferred(if_shared_ctx_t);
void iflib_link_intr_deferred(if_shared_ctx_t);
void iflib_link_state_change(if_shared_ctx_t, uint64_t, int);

int iflib_tx_cidx_get(if_shared_ctx_t, int);
void iflib_tx_credits_update(if_shared_ctx_t, int, int);


void iflib_stats_update(if_shared_ctx_t);
void iflib_add_int_delay_sysctl(if_shared_ctx_t, const char *, const char *,
								if_int_delay_info_t, int, int);

#endif /*  __IFLIB_H_ */
