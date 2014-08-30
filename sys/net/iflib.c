#include <sys/param.h>
#include <sys/kobj.h>

#include <net/iflib.h>
#include "iflib_if.h"


struct iflib_ctx {
	KOBJ_FIELDS;
	/*
	 * Pointer to hardware driver's softc
	 */
	iflib_shared_ctx_t *ifc_sctx;
	int ifc_ntxq;
	int ifc_ntxd;
	int ifc_nrxq;
	int ifc_nrxd;
	if_t ifc_ifp;
	device_t ifc_dev;
	iflib_txq_t ifc_txq;
	iflib_rxq_t ifc_rxq;
};

typedef struct iflib_txq {
	iflib_ctx_t             ift_ctx;
	struct mtx              ift_mtx;
	char                    ift_mtx_name[16];
	iflib_buffer_t          ift_bufs;
	struct buf_ring        *ift_br;
	bus_dma_tag_t		    ift_txtag;
	u32                     ift_next_avail_desc;
	u32                     ift_next_to_clean;
	volatile u16            ift_tx_avail;
	unsigned long		    ift_no_desc_avail;
	void                   *ift_hw_txr;
	struct task             ift_task;
	int			            ift_qstatus;
	int                     ift_watchdog_time;

} *iflib_txq_t;

typedef struct iflib_rxq {
	iflib_ctx_t             ifr_ctx;
	struct mtx              ifr_mtx;
	char                    ifr_mtx_name[16];
	struct task             ifr_task;
	iflib_sd_t              ifr_sds;
	bus_dma_tag_t           ifr_tag;
} *iflib_rxq_t;


#define RX_SW_DESC_MAP_CREATED	(1 << 0)
#define TX_SW_DESC_MAP_CREATED	(1 << 1)
#define RX_SW_DESC_INUSE        (1 << 3)
#define TX_SW_DESC_MAPPED       (1 << 4)

typedef struct iflib_sw_desc {
	bus_dmamap_t    ifsd_map;         /* bus_dma map for packet */
	struct mbuf    *ifsd_m;           /* rx: uninitialized mbuf
									   * tx: pkthdr for the packet
x									   */
	caddr_t         ifsd_cl;          /* direct cluster pointer for rx */
	/* XXX needed? */
	int		        ifsd_next_eop;    /* Index of the last buffer */
	int             ifsd_flags;
} *iflib_sd_t;

struct iflib_irq {
	struct resource  *ifi_res;
	int               ifi_rid;
	void             *ifi_tag;
};

struct iflib_dma_info_t {
        bus_addr_t              ifd_paddr;
        caddr_t                 ifd_vaddr;
        bus_dma_tag_t           ifd_tag;
        bus_dmamap_t            ifd_map;
        bus_dma_segment_t       ifd_seg;
        int                     ifd_nseg;
};

void *
iflib_softc_get(iflib_ctx_t ctx)
{

	return (ctx->ifc_sctx);
}

void *
iflib_device_get(iflib_ctx_t ctx)
{

	return (ctx->ifc_dev);
}


static void
_recycle_rx_buf(iflib_ctx_t ctx, iflib_rxq_t rxq, uint32_t idx)
{
	rxq->ifr_sds[rxq->ifr_pidx] = rxq->ifr_sds[idx];
	IFC_RECYCLE_RX_BUF(ctx, rxq, idx);
	rxq->ifr_credits++;
	if (++rxq->ifr_pidx == rxq->ifr_size)
		q->pidx = 0;
}

/*
 * Internal service routines
 */

#if !defined(__i386__) && !defined(__amd64__)
struct refill_rxq_cb_arg {
	int               error;
	bus_dma_segment_t seg;
	int               nseg;
};

static void
_refill_rxq_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct refill_rxq_cb_arg *cb_arg = arg;
	
	cb_arg->error = error;
	cb_arg->seg = segs[0];
	cb_arg->nseg = nseg;

}
#endif

/**
 *	refill_rxq - refill an SGE free-buffer list
 *	@sc: the controller softc
 *	@q: the free-list to refill
 *	@n: the number of new buffers to allocate
 *
 *	(Re)populate an SGE free-buffer list with up to @n new packet buffers.
 *	The caller must assure that @n does not exceed the queue's capacity.
 */
static void
_iflib_refill_rxq(iflib_ctx_t ctx, iflib_rxq_t rxq, int n)
{
	struct rx_sw_desc *rxsd = &rxq->ifr_sd[rxq->ifr_pidx];
	struct mbuf *m;
	caddr_t cl;
	int err;
	uint64_t phys_addr;
	
	while (n--) {
		/*
		 * We allocate an uninitialized mbuf + cluster, mbuf is
		 * initialized after rx.
		 */
		if ((cl = m_cljget(NULL, M_NOWAIT, rxq->ifr_buf_size)) == NULL)
			break;
		if ((m = m_gethdr(M_NOWAIT, MT_NOINIT)) == NULL) {
			uma_zfree(rxq->ifr_zone, cl);
			break;
		}
		if ((rxsd->ifsd_flags & RX_SW_DESC_MAP_CREATED) == 0) {
			if ((err = bus_dmamap_create(rxq->ifr_qtag, 0, &rxsd->ifsd_map))) {
				log(LOG_WARNING, "bus_dmamap_create failed %d\n", err);
				uma_zfree(rxq->ifr_zone, cl);
				goto done;
			}
			rxsd->ifsd_flags |= RX_SW_DESC_MAP_CREATED;
		}
#if !defined(__i386__) && !defined(__amd64__)
		{
			struct refill_fl_cb_arg cb_arg;
			cb_arg.error = 0;
			err = bus_dmamap_load(q->entry_tag, sd->map,
		         cl, q->buf_size, refill_fl_cb, &cb_arg, 0);
		
			if (err != 0 || cb_arg.error) {
				if (q->zone == zone_pack)
					uma_zfree(q->zone, cl);
				m_free(m);
				goto done;
			}
			phys_addr = cb_arg.seg.ds_addr;
		}
#else
		phys_addr = pmap_kextract((vm_offset_t)cl);
#endif		
		rxsd->ifsd_flags |= RX_SW_DESC_INUSE;
		rxsd->ifsd_cl = cl;
		rxsd->ifsd_m = m;
		IFC_RXD_REFILL_FLUSH(ctx, rxq->ifr_id, rxq->ifr_pidx, phys_addr);

		if (++rxq->ifr_pidx == rxq->ifr_qsize) {
			rxq->ifr_pidx = 0;
			rxsd = rxq->ifr_sd;
		}
		rxq->ifr_credits++;
	}

done:
	IFC_RXD_REFILL_FLUSH(ctx, q->ifr_id, q->ifr_pidx);
}

/*
 * Process one software descriptor
 */
static void
_iflib_packet_get(iflib_ctx_t ctx, rxd_info_t ri)
{
	iflib_rxq_t rxq = ctx->ifc_rxqs[ri->qidx];
	iflib_sd_t sd = &rxq->ift_sds[ri->cidx];
	uint32_t flags = M_EXT;
	caddr_t cl;
	struct mbuf *m;
	int len = ri->ri_len;
	/* 
	 * most drivers only support 1 segment on receive
	 * so ignore chaining to start
	 */
	if (ri->ri_flags != RXD_SOP_EOP)
		panic("chaining unsupported");

	if (recycle_enable & ri->ri_len <= IFLIB_RX_COPY_THRES) {
		if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL)
			goto skip_recycle;
		cl = mtod(m, void *);
		memcpy(cl, sd->ifsd_cl, ri->ri_len);
		_recycle_rx_buf(ctx, rxq, ri->ri_cidx);
		m->m_pkthdr.len = m->m_len = len;
		m->m_flags = 0;
	} else {
	skip_recycle:
		bus_dmamap_unload(rxq->ifr_dtag, sd->map);
		cl = sd->ifs_cl;
		m = sd->ifs_m;
		flags |= M_PKTHDR;
		m_init(m, rxq->ifr_zone, rxq->ifr_buf_size, M_NOWAIT, MT_DATA, flags);
		m_cljset(m, cl, rxq->ifr_cltype);
		m->m_len = len;
		m->m_pkthdr.len = len;
	}
	m->m_pkthdr.rcvif = (struct ifnet *)ctx->ifc_ifp;
	if (ri->ri_flags & RXD_VLAN) {
		if_setvtag(m, ri->ri_vtag);
		m->m_flags |= M_VLANTAG;
	}
	ri->ri_m = m;
	
	/*
	 *  XXX should be per-cpu counter
	 */
	if_incipackets(ifp, 1);
}

static bool
iflib_rxeof(iflib_rxq_t rxq, int budget)
{
	iflib_ctx_t ctx = rxq->ctx;
	int cidx = rxq->ifr_cidx;
	struct rxd_info ri;

	/*
	 * XXX early demux data packets so that if_input processing only handles
	 * acks in interrupt context
	 */
	struct mbuf *mh, *mt;

	if (!RX_TRYLOCK(rxq))
		return (false);
#ifdef DEV_NETMAP
	if (netmap_rx_irq(ifp, rxr->me, &processed)) {
		RX_UNLOCK(rxq);
		return (FALSE);
	}
#endif /* DEV_NETMAP */

	mh = mt = NULL;
	while (__predict_true(budget_left--)) {
		if (__predict_false(!iflib_getactive(ctx)))
			break;
		bus_dmamap_sync(rxr->rxdma.dma_tag, rxr->rxdma.dma_map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);		
		if (__predict_false(!IFC_IS_NEW(ctx, rxq->ifr_hwq, cidx)))
			break;		
		err = IFC_PACKET_GET(ctx, rxq->ifr_hwq, cidx, &ri);
		bus_dmamap_unload(rxq->ifr_dtag, rxq->ifr_sds[cidx].ifsd_map);

		if (++cidx == ctx->ifc_nrxd)
			cidx = 0;
		if (err) {
			_recycle_rx_buf(ctx, rxq, ri->ri_cidx);
			continue;
		}

		_iflib_packet_get(ctx, &ri);
		m = ri->ri_m;
		if (mh == NULL)
			mh = mt = m;
		else {
			mt->m_nextpkt = m;
			mt = m;
		}
		_iflib_rxq_refill_lt(ctx, rxq, /* XXX em value */ 8);
	}
	rxq->ifr_cidx = cidx;
	RX_UNLOCK(rxq);

	while (mh != NULL) {
		m = mh;
		mh = mh->m_nextpkt;
		m->m_nextpkt = NULL;
		if_input(ctx->ifc_ifp, m);
	}
}

/*
 * Trivial ifnet accessor wrappers
 *
 */
static void
iflib_setcapabilitiesbit(iflib_ctx_t ctx, int flags)
{
	if_setcapabilitiesbit(ctx->ifc_ifp, flags, 0);
}

void
iflib_setcapenable(iflib_ctx_t ctx)
{
	if_t ifp = ctx->ifc_ifp;

	if_setcapenable(ifp, if_getcapabilities(ifp));
}

int
iflib_getcapenable(iflib_ctx_t ctx)
{

	return (if_getcapenable(ctx->ifc_ifp));
}

static void
iflib_setifheaderlen(iflib_ctx_t ctx, int len)
{
	if_setifheaderlen(ctx->ifc_ifp, len);
}

void
iflib_setactive(iflib_ctx_t ctx)
{

    /* Set the interface as ACTIVE */
	if_setdrvflagbits(ctx->ifc_ifp, IFF_DRV_RUNNING, 0);
}

int
iflib_getactive(iflib_ctx_t ctx)
{
		return ((if_getdrvflags(ctx->ifc_ifp) & IFF_DRV_RUNNING) != 0)
}

void
iflib_clearactive(iflib_ctx_t ctx)
{
	/* Tell the stack that the interface is not active */
	if_setdrvflagbits(ctx->ifc_ifp, 0, IFF_DRV_RUNNING);
}
	
void
iflib_sethwassist(iflib_ctx_t ctx)
{
	struct ifnet *ifp = ctx->ifc_ifp;

	if_clearhwassist(ifp);
	if (if_getcapenable(ifp) & IFCAP_TXCSUM)
		if_sethwassistbits(ifp, CSUM_TCP | CSUM_UDP, 0);
	if (if_getcapenable(ifp) & IFCAP_TSO4)
		if_sethwassistbits(ifp, CSUM_TSO, 0);
}

int
iflib_multiaddr_count(iflib_ctx_t ctx, int n)
{

	return (if_multiaddr_count(ctx->ifc_ifp, n));
}

void
iflib_multiaddr_array(iflib_ctx_t ctx, u8 *mta, int *mcnt, int n)
{

	iflib_multiaddr_array(ctx->ifc_ifp, mta, mcnt, n);
}

/*
 * Trivial device routines
 */

int
iflib_printf(iflib_ctx_t ctx, const char * fmt, ...)
{
	va_list ap;
	int retval;

	retval = device_print_prettyname(dev);
	va_start(ap, fmt);
	retval += vprintf(fmt, ap);
	va_end(ap);
	return (retval);
}

static int
_iflib_irq_alloc(iflib_ctx_t ctx, iflib_irq_t *irq, int rid, driver_filter_t filter,
				 driver_intr_t handler, void *arg, char *name)
{
	int rc;
	struct resource *res;
	void *tag;
	device_t dev = ctx->ifc_dev;

	irq->ifi_rid = rid;	
	res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq->ifi_rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (res == NULL) {
		device_printf(dev,
		    "failed to allocate IRQ for rid %d, name %s.\n", rid, name);
		return (ENOMEM);
	}

	/*
	 * Sort out handler versus filter XXX 
	 */
	rc = bus_setup_intr(dev, res, INTR_MPSAFE | INTR_TYPE_NET,
	    NULL, handler, arg, &tag);
	if (rc != 0) {
		device_printf(dev,
		    "failed to setup interrupt for rid %d, name %s: %d\n",
					  rid, name : name ? "unknown", rc);
	} else if (name)
		bus_describe_intr(dev, res, tag, name);

	irq->ifi_tag = tag;
	irq->ifi_res = res;
	return (0);	
}


/*
 * Services routines exported downward to the driver
 */

int
iflib_irq_alloc(iflib_ctx_t ctx, iflib_irq_t *irq, int rid,
				driver_intr_t handler, void *arg, char *name)
{

	return (_iflib_irq_alloc(ctx, irq, rid, NULL, handler, arg, name));	
}

int
iflib_irq_alloc_generic(iflib_ctx_t ctx, iflib_irq_t *irq, int rid,
						intr_type_t type, void *arg, char *name)
{
	switch (type) {
	case IFLIB_INTR_TX:
		break;
	case IFLIB_INTR_RX:
		break;
	case IFLIB_INTR_LINK:
		break;
	}
	
}

static void
_iflib_init(iflib_ctx_t ctx)
{
	IFC_DISABLE_INTR(ctx);
	callout_stop(ctx->ifc_timer);
	IFC_INIT(ctx);
	if_setdrvflagbits(ctx->ifc_ifp, IFF_DRV_RUNNING, 0);
	callout_reset(ctx->ifc_timer, hz, iflib_timer, ctx);
}

void
iflib_init(iflib_ctx_t *ctx)
{
	CTX_LOCK(ctx);
	_iflib_init(ctx);
	CTX_UNLOCK(ctx);
}

static void
_task_fn_legacy_intr(void *context, int pending)
{
	iflib_ctx_t ctx = context;
	if_t ifp = ctx->ifc_ifp;
	iflib_txq_t	txq = ctx->ifc_txqs;
	iflib_rxq_t	rxq = ctx->ifc_rxqs;
	bool more;

	/* legacy do it all crap function */
	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING)  == 0)
		goto enable;

	more = iflib_rxeof(rxq, ctx->rx_process_limit);

	if(TXR_TRYLOCK(tx)) {
		_iflib_txr_transmit(txr, NULL);
		TXR_UNLOCK(tx);
	}

	if (more) {
		iflib_legacy_intr_deferred(ctx);
		return;
	}

enable:
	ctx->ifc_intr_enable(ctx);
}

static void
_task_fn_tx(void *context, int pending)
{
	iflib_txq_t txq = context;
	iflib_ctx_t ctx = txq->ift_ctx;
	
	TXQ_LOCK(txq);
	_iflib_txq_transmit(txq, NULL);
	IFC_TX_INTR_ENABLE(ctx, txq->ift_hwq);
	TXQ_UNLOCK(txq);
}

static void
_task_fn_rx(void *context, int pending)
{
	iflib_rxq_t rxq = context;
	iflib_ctx_t ctx = rxq->ift_ctx;
	int more = 0;

	if (!(if_getdrvflags(ctx->ifc_ifp) & IFF_DRV_RUNNING))
		return;

	
	if (RXQ_TRY_LOCK(rxq)) {
		if ((more = iflib_rxeof(rxq, ctx->rx_process_limit)) == 0)
			IFC_RX_INTR_ENABLE(ctx, rxq);
		RXQ_UNLOCK(rxq);
	}
	if (more)
		taskqueue_enqueue(rxr->ifr_tq, &rxr->ifr_task);


}

static void
_task_fn_link(void *context, int pending)
{
	iflib_ctx_t ctx = context;
	iflib_txq_t txr = ctx->ifc_txr;

	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
		return;

	CTX_LOCK(ctx);
	callout_stop(&ctx->ifc_timer);
	IFC_UPDATE_LINK_STATUS(ctx);
	callout_reset(&ctx->ifc_timer, hz, iflib_timer, ctx);
	IFC_LINK_INTR_ENABLE(ctx);	
	CTX_UNLOCK(ctx);

	if (ctx->ifc_link_active == 0)
		return;
	
	for (int i = 0; i < ctx->ifc_nqueues; i++, txr++) {
		if (TXQ_TRYLOCK(txq) == 0)
			continue;
		_iflib_txq_transmit(txq, NULL);
		TXQ_UNLOCK(txq);
	}
}

int
iflib_legacy_setup(iflib_ctx_t ctx, driver_filter_t filter, int *rid)
{
	struct resource *res;
	struct taskqueue *tq, *tx_tq;
	device_t dev = ctx->ifc_dev;
	iflib_irq_t irq = &ctx->ifc_legacy_irq;

	/* We allocate a single interrupt resource */
	if ((err = iflib_irq_alloc(ctx, &irq, *rid, filter, ctx, NULL)) != 0)
		return (err);

	/*
	 * Allocate a fast interrupt and the associated
	 * deferred processing contexts.
	 */
	TASK_INIT(&ctx->ifc_legacy_task, 0, _task_fn_legacy_intr, ctx);
	tq = taskqueue_create_fast("legacy_taskq", M_NOWAIT,
	    taskqueue_thread_enqueue, &tq);
	taskqueue_start_threads(&tq, 1, PI_NET, "%s que",
	    device_get_nameunit(dev));
	/* Use a TX only tasklet for local timer */
	TASK_INIT(&txr->tx_task, 0, _task_fn_tx, txr);
	tx_tq = taskqueue_create_fast("iflib_txq", M_NOWAIT,
	    taskqueue_thread_enqueue, tx_tq);
	taskqueue_start_threads(tx_tq, 1, PI_NET, "%s txq",
	    device_get_nameunit(dev));
	TASK_INIT(&ctx->ifc_link_task, 0, _task_fn_link, ctx);
	return (0);
}

void
iflib_legacy_intr_deferred(iflib_ctx_t ctx)
{

	taskqueue_enqueue(ctx->ifc_tq, &adapter->ifc_legacy_task);
}

void
iflib_link_intr_deferred(iflib_ctx_t ctx)
{

	taskqueue_enqueue(taskqueue_fast, &ctx->ifc_link_task);
}

/*********************************************************************
 *
 *  Allocate memory for tx_buffer structures. The tx_buffer stores all
 *  the information needed to transmit a packet on the wire. This is
 *  called only once at attach, setup is done every reset.
 *
 **********************************************************************/

static int
iflib_txsd_alloc(iflib_txq_t txr)
{
	iflib_ctx_t ctx = txr->ift_ctx;
	device_t dev = ctx->ifc_dev;
	iflib_buf_t txbuf;
	int err, i;
	
	/*
	 * Setup DMA descriptor areas.
	 */
	if ((err = bus_dma_tag_create(bus_get_dma_tag(dev),
			       1, 0,			/* alignment, bounds */
			       BUS_SPACE_MAXADDR,	/* lowaddr */
			       BUS_SPACE_MAXADDR,	/* highaddr */
			       NULL, NULL,		/* filter, filterarg */
			       ctx->ifc_tx_maxsize,		/* maxsize */
			       ctx->ifc_tx_nsegments,	/* nsegments */
			       ctx->ifc_tx_maxsegsize,	/* maxsegsize */
			       0,			/* flags */
			       NULL,			/* lockfunc */
			       NULL,			/* lockfuncarg */
			       &txr->ift_tag))) {
		device_printf(dev,"Unable to allocate TX DMA tag\n");
		goto fail;
	}

	if (!(txq->ift_sds =
	    (iflib_buf_t) malloc(sizeof(struct iflib_buf) *
	    ctx->ifc_ntxd, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate tx_buffer memory\n");
		err = ENOMEM;
		goto fail;
	}

        /* Create the descriptor buffer dma maps */
	txbuf = txq->ift_sds;
	for (i = 0; i < ctx->ifc_ntxd; i++, txbuf++) {
		err = bus_dmamap_create(txq->ift_tag, 0, &txbuf->ifsd_map);
		if (err != 0) {
			device_printf(dev, "Unable to create TX DMA map\n");
			goto fail;
		}
	}

	return 0;
fail:
	/* We free all, it handles case where we are in the middle */
	iflib_tx_structures_free(ctx);
	return (err);
}

static void
iflib_txq_destroy(iflib_txq_t txq)
{
	iflib_ctx_t ctx = txq->ift_ctx;
	iflib_sd_t sd = txq->ift_sds;
	
	for (int i = 0; i < ctx->ifc_ntxd; i++, sd++)
		iflib_txbuf_destroy(ctx, txq, sd);
	if (txq->ift_sds != NULL) {
		free(txq->ift_sds, M_DEVBUF);
		txq->ift_sds = NULL;
	}
	if (txq->ift_tag != NULL) {
		bus_dma_tag_destroy(txq->ift_tag);
		txq->ift_tag = NULL;
	}
	TXQ_LOCK_DESTROY(txq);
}

static void
iflib_txsd_free(iflib_txq_t txq)
{
	if (txq->ift_sds == NULL)
		return;
	iflib_txq_destroy(txq);
}

/*********************************************************************
 *
 *  Allocate memory for rx_buffer structures. Since we use one
 *  rx_buffer per received packet, the maximum number of rx_buffer's
 *  that we'll need is equal to the number of receive descriptors
 *  that we've allocated.
 *
 **********************************************************************/
static int
iflib_rxsd_alloc(struct iflib_rxq_t rxq)
{
	iflib_ctx_t ctx = rxq->ifr_ctx;
	device_t dev = ctx->ifc_dev;
	iflib_sd_t	rxbuf;
	int			err;

	rxq->ifr_sds = malloc(sizeof(struct iflib_sw_desc) *
	    ctx->ifc_nrxd, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (rxq->ifr_sds == NULL) {
		device_printf(dev, "Unable to allocate rx_buffer memory\n");
		return (ENOMEM);
	}

	err = bus_dma_tag_create(bus_get_dma_tag(dev), /* parent */
				1, 0,			/* alignment, bounds */
				BUS_SPACE_MAXADDR,	/* lowaddr */
				BUS_SPACE_MAXADDR,	/* highaddr */
				NULL, NULL,		/* filter, filterarg */
				ctx->ifc_rx_maxsize,	/* maxsize */
				ctx->ifc_rx_nsegments,	/* nsegments */
				ctx->ifc_rx_maxsegsize,	/* maxsegsize */
				0,			/* flags */
				NULL,			/* lockfunc */
				NULL,			/* lockarg */
				&rxq->ifr_tag);
	if (err) {
		device_printf(dev, "%s: bus_dma_tag_create failed %d\n",
		    __func__, err);
		goto fail;
	}

	rxbuf = rxq->ifr_sds;
	for (int i = 0; i < ctx->ifc_nrxd; i++, rxbuf++) {
		err = bus_dmamap_create(rxr->ifr_tag, 0, &rxbuf->ifsd_map);
		if (err) {
			device_printf(dev, "%s: bus_dmamap_create failed: %d\n",
			    __func__, err);
			goto fail;
		}
	}

	return (0);

fail:
	iflib_receive_structures_free(ctx);
	return (err);
}

static void
iflib_rx_bufs_free(iflib_rxq_t rxq)
{
	uint32_t cidx = rxq->ifr_cidx;

	while (rxq->ifr_credits--) {
		iflib_sd_t d = rxq->ifr_sds[cidx];

		if (d->ifsd_flags & RX_SW_DESC_INUSE) {
			bus_dmamap_unload(q->ifr_dtag, d->ifsd_map);
			bus_dmamap_destroy(q->ifr_dtag, d->ifsd_map);
			m_init(d->m, zone_mbuf, MLEN,
				   M_NOWAIT, MT_DATA, 0);
			uma_zfree(zone_mbuf, d->ifsd_m);
			uma_zfree(q->rxq_zone, d->ifsd_cl);
		}				
		d->ifsd_cl = NULL;
		d->ifsd_m = NULL;
		if (++cidx == q->ifr_size)
			cidx = 0;
	}
}

/*********************************************************************
 *
 *  Initialize a receive ring and its buffers.
 *
 **********************************************************************/
static int
iflib_rxq_setup(iflib_rxq_t rxq)
{
	iflib_ctx_t ctx = rxq->ifr_ctx;
	iflib_sd_t	rxbuf;
	bus_dma_segment_t	seg[1];
	int			i, rsize, nsegs, err = 0;
#ifdef DEV_NETMAP
	struct netmap_slot *slot;
	struct netmap_adapter *na = netmap_getna(ctx->ifc_ifp);
#endif

	/* Clear the ring contents */
	RX_LOCK(rxq);
	bzero((void *)rxr->rx_base, ctx->ifc_rxq_size);
#ifdef DEV_NETMAP
	slot = netmap_reset(na, NR_RX, rxr->ifr_id, 0);
#endif

	/*
	** Free current RX buffer structs and their mbufs
	*/
	for (i = 0; i < ctx->ifc_nrxd; i++)
		iflib_rx_free(ctx, rxr, &rxr->ifr_buffers[i]);

	/* Now replenish the mbufs */
	iflib_refill_rxq(ctx, rxq, ctx->ifc_nrxd);

	bus_dmamap_sync(rxr->rxdma.dma_tag, rxr->rxdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

fail:
	RX_UNLOCK(rxr);
	return (err);
}

/*********************************************************************
 *
 *  Initialize all receive rings.
 *
 **********************************************************************/
static int
iflib_rx_structures_setup(iflib_ctx_t ctx)
{
	iflib_rxq_t rxq = ctx->ifc_rxqs;
	iflib_sd_t rxbuf;
	int i, n, q;

	for (q = 0; q < ctx->ifc_nqueues; q++, rxq++)
		if (iflib_rxq_setup(rxr))
			goto fail;

	return (0);
fail:
	/*
	 * Free RX software descriptors allocated so far, we will only handle
	 * the rings that completed, the failing case will have
	 * cleaned up for itself. 'q' failed, so its the terminus.
	 */
	rxq = ctx->ifc_rxqs;
	for (i = 0; i < q; ++i; rxq++) {
		iflib_rx_sds_free(ctx, rxq);
		rxq->next_to_check = 0;
		rxq->next_to_refresh = 0;
	}

	return (ENOBUFS);
}

/*********************************************************************
 *
 *  Free all receive rings.
 *
 **********************************************************************/
static void
iflib_rx_structures_free(iflib_ctx_t ctx)
{
	iflib_txq_t rxq = ctx->ifc_rxq;

	for (int i = 0; i < ctx->ifc_nqueues; i++, rxq++) {
		iflib_rx_sds_free(rxq);
		iflib_dma_free(&rxq->ift_dma_info);
	}
}


/*********************************************************************
 *
 *  Free receive ring data structures
 *
 **********************************************************************/
static void
iflib_rx_sds_free(iflib_rxq_t rxq)
{
	iflib_ctx_t ctx = rxq->ifr_ctx;

	INIT_DEBUGOUT("free_receive_buffers: begin");

	if (rxq->ifr_sds != NULL) {

		free(rxq->ifr_sds, M_DEVBUF);
		rxq->ifr_sds = NULL;
		rxr->next_to_check = 0;
		rxr->next_to_refresh = 0;
	}

	if (rxr->rxtag != NULL) {
		bus_dma_tag_destroy(rxr->rxtag);
		rxr->rxtag = NULL;
	}

	return;
}

static void
_iflib_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error)
		return;
	*(bus_addr_t *) arg = segs[0].ds_addr;
}

static int
iflib_dma_alloc(iflib_ctx_t ctx, bus_size_t size, iflib_dma_info_t dma,
				int mapflags)
{
	int err;

	err = bus_dma_tag_create(bus_get_dma_tag(ctx->ifc_dev), /* parent */
				ctx->ifc_q_align, 0,	/* alignment, bounds */
				BUS_SPACE_MAXADDR,	/* lowaddr */
				BUS_SPACE_MAXADDR,	/* highaddr */
				NULL, NULL,		/* filter, filterarg */
				size,			/* maxsize */
				1,			/* nsegments */
				size,			/* maxsegsize */
				0,			/* flags */
				NULL,			/* lockfunc */
				NULL,			/* lockarg */
				&dma->ifd_tag);
	if (err) {
		iflib_printf(ctx,
		    "%s: bus_dma_tag_create failed: %d\n",
		    __func__, err);
		goto fail_0;
	}

	err = bus_dmamem_alloc(dma->ifd_tag, (void**) &dma->ifd_vaddr,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT, &dma->ifd_map);
	if (err) {
		iflib_printf(ctx,
		    "%s: bus_dmamem_alloc(%ju) failed: %d\n",
		    __func__, (uintmax_t)size, err);
		goto fail_2;
	}

	dma->ifd_paddr = 0;
	err = bus_dmamap_load(dma->ifd_tag, dma->ifd_map, dma->ifd_vaddr,
	    size, _iflib_dmamap_cb, &dma->ifd_paddr, mapflags | BUS_DMA_NOWAIT);
	if (err || dma->ifd_paddr == 0) {
		device_printf(adapter->dev,
		    "%s: bus_dmamap_load failed: %d\n",
		    __func__, err);
		goto fail_3;
	}

	return (0);

fail_3:
	bus_dmamap_unload(dma->ifd_tag, dma->ifd_map);
fail_2:
	bus_dmamem_free(dma->ifd_tag, dma->ifd_vaddr, dma->ifd_map);
	bus_dma_tag_destroy(dma->ifd_tag);
fail_0:
	dma->ifd_tag = NULL;

	return (err);
}

static void
iflib_dma_free(iflib_dma_info_t dma)
{
	if (dma->ifd_tag == NULL)
		return;
	if (dma->ifd_paddr != 0) {
		bus_dmamap_sync(dma->ifd_tag, dma->ifd_map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(dma->ifd_tag, dma->ifd_map);
		dma->ifd_paddr = 0;
	}
	if (dma->ifd_vaddr != NULL) {
		bus_dmamem_free(dma->ifd_tag, dma->ifd_vaddr, dma->ifd_map);
		dma->ifd_vaddr = NULL;
	}
	bus_dma_tag_destroy(dma->ifd_tag);
	dma->ifd_tag = NULL;
}
	
void
iflib_tx_structures_free(iflib_ctx_t ctx)
{
	iflib_txq_t txq = ctx->ifc_txq;

	for (int i = 0; i < ctx->ifc_nqueues; i++, txq++) {
		iflib_txq_destroy(txq);
		iflib_dma_free(&txq->ift_dma_info);
	}
	free(ctx->ifc_txqs, M_DEVBUF);	
}

static int
iflib_queues_alloc(iflib_ctx_t ctx, int txq_size, int rxq_size)
{
	int nqueues = ctx->ifc_nqueues;
	device_t dev = ctx->ifc_dev;
	iflib_txq_t rxq = NULL;
	iflib_rxq_t rxq = NULL;
	int i, err;

	/* Allocate the TX ring struct memory */
	if (!(txq =
	    (iflib_txq_t) malloc(sizeof(struct iflib_txq) *
	    nqueues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate TX ring memory\n");
		err = ENOMEM;
		goto fail;
	}
	
	/* Now allocate the RX */
	if (!(rxq = 
	    (iflib_rxq_t) malloc(sizeof(struct iflib_rxq) *
	    nqueues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate RX ring memory\n");
		err = ENOMEM;
		goto rx_fail;
	}

	/*
	 * XXX handle allocation failure
	 */
	for (i = 0; i < nqueues; i++, txconf++) {
		/* Set up some basics */
		txq = &ctx->ifc_txqs[i];
		txq->ift_ctx = ctx;
		txq->ift_id = i;
		
		if (iflib_allocate_transmit_buffers(txr)) {
			device_printf(dev,
						  "Critical Failure setting up transmit buffers\n");
			err = ENOMEM;
			goto err_tx_desc;
		}

		/* Initialize the TX lock */
		snprintf(txq->ift_mtx_name, sizeof(txq->ift_mtx_name), "%s:tx(%d)",
		    device_get_nameunit(dev), txq->ift_id);
		mtx_init(&txq->ift_mtx, txq->ift_mtx_name, NULL, MTX_DEF);

		if (iflib_dma_alloc(ctx, txq_size,
			&txq->ift_dma_info, BUS_DMA_NOWAIT)) {
			device_printf(dev,
			    "Unable to allocate TX Descriptor memory\n");
			err = ENOMEM;
			goto err_tx_desc;
		}
		bzero((void *)txq->ift_dma_info.ifd_vaddr, txq_size);
		/* Allocate a buf ring */
		txr->br = buf_ring_alloc(4096, M_DEVBUF,
		    M_WAITOK, &txr->ift_mtx);
		/*
		 * Next the RX queues...
		 */
		rxq = &ctx->ifc_rxqs[i];
		rxq->ift_ctx = ctx;
		rxq->ift_id = i;

        /* Allocate receive buffers for the ring*/
		if (iflib_rxsd_alloc(rxq)) {
			device_printf(dev,
			    "Critical Failure setting up receive buffers\n");
			err = ENOMEM;
			goto err_rx_desc;
		}
		
		/* Initialize the RX lock */
		snprintf(rxq->ifr_mtx_name, sizeof(rxq->ifr_mtx_name), "%s:rx(%d)",
		    device_get_nameunit(dev), rxq->ifr_id);
		mtx_init(&rxq->ifr_mtx, rxq->ifr_mtx_name, NULL, MTX_DEF);
		
		if (iflib_dma_alloc(ctx, rxq_size,
			&rxq->ift_dma_info, BUS_DMA_NOWAIT)) {
			device_printf(dev,
			    "Unable to allocate TX Descriptor memory\n");
			err = ENOMEM;
			goto err_tx_desc;
		}
		bzero((void *)rxq->ifr_dma_info.ifd_vaddr, rxq_size);
	}
	ctx->ifc_txqs = txqs;
	ctx->ifc_rxqs = rxqs;
	ctx->ifc_txq_size = txq_size;
	ctx->ifc_rxq_size = rxq_size;
	if ((err = ctx->ifc_queues_alloc(ctx)) != 0)
		iflib_tx_structures_free(ctx);
		
	return (0);
err_rx_desc:
	for (rxq = ctx->ifc_rxqs; rxconf > 0; rxq++, rxconf--)
		iflib_dma_free(ctx, &rxq->ifr_dma_info);
err_tx_desc:
	for (txq = ctx->ifc_txqs; txconf > 0; txq++, txconf--)
		iflib_dma_free(adapter, &txq->ifr_dma_info);
	free(ctx->ifc_rxqs, M_DEVBUF);
rx_fail:
	free(ctx->ifc_txqs, M_DEVBUF);
fail:
	return (err);
}


/*
 *
 *
 */

static void
iflib_txq_setup(iflib_txq_t txq)
{
	iflib_ctx_t ctx = txq->ift_ctx;
	iflib_buf_t txbuf;
#ifdef DEV_NETMAP
	struct netmap_slot *slot;
	struct netmap_adapter *na = netmap_getna(adapter->ifp);
#endif /* DEV_NETMAP */
	
	TX_LOCK(txr);
#ifdef DEV_NETMAP
	slot = netmap_reset(na, NR_TX, txr->me, 0);
#endif /* DEV_NETMAP */	

    /* Set number of descriptors available */
	txr->tx_avail = adapter->ntxd;
	txr->qstatus = IFLIB_QUEUE_IDLE;

	/* Reset indices */
	txr->next_avail_desc = 0;
	txr->next_to_clean = 0;

	/* Free any existing tx buffers. */
	txbuf = txr->ift_buffers;
	for (i = 0; i < adapter->ntxd; i++, txbuf++) {
		iflib_tx_free(ctx, txr, txbuf);
#ifdef DEV_NETMAP
		if (slot) {
			int si = netmap_idx_n2k(&na->tx_rings[txr->me], i);
			uint64_t paddr;
			void *addr;

			addr = PNMB(na, slot + si, &paddr);
			txr->tx_base[i].buffer_addr = htole64(paddr);
			/* reload the map for netmap mode */
			netmap_load_map(na, txr->txtag, txbuf->map, addr);
		}
#endif /* DEV_NETMAP */

		/* clear the watch index */
		txbuf->next_eop = -1;
	}

	IFC_TXQ_SETUP(ctx, txr->ift_hw_txr);
	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	TX_UNLOCK(txr);	
}

void
iflib_tx_structures_setup(iflib_ctx_t ctx)
{
	iflib_txq_t txr = ctx->ifc_txr;

	for (int i = 0; i < ctx->ifc_nqueues; i++, txr++)
		iflib_txq_setup(txq);
}


int
iflib_attach(device_t dev, driver_t *driver)
{
	iflib_shared_ctx_t sctx = device_get_softc(dev);
	iflib_ctx_t ctx;
	if_t ifp;
	
	ctx = malloc(sizeof(struct iflib_ctx), M_DEVBUF, M_WAITOK);
	if (ctx == NULL)
		return (ENOMEM);
	CTX_LOCK_INIT(ctx);
	callout_init_mtx(&ctx->ifc_timer, &ctx->ifc_mtx, 0);
	sctx->isc_ctx = ctx;
	ctx->ifc_sctx = sctx;
	ifp = ctx->ifc_ifp = if_gethandle(IFT_ETHER);
	if (ifp == NULL) {
				device_printf(dev, "can not allocate ifnet structure\n");
		return (-1);
	}

	/*
	 * Initialize our contexts device specific methods
	 */
	driver->size = sizeof(*iflib_ctx_t);
	kobj_init((kobj_t) ctx, (kobj_class_t) driver);
	kobj_class_compile((kobj_class_t) driver);
	driver->refs++;

	if_initname_drv(ifp, device_get_name(dev), device_get_unit(dev));
	if_setdev(ifp, dev);
	if_setinitfn(ifp, iflib_if_init);
	if_setsoftc(ifp, ctx);
	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setioctlfn(ifp, iflib_if_ioctl);
	if_settransmitfn(ifp, iflib_if_transmit);
	if_setqflushfn(ifp, iflib_if_qflush);
	ether_ifattach_drv(ifp, ctx->ifc_hw.mac.addr);

	if_setcapabilities(ifp, 0);
	if_setcapenable(ifp, 0);

	ctx->vlan_attach_event =
		EVENTHANDLER_REGISTER(vlan_config, iflib_vlan_register, ctx,
							  EVENTHANDLER_PRI_FIRST);
	ctx->vlan_detach_event =
		EVENTHANDLER_REGISTER(vlan_unconfig, iflib_vlan_unregister, adapter,
							  EVENTHANDLER_PRI_FIRST);
	return (ctx);
}


static void
_iflib_init(iflib_ctx_t ctx)
{
	ctx->ifc_disable_intr(ctx);
	callout_stop(ctx->ifc_timer);
	ctx->ifc_init(ctx);
	if_setdrvflagbits(ctx->ifc_ifp, IFF_DRV_RUNNING, 0);
	callout_reset(ctx->ifc_timer, hz, iflib_timer, ctx);
}

void
iflib_init(iflib_ctx_t *ctx)
{
	CTX_LOCK(ctx);
	_iflib_init(ctx);
	CTX_UNLOCK(ctx);
}

static void
_task_fn_legacy_intr(void *context, int pending)
{
	iflib_ctx_t ctx = context;
	if_t ifp = ctx->ifc_ifp;
	iflib_txq_t	txq = ctx->ifc_txqs;
	iflib_rxq_t	rxq = ctx->ifc_rxqs;
	bool more;

	/* legacy do it all crap function */
	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING)  == 0)
		goto enable;

	more = iflib_rxeof(rxq, ctx->rx_process_limit);

	if(TXR_TRYLOCK(tx)) {
		_iflib_txr_transmit(txr, NULL);
		TXR_UNLOCK(tx);
	}

	if (more) {
		iflib_legacy_intr_deferred(ctx);
		return;
	}

enable:
	ctx->ifc_intr_enable(ctx);
}

static void
_task_fn_tx(void *context, int pending)
{
	iflib_txq_t txq = context;
	iflib_ctx_t ctx = trx->ift_ctx;
	
	TXQ_LOCK(tx);
	_iflib_txq_transmit(txq, NULL);
	IFC_TX_INTR_ENABLE(ctx, txq->ift_hwq);
	TXQ_UNLOCK(tx);
}

static void
_task_fn_rx(void *context, int pending)
{
	iflib_rxq_t rxq = arg;
	iflib_ctx_t ctx = rxq->ift_ctx;
	int more = 0;

	if (!(if_getdrvflags(ctx->ifc_ifp) & IFF_DRV_RUNNING))
		return;

	
	if (RXQ_TRY_LOCK(rxq)) {
		if ((more = iflib_rxeof(rxq, ctx->rx_process_limit)) == 0)
			IFC_RX_INTR_ENABLE(ctx, rxq->ifr_hwq);
		RXQ_UNLOCK(rxq);
	}
	if (more)
		taskqueue_enqueue(rxr->ifr_tq, &rxq->ifr_task);


}

static void
_task_fn_link(void *context, int pending)
{
	iflib_ctx_t ctx = context;
	iflib_txq_t txq = ctx->ifc_txq;

	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
		return;

	CTX_LOCK(ctx);
	callout_stop(&ctx->ifc_timer);
	IFC_UPDATE_LINK_STATUS(ctx);
	callout_reset(&ctx->ifc_timer, hz, iflib_timer, ctx);
	IFC_LINK_INTR_ENABLE(ctx);	
	CTX_UNLOCK(ctx);

	if (ctx->ifc_link_active == 0)
		return;
	
	for (int i = 0; i < ctx->ifc_nqueues; i++, txr++) {
		if (TXQ_TRYLOCK(txr) == 0)
			continue;
		_iflib_txq_transmit(txq, NULL);
		TXQ_UNLOCK(txr);
	}
}

int
iflib_legacy_setup(iflib_ctx_t ctx, driver_filter_t filter, int *rid)
{
	struct resource *res;
	struct taskqueue *tq, *tx_tq;
	device_t dev = ctx->ifc_dev;
	iflib_irq_t irq = &ctx->ifc_legacy_irq;


	/* We allocate a single interrupt resource */
	if ((err = iflib_irq_alloc(ctx, &irq, *rid, filter, ctx, NULL)) != 0)
		return (err);

     /*
	* Allocate a fast interrupt and the associated
	* deferred processing contexts.
	*/
	TASK_INIT(&ctx->ifc_legacy_task, 0, _task_fn_legacy_intr, ctx);
	tq = taskqueue_create_fast("legacy_taskq", M_NOWAIT,
	    taskqueue_thread_enqueue, &tq);
	taskqueue_start_threads(&tq, 1, PI_NET, "%s que",
	    device_get_nameunit(dev));
	/* Use a TX only tasklet for local timer */
	TASK_INIT(&txr->tx_task, 0, _task_fn_tx, txr);
	tx_tq = taskqueue_create_fast("iflib_txq", M_NOWAIT,
	    taskqueue_thread_enqueue, tx_tq);
	taskqueue_start_threads(tx_tq, 1, PI_NET, "%s txq",
	    device_get_nameunit(dev));
	TASK_INIT(&ctx->ifc_link_task, 0, _task_fn_link, ctx);

	ctx->ifc_tq = tq;
	txr->ift_tq = tx_tq;
	return (0);
}

void
iflib_legacy_intr_deferred(iflib_ctx_t ctx)
{

	taskqueue_enqueue(ctx->ifc_tq, &adapter->ifc_legacy_task);
}

void
iflib_link_intr_deferred(iflib_ctx_t ctx)
{

	taskqueue_enqueue(taskqueue_fast, &ctx->ifc_link_task);
}

/*********************************************************************
 *
 *  Allocate memory for tx_buffer structures. The tx_buffer stores all
 *  the information needed to transmit a packet on the wire. This is
 *  called only once at attach, setup is done every reset.
 *
 **********************************************************************/

static int
iflib_txsd_alloc(iflib_txq_t txr)
{
	iflib_ctx_t ctx = txr->ift_ctx;
	device_t dev = ctx->ifc_dev;
	iflib_buf_t txbuf;
	int err, i;
	
	/*
	 * Setup DMA descriptor areas.
	 */
	if ((err = bus_dma_tag_create(bus_get_dma_tag(dev),
			       1, 0,			/* alignment, bounds */
			       BUS_SPACE_MAXADDR,	/* lowaddr */
			       BUS_SPACE_MAXADDR,	/* highaddr */
			       NULL, NULL,		/* filter, filterarg */
			       ctx->ifc_tx_maxsize,		/* maxsize */
			       ctx->ifc_tx_nsegments,	/* nsegments */
			       ctx->ifc_tx_maxsegsize,	/* maxsegsize */
			       0,			/* flags */
			       NULL,			/* lockfunc */
			       NULL,			/* lockfuncarg */
			       &txr->ift_tag))) {
		device_printf(dev,"Unable to allocate TX DMA tag\n");
		goto fail;
	}

	if (!(txq->ift_sds =
	    (iflib_buf_t) malloc(sizeof(struct iflib_buf) *
	    ctx->ifc_ntxd, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate tx_buffer memory\n");
		err = ENOMEM;
		goto fail;
	}

        /* Create the descriptor buffer dma maps */
	txbuf = txq->ift_sds;
	for (i = 0; i < ctx->ifc_ntxd; i++, txbuf++) {
		err = bus_dmamap_create(txq->ift_tag, 0, &txbuf->ifsd_map);
		if (err != 0) {
			device_printf(dev, "Unable to create TX DMA map\n");
			goto fail;
		}
	}

	return 0;
fail:
	/* We free all, it handles case where we are in the middle */
	iflib_tx_structures_free(ctx);
	return (err);
}

static void
iflib_txq_destroy(iflib_txq_t txq)
{
	iflib_ctx_t ctx = txq->ift_ctx;
	iflib_sd_t sd = txq->ift_sds;
	
	for (int i = 0; i < ctx->ifc_ntxd; i++, sd++)
		iflib_txbuf_destroy(ctx, txq, sd);
	if (txq->ift_sds != NULL) {
		free(txq->ift_sds, M_DEVBUF);
		txq->ift_sds = NULL;
	}
	if (txq->ift_tag != NULL) {
		bus_dma_tag_destroy(txq->ift_tag);
		txq->ift_tag = NULL;
	}
	TXQ_LOCK_DESTROY(txq);
}

static void
iflib_txsd_free(iflib_txq_t txq)
{
	if (txq->ift_sds == NULL)
		return;
	iflib_txq_destroy(txq);
}

/*********************************************************************
 *
 *  Allocate memory for rx_buffer structures. Since we use one
 *  rx_buffer per received packet, the maximum number of rx_buffer's
 *  that we'll need is equal to the number of receive descriptors
 *  that we've allocated.
 *
 **********************************************************************/
static int
iflib_rxsd_alloc(struct iflib_rxq_t rxq)
{
	iflib_ctx_t ctx = rxq->ifr_ctx;
	device_t dev = ctx->ifc_dev;
	iflib_buf_t	rxbuf;
	int			err;

	rxq->ifr_sds = malloc(sizeof(struct iflib_sw_desc) *
	    ctx->ifc_nrxd, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (rxr->ifr_sds == NULL) {
		device_printf(dev, "Unable to allocate rx_buffer memory\n");
		return (ENOMEM);
	}

	err = bus_dma_tag_create(bus_get_dma_tag(dev), /* parent */
				1, 0,			/* alignment, bounds */
				BUS_SPACE_MAXADDR,	/* lowaddr */
				BUS_SPACE_MAXADDR,	/* highaddr */
				NULL, NULL,		/* filter, filterarg */
				ctx->ifc_rx_maxsize,	/* maxsize */
				ctx->ifc_rx_nsegments,	/* nsegments */
				ctx->ifc_rx_maxsegsize,	/* maxsegsize */
				0,			/* flags */
				NULL,			/* lockfunc */
				NULL,			/* lockarg */
				&rxr->ifr_tag);
	if (err) {
		device_printf(dev, "%s: bus_dma_tag_create failed %d\n",
		    __func__, err);
		goto fail;
	}

	rxbuf = rxr->ifr_sds;
	for (int i = 0; i < ctx->ifc_nrxd; i++, rxbuf++) {
		err = bus_dmamap_create(rxr->ifr_tag, 0, &rxbuf->ifsd_map);
		if (err) {
			device_printf(dev, "%s: bus_dmamap_create failed: %d\n",
			    __func__, err);
			goto fail;
		}
	}

	return (0);

fail:
	iflib_receive_structures_free(ctx);
	return (err);
}

static void
iflib_rx_bufs_free(iflib_rxq_t rxq)
{
	uint32_t cidx = rxq->ifr_cidx;

	while (rxq->ifr_credits--) {
		iflib_sd_t d = rxq->ifr_sds[cidx];

		if (d->ifsd_flags & RX_SW_DESC_INUSE) {
			bus_dmamap_unload(q->ifr_dtag, d->ifsd_map);
			bus_dmamap_destroy(q->ifr_dtag, d->ifsd_map);
			m_init(d->m, zone_mbuf, MLEN,
				   M_NOWAIT, MT_DATA, 0);
			uma_zfree(zone_mbuf, d->m);
			uma_zfree(q->rxq_zone, d->ifsd_cl);
		}				
		d->ifsd_cl = NULL;
		d->ifsd_m = NULL;
		if (++cidx == q->ifr_size)
			cidx = 0;
	}
}

/*********************************************************************
 *
 *  Initialize a receive ring and its buffers.
 *
 **********************************************************************/
static int
iflib_rxq_setup(iflib_rxq_t rxq)
{
	iflib_ctx_t ctx = rxq->ifr_ctx;
	iflib_sd_t	rxbuf;
	bus_dma_segment_t	seg[1];
	int			i, rsize, nsegs, err = 0;
#ifdef DEV_NETMAP
	struct netmap_slot *slot;
	struct netmap_adapter *na = netmap_getna(ctx->ifc_ifp);
#endif

	/* Clear the ring contents */
	RX_LOCK(rxq);
	bzero((void *)rxr->rx_base, ctx->ifc_rxq_size);
#ifdef DEV_NETMAP
	slot = netmap_reset(na, NR_RX, rxr->ifr_id, 0);
#endif

	/*
	** Free current RX buffer structs and their mbufs
	*/
	for (i = 0; i < ctx->ifc_nrxd; i++)
		iflib_rx_free(ctx, rxr, &rxr->ifr_buffers[i]);

	/* Now replenish the mbufs */
	iflib_refill_rxq(ctx, rxq, ctx->ifc_nrxd);

	bus_dmamap_sync(rxr->rxdma.dma_tag, rxr->rxdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

fail:
	RX_UNLOCK(rxr);
	return (err);
}

/*********************************************************************
 *
 *  Initialize all receive rings.
 *
 **********************************************************************/
static int
iflib_rx_structures_setup(iflib_ctx_t ctx)
{
	iflib_rxq_t rxq = ctx->ifc_rxqs;
	iflib_sd_t rxbuf;
	int i, n, q;

	for (q = 0; q < ctx->ifc_nqueues; q++, rxq++)
		if (iflib_rxq_setup(rxr))
			goto fail;

	return (0);
fail:
	/*
	 * Free RX software descriptors allocated so far, we will only handle
	 * the rings that completed, the failing case will have
	 * cleaned up for itself. 'q' failed, so its the terminus.
	 */
	rxq = ctx->ifc_rxqs;
	for (i = 0; i < q; ++i; rxq++) {
		iflib_rx_sds_free(ctx, rxq);
		rxq->next_to_check = 0;
		rxq->next_to_refresh = 0;
	}

	return (ENOBUFS);
}

/*********************************************************************
 *
 *  Free all receive rings.
 *
 **********************************************************************/
static void
iflib_rx_structures_free(iflib_ctx_t ctx)
{
	iflib_txq_t rxq = ctx->ifc_rxq;

	for (int i = 0; i < ctx->ifc_nqueues; i++, rxq++) {
		iflib_rx_sds_free(rxq);
		iflib_dma_free(&rxq->ift_dma_info);
	}
}


/*********************************************************************
 *
 *  Free receive ring data structures
 *
 **********************************************************************/
static void
iflib_rx_sds_free(iflib_rxq_t rxq)
{
	iflib_ctx_t ctx = rxq->ifr_ctx;

	INIT_DEBUGOUT("free_receive_buffers: begin");

	if (rxq->ifr_sds != NULL) {

		free(rxq->ifr_sds, M_DEVBUF);
		rxq->ifr_sds = NULL;
		rxr->next_to_check = 0;
		rxr->next_to_refresh = 0;
	}

	if (rxr->rxtag != NULL) {
		bus_dma_tag_destroy(rxr->rxtag);
		rxr->rxtag = NULL;
	}

	return;
}

static void
_iflib_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error)
		return;
	*(bus_addr_t *) arg = segs[0].ds_addr;
}

static int
iflib_dma_alloc(iflib_ctx_t ctx, bus_size_t size, iflib_dma_info_t dma,
				int mapflags)
{
	int err;

	err = bus_dma_tag_create(bus_get_dma_tag(ctx->ifc_dev), /* parent */
				ctx->ifc_q_align, 0,	/* alignment, bounds */
				BUS_SPACE_MAXADDR,	/* lowaddr */
				BUS_SPACE_MAXADDR,	/* highaddr */
				NULL, NULL,		/* filter, filterarg */
				size,			/* maxsize */
				1,			/* nsegments */
				size,			/* maxsegsize */
				0,			/* flags */
				NULL,			/* lockfunc */
				NULL,			/* lockarg */
				&dma->ifd_tag);
	if (err) {
		iflib_printf(ctx,
		    "%s: bus_dma_tag_create failed: %d\n",
		    __func__, err);
		goto fail_0;
	}

	err = bus_dmamem_alloc(dma->ifd_tag, (void**) &dma->ifd_vaddr,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT, &dma->ifd_map);
	if (err) {
		iflib_printf(ctx,
		    "%s: bus_dmamem_alloc(%ju) failed: %d\n",
		    __func__, (uintmax_t)size, err);
		goto fail_2;
	}

	dma->ifd_paddr = 0;
	err = bus_dmamap_load(dma->ifd_tag, dma->ifd_map, dma->ifd_vaddr,
	    size, _iflib_dmamap_cb, &dma->ifd_paddr, mapflags | BUS_DMA_NOWAIT);
	if (err || dma->ifd_paddr == 0) {
		device_printf(adapter->dev,
		    "%s: bus_dmamap_load failed: %d\n",
		    __func__, err);
		goto fail_3;
	}

	return (0);

fail_3:
	bus_dmamap_unload(dma->ifd_tag, dma->ifd_map);
fail_2:
	bus_dmamem_free(dma->ifd_tag, dma->ifd_vaddr, dma->ifd_map);
	bus_dma_tag_destroy(dma->ifd_tag);
fail_0:
	dma->ifd_tag = NULL;

	return (err);
}

static void
iflib_dma_free(iflib_dma_info_t dma)
{
	if (dma->ifd_tag == NULL)
		return;
	if (dma->ifd_paddr != 0) {
		bus_dmamap_sync(dma->ifd_tag, dma->ifd_map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(dma->ifd_tag, dma->ifd_map);
		dma->ifd_paddr = 0;
	}
	if (dma->ifd_vaddr != NULL) {
		bus_dmamem_free(dma->ifd_tag, dma->ifd_vaddr, dma->ifd_map);
		dma->ifd_vaddr = NULL;
	}
	bus_dma_tag_destroy(dma->ifd_tag);
	dma->ifd_tag = NULL;
}
	
void
iflib_tx_structures_free(iflib_ctx_t ctx)
{
	iflib_txq_t txq = ctx->ifc_txq;

	for (int i = 0; i < ctx->ifc_nqueues; i++, txq++) {
		iflib_txq_destroy(txq);
		iflib_dma_free(&txq->ift_dma_info);
	}
	free(ctx->ifc_txqs, M_DEVBUF);	
}

static int
iflib_queues_alloc(iflib_ctx_t ctx, int txq_size, int rxq_size)
{
	int nqueues = ctx->ifc_nqueues;
	device_t dev = ctx->ifc_dev;
	iflib_txq_t txq = NULL;
	iflib_rxq_t rxq = NULL;
	int i, err;

	/* Allocate the TX ring struct memory */
	if (!(txq =
	    (iflib_txq_t) malloc(sizeof(struct iflib_txq) *
	    nqueues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate TX ring memory\n");
		err = ENOMEM;
		goto fail;
	}
	
	/* Now allocate the RX */
	if (!(rxq = 
	    (iflib_rxq_t) malloc(sizeof(struct iflib_rxq) *
	    nqueues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate RX ring memory\n");
		err = ENOMEM;
		goto rx_fail;
	}

	/*
	 * XXX handle allocation failure
	 */
	for (i = 0; i < nqueues; i++, txconf++) {
		/* Set up some basics */
		txq = &ctx->ifc_txqs[i];
		txq->ift_ctx = ctx;
		txq->ift_id = i;

		/*
		 * XXX undefined
		 */
		if (iflib_allocate_transmit_buffers(txq)) {
			device_printf(dev,
						  "Critical Failure setting up transmit buffers\n");
			err = ENOMEM;
			goto err_tx_desc;
		}

		/* Initialize the TX lock */
		snprintf(txq->ift_mtx_name, sizeof(txq->ift_mtx_name), "%s:tx(%d)",
		    device_get_nameunit(dev), txq->ift_id);
		mtx_init(&txq->ift_mtx, txq->ift_mtx_name, NULL, MTX_DEF);

		if (iflib_dma_alloc(ctx, txq_size,
			&txq->ift_dma_info, BUS_DMA_NOWAIT)) {
			device_printf(dev,
			    "Unable to allocate TX Descriptor memory\n");
			err = ENOMEM;
			goto err_tx_desc;
		}
		bzero((void *)txq->ift_dma_info.ifd_vaddr, txq_size);
		/* Allocate a buf ring */
		txr->br = buf_ring_alloc(4096, M_DEVBUF, M_WAITOK, &txr->ift_mtx);
		/*
		 * Next the RX queues...
		 */
		rxq = &ctx->ifc_rxqs[i];
		rxq->ift_ctx = ctx;
		rxq->ift_id = i;

        /* Allocate receive buffers for the ring*/
		if (iflib_rxsd_alloc(rxq)) {
			device_printf(dev,
			    "Critical Failure setting up receive buffers\n");
			err = ENOMEM;
			goto err_rx_desc;
		}
		
		/* Initialize the RX lock */
		snprintf(rxq->ifr_mtx_name, sizeof(rxq->ifr_mtx_name), "%s:rx(%d)",
		    device_get_nameunit(dev), rxq->ifr_id);
		mtx_init(&rxq->ifr_mtx, rxq->ifr_mtx_name, NULL, MTX_DEF);
		
		if (iflib_dma_alloc(ctx, rxq_size,
			&rxq->ift_dma_info, BUS_DMA_NOWAIT)) {
			device_printf(dev,
			    "Unable to allocate TX Descriptor memory\n");
			err = ENOMEM;
			goto err_tx_desc;
		}
		bzero((void *)rxq->ifr_dma_info.ifd_vaddr, rxq_size);
	}
	ctx->ifc_txqs = txqs;
	ctx->ifc_rxqs = rxqs;
	ctx->ifc_txq_size = txq_size;
	ctx->ifc_rxq_size = rxq_size;
	if ((err = IFC_QUEUES_ALLOC(ctx)) != 0)
		iflib_tx_structures_free(ctx);
		
	return (0);
err_rx_desc:
	for (rxq = ctx->ifc_rxqs; rxconf > 0; rxq++, rxconf--)
		iflib_dma_free(ctx, &rxq->ifr_dma_info);
err_tx_desc:
	for (txq = ctx->ifc_txqs; txconf > 0; txq++, txconf--)
		iflib_dma_free(adapter, &txq->ifr_dma_info);
	free(ctx->ifc_rxqs, M_DEVBUF);
rx_fail:
	free(ctx->ifc_txqs, M_DEVBUF);
fail:
	return (err);
}


/*
 *
 *
 */

static void
iflib_txq_setup(iflib_txq_t txq)
{
	iflib_ctx_t ctx = txq->ift_ctx;
	iflib_buf_t txbuf;
#ifdef DEV_NETMAP
	struct netmap_slot *slot;
	struct netmap_adapter *na = netmap_getna(adapter->ifp);
#endif /* DEV_NETMAP */
	
	TX_LOCK(txr);
#ifdef DEV_NETMAP
	slot = netmap_reset(na, NR_TX, txr->me, 0);
#endif /* DEV_NETMAP */	

    /* Set number of descriptors available */
	txr->tx_avail = adapter->ntxd;
	txr->qstatus = IFLIB_QUEUE_IDLE;

	/* Reset indices */
	txr->next_avail_desc = 0;
	txr->next_to_clean = 0;

	/* Free any existing tx buffers. */
	txbuf = txr->ift_buffers;
	for (i = 0; i < adapter->ntxd; i++, txbuf++) {
		iflib_tx_free(ctx, txr, txbuf);
#ifdef DEV_NETMAP
		if (slot) {
			int si = netmap_idx_n2k(&na->tx_rings[txr->me], i);
			uint64_t paddr;
			void *addr;

			addr = PNMB(na, slot + si, &paddr);
			txr->tx_base[i].buffer_addr = htole64(paddr);
			/* reload the map for netmap mode */
			netmap_load_map(na, txr->txtag, txbuf->map, addr);
		}
#endif /* DEV_NETMAP */

		/* clear the watch index */
		txbuf->next_eop = -1;
	}

	IFC_TXQ_SETUP(ctx, txq->ift_hwq);
	/* 
	 * XXX tx busdma state in disarray
	 */
	bus_dmamap_sync(txq->ift_qtag, txq->ift_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	TX_UNLOCK(txr);	
}

void
iflib_tx_structures_setup(iflib_ctx_t ctx)
{
	iflib_txq_t txr = ctx->ifc_txr;

	for (int i = 0; i < ctx->ifc_nqueues; i++, txr++)
		iflib_txq_setup(txq);
}


int
iflib_attach(device_t dev)
{
	iflib_shared_ctx_t sctx = device_get_softc(dev);
	iflib_ctx_t ctx;
	if_t ifp;
		device_printf(dev, "can not allocate ifnet structure\n");
	ctx = malloc(sizeof(struct iflib_ctx), M_DEVBUF, M_WAITOK);
	if (ctx == NULL)
		return (ENOMEM);
	CTX_LOCK_INIT(ctx);
	callout_init_mtx(&ctx->ifc_timer, &ctx->ifc_mtx, 0);
	sctx->isc_ctx = ctx;
	ctx->ifc_sctx = sctx;
	ifp = ctx->ifc_ifp = if_gethandle(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "can not allocate ifnet structure\n");
		return (ENOSPC);
	}

	if_initname_drv(ifp, device_get_name(dev), device_get_unit(dev));
	if_setdev(ifp, dev);
	if_setinitfn(ifp, iflib_if_init);
	if_setsoftc(ifp, ctx);
	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setioctlfn(ifp, iflib_if_ioctl);
	if_settransmitfn(ifp, iflib_if_transmit);
	if_setqflushfn(ifp, iflib_if_qflush);
	ether_ifattach_drv(ifp, ctx->ifc_hw.mac.addr);

	if_setcapabilities(ifp, 0);
	if_setcapenable(ifp, 0);
	
	return (0);
}


static void
iflib_stop(iflib_ctx_t ctx)
{
	iflib_txq_t txr = ctx->ifc_txqs;

	IFC_DISABLE_INTR(ctx);
	callout_stop(ctx->ifc_timer);
	/* Tell the stack that the interface is no longer active */
	if_setdrvflagbits(ctx->ifc_ifp, IFF_DRV_OACTIVE, IFF_DRV_RUNNING);

	/* Wait for curren tx queue users to exit to disarm watchdog timer. */
	for (int i = 0; i < ctx->ifc_nqueues; i++, txr++) {
		TX_LOCK(txr);
		txr->ift_qstatus = IFLIB_QUEUE_IDLE;
		TX_UNLOCK(txr);
	}
	IFC_STOP(ctx);
}


int
iflib_detach(device_t dev)
{
	iflib_shared_ctx_t sctx = device_get_softc(dev);
	iflib_ctx_t ctx = sctx->isc_ctx;
	if_t ifp = ctx->ifc_ifp;

	/* Make sure VLANS are not using driver */
	if (if_vlantrunkinuse(ifp)) {
		device_printf(dev,"Vlan in use, detach first\n");
		return (EBUSY);
	}

	CTX_LOCK(ctx);
	ctx->in_detach = 1;
	iflib_stop(ctx);
	CTX_UNLOCK(ctx);
	CTX_LOCK_DESTROY(ctx);

	/* Unregister VLAN events */
	if (ctx->ifc_vlan_attach_event != NULL)
		EVENTHANDLER_DEREGISTER(vlan_config, ctx->ifc_vlan_attach_event);
	if (ctx->ifc_vlan_detach_event != NULL)
		EVENTHANDLER_DEREGISTER(vlan_unconfig, ctx->ifc_vlan_detach_event); 

	ether_ifdetach_drv(ctx->ifc_ifp);
	if (ctx->ifc_led_dev != NULL)
		led_destroy(ctx->ifc_led_dev);
	IFC_DETACH(ctx);
	callout_drain(&adapter->timer);

#ifdef DEV_NETMAP
	netmap_detach(ifp);
#endif /* DEV_NETMAP */

	bus_generic_detach(dev);
	iflib_ctx_free(ctx);

	iflib_transmit_structures_free(ctx);
	iflib_receive_structures_free(ctx);
	return (0);
}


int
iflib_suspend(device_t dev)
{
	iflib_shared_ctx_t sctx = device_get_softc(dev);
	iflib_ctx_t ctx = sctx->isc_ctx;
	
	CTX_LOCK(ctx);
	IFC_SUSPEND(ctx);
	CTX_UNLOCK(ctx);

	return bus_generic_suspend(dev);
}

int
iflib_resume(device_t dev)
{
	iflib_shared_ctx_t sctx = device_get_softc(dev);
	iflib_ctx_t ctx = sctx->isc_ctx;
	
	CTX_LOCK(ctx);
	IFC_RESUME(ctx);
	CTX_UNLOCK(ctx);
	for (int i = 0; i < ctx->ifc_ntxr; i++)
		iflib_txr_transmit(&ctx->ifc_txr[i], NULL);

	return (bus_generic_resume(dev));
}

void
iflib_vlan_register(void *arg, if_t ifp, u16 vtag)
{
	iflib_ctx_t ctx = if_getsoftc(ifp);
	u32 index, bit;

	if ((void *)ctx != arg)
		return;

	if ((vtag == 0) || (vtag > 4095))
		return;

	CTX_LOCK(ctx);
	IFC_VLAN_REGISTER(ctx, vtag);
	/* Re-init to load the changes */
	if (if_getcapenable(ifp) & IFCAP_VLAN_HWFILTER)
		_iflib_init(ctx);
	CTX_UNLOCK(ctx);
}

void
iflib_vlan_unregister(void *arg, if_t ifp, u16 vtag)
{
	iflib_ctx_t ctx = if_getsoftc(ifp);

	if ((void *)ctx != arg)
		return;

	if ((vtag == 0) || (vtag > 4095))
		return;

	CTX_LOCK(ctx);
	IFC_VLAN_UNREGISTER(ctx, vtag);
	/* Re-init to load the changes */
	if (if_getcapenable(ifp) & IFCAP_VLAN_HWFILTER)
		_iflib_init(ctx);
	CTX_UNLOCK(ctx);
}

void
iflib_ctx_free(iflib_ctx_t ctx)
{
	if_free_drv(ctx->ifc_ifp);
}


void
iflib_promisc_config(iflib_ctx_t ctx)
{

	IFC_PROMISC_CONFIG(ctx, if_getflags(ctx->ifc_ifp));
}


/*
 * Ifnet functions exported upward to network stack
 */

static void
iflib_if_init(void *arg)
{
	iflib_ctx_t ctx = arg;

	iflib_init(ctx);
}

static int
iflib_if_transmit(if_t ifp, struct mbuf *m)
{
	ctx_t	*ctx = if_getsoftc(ifp);
	struct tx_ring	*txr;
	int 		err;

	/*
	* XXX calculate txr and buf_ring based on flowid
	* or other policy flag
	*/
	txr = &ctx->ifc_txqs[0];
	br = &txr->tx_br[0];
	err = 0;
	if (TX_TRYLOCK(txr)) {
		err = _iflib_txr_transmit(txr, m);
		TX_UNLOCK(txr);
	} else if (m != NULL)
		err = drbr_enqueue(ifp, br, m);

	return (err);
}

static void
iflib_if_qflush(if_t ifp)
{
	ctx_t ctx = if_getsoftc(ifp);
	struct tx_ring  *txr = ctx->ifc_txqs;
	struct mbuf     *m;

	for (int i = 0; i < adapter->nqueues; i++, txr++) {
		TX_LOCK(txr);
		for (int j = 0; j < txr->tx_nbr; j++) {
			while ((m = buf_ring_dequeue_sc(&txr->br[j])) != NULL)
				m_freem(m);
		}
		TX_UNLOCK(txr);
	}
	if_qflush(ifp);
	
}

static int
iflib_if_ioctl(if_t ifp, u_long command, caddr_t data)
{

	iflib_ctx_t ctx = if_getsoftc(ifp);
	struct ifreq	*ifr = (struct ifreq *)data;
#if defined(INET) || defined(INET6)
	struct ifaddr	*ifa = (struct ifaddr *)data;
#endif
	bool		avoid_reset = FALSE;
	int		err = 0;

	switch (command) {
	case SIOCSIFADDR:
#ifdef INET
		if (ifa->ifa_addr->sa_family == AF_INET)
			avoid_reset = TRUE;
#endif
#ifdef INET6
		if (ifa->ifa_addr->sa_family == AF_INET6)
			avoid_reset = TRUE;
#endif
		/*
		** Calling init results in link renegotiation,
		** so we avoid doing it when possible.
		*/
		if (avoid_reset) {
			if_setflagbits(ifp, IFF_UP,0);
			if (!(if_getdrvflags(ifp)& IFF_DRV_RUNNING))
				iflib_init(ctx);
#ifdef INET
			if (!(if_getflags(ifp) & IFF_NOARP))
				arp_ifinit_drv(ifp, ifa);
#endif
		} else
			err = ether_ioctl_drv(ifp, command, data);
		break;
	case SIOCSIFMTU:
		CTX_LOCK(ctx);
		IFC_MTU_SET(ctx, mtu);
		_iflib_init(ctx);
		if_setmtu(ifp, mtu);
		CTX_UNLOCK(ctx);
		break;
	case SIOCSIFFLAGS:
		CTX_LOCK(ctx);
		if (if_getflags(ifp) & IFF_UP) {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
				if ((if_getflags(ifp) ^ adapter->if_flags) &
				    (IFF_PROMISC | IFF_ALLMULTI)) {
					IFC_PROMISC_CONFIG(ctx->ifc_sctx, if_getflags(ifp));
				}
			} else
				ctx->ifc_init(ctx);
		} else
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING)
				IFC_STOP(ctx);
		adapter->if_flags = if_getflags(ifp);
		CTX_UNLOCK(ctx);
		break;

		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		IOCTL_DEBUGOUT("ioctl rcv'd: SIOC(ADD|DEL)MULTI");
		if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
			CTX_LOCK(ctx);
			ctx->ifc_intr_disable(ctx);
			ctx->ifc_multi_set(ctx);
			ctx->ifc_intr_enable(ctx);
			CTX_LOCK(ctx);
		}
		break;
	case SIOCSIFMEDIA:
		CTX_LOCK(ctx);
		IFC_MEDIA_SET(ctx);
		CTX_UNLOCK(ctx);
		/* falls thru */
	case SIOCGIFMEDIA:
		err = ifmedia_ioctl_drv(ifp, ifr, &adapter->media, command);
		break;
	case SIOCSIFCAP:
	    {
		int mask, reinit;

		IOCTL_DEBUGOUT("ioctl rcv'd: SIOCSIFCAP (Set Capabilities)");
		reinit = 0;
		mask = ifr->ifr_reqcap ^ if_getcapenable(ifp);
		if (mask & IFCAP_POLLING) {
			if (ifr->ifr_reqcap & IFCAP_POLLING) {
#ifdef notyet
				err = ether_poll_register_drv(em_poll, ifp);
#else
				err = ENOTSUP;
#endif
				if (err)
					return (err);
				CTX_LOCK(ctx);
				IFC_DISABLE_INTR(ctx);
				if_setcapenablebit(ifp, IFCAP_POLLING, 0);
				CTX_UNLOCK(ctx);
			} else {
				err = ether_poll_deregister_drv(ifp);
				/* Enable interrupt even in err case */
				CTX_LOCK(ctx);
				IFC_INTR_ENABLE(ctx);
				if_setcapenablebit(ifp, 0, IFCAP_POLLING);
				CTX_UNLOCK(ctx);
			}
		}
		if (mask & IFCAP_HWCSUM) {
			if_togglecapenable(ifp, IFCAP_HWCSUM);
			reinit = 1;
		}
		if (mask & IFCAP_TSO4) {
			if_togglecapenable(ifp, IFCAP_TSO4);
			reinit = 1;
		}
		if (mask & IFCAP_VLAN_HWTAGGING) {
			if_togglecapenable(ifp, IFCAP_VLAN_HWTAGGING);
			reinit = 1;
		}
		if (mask & IFCAP_VLAN_HWFILTER) {
			if_togglecapenable(ifp, IFCAP_VLAN_HWFILTER);
			reinit = 1;
		}
		if (mask & IFCAP_VLAN_HWTSO) {
			if_togglecapenable(ifp, IFCAP_VLAN_HWTSO);
			reinit = 1;
		}
		if ((mask & IFCAP_WOL) &&
		    (if_getcapabilities(ifp) & IFCAP_WOL) != 0) {
			if (mask & IFCAP_WOL_MCAST)
				if_togglecapenable(ifp, IFCAP_WOL_MCAST);
			if (mask & IFCAP_WOL_MAGIC)
				if_togglecapenable(ifp, IFCAP_WOL_MAGIC);
		}
		if (reinit && (if_getdrvflags(ifp) & IFF_DRV_RUNNING)) {
			iflib_init(ctx);
		}
		if_vlancap(ifp);
		break;
	    }

	default:
		err = ether_ioctl_drv(ifp, command, data);
		break;
	}

	return (err);
}

/*
 * MI independent logic
 *
 */
static void
iflib_timer(void *arg)
{
	iflib_ctx_t ctx = arg;
	iflib_txq_t txr = ctx->ifc_txr;
	CTX_LOCK(ctx);
	
	/*
	** Check on the state of the TX queue(s), this 
	** can be done without the lock because its RO
	** and the HUNG state will be static if set.
	*/
	IFC_TIMER(ctx);
	for (int i = 0; i < ctx->ifc_nqueues; i++, txr++) {
		if ((txr->ift_qstatus == IFLIB_QUEUE_HUNG) &&
		    (ctx->_ifc_pause_frames == 0))
			goto hung;

		if (txr->tx_avail <= ctx->ifc_max_scatter)
			taskqueue_enqueue(txr->ift_tq, &txr->ift_task);
	}
	ctx->ifc_pause_frames = 0;
	callout_reset(&ctx->ifc_timer, hz, iflib_timer, ctx);
	goto unlock;
	
hung:
	if_setdrvflagbits(ifp, 0, IFF_DRV_RUNNING);
	ctx->ifc_watchdog_reset(ctx);
	ctx->ifc_watchdog_events++;
	ctx->ifc_pause_frames = 0;

	IFC_INIT(ctx);
unlock:
	CTX_UNLOCK(ctx);
}

static void
_iflib_led_func(void *arg, int onoff)
{
	iflib_ctx_t ctx = arg;

	CTX_LOCK(ctx);
	IFC_LED_FUNC(ctx, onoff);
	CTX_UNLOCK(ctx);
}

void
iflib_led_create(iflib_ctx_t ctx)
{
	ctx->ifc_led_dev = led_create(_iflib_led_func, ctx,
								  device_get_nameunit(ctx->ifc_dev));
}

static int
_iflib_tx(iflib_txq_t txr, struct mbuf **m_headp)
{
	struct adapter		*adapter = txr->adapter;
	bus_dma_segment_t	*segs /* [EM_MAX_SCATTER]*/;
	bus_dmamap_t		map;
	iflib_buf_t	tx_buffer, tx_buffer_mapped;
	struct mbuf		*m_head;
	struct ether_header	*eh;
	struct ip		*ip = NULL;
	struct tcphdr		*tp = NULL;
	int			ip_off, poff;
	int			nsegs, i, j, first, last = 0;
	int			err, do_tso, tso_desc = 0, remap = 1;

	segs = txq->ift_segs;
	m_head = *m_headp;
	txd_upper = txd_lower = txd_used = txd_saved = 0;
	do_tso = ((m_head->m_pkthdr.csum_flags & CSUM_TSO) != 0);
	ip_off = poff = 0;

	/*
	 * Intel recommends entire IP/TCP header length reside in a single
	 * buffer. If multiple descriptors are used to describe the IP and
	 * TCP header, each descriptor should describe one or more
	 * complete headers; descriptors referencing only parts of headers
	 * are not supported. If all layer headers are not coalesced into
	 * a single buffer, each buffer should not cross a 4KB boundary,
	 * or be larger than the maximum read request size.
	 * Controller also requires modifing IP/TCP header to make TSO work
	 * so we firstly get a writable mbuf chain then coalesce ethernet/
	 * IP/TCP header into a single buffer to meet the requirement of
	 * controller. This also simplifies IP/TCP/UDP checksum offloading
	 * which also has similiar restrictions.
	 */
	if (do_tso || m_head->m_pkthdr.csum_flags & CSUM_OFFLOAD) {
		if (do_tso || (m_head->m_next != NULL && 
		    m_head->m_pkthdr.csum_flags & CSUM_OFFLOAD)) {
			if (M_WRITABLE(*m_headp) == 0) {
				m_head = m_dup(*m_headp, M_NOWAIT);
				m_freem(*m_headp);
				if (m_head == NULL) {
					*m_headp = NULL;
					return (ENOBUFS);
				}
				*m_headp = m_head;
			}
		}
		/*
		 * XXX
		 * Assume IPv4, we don't have TSO/checksum offload support
		 * for IPv6 yet.
		 */
		ip_off = sizeof(struct ether_header);
		m_head = m_pullup(m_head, ip_off);
		if (m_head == NULL) {
			*m_headp = NULL;
			return (ENOBUFS);
		}
		eh = mtod(m_head, struct ether_header *);
		if (eh->ether_type == htons(ETHERTYPE_VLAN)) {
			ip_off = sizeof(struct ether_vlan_header);
			m_head = m_pullup(m_head, ip_off);
			if (m_head == NULL) {
				*m_headp = NULL;
				return (ENOBUFS);
			}
		}
		m_head = m_pullup(m_head, ip_off + sizeof(struct ip));
		if (m_head == NULL) {
			*m_headp = NULL;
			return (ENOBUFS);
		}
		ip = (struct ip *)(mtod(m_head, char *) + ip_off);
		poff = ip_off + (ip->ip_hl << 2);
		if (do_tso) {
			m_head = m_pullup(m_head, poff + sizeof(struct tcphdr));
			if (m_head == NULL) {
				*m_headp = NULL;
				return (ENOBUFS);
			}
			tp = (struct tcphdr *)(mtod(m_head, char *) + poff);
			/*
			 * TSO workaround:
			 *   pull 4 more bytes of data into it.
			 */
			m_head = m_pullup(m_head, poff + (tp->th_off << 2) + 4);
			if (m_head == NULL) {
				*m_headp = NULL;
				return (ENOBUFS);
			}
			ip = (struct ip *)(mtod(m_head, char *) + ip_off);
			ip->ip_len = 0;
			ip->ip_sum = 0;
			/*
			 * The pseudo TCP checksum does not include TCP payload
			 * length so driver should recompute the checksum here
			 * what hardware expect to see. This is adherence of
			 * Microsoft's Large Send specification.
			 */
			tp = (struct tcphdr *)(mtod(m_head, char *) + poff);
			tp->th_sum = in_pseudo(ip->ip_src.s_addr,
			    ip->ip_dst.s_addr, htons(IPPROTO_TCP));
		} else if (m_head->m_pkthdr.csum_flags & CSUM_TCP) {
			m_head = m_pullup(m_head, poff + sizeof(struct tcphdr));
			if (m_head == NULL) {
				*m_headp = NULL;
				return (ENOBUFS);
			}
			tp = (struct tcphdr *)(mtod(m_head, char *) + poff);
			m_head = m_pullup(m_head, poff + (tp->th_off << 2));
			if (m_head == NULL) {
				*m_headp = NULL;
				return (ENOBUFS);
			}
			ip = (struct ip *)(mtod(m_head, char *) + ip_off);
			tp = (struct tcphdr *)(mtod(m_head, char *) + poff);
		} else if (m_head->m_pkthdr.csum_flags & CSUM_UDP) {
			m_head = m_pullup(m_head, poff + sizeof(struct udphdr));
			if (m_head == NULL) {
				*m_headp = NULL;
				return (ENOBUFS);
			}
			ip = (struct ip *)(mtod(m_head, char *) + ip_off);
		}
		*m_headp = m_head;
	}

	/*
	 * Map the packet for DMA
	 *
	 * Capture the first descriptor index,
	 * this descriptor will have the index
	 * of the EOP which is the only one that
	 * now gets a DONE bit writeback.
	 */
	first = txr->next_avail_desc;
	tx_buffer = &txr->tx_buffers[first];
	tx_buffer_mapped = tx_buffer;
	map = tx_buffer->ifsd_map;

retry:
	err = bus_dmamap_load_mbuf_sg(txr->txtag, map,
	    *m_headp, segs, &nsegs, BUS_DMA_NOWAIT);

	/*
	 * There are two types of errors we can (try) to handle:
	 * - EFBIG means the mbuf chain was too long and bus_dma ran
	 *   out of segments.  Defragment the mbuf chain and try again.
	 * - ENOMEM means bus_dma could not obtain enough bounce buffers
	 *   at this point in time.  Defer sending and try again later.
	 * All other errors, in particular EINVAL, are fatal and prevent the
	 * mbuf chain from ever going through.  Drop it and report error.
	 */
	if (err == EFBIG && remap) {
		struct mbuf *m;

		m = m_defrag(*m_headp, M_NOWAIT);
		if (m == NULL) {
			adapter->mbuf_alloc_failed++;
			m_freem(*m_headp);
			*m_headp = NULL;
			return (ENOBUFS);
		}
		*m_headp = m;

		/* Try it again, but only once */
		remap = 0;
		goto retry;
	} else if (err == ENOMEM) {
		adapter->no_tx_dma_setup++;
		return (err);
	} else if (err != 0) {
		adapter->no_tx_dma_setup++;
		m_freem(*m_headp);
		*m_headp = NULL;
		return (err);
	}

	if (nsegs > (txr->tx_avail - 2)) {
		txr->no_desc_avail++;
		bus_dmamap_unload(txr->ift_txtag, map);
		return (ENOBUFS);
	}
	m_head = *m_headp;

	pi->pi_segs = segs;
	pi->pi_nsegs = nsegs;
	if ((err = IFC_TX(ctx, txq->ift_hwq, pi)) == 0)
		tx_buffer->next_eop = pi->pi_last;
	return (err);
}

static int
_iflib_txr_transmit(iflib_txq_t txr, struct mbuf *m)
{
	iflib_ctx_t ctx = txr->txr_ctx;
	if_t ifp = ctx->ifc_if;
	struct mbuf     *next, *mp = m;
	int             err = 0, enq = 0;

	if (((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != IFF_DRV_RUNNING) ||
			adapter->link_active == 0) {
		if (m != NULL)
			err = drbr_enqueue(ifp, txr->br, m);
		return (err);
	}
	/*
	* Transmit in the order of arrival if other packets
	* are already waiting
	*/
	if ((m != NULL) && (drbr_peek(ifp, txr->br) != NULL)) {
		if ((err = drbr_enqueue(ifp, txr->br, m)) != 0)
			return (err);
		mp = NULL;
	}
		
	/*
	* Handle immediate fast path
	*/
	if (mp != NULL) {
		if ((err = _iflib_tx(txr, &m)) != 0) {
			if (drbr_enqueue(ifp, txr->br, m) != 0)
				m_freem(m);
			goto done;
		}
		enq = 1;
	}
	/* Process the queue */
	while ((next = drbr_peek(ifp, txr->br)) != NULL) {
		if ((err = _iflib_tx(txr, &next)) != 0) {
			if (next == NULL)
				drbr_advance(ifp, txr->br);
			else 
				drbr_putback(ifp, txr->br, next);
			break;
		}
		drbr_advance(ifp, txr->br);
		enq++;
		if_incobytes(ifp,  next->m_pkthdr.len);
		if (next->m_flags & M_MCAST)
			if_incomcasts(ifp, 1);
		if_etherbpfmtap(ifp, next);
		if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0)
                        break;
	}
	done:
	if (enq > 0) {
		/* Set the watchdog */
		txr->ift_qstatus = IFLIB_QUEUE_WORKING;
		txr->ift_watchdog_time = ticks;
	}

	if (txr->ift_tx_avail < ctx->ifc_max_scatter)
		iflib_txeof(txr);
	if (txr->ift_tx_avail < ctx->ifc_max_scatter) 
		txr->ift_tx_active = 0;
	return (err);
	
}

iflib_incopackets(iflib_ctx_t, int n)
{

	if_incopackets(ctx->ifp, n);
}

void
iflib_linkstate_change(iflib_ctx_t ctx, uint64_t baudrate, int link_state)
{
	if_t ifp = ctx->ifp;
	iflib_txq_t txq = ctx->ifc_txq;

	if_setbaudrate(ifp, baudrate);
	/* If link down, disable watchdog */
	if ((ctx->ifc_link_state == LINK_STATE_UP) && (link_state == LINK_STATE_DOWN)) {
		for (int i = 0; i < ctx->ifc_nqueues; i++, txq++)
			txq->ift_qstatus = IFLIB_QUEUE_IDLE;
	}
	if_linkstate_change_drv(ifp, link_state);
}

void
iflib_intr_rx(void *arg)
{
	struct rx_ring *rxr = arg;

	++rxr->rx_irq;
	_task_fn_rx(arg, 0);
}

void
iflib_intr_tx(void *arg)
{
	iflib_txq_t txr = arg;

	++txr->tx_irq;
	_task_fn_tx(arg, 0);
}

void
iflib_intr_link(void *arg)
{
	iflib_ctx_t ctx = arg;

	++ctx->ifc_link_irq;
	_task_fn_link(arg, 0);
}

void
iflib_txbuf_free(iflib_ctx_t ctx, struct tx_ring *txr, iflib_buf_t *tx_buffer)
{
	if (tx_buffer->m_head == NULL)
		return;
	bus_dmamap_sync(txr->txtag,
				    tx_buffer->map,
				    BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(txr->txtag,
					  tx_buffer->map);
	m_freem(tx_buffer->m_head);
	tx_buffer->m_head = NULL;
}

void
iflib_txbuf_destroy(iflib_ctx_t ctx, struct tx_ring *txr, iflib_buf_t *tx_buffer)
{
	if (tx_buffer->m_head != NULL) {
		iflib_tx_free(ctx, txr, tx_buffer);
		if (txbuf->map != NULL) {
			bus_dmamap_destroy(txr->txtag,
				    txbuf->map);
			txbuf->map = NULL;
		}
	} else if (txbuf->map != NULL) {
		bus_dmamap_unload(txr->txtag,
						  tx_buffer->map);
		bus_dmamap_destroy(txr->txtag,
						   tx_buffer->map);
		tx_buffer->map = NULL;
	} 
}
