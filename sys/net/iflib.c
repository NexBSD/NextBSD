struct iflib_ctx {
	int ifc_ntxr;
	if_t ifc_ifp;
	iflib_tx_ring_t ifc_txr;

	
	ifc_init(iflib_ctx_t);
	ifc_resume(iflib_ctx_t);
	
	
};

struct iflib_tx_ring {
	iflib_ctx_t             ift_ctx;
	struct mtx              ift_mtx;
	char                    ift_mtx_name[16];
	iflib_buffer_t          ift_buffers;
	struct buf_ring        *ift_br;
	bus_dma_tag_t		    ift_txtag;
	u32                     ift_next_avail_desc;
	u32                     ift_next_to_clean;
	volatile u16            ift_tx_avail;
	unsigned long		    ift_no_desc_avail;
	void                   *ift_hw_txr;
	struct task             ift_task;
	struct taskqueue       *ift_tq;
	int			            ift_queue_status;
	int                     ift_watchdog_time;

};

struct iflib_rx_ring {
	iflib_ctx_t             ifr_ctx;
	struct mtx              ifr_mtx;
	char                    ifr_mtx_name[16];
	struct task             ifr_task;
	struct taskqueue       *ifr_tq;
	iflib_buffer_t          ifr_buffers;
	bus_dma_tag_t           ifr_tag;
};

struct iflib_buffer {
	int		        ifb_next_eop;    /* Index of the last buffer */
	struct mbuf    *ifb_m;
	bus_dmamap_t    ifb_map;         /* bus_dma map for packet */
};

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

/*
 * XXX Incomplete
 */

iflib_ctx_t
iflib_dev_init(void *sc)
{
	ctx->ifc_sc = sc;
	CTX_LOCK_INIT(ctx);
	callout_init_mtx(&ctx->ifc_timer, &ctx->ifc_mtx, 0);
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
	

/*
 * Services routines exported downward to the driver
 */
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

iflib_ctx_t
iflib_drv_init(device_t dev, int unit, void *sc)
{
	if_t ifp;
	iflib_ctx_t ctx;
	
	ctx = malloc(sizeof(struct iflib_ctx), M_DEVBUF, M_WAITOK);
	
	ifp = ctx->ifc_ifp = if_gethandle(IFT_ETHER);
	ctx->ifc_sc = sc;

	if (ifp == 0) {
		device_printf(dev, "can not allocate ifnet structure\n");
		return (-1);
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

	return (ctx);
}

void
iflib_init(iflib_ctx_t *ctx)
{

	iflib_if_init(ctx);
}

static void
_task_fn_legacy_intr(void *context, int pending)
{
	iflib_ctx_t ctx = context;
	if_t ifp = ctx->ifp;
	iflib_tx_ring_t	txr = adapter->tx_rings;
	iflib_rx_ring_t	rxr = adapter->rx_rings;
	bool more;

	/* legacy do it all crap function */
	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING)  == 0)
		goto enable;

	more = em_rxeof(rxr, adapter->rx_process_limit, NULL);

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
	iflib_tx_ring_t txr = context;
	iflib_ctx_t ctx = trx->ift_ctx;
	
	TXR_LOCK(tx);
	_iflib_txr_transmit(txr, NULL);
	ctx->ifc_tx_intr_enable(txr);
	TXR_UNLOCK(tx);
}

static void
_task_fn_rx(void *context, int pending)
{
	iflib_rx_ring_t rxr = arg;
	iflib_ctx_t ctx = rxr->ift_ctx;
	int more = 0;

	if (!(if_getdrvflags(ctx->ifc_ifp) & IFF_DRV_RUNNING))
		return;

	
	if (RX_TRY_LOCK(rxr)) {
		if ((more = ctx->ifc_rxeof(rxr, ctx->rx_process_limit, NULL)) == 0)
			ctx->ifc_rx_intr_enable(rxr);
		RX_UNLOCK(rxr);
	}
	if (more)
		taskqueue_enqueue(rxr->ifr_tq, &rxr->ifr_task);


}

static void
_task_fn_link(void *context, int pending)
{
	iflib_ctx_t ctx = context;
	iflib_tx_ring_t txr = ctx->ifc_txr;

	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
		return;

	CTX_LOCK(ctx);
	callout_stop(&ctx->ifc_timer);
	ctx->ifc_update_link_status(ctx);
	callout_reset(&ctx->ifc_timer, hz, iflib_timer, ctx);
	ctx->ifc_link_intr_enable(ctx);	
	CTX_UNLOCK(ctx);

	if (ctx->ifc_link_active == 0)
		return;
	
	for (int i = 0; i < ctx->ifc_num_queues; i++, txr++) {
		if (TXR_TRYLOCK(txr) == 0)
			continue;
		_iflib_txr_transmit(txr, NULL);
		TXR_UNLOCK(txr);
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
iflib_transmit_buffers_alloc(iflib_tx_ring_t txr)
{
	iflib_ctx_t ctx = txr->ift_ctx;
	device_t dev = ctx->ifc_dev;
	iflib_buffer_t txbuf;
	int error, i;
	
	/*
	 * Setup DMA descriptor areas.
	 */
	if ((error = bus_dma_tag_create(bus_get_dma_tag(dev),
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

	if (!(txr->tx_buffers =
	    (struct em_buffer *) malloc(sizeof(struct iflib_buffer) *
	    ctx->ifc_num_tx_desc, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate tx_buffer memory\n");
		error = ENOMEM;
		goto fail;
	}

        /* Create the descriptor buffer dma maps */
	txbuf = txr->tx_buffers;
	for (i = 0; i < ctx->ifc_num_tx_desc; i++, txbuf++) {
		error = bus_dmamap_create(txr->ift_tag, 0, &txbuf->ifb_map);
		if (error != 0) {
			device_printf(dev, "Unable to create TX DMA map\n");
			goto fail;
		}
	}

	return 0;
fail:
	/* We free all, it handles case where we are in the middle */
	iflib_transmit_structures_free(adapter);
	return (error);
}



static void
iflib_txring_destroy(struct tx_ring *txr)
{
	iflib_ctx_t ctx = txr->ctx;
	
	for (int i = 0; i < adapter->num_tx_desc; i++)
		iflib_txbuf_destroy(ctx, txr, &txr->tx_buffers[i]);
	if (txr->tx_buffers != NULL) {
		free(txr->tx_buffers, M_DEVBUF);
		txr->tx_buffers = NULL;
	}
	if (txr->txtag != NULL) {
		bus_dma_tag_destroy(txr->txtag);
		txr->txtag = NULL;
	}
	TXR_LOCK_DESTROY(txr);
}

static void
iflib_transmit_buffers_free(iflib_tx_ring_t txr)
{
	if (txr->ift_buffers == NULL)
		return;
	iflib_txring_destroy(txr);
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
em_allocate_receive_buffers(struct iflib_rx_ring_t rxr)
{
	iflib_ctx_t ctx = rxr->ifr_ctx;
	device_t dev = ctx->ifc_dev;
	iflib_buffer_t	rxbuf;
	int			error;

	rxr->rx_buffers = malloc(sizeof(struct iflib_buffer) *
	    ctx->ifc_num_rx_desc, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (rxr->ifr_buffers == NULL) {
		device_printf(dev, "Unable to allocate rx_buffer memory\n");
		return (ENOMEM);
	}

	error = bus_dma_tag_create(bus_get_dma_tag(dev), /* parent */
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
	if (error) {
		device_printf(dev, "%s: bus_dma_tag_create failed %d\n",
		    __func__, error);
		goto fail;
	}

	rxbuf = rxr->ifr_buffers;
	for (int i = 0; i < ctx->ifc_num_rx_desc; i++, rxbuf++) {
		error = bus_dmamap_create(rxr->ifr_tag, 0, &rxbuf->ifb_map);
		if (error) {
			device_printf(dev, "%s: bus_dmamap_create failed: %d\n",
			    __func__, error);
			goto fail;
		}
	}

	return (0);

fail:
	iflib_receive_structures_free(ctx);
	return (error);
}


/*********************************************************************
 *
 *  Initialize a receive ring and its buffers.
 *
 **********************************************************************/
static int
iflib_receive_ring_setup(iflib_rx_ring_t rxr)
{
	iflib_ctx_t ctx = rxr->ifr_ctx;
	iflib_buffer_t	rxbuf;
	bus_dma_segment_t	seg[1];
	int			i, rsize, nsegs, error = 0;
#ifdef DEV_NETMAP
	struct netmap_slot *slot;
	struct netmap_adapter *na = netmap_getna(adapter->ifp);
#endif

	/* Clear the ring contents */
	RX_LOCK(rxr);
	rsize = roundup2(adapter->num_rx_desc *
	    sizeof(struct e1000_rx_desc), EM_DBA_ALIGN);
	bzero((void *)rxr->rx_base, rsize);
#ifdef DEV_NETMAP
	slot = netmap_reset(na, NR_RX, rxr->ifr_id, 0);
#endif

	/*
	** Free current RX buffer structs and their mbufs
	*/
	for (i = 0; i < ctx->ifc_num_rx_desc; i++)
		iflib_rx_free(ctx, rxr, &rxr->ifr_buffers[i]);

	/* Now replenish the mbufs */
	for (i = 0; i != ctx->ifc_num_rx_desc; ++i) {
		rxbuf = &rxr->ifr_buffers[i];
#ifdef DEV_NETMAP
		if (slot) {
			int si = netmap_idx_n2k(&na->rx_rings[rxr->ifr_id], i);
			uint64_t paddr;
			void *addr;

			addr = PNMB(na, slot + si, &paddr);
			netmap_load_map(na, rxr->ifr_tag, rxbuf->ifb_map, addr);
			/* Update descriptor */
			rxr->rx_base[i].buffer_addr = htole64(paddr);
			continue;
		}
#endif /* DEV_NETMAP */
		rxbuf->ifb_cl = m_cljget(NULL, M_NOWAIT, ctx->ifc_rx_mbuf_size))
		rxbuf->ifb_m = m_getjcl(M_NOWAIT, MT_DATA,
		    M_PKTHDR, adapter->rx_mbuf_sz);
		if (rxbuf->m_head == NULL) {
			error = ENOBUFS;
			goto fail;
		}
		rxbuf->m_head->m_len = adapter->rx_mbuf_sz;
		rxbuf->m_head->m_flags &= ~M_HASFCS; /* we strip it */
		rxbuf->m_head->m_pkthdr.len = adapter->rx_mbuf_sz;

		/* Get the memory mapping */
		error = bus_dmamap_load_mbuf_sg(rxr->rxtag,
		    rxbuf->map, rxbuf->m_head, seg,
		    &nsegs, BUS_DMA_NOWAIT);
		if (error != 0) {
			m_freem(rxbuf->m_head);
			rxbuf->m_head = NULL;
			goto fail;
		}
		bus_dmamap_sync(rxr->rxtag,
		    rxbuf->map, BUS_DMASYNC_PREREAD);

		/* Update descriptor */
		rxr->rx_base[j].buffer_addr = htole64(seg[0].ds_addr);
	}
	rxr->next_to_check = 0;
	rxr->next_to_refresh = 0;
	bus_dmamap_sync(rxr->rxdma.dma_tag, rxr->rxdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

fail:
	EM_RX_UNLOCK(rxr);
	return (error);
}

/*********************************************************************
 *
 *  Initialize all receive rings.
 *
 **********************************************************************/
static int
em_setup_receive_structures(struct adapter *adapter)
{
	struct rx_ring *rxr = adapter->rx_rings;
	int q;

	for (q = 0; q < adapter->num_queues; q++, rxr++)
		if (em_setup_receive_ring(rxr))
			goto fail;

	return (0);
fail:
	/*
	 * Free RX buffers allocated so far, we will only handle
	 * the rings that completed, the failing case will have
	 * cleaned up for itself. 'q' failed, so its the terminus.
	 */
	for (int i = 0; i < q; ++i) {
		rxr = &adapter->rx_rings[i];
		for (int n = 0; n < adapter->num_rx_desc; n++) {
			struct em_buffer *rxbuf;
			rxbuf = &rxr->rx_buffers[n];
			if (rxbuf->m_head != NULL) {
				bus_dmamap_sync(rxr->rxtag, rxbuf->map,
			  	  BUS_DMASYNC_POSTREAD);
				bus_dmamap_unload(rxr->rxtag, rxbuf->map);
				m_freem(rxbuf->m_head);
				rxbuf->m_head = NULL;
			}
		}
		rxr->next_to_check = 0;
		rxr->next_to_refresh = 0;
	}

	return (ENOBUFS);
}

/*********************************************************************
 *
 *  Free all receive rings.
 *
 **********************************************************************/
static void
em_free_receive_structures(struct adapter *adapter)
{
	struct rx_ring *rxr = adapter->rx_rings;

	for (int i = 0; i < adapter->num_queues; i++, rxr++) {
		em_free_receive_buffers(rxr);
		/* Free the ring memory as well */
		em_dma_free(adapter, &rxr->rxdma);
		EM_RX_LOCK_DESTROY(rxr);
	}

	free(adapter->rx_rings, M_DEVBUF);
}


/*********************************************************************
 *
 *  Free receive ring data structures
 *
 **********************************************************************/
static void
em_free_receive_buffers(iflib_rx_ring_t rxr)
{
	iflib_ctx_t ctx = rxr->ifr_ctx;

	INIT_DEBUGOUT("free_receive_buffers: begin");

	if (rxr->rx_buffers != NULL) {
		for (int i = 0; i < ctx->ifc_num_rx_desc; i++)
			iflib_rx_free(ctx, rxr, &rxr->ifr_buffers[i])

		free(rxr->ifr_buffers, M_DEVBUF);
		rxr->rx_buffers = NULL;
		rxr->next_to_check = 0;
		rxr->next_to_refresh = 0;
	}

	if (rxr->rxtag != NULL) {
		bus_dma_tag_destroy(rxr->rxtag);
		rxr->rxtag = NULL;
	}

	return;
}

static int
iflib_dma_alloc(iflib_ctx_t ctx, bus_size_t size, iflib_dma_info_t dma,
				int mapflags)
{
	int error;

	error = bus_dma_tag_create(bus_get_dma_tag(ctx->ifc_dev), /* parent */
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
	if (error) {
		iflib_printf(ctx,
		    "%s: bus_dma_tag_create failed: %d\n",
		    __func__, error);
		goto fail_0;
	}

	error = bus_dmamem_alloc(dma->ifd_tag, (void**) &dma->ifd_vaddr,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT, &dma->ifd_map);
	if (error) {
		iflib_printf(ctx,
		    "%s: bus_dmamem_alloc(%ju) failed: %d\n",
		    __func__, (uintmax_t)size, error);
		goto fail_2;
	}

	dma->ifd_paddr = 0;
	error = bus_dmamap_load(dma->ifd_tag, dma->ifd_map, dma->ifd_vaddr,
	    size, em_dmamap_cb, &dma->ifd_paddr, mapflags | BUS_DMA_NOWAIT);
	if (error || dma->ifd_paddr == 0) {
		device_printf(adapter->dev,
		    "%s: bus_dmamap_load failed: %d\n",
		    __func__, error);
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

	return (error);
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
iflib_transmit_structures_free(iflib_ctx_t ctx)
{
	iflib_tx_ring_t txr = ctx->ifc_txr;

	for (int i = 0; i < ctx->ifc_num_queues; i++, txr++) {
		iflib_transmit_buffers_free(txr);
		iflib_dma_free(&txr->ift_dma_info);
	}
	free(adapter->tx_rings, M_DEVBUF);	
}

static int
iflib_queues_alloc(iflib_ctx_t ctx, int txq_size, int rxq_size)
{
	device_t dev = ctx->ifc_dev;
	iflib_tx_ring_t txr = NULL;
	iflib_tx_ring_t rxr = NULL;
	int num_queues = ctx->ifc_num_queues;

	/* Allocate the TX ring struct memory */
	if (!(txr =
	    (iflib_tx_ring_t) malloc(sizeof(struct iflib_tx_ring) *
	    num_queues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate TX ring memory\n");
		error = ENOMEM;
		goto fail;
	}
	
	/* Now allocate the RX */
	if (!(adapter->rx_rings =
	    (iflib_rx_ring_t) malloc(sizeof(struct iflib_rx_ring) *
	    num_queues, M_DEVBUF, M_NOWAIT | M_ZERO))) {
		device_printf(dev, "Unable to allocate RX ring memory\n");
		error = ENOMEM;
		goto rx_fail;
	}

	/*
	 * XXX handle allocation failure
	 */
	for (int i = 0; i < num_queues; i++, txconf++) {
		/* Set up some basics */
		txr = &ctx->ifc_txr[i];
		txr->ift_ctx = ctx;
		txr->ift_id = i;
		
		if (iflib_allocate_transmit_buffers(txr)) {
			device_printf(dev,
						  "Critical Failure setting up transmit buffers\n");
			error = ENOMEM;
			goto err_tx_desc;
		}

		/* Initialize the TX lock */
		snprintf(txr->mtx_name, sizeof(txr->mtx_name), "%s:tx(%d)",
		    device_get_nameunit(dev), txr->me);
		mtx_init(&txr->tx_mtx, txr->mtx_name, NULL, MTX_DEF);

		if (iflib_dma_alloc(ctx, txq_size,
			&txr->ift_dma_info, BUS_DMA_NOWAIT)) {
			device_printf(dev,
			    "Unable to allocate TX Descriptor memory\n");
			error = ENOMEM;
			goto err_tx_desc;
		}
		
		/* Allocate a buf ring */
		txr->br = buf_ring_alloc(4096, M_DEVBUF,
		    M_WAITOK, &txr->tx_mtx);
		/*
		 * Next the RX queues...
		 */
		rxr = &ctx->ifc_rxr[i];
		rxr->ift_ctx = ctx;
		rxr->ift_id = i;

        /* Allocate receive buffers for the ring*/
		if (iflib_receive_buffers_alloc(rxr)) {
			device_printf(dev,
			    "Critical Failure setting up receive buffers\n");
			error = ENOMEM;
			goto err_rx_desc;
		}
		
		/* Initialize the RX lock */
		snprintf(rxr->mtx_name, sizeof(rxr->mtx_name), "%s:rx(%d)",
		    device_get_nameunit(dev), txr->me);
		mtx_init(&rxr->rx_mtx, rxr->mtx_name, NULL, MTX_DEF);
		
		if (iflib_dma_alloc(ctx, txq_size,
			&txr->ift_dma_info, BUS_DMA_NOWAIT)) {
			device_printf(dev,
			    "Unable to allocate TX Descriptor memory\n");
			error = ENOMEM;
			goto err_tx_desc;
		}
		
	}
	err = ctx->ifc_queues_alloc(ctx, txq_size, rxq_size);

	return (0);

}

static void
iflib_transmit_ring_setup(iflib_tx_ring_t txr)
{
	iflib_ctx_t ctx = txr->ift_ctx;
	iflib_buffer_t txbuf;
#ifdef DEV_NETMAP
	struct netmap_slot *slot;
	struct netmap_adapter *na = netmap_getna(adapter->ifp);
#endif /* DEV_NETMAP */
	
	TX_LOCK(txr);
#ifdef DEV_NETMAP
	slot = netmap_reset(na, NR_TX, txr->me, 0);
#endif /* DEV_NETMAP */	

    /* Set number of descriptors available */
	txr->tx_avail = adapter->num_tx_desc;
	txr->queue_status = IFLIB_QUEUE_IDLE;

	/* Reset indices */
	txr->next_avail_desc = 0;
	txr->next_to_clean = 0;

	/* Free any existing tx buffers. */
	txbuf = txr->ift_buffers;
	for (i = 0; i < adapter->num_tx_desc; i++, txbuf++) {
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

	ctx->ifc_transmit_ring_setup(txr->ift_hw_txr);
	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	TX_UNLOCK(txr);	
}

void
iflib_transmit_structures_setup(iflib_ctx_t ctx)
{
	iflib_tx_ring_t txr = ctx->ifc_txr;

	for (int i = 0; i < ctx->ifc_num_queues; i++, txr++)
		iflib_transmit_ring_setup(txr);
}


int
iflib_attach(device_t dev)
{
	iflib_ctx_t ctx = iflib_dev_init(dev);

	return(ctx->ifc_attach(ctx));
}


static void
iflib_stop(iflib_ctx_t ctx)
{
	iflib_tx_ring_t txr = ctx->ifc_tx_rings;

	ctx->ifc_disable_intr(ctx);
	callout_stop(ctx->ifc_timer);
	/* Tell the stack that the interface is no longer active */
	if_setdrvflagbits(ctx->ifc_ifp, IFF_DRV_OACTIVE, IFF_DRV_RUNNING);

	/* Wait for curren tx queue users to exit to disarm watchdog timer. */
	for (int i = 0; i < ctx->ifc_num_queues; i++, txr++) {
		TX_LOCK(txr);
		txr->ift_queue_status = IFLIB_QUEUE_IDLE;
		TX_UNLOCK(txr);
	}
	ctx->ifc_stop(ctx);
}

int
iflib_detach(device_t dev)
{
	iflib_ctx_t ctx = device_get_softc(dev);
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
	ctx->ifc_detach(ctx);
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
	iflib_ctx_t ctx = device_get_softc(dev);
	
	CTX_LOCK(ctx);
	ctx->ifc_suspend(ctx);
	CTX_UNLOCK(ctx);

	return bus_generic_suspend(dev);
}

int
iflib_resume(device_t dev)
{
	iflib_ctx_t ctx = device_get_softc(dev);
	
	CTX_LOCK(ctx);
	ctx->ifc_resume(ctx);
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
	ctx->ifc_vlan_register(ctx, vtag);
	/* Re-init to load the changes */
	if (if_getcapenable(ifp) & IFCAP_VLAN_HWFILTER)
		iflib_init(ctx);
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
	ctx->ifc_vlan_unregister(ctx, vtag);
	/* Re-init to load the changes */
	if (if_getcapenable(ifp) & IFCAP_VLAN_HWFILTER)
		iflib_init(ctx);
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

	ctx->ifc_promisc_config(ctx, if_getflags(ctx->ifc_ifp));
}


/*
 * Ifnet functions exported upward to network stack
 */

static void
iflib_if_init(void *arg)
{
	iflib_ctx_t ctx = arg;
	
	CTX_LOCK(ctx);
	ctx->ifc_disable_intr(ctx);
	callout_stop(ctx->ifc_timer);
	ctx->ifc_init(ctx);
	if_setdrvflagbits(ctx->ifc_ifp, IFF_DRV_RUNNING, 0);
	callout_reset(ctx->ifc_timer, hz, iflib_timer, ctx);
	CTX_UNLOCK(ctx);
}

static int
iflib_if_transmit(if_t ifp, struct mbuf *m)
{
	ctx_t	*ctx = if_getsoftc(ifp);
	struct tx_ring	*txr;
	int 		error;

	/*
	* XXX calculate txr and buf_ring based on flowid
	* or other policy flag
	*/
	txr = &ctx->tx_rings[0];
	br = &txr->tx_br[0];
	error = 0;
	if (TX_TRYLOCK(txr)) {
		error = _iflib_txr_transmit(txr, m);
		TX_UNLOCK(txr);
	} else if (m != NULL)
		error = drbr_enqueue(ifp, br, m);

	return (error);
}

static void
iflib_if_qflush(if_t ifp)
{
	ctx_t ctx = if_getsoftc(ifp);
	struct tx_ring  *txr = ctx->tx_rings;
	struct mbuf     *m;

	for (int i = 0; i < adapter->num_queues; i++, txr++) {
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
	int		error = 0;

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
			error = ether_ioctl_drv(ifp, command, data);
		break;
	case SIOCSIFMTU:
		CTX_LOCK(ctx);
		ctx->ifc_set_mtu(ctx, mtu);
		CTX_UNLOCK(ctx);
		if_setmtu(ifp, mtu);
	case SIOCSIFFLAGS:
		CTX_LOCK(ctx);
		if (if_getflags(ifp) & IFF_UP) {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
				if ((if_getflags(ifp) ^ adapter->if_flags) &
				    (IFF_PROMISC | IFF_ALLMULTI)) {
					ctx->ifc_promisc_config(ctx->ifc_sc, if_getflags(ifp));
				}
			} else
				ctx->ifc_init(ctx);
		} else
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING)
				ctx->ifc_stop(ctx);
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
		ctx->ifc_set_media(ctx);
		CTX_UNLOCK(ctx);
		/* falls thru */
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl_drv(ifp, ifr, &adapter->media, command);
		break;
	case SIOCSIFCAP:
	    {
		int mask, reinit;

		IOCTL_DEBUGOUT("ioctl rcv'd: SIOCSIFCAP (Set Capabilities)");
		reinit = 0;
		mask = ifr->ifr_reqcap ^ if_getcapenable(ifp);
		if (mask & IFCAP_POLLING) {
			if (ifr->ifr_reqcap & IFCAP_POLLING) {
				error = ether_poll_register_drv(em_poll, ifp);
				if (error)
					return (error);
				CTX_LOCK(ctx);
				ctx->ifc_disable_intr(ctx);
				if_setcapenablebit(ifp, IFCAP_POLLING, 0);
				CTX_UNLOCK(ctx);
			} else {
				error = ether_poll_deregister_drv(ifp);
				/* Enable interrupt even in error case */
				CTX_LOCK(ctx);
				ctx->ifc_intr_enable(ctx);
				if_setcapenablebit(ifp, 0, IFCAP_POLLING);
				CTX_UNLOCK(ctx);
			}
		}
		if (mask & IFCAP_HWCSUM) {
			if_togglecapenable(ifp,IFCAP_HWCSUM);
			reinit = 1;
		}
		if (mask & IFCAP_TSO4) {
			if_togglecapenable(ifp,IFCAP_TSO4);
			reinit = 1;
		}
		if (mask & IFCAP_VLAN_HWTAGGING) {
			if_togglecapenable(ifp,IFCAP_VLAN_HWTAGGING);
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
		error = ether_ioctl_drv(ifp, command, data);
		break;
	}

	return (error);
}

/*
 * MI independent logic
 *
 */
static void
iflib_timer(void *arg)
{
	iflib_ctx_t ctx = arg;
	iflib_tx_ring_t txr = ctx->ifc_txr;
	CTX_LOCK(ctx);
	
	/*
	** Check on the state of the TX queue(s), this 
	** can be done without the lock because its RO
	** and the HUNG state will be static if set.
	*/
	ctx->ifc_timer(ctx->ifc_sc);
	for (int i = 0; i < ctx->ifc_num_queues; i++, txr++) {
		if ((txr->ift_queue_status == IFLIB_QUEUE_HUNG) &&
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

	ctx->ifc_init(ctx);
unlock:
	CTX_UNLOCK(ctx);
}



/*
 * XXX BLEH
 *
 */
static void
(tx_ring_t *txr)
{
}


static int
_iflib_tx(iflib_tx_ring_t txr, struct mbuf **m_headp)
{
	struct adapter		*adapter = txr->adapter;
	bus_dma_segment_t	segs[EM_MAX_SCATTER];
	bus_dmamap_t		map;
	iflib_buffer_t	tx_buffer, tx_buffer_mapped;
	struct mbuf		*m_head;
	struct ether_header	*eh;
	struct ip		*ip = NULL;
	struct tcphdr		*tp = NULL;
	int			ip_off, poff;
	int			nsegs, i, j, first, last = 0;
	int			error, do_tso, tso_desc = 0, remap = 1;

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
	map = tx_buffer->ifb_map;

retry:
	error = bus_dmamap_load_mbuf_sg(txr->txtag, map,
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
	if (error == EFBIG && remap) {
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
	} else if (error == ENOMEM) {
		adapter->no_tx_dma_setup++;
		return (error);
	} else if (error != 0) {
		adapter->no_tx_dma_setup++;
		m_freem(*m_headp);
		*m_headp = NULL;
		return (error);
	}

	if (nsegs > (txr->tx_avail - 2)) {
		txr->no_desc_avail++;
		bus_dmamap_unload(txr->ift_txtag, map);
		return (ENOBUFS);
	}
	m_head = *m_headp;

	pi->pi_segs = segs;
	pi->pi_nsegs = nsegs;
	if ((err = ctx->ifc_tx(txr->ift_hw_txr, pi)) == 0)
		tx_buffer->next_eop = pi->pi_last;
	return (err);
}

static int
_iflib_txr_transmit(iflib_tx_ring_t txr, struct mbuf *m)
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
		txr->ift_queue_status = IFLIB_QUEUE_WORKING;
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
	iflib_tx_ring_t txr = ctx->ifc_txr;

	if_setbaudrate(ifp, baudrate);
	/* If link down, disable watchdog */
	if ((ctx->ifc_link_state == LINK_STATE_UP) && (link_state == LINK_STATE_DOWN)) {
		for (int i = 0; i < ctx->ifc_num_queues; i++, txr++)
			txr->ift_queue_status = IFLIB_QUEUE_IDLE;
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
	iflib_tx_ring_t txr = arg;

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
iflib_txbuf_free(iflib_ctx_t ctx, struct tx_ring *txr, iflib_buffer_t *tx_buffer)
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
iflib_txbuf_destroy(iflib_ctx_t ctx, struct tx_ring *txr, iflib_buffer_t *tx_buffer)
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

static void
iflib_rxbuf_free(iflib_ctx_t ctx, struct rx_ring *rxr, iflib_buffer_t *rx_buffer)
{
	if (rxbuf->map != NULL) {
		bus_dmamap_sync(rxr->rxtag, rxbuf->map,
						BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(rxr->rxtag, rxbuf->map);
		bus_dmamap_destroy(rxr->rxtag, rxbuf->map);
	}
	if (rxbuf->m_head != NULL) {
		m_freem(rxbuf->m_head);
		rxbuf->m_head = NULL;
	}
}

static void
iflib_vlan_config_register(iflib_ctx_t ctx, register_vlan, unregister_vlan)
{
	ctx->ifc_register_vlan = register_vlan;
	ctx->ifc_unregister_vlan = unregister_vlan;
	
	ctx->vlan_attach_event =
		EVENTHANDLER_REGISTER(vlan_config, iflib_register_vlan, adapter,
							  EVENTHANDLER_PRI_FIRST);
	ctx->vlan_detach_event =
		EVENTHANDLER_REGISTER(vlan_unconfig, iflib_unregister_vlan, adapter,
							  EVENTHANDLER_PRI_FIRST); 
}
