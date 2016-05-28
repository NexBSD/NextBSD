#include "if_em.h"

#ifdef	RSS
#include <net/rss_config.h>
#include <netinet/in_rss.h>
#endif

/*********************************************************************
 *  Local Function prototypes
 *********************************************************************/
static void em_tso_setup(struct adapter *adapter, if_pkt_info_t pi, u32 *txd_upper, u32 *txd_lower);
static void em_transmit_checksum_setup(struct adapter *adapter, if_pkt_info_t pi, u32 *txd_upper, u32 *txd_lower);
static int em_isc_txd_encap(void *arg, if_pkt_info_t pi);
static void em_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx);
static int em_isc_txd_credits_update(void *arg, uint16_t txqid, uint32_t cidx_init, bool clear);

static void em_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				   uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count);
static void em_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx);
static int em_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx);
static int em_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri);
static void em_receive_checksum(uint32_t status, if_rxd_info_t ri);

extern int em_intr(void *arg);

struct if_txrx em_txrx  = {
	em_isc_txd_encap,
	em_isc_txd_flush,
	em_isc_txd_credits_update,
	em_isc_rxd_available,
	em_isc_rxd_pkt_get,
	em_isc_rxd_refill,
	em_isc_rxd_flush,
	em_intr
};

extern if_shared_ctx_t em_sctx; 

/**********************************************************************
 *
 *  Setup work for hardware segmentation offload (TSO) on
 *  adapters using advanced tx descriptors
 *
 **********************************************************************/
static void
em_tso_setup(struct adapter *adapter, if_pkt_info_t pi, u32 *txd_upper, u32 *txd_lower)
{
        struct em_tx_queue *que = &adapter->tx_queues[pi->ipi_qsidx];
        struct tx_ring *txr = &que->txr;
	struct e1000_context_desc *TXD; 
        int cur, hdr_len;

	hdr_len = pi->ipi_ehdrlen + pi->ipi_ip_hlen + pi->ipi_tcp_hlen;
  	*txd_lower = (E1000_TXD_CMD_DEXT |	/* Extended descr type */
		      E1000_TXD_DTYP_D |	/* Data descr type */
		      E1000_TXD_CMD_TSE);	/* Do TSE on this packet */

	/* IP and/or TCP header checksum calculation and insertion. */
	*txd_upper = (E1000_TXD_POPTS_IXSM | E1000_TXD_POPTS_TXSM) << 8;

	cur = pi->ipi_pidx;
        TXD = (struct e1000_context_desc *)&txr->tx_base[cur]; 
	
	 /*
	 * Start offset for header checksum calculation.
	 * End offset for header checksum calculation.
	 * Offset of place put the checksum.
	 */
	 TXD->lower_setup.ip_fields.ipcss = pi->ipi_ehdrlen; 
	TXD->lower_setup.ip_fields.ipcse =
	    htole16(pi->ipi_ehdrlen + pi->ipi_ip_hlen - 1);
	TXD->lower_setup.ip_fields.ipcso = pi->ipi_ehdrlen + offsetof(struct ip, ip_sum);

         /*
	 * Start offset for payload checksum calculation.
	 * End offset for payload checksum calculation.
	 * Offset of place to put the checksum.
	 */
	TXD->upper_setup.tcp_fields.tucss = pi->ipi_ehdrlen + pi->ipi_ip_hlen;
	TXD->upper_setup.tcp_fields.tucse = 0;
	TXD->upper_setup.tcp_fields.tucso =
	    pi->ipi_ehdrlen + pi->ipi_ip_hlen + offsetof(struct tcphdr, th_sum);
	
         /*
	 * Payload size per packet w/o any headers.
	 * Length of all headers up to payload.
	 */
	TXD->tcp_seg_setup.fields.mss = htole16(pi->ipi_tso_segsz);
	TXD->tcp_seg_setup.fields.hdr_len = hdr_len;

	TXD->cmd_and_length = htole32(adapter->txd_cmd |
				E1000_TXD_CMD_DEXT |	/* Extended descr */
				E1000_TXD_CMD_TSE |	/* TSE context */
				E1000_TXD_CMD_IP |	/* Do IP csum */
				E1000_TXD_CMD_TCP |	/* Do TCP checksum */
				      (pi->ipi_len - hdr_len)); /* Total len */
	txr->tx_tso = TRUE; 
}


static void
em_transmit_checksum_setup(struct adapter *adapter, if_pkt_info_t pi, u32 *txd_upper, u32 *txd_lower)
{
        struct e1000_context_desc   *TXD = NULL;
 	struct em_tx_queue          *que = &adapter->tx_queues[pi->ipi_qsidx];
	struct tx_ring              *txr = &que->txr; 
	int                         csum_flags = pi->ipi_csum_flags;
	int                         ip_off = pi->ipi_ehdrlen; 
	int                         cur, hdr_len;
	u32                         cmd = 0; 
	u16			    offload = 0;
	u8                          ipcso, ipcss, tucss, tucso;

	ipcso = ipcss = tucss = tucso = 0; 
	cur = pi->ipi_pidx;
	hdr_len = ip_off + pi->ipi_ip_hlen;
	
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
		 * The 82574L can only remember the *last* context used
		 * regardless of queue that it was use for.  We cannot reuse
		 * contexts on this hardware platform and must generate a new
		 * context every time.  82574L hardware spec, section 7.2.6,
		 * second note.
		 */
		if (adapter->num_tx_queues < 2) {
 			/*
 		 	* Setting up new checksum offload context for every
			* frames takes a lot of processing time for hardware.
			* This also reduces performance a lot for small sized
			* frames so avoid it if driver can use previously
			* configured checksum offload context.
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
		}
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
		 * The 82574L can only remember the *last* context used
		 * regardless of queue that it was use for.  We cannot reuse
		 * contexts on this hardware platform and must generate a new
		 * context every time.  82574L hardware spec, section 7.2.6,
		 * second note.
		 */
		if (adapter->num_tx_queues < 2) {
 			/*
 		 	* Setting up new checksum offload context for every
			* frames takes a lot of processing time for hardware.
			* This also reduces performance a lot for small sized
			* frames so avoid it if driver can use previously
			* configured checksum offload context.
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
		}
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
}

static int
em_isc_txd_encap(void *arg, if_pkt_info_t pi)
{
        struct adapter *sc       = arg;
	struct em_tx_queue *que  = &sc->tx_queues[pi->ipi_qsidx];
	struct tx_ring *txr      = &que->txr;
	bus_dma_segment_t *segs  = pi->ipi_segs;
	int nsegs                = pi->ipi_nsegs;
	int csum_flags           = pi->ipi_csum_flags;
        int i, j, first, cidx_last; 
	u32                     txd_upper = 0, txd_lower = 0; 
	
	struct em_txbuffer *tx_buffer;
	struct e1000_tx_desc *ctxd = NULL;
	bool do_tso, tso_desc; 
	
	i = first = pi->ipi_pidx;         
	do_tso = (csum_flags & CSUM_TSO);
	tso_desc = FALSE;

        /*
	 * TSO Hardware workaround, if this packet is not
	 * TSO, and is only a single descriptor long, and
	 * it follows a TSO burst, then we need to add a
	 * sentinel descriptor to prevent premature writeback.
	 */
	if ((!do_tso) && (txr->tx_tso == TRUE)) {
		if (nsegs == 1)
			tso_desc = TRUE;
		txr->tx_tso = FALSE;
	}

	/* Do hardware assists */
	if (do_tso) {
	  em_tso_setup(sc, pi, &txd_upper, &txd_lower);
	  tso_desc = TRUE; 
	} else if (csum_flags & CSUM_OFFLOAD) {
          em_transmit_checksum_setup(sc, pi, &txd_upper, &txd_lower);
	}

	if (do_tso || (csum_flags & CSUM_OFFLOAD)) {
		i++; 
	}

	if (pi->ipi_mflags & M_VLANTAG) {
	  /* Set the vlan id. */
		txd_upper |= htole16(pi->ipi_vtag) << 16;
                /* Tell hardware to add tag */
                txd_lower |= htole32(E1000_TXD_CMD_VLE);
	}

	/* Set up our transmit descriptors */
	for (j = 0; j < nsegs; j++) {
	  bus_size_t seg_len;
	  bus_addr_t seg_addr;
	  
	  ctxd = &txr->tx_base[i];
	  tx_buffer = &txr->tx_buffers[i];
	  seg_addr = segs[j].ds_addr;
	  seg_len = segs[j].ds_len;

	  /*
	  ** TSO Workaround:
	  ** If this is the last descriptor, we want to
	  ** split it so we have a small final sentinel
	  */
	  if (tso_desc && (j == (nsegs - 1)) && (seg_len > 8)) {
		  /*  seg_len -= TSO_WORKAROUND;  */
	    ctxd->buffer_addr = htole64(seg_addr);
	    ctxd->lower.data = htole32(sc->txd_cmd | txd_lower | seg_len);
	    ctxd->upper.data = htole32(txd_upper);

	    /* Now make the sentinel */	
	    ctxd = &txr->tx_base[i];
	    tx_buffer = &txr->tx_buffers[i];
	    ctxd->buffer_addr = htole64(seg_addr + seg_len);
	    /*    ctxd->lower.data = htole32(adapter->txd_cmd | txd_lower | TSO_WORKAROUND); */
	    ctxd->upper.data = htole32(txd_upper);
	  } else {
	    ctxd->buffer_addr = htole64(seg_addr);
	    ctxd->lower.data = htole32(sc->txd_cmd | txd_lower | seg_len);
	    ctxd->upper.data = htole32(txd_upper);
	  }
          cidx_last = i;
	   if (++i == em_sctx->isc_ntxd)
	      i = 0;
	  
	  tx_buffer = &txr->tx_buffers[first];
	  tx_buffer->eop = cidx_last;
	  pi->ipi_new_pidx = i; 
	}
	return (0); 
}

static void
em_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx)
{
  struct adapter *adapter = arg;
  struct em_tx_queue *que = &adapter->tx_queues[txqid];
  struct tx_ring *txr = &que->txr;

  E1000_WRITE_REG(&adapter->hw, E1000_TDT(txr->me), pidx);
}

static int
em_isc_txd_credits_update(void *arg, uint16_t txqid, uint32_t cidx_init, bool clear)
{
  struct adapter *adapter = arg;
  struct em_tx_queue *que = &adapter->tx_queues[txqid];
  struct tx_ring *txr = &que->txr;

  u32 cidx, processed = 0;
  u32 limit = adapter->tx_process_limit; 
  struct em_txbuffer *buf;
  struct e1000_tx_desc *tx_desc;

  cidx = cidx_init;
  buf = &txr->tx_buffers[cidx];
  tx_desc = &txr->tx_base[cidx];

  do {
	  int eop = buf->eop;
	  struct e1000_tx_desc *eop_desc;
	  
	  if (eop == -1) /* No work */
		  break;

	  eop_desc = &txr->tx_base[eop]; 
	  if ((eop_desc->upper.fields.status & E1000_TXD_STAT_DD) == 0)
		  break;  /* I/O not complete */
	  
	  if (clear)
		  eop = -1;  /* clear indicates processed */ 
	  
	  /* We clean the range of the packet */
	  while (tx_desc != eop_desc) {
		  tx_desc->upper.data = 0;
		  tx_desc->lower.data = 0;
		  tx_desc->buffer_addr = 0;
		  tx_desc++;
		  buf++;
		  cidx++;
		  processed++;
		  
		  /* wrap the ring ? */
		  if (cidx == em_sctx->isc_ntxd) {
			  buf = txr->tx_buffers;
			  tx_desc = txr->tx_base;
			  cidx = 0;
		  }
		  prefetch(tx_desc);
		  prefetch(tx_desc+1); 
	  }
  } while (__predict_true(--limit) && cidx != cidx_init);
  
  return(processed); 
}

static void
em_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				   uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count)
{
  struct adapter *sc = arg;
  struct em_rx_queue *que = &sc->rx_queues[rxqid];
  struct rx_ring *rxr = &que->rxr;
  union e1000_rx_desc_extended *rxd; 
  int i;
  uint32_t next_pidx;

  for (i = 0, next_pidx = pidx; i < count; i++) {
        rxd = &rxr->rx_base[i];
        rxd->read.buffer_addr = htole64(paddrs[i]); 
        rxd->wb.upper.status_error = 0; 
	
	if (++next_pidx == em_sctx->isc_nrxd)
	  next_pidx = 0;
  }  
}

static void
em_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx)
{
    	struct adapter *sc       = arg;
	struct em_rx_queue *que     = &sc->rx_queues[rxqid];
	struct rx_ring *rxr      = &que->rxr;

        E1000_WRITE_REG(&sc->hw, E1000_RDT(rxr->me), pidx);
}

static int
em_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx)
{
       	struct adapter *sc         = arg;
	struct em_rx_queue *que   = &sc->rx_queues[rxqid];
	struct rx_ring *rxr        = &que->rxr;
	union e1000_rx_desc_extended *rxd;
	u32                      staterr = 0;
	int                      cnt, i;
  
	for (cnt = 0, i = idx; cnt < em_sctx->isc_nrxd;) {
		rxd = &rxr->rx_base[i];
		staterr = le32toh(rxd->wb.upper.status_error);

		if ((staterr & E1000_RXD_STAT_DD) == 0)
			break;
		
		if (++i == em_sctx->isc_nrxd) {
			i = 0;
		}

		if (staterr & E1000_RXD_STAT_EOP)
			cnt++; 
	}

	return (cnt);
}

static int
em_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{
      	struct adapter           *adapter = arg;
	struct em_rx_queue       *que = &adapter->rx_queues[ri->iri_qsidx];
	struct rx_ring           *rxr = &que->rxr;
	union e1000_rx_desc_extended *rxd;

	u16                      len; 
	u32                      staterr = 0;
	bool                     eop;
	int                      i, cidx, vtag;

	vtag = 0; 
	i = 0;
	cidx = ri->iri_cidx;

	do {
		rxd = &rxr->rx_base[cidx];
		staterr = le32toh(rxd->wb.upper.status_error);	
	
		/* Error Checking then decrement count */
		MPASS ((staterr & E1000_RXD_STAT_DD) != 0);

		len = le16toh(rxd->wb.upper.length);
		ri->iri_len = len; 

		eop = (staterr & E1000_RXD_STAT_EOP) != 0;

		/* Make sure bad packets are discarded */
		if (staterr & E1000_RXDEXT_ERR_FRAME_ERR_MASK) {
			adapter->dropped_pkts++;
			return (EBADMSG); 
		}

		ri->iri_frags[i].irf_flid = 0;
		ri->iri_frags[i].irf_idx = cidx;
		if (++cidx == em_sctx->isc_nrxd)
			cidx = 0;
		i++;
	} while (!eop);

	em_receive_checksum(staterr, ri);
	
	if (staterr & E1000_RXD_STAT_VP) {
		vtag = le16toh(rxd->wb.upper.vlan);
	}

	/* Zero out the receive descriptors status. */
	rxd->wb.upper.status_error &= htole32(~0xFF);
	
	ri->iri_vtag = vtag;
	ri->iri_nfrags = i;
	if (vtag)
		ri->iri_flags |= M_VLANTAG;
		
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
em_receive_checksum(uint32_t status, if_rxd_info_t ri)
{
	ri->iri_csum_flags = 0;

	/* Ignore Checksum bit is set */
	if (status & E1000_RXD_STAT_IXSM)
		return;

	/* If the IP checksum exists and there is no IP Checksum error */
	if ((status & (E1000_RXD_STAT_IPCS | E1000_RXDEXT_STATERR_IPE)) ==
		E1000_RXD_STAT_IPCS) {
		ri->iri_csum_flags = (CSUM_IP_CHECKED | CSUM_IP_VALID);
	}

	/* TCP or UDP checksum */
	if ((status & (E1000_RXD_STAT_TCPCS | E1000_RXDEXT_STATERR_TCPE)) ==
	    E1000_RXD_STAT_TCPCS) {
		ri->iri_csum_flags |= (CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
		ri->iri_csum_data = htons(0xffff);
	}
	if (status & E1000_RXD_STAT_UDPCS) {
		ri->iri_csum_flags |= (CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
		ri->iri_csum_data = htons(0xffff);
	}
}
