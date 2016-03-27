#include "if_igb.h"

#ifdef	RSS
#include <net/rss_config.h>
#include <netinet/in_rss.h>
#endif

/*********************************************************************
 *  Local Function prototypes
 *********************************************************************/
static int igb_isc_txd_encap(void *arg, if_pkt_info_t pi);
static void igb_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx);
static int igb_isc_txd_credits_update(void *arg, uint16_t txqid, uint32_t cidx);

static void igb_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				   uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count);
static void igb_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx);
static int igb_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx);
static int igb_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri);

static int igb_tso_setup(struct adapter *adapter, struct e1000_adv_tx_context_desc *TXD, if_pkt_info_t pi);
static int igb_tx_ctx_setup(struct adapter *adapter, struct e1000_adv_tx_context_desc *TXD, if_pkt_info_t pi);
static void igb_rx_checksum(u32 staterr, if_rxd_info_t ri, u32 ptype);
static int igb_determine_rsstype(u16 pkt_info);	

extern void igb_if_enable_intr(if_ctx_t ctx);
extern int igb_intr(void *arg);

struct if_txrx igb_txrx  = {
	igb_isc_txd_encap,
	igb_isc_txd_flush,
	igb_isc_txd_credits_update,
	igb_isc_rxd_available,
	igb_isc_rxd_pkt_get,
	igb_isc_rxd_refill,
	igb_isc_rxd_flush,
	igb_intr
};

extern if_shared_ctx_t igb_sctx;

/*********************************************************************
 *
 *  Advanced Context Descriptor setup for VLAN, CSUM or TSO
 *
 **********************************************************************/
static int
igb_tx_ctx_setup(struct adapter *adapter, struct e1000_adv_tx_context_desc *TXD, if_pkt_info_t pi)
{
	struct igb_queue *que = &adapter->queues[pi->ipi_qsidx];
	struct tx_ring *txr = &que->txr; 
	u32 olinfo_status = 0; 
	u32 vlan_macip_lens = 0, type_tucmd_mlhl = 0, mss_l4len_idx = 0;
	u16	vtag = 0;

	/*
	** In advanced descriptors the vlan tag must 
	** be placed into the context descriptor. Hence
	** we need to make one even if not doing offloads.
	*/
	if (pi->ipi_flags & M_VLANTAG) {
		vtag = htole16(pi->ipi_vtag);
		vlan_macip_lens |= (vtag << E1000_ADVTXD_VLAN_SHIFT);
	} 

	/* Set the ether header length */
	TXD->vlan_macip_lens |= pi->ipi_ehdrlen << E1000_ADVTXD_MACLEN_SHIFT;

	/* First check if TSO is to be used */
	if (pi->ipi_csum_flags & CSUM_TSO)
		return (igb_tso_setup(adapter, TXD, pi));
        
	olinfo_status |= pi->ipi_len << E1000_ADVTXD_PAYLEN_SHIFT;
	if (pi->ipi_flags & IPI_TX_IPV4)
		type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_IPV4;
	else if (pi->ipi_flags & IPI_TX_IPV6)	
		type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_IPV6;
	else
		goto no_offloads;

	switch (pi->ipi_ipproto) {
		case IPPROTO_TCP:
			if (pi->ipi_csum_flags & CSUM_TCP)
				type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_TCP;
			break;
		case IPPROTO_UDP:
			if (pi->ipi_csum_flags & CSUM_UDP)
				type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_UDP;
			break;

#if __FreeBSD_version >= 800000
		case IPPROTO_SCTP:
			if (pi->ipi_csum_flags & CSUM_SCTP)
				type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_SCTP;
			break;
#endif
		default:
			goto no_offloads;
			break;
	}

	olinfo_status |= E1000_TXD_POPTS_TXSM << 8;

	/* 82575 needs the queue index added */
	if (adapter->hw.mac.type == e1000_82575)
		mss_l4len_idx = txr->me << 4;

no_offloads:
	TXD->type_tucmd_mlhl |= E1000_ADVTXD_DCMD_DEXT | E1000_ADVTXD_DTYP_CTXT;
	
	/* Now copy bits into descriptor */
	TXD->vlan_macip_lens = htole32(vlan_macip_lens);
	TXD->type_tucmd_mlhl = htole32(type_tucmd_mlhl);
	TXD->seqnum_seed = htole32(0);
	TXD->mss_l4len_idx = htole32(mss_l4len_idx);
	
	return (olinfo_status);
}

/**********************************************************************
 *
 *  Setup work for hardware segmentation offload (TSO) on
 *  adapters using advanced tx descriptors
 *
 **********************************************************************/
static int
igb_tso_setup(struct adapter *adapter, struct e1000_adv_tx_context_desc *TXD, if_pkt_info_t pi)
{
	struct igb_queue *que = &adapter->queues[pi->ipi_qsidx];
	struct tx_ring *txr = &que->txr; 
	u32 vlan_macip_lens, type_tucmd_mlhl;
	u32 mss_l4len_idx, paylen;
        u32 olinfo_status; 

	mss_l4len_idx = vlan_macip_lens = type_tucmd_mlhl = olinfo_status = 0; 
	if (pi->ipi_flags & IPI_TX_IPV4)
		type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_IPV4;
	else if (pi->ipi_flags & IPI_TX_IPV6)
		type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_IPV6;
	else
		panic("CSUM_TSO but no supported IP version");

	/* This is used in the transmit desc in encap */
	paylen = pi->ipi_len - pi->ipi_ehdrlen - pi->ipi_ip_hlen - pi->ipi_tcp_hlen;

	/* ADV DTYPE TUCMD */
	type_tucmd_mlhl |= E1000_ADVTXD_DCMD_DEXT | E1000_ADVTXD_DTYP_CTXT;
	type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_TCP;
	TXD->type_tucmd_mlhl = htole32(type_tucmd_mlhl);

	/* MSS L4LEN IDX */
	mss_l4len_idx |= (pi->ipi_tso_segsz << E1000_ADVTXD_MSS_SHIFT);
	mss_l4len_idx |= (pi->ipi_tcp_hlen << E1000_ADVTXD_L4LEN_SHIFT);

	/* 82575 needs the queue index added */
	if (adapter->hw.mac.type == e1000_82575)
		mss_l4len_idx |= txr->me << 4;
	TXD->mss_l4len_idx = htole32(mss_l4len_idx);

	TXD->seqnum_seed = htole32(0);

        olinfo_status |= E1000_TXD_POPTS_IXSM << 8;
	olinfo_status |= paylen << E1000_ADVTXD_PAYLEN_SHIFT;
	return (olinfo_status);
}

static int
igb_isc_txd_encap(void *arg, if_pkt_info_t pi)
{
	struct adapter *sc     = arg;
	struct igb_queue *que  = &sc->queues[pi->ipi_qsidx];
	struct tx_ring *txr    = &que->txr;
	struct igb_tx_buf *buf;
	int nsegs              = pi->ipi_nsegs;
	bus_dma_segment_t *segs = pi->ipi_segs;
	union e1000_adv_tx_desc *txd = NULL;
	struct e1000_adv_tx_context_desc *TXD; 

	int                    i, j, first;
	u32                    olinfo_status = 0, cmd_type_len;

	/* Basic descriptor defines */
	cmd_type_len = (E1000_ADVTXD_DTYP_DATA |
					E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT);
	
	if (pi->ipi_mflags & M_VLANTAG)
		cmd_type_len |= E1000_ADVTXD_DCMD_VLE;

	i = first = pi->ipi_pidx;

        /* Indicate the whole packet as payload when not doing TSO */
	TXD = (struct e1000_adv_tx_context_desc *) &txr->tx_base[first]; 
	olinfo_status |= pi->ipi_len << E1000_ADVTXD_PAYLEN_SHIFT;
	
	if(pi->ipi_csum_flags & CSUM_OFFLOAD) {
		/*********************************************
		 * Set up the appropriate offload context
		 * this will consume the first descriptor
		 *********************************************/	
		olinfo_status = igb_tx_ctx_setup(sc, TXD, pi);
		if (pi->ipi_csum_flags & CSUM_TSO) {
			cmd_type_len |= E1000_ADVTXD_DCMD_TSE;
			++txr->tso_tx; 
		}
		++i; 
	}

	/* 82575 needs the queue index added */
	if (sc->hw.mac.type == e1000_82575)
		olinfo_status |= txr->me << 4;
    
	for (j = 0; j < nsegs; j++) {
		bus_size_t seglen;
		bus_addr_t segaddr;

		buf = &txr->tx_buffers[i];
		txd = &txr->tx_base[i];
		seglen = segs[j].ds_len;
		segaddr = htole64(segs[j].ds_addr);

		txd->read.buffer_addr = segaddr;
		txd->read.cmd_type_len = htole32(E1000_TXD_CMD_IFCS |
		    cmd_type_len | seglen);
		txd->read.olinfo_status = htole32(olinfo_status);

		if (++i == igb_sctx->isc_ntxd)
			i = 0;
	}
	
	txd->read.cmd_type_len |=
	    htole32(E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);
		
	/* Set the EOP descriptor that will be marked done */
	buf = &txr->tx_buffers[first];
	buf->eop = txd;
	
	txr->bytes += pi->ipi_len;
	pi->ipi_new_pidx = i;

	++txr->total_packets;
  
    return (0);
}

static void
igb_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx)
{
	struct adapter *adapter = arg;
  struct igb_queue *que     = &adapter->queues[txqid];
  struct tx_ring *txr      = &que->txr;
  
  E1000_WRITE_REG(&adapter->hw, E1000_TDT(txr->me), pidx);
}


static int
igb_isc_txd_credits_update(void *arg, uint16_t txqid, uint32_t cidx_init)
{
	struct adapter *adapter = arg;
	struct igb_queue *que = &adapter->queues[txqid];
	struct tx_ring *txr = &que->txr;

	u32       cidx, processed = 0;
	u32       limit = adapter->tx_process_limit; 

	struct igb_tx_buf *buf;
	union e1000_adv_tx_desc *txd;

	cidx = cidx_init;
	buf = &txr->tx_buffers[cidx];
	txd = &txr->tx_base[cidx];

	do {
		union e1000_adv_tx_desc *eop = buf->eop;
		if (eop == NULL) /* No work */
			break;
		
		if ((eop->wb.status & E1000_TXD_STAT_DD) == 0)
			break;	/* I/O not complete */
		
		buf->eop = NULL; /* clear indicate processed */
		
			/* We clean the range if multi segment */
		while (txd != eop) {
			++txd;
			++buf;
            cidx++;
			processed++;
			/* wrap the ring? */
           	/* wrap the ring? */
			if (cidx == igb_sctx->isc_ntxd) {
				buf = txr->tx_buffers;
				txd = txr->tx_base;
				cidx = 0;
			}
			buf->eop = NULL;
		}
		++txr->packets;
		++processed;
    
		/* Try the next packet */
		txd++;
		buf++;
		cidx++;
		/* reset with a wrap */
		if (__predict_false(cidx == igb_sctx->isc_ntxd)) {
			cidx = 0;
			buf = txr->tx_buffers;
			txd = txr->tx_base;
		}
		prefetch(txd);
	} while (__predict_true(--limit) && cidx != cidx_init);
	
	return (processed);
}

static void igb_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				 uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count)
{
	struct adapter *sc       = arg;
	struct igb_queue *que     = &sc->queues[rxqid];
	struct rx_ring *rxr      = &que->rxr;
	int			i;
	uint32_t next_pidx;

	for (i = 0, next_pidx = pidx; i < count; i++) {
		rxr->rx_base[next_pidx].read.pkt_addr = htole64(paddrs[i]);
		if (++next_pidx == igb_sctx->isc_nrxd)
			next_pidx = 0;
	}
}

static void igb_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx)
{
	struct adapter *sc       = arg;
	struct igb_queue *que     = &sc->queues[rxqid];
	struct rx_ring *rxr      = &que->rxr;

    E1000_WRITE_REG(&sc->hw, rxr->tail, pidx);
}

static int igb_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx)
{
	struct adapter *sc       = arg;
	struct igb_queue *que     = &sc->queues[rxqid];
	struct rx_ring *rxr      = &que->rxr;
	union e1000_adv_rx_desc *rxd;
	u16                      pkt_info;
	u32                      staterr = 0;
	int                      cnt, i;
  
	for (cnt = 0, i = idx; cnt < igb_sctx->isc_nrxd;) {
		rxd = &rxr->rx_base[i];
		staterr = le32toh(rxd->wb.upper.status_error);
		pkt_info = le16toh(rxd->wb.lower.lo_dword.hs_rss.pkt_info);

		if ((staterr & E1000_RXD_STAT_DD) == 0)
			break;

		cnt++;
		if (++i == igb_sctx->isc_nrxd) {
			i = 0;
		}
	}

	return (cnt);
}

/****************************************************************
 * Routine sends data which has been dma'ed into host memory
 * to upper layer. Initialize ri structure. 
 *
 * Returns 0 upon success, errno on failure
 ***************************************************************/

static int
igb_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{
	struct adapter           *adapter = arg;
	struct igb_queue          *que = &adapter->queues[ri->iri_qsidx];
	struct rx_ring           *rxr = &que->rxr;
	struct ifnet             *ifp = iflib_get_ifp(adapter->ctx);
	union e1000_adv_rx_desc  *rxd;

	u16                      pkt_info, len;
	u16                      vtag = 0;
	u32                      ptype;
	u32                      staterr = 0;
	bool                     eop;
  
	rxd = &rxr->rx_base[ri->iri_cidx];
	staterr = le32toh(rxd->wb.upper.status_error);
	pkt_info = le16toh(rxd->wb.lower.lo_dword.hs_rss.pkt_info);

	/* Error Checking then decrement count */
	MPASS ((staterr & E1000_RXD_STAT_DD) != 0);

	len = le16toh(rxd->wb.upper.length);
	ptype = le32toh(rxd->wb.lower.lo_dword.data) &  IGB_PKTTYPE_MASK;

	ri->iri_len = len;

	rxd->wb.upper.status_error = 0;
	eop = ((staterr & E1000_RXD_STAT_EOP) == E1000_RXD_STAT_EOP);

	if (((adapter->hw.mac.type == e1000_i350) ||
		 (adapter->hw.mac.type == e1000_i354)) &&
		(staterr & E1000_RXDEXT_STATERR_LB))
		vtag = be16toh(rxd->wb.upper.vlan);
	else
		vtag = le16toh(rxd->wb.upper.vlan);

	/* Make sure bad packets are discarded */
	if (eop && ((staterr & E1000_RXDEXT_ERR_FRAME_ERR_MASK) != 0)) {
		adapter->dropped_pkts++;
		++rxr->rx_discarded;
		return (EBADMSG);
	}
	
	/* Prefetch the next buffer */
	if (!eop) {
		ri->iri_next_offset = 1; 
	} else {
        rxr->packets++;
		rxr->rx_packets++;
        rxr->bytes += ri->iri_len;
		rxr->rx_bytes += ri->iri_len; 
		
		if ((ifp->if_capenable & IFCAP_RXCSUM) != 0)
			igb_rx_checksum(staterr, ri, ptype);

		if ((ifp->if_capenable & IFCAP_VLAN_HWTAGGING) != 0 &&
			(staterr & E1000_RXD_STAT_VP) != 0) {
			ri->iri_vtag = vtag;
			ri->iri_flags |= M_VLANTAG;
		}

		ri->iri_flowid =
			le32toh(rxd->wb.lower.hi_dword.rss);

		ri->iri_rsstype = igb_determine_rsstype(pkt_info);

		ri->iri_next_offset = 0;
	}
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
igb_rx_checksum(u32 staterr, if_rxd_info_t ri, u32 ptype)
{
	u16 status = (u16)staterr;
	u8  errors = (u8) (staterr >> 24);
	bool sctp = FALSE; 

	/* Ignore Checksum bit is set */
	if (status & E1000_RXD_STAT_IXSM) {
		ri->iri_csum_flags = 0;
		return;
	}

	if ((ptype & E1000_RXDADV_PKTTYPE_ETQF) == 0 &&
	    (ptype & E1000_RXDADV_PKTTYPE_SCTP) != 0)
		sctp = 1;
	else
		sctp = 0;

	if (status & E1000_RXD_STAT_IPCS) {
		/* Did it pass? */
		if (!(errors & E1000_RXD_ERR_IPE)) {
			/* IP Checksum Good */
			ri->iri_csum_flags = CSUM_IP_CHECKED | CSUM_IP_VALID;
		} else
			ri->iri_csum_flags = 0;
	}

	if (status & (E1000_RXD_STAT_TCPCS | E1000_RXD_STAT_UDPCS)) {
		u64 type = (CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
#if __FreeBSD_version >= 800000
		if (sctp) /* reassign */
			type = CSUM_SCTP_VALID;
#endif
		/* Did it pass? */
		if (!(errors & E1000_RXD_ERR_TCPE)) {
			ri->iri_csum_flags |= type;
			if (sctp == 0)
				ri->iri_csum_data = htons(0xffff);
		}
	}
	return;
}

/********************************************************************
 *
 *  Parse the packet type to determine the appropriate hash
 *
 ******************************************************************/
static int 
igb_determine_rsstype(u16 pkt_info)	
{
   	switch (pkt_info & E1000_RXDADV_RSSTYPE_MASK) {
	case E1000_RXDADV_RSSTYPE_IPV4_TCP:
		return M_HASHTYPE_RSS_TCP_IPV4;
	case E1000_RXDADV_RSSTYPE_IPV4:
		return M_HASHTYPE_RSS_IPV4;
	case E1000_RXDADV_RSSTYPE_IPV6_TCP:
		return M_HASHTYPE_RSS_TCP_IPV6;
	case E1000_RXDADV_RSSTYPE_IPV6_EX:
		return M_HASHTYPE_RSS_IPV6_EX;
	case E1000_RXDADV_RSSTYPE_IPV6:
		return M_HASHTYPE_RSS_IPV6;
	case E1000_RXDADV_RSSTYPE_IPV6_TCP_EX:
		return M_HASHTYPE_RSS_TCP_IPV6_EX;
	default:
		return M_HASHTYPE_OPAQUE;
	}
}
