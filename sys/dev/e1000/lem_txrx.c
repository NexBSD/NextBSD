#include "if_lem.h"

/*********************************************************************
 *  Local Function prototypes
 *********************************************************************/

int lem_intr(void *arg);
static int	lem_82547_fifo_workaround(struct adapter *, int);
static void	lem_82547_update_fifo_head(struct adapter *, int);
static int	lem_82547_tx_fifo_reset(struct adapter *);
static void	lem_82547_move_tail(void *arg, int i);

extern void lem_if_enable_intr(if_ctx_t ctx);
extern void lem_update_link_status(struct adapter *adapter);

static int lem_isc_txd_encap(void *arg, if_pkt_info_t pi);
static void lem_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx);
static int lem_isc_txd_credits_update(void *arg, uint16_t qid, uint32_t cidx);
static int lem_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx);
static int lem_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri);
static void lem_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
							   uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count);
static void lem_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx);

static int lem_tx_checksum_setup(struct adapter *adapter, struct mbuf *m_head, int i, u32 *txd_upper, u32 *txd_lower);
static void lem_receive_checksum(struct adapter *adapter, struct e1000_rx_desc *rx_desc, if_rxd_info_t ri);

struct if_txrx lem_txrx = {
    lem_isc_txd_encap,
	lem_isc_txd_flush,
	lem_isc_txd_credits_update,
	lem_isc_rxd_available,
	lem_isc_rxd_pkt_get,
	lem_isc_rxd_refill,
	lem_isc_rxd_flush, 
    lem_intr
};

extern if_shared_ctx_t lem_sctx;

/*********************************************************************
 *
 *  Legacy Interrupt Service routine
 *
 **********************************************************************/
int 
lem_intr(void *arg)
{
   	struct adapter *adapter = arg;
	if_t ifp = iflib_get_ifp(adapter->ctx); 
    u32 reg_icr;

		if ((if_getcapenable(ifp) & IFCAP_POLLING) ||
	    ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0))
			return (FILTER_HANDLED);

	reg_icr = E1000_READ_REG(&adapter->hw, E1000_ICR);
	if (reg_icr & E1000_ICR_RXO)
		adapter->rx_overruns++;

	if ((reg_icr == 0xffffffff) || (reg_icr == 0)) {
		return (FILTER_HANDLED);
	}

	if (reg_icr & (E1000_ICR_RXSEQ | E1000_ICR_LSC)) {
		adapter->hw.mac.get_link_status = 1;
		lem_update_link_status(adapter);
		return (FILTER_HANDLED);
	}
	
	return (FILTER_SCHEDULE_THREAD); 
}

static int
lem_isc_txd_encap(void *arg, if_pkt_info_t pi)
{
	struct adapter *adapter     = arg;
	struct mbuf *m_head         = pi->ipi_m;
	bus_dma_segment_t *segs     = pi->ipi_segs;
	int nsegs                   = pi->ipi_nsegs;
    int first                   = pi->ipi_pidx; 
    int prev_pidx, error, i, j, last = 0;;   
    u32   txd_upper, txd_lower;
	
	struct em_buffer *tx_buffer;
	struct e1000_tx_desc *ctxd   = NULL;

	txd_upper = txd_lower = 0;
    i = first; 
	
    /*
	** When doing checksum offload, it is critical to
	** make sure the first mbuf has more than header,
	** because that routine expects data to be present.
	*/
	if ((m_head->m_pkthdr.csum_flags & CSUM_OFFLOAD) &&
	    (m_head->m_len < ETHER_HDR_LEN + sizeof(struct ip))) {
		m_head = m_pullup(m_head, ETHER_HDR_LEN + sizeof(struct ip));
		if (m_head == NULL)
			return (ENOBUFS);
	}

    /* Do hardware assists */
	if (m_head->m_pkthdr.csum_flags & CSUM_OFFLOAD) {
		error = lem_tx_checksum_setup(adapter, m_head, first, &txd_lower, &txd_upper);
		return error;
	}
	
	/* Set up our transmit descriptors */
	for (j = 0; j < nsegs; j++) {
		bus_size_t seg_len;
		bus_addr_t seg_addr;

		tx_buffer = &adapter->tx_buffer_area[i];
	    ctxd = &adapter->tx_desc_base[i];
		seg_addr = segs[j].ds_addr;
		seg_len  = segs[j].ds_len;
		ctxd->buffer_addr = htole64(seg_addr);
		ctxd->lower.data = htole32(
			adapter->txd_cmd | txd_lower | seg_len);
		ctxd->upper.data =
			htole32(txd_upper);
		last = i; 
		if (++i == lem_sctx->isc_ntxd)
			i = 0;
		tx_buffer->next_eop = -1; 
	}
	
	if (m_head->m_flags & M_VLANTAG) {
		/* Set the vlan id. */
		ctxd->upper.fields.special =
		    htole16(m_head->m_pkthdr.ether_vtag);
		/* Tell hardware to add tag */
		ctxd->lower.data |= htole32(E1000_TXD_CMD_VLE);
	}
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
	tx_buffer = &adapter->tx_buffer_area[first];
	tx_buffer->next_eop = last;
	pi->ipi_new_pidx = i;
    prev_pidx = i - 1; 
	
    #ifdef NIC_SEND_COMBINING
	if (adapter->sc_enable) {
		if (adapter->shadow_tdt & MIT_PENDING_INT) {
			/* signal intr and data pending */
			adapter->shadow_tdt = MIT_PENDING_TDT | (i & 0xffff);
			return (0);
		} else {
			adapter->shadow_tdt = MIT_PENDING_INT;
		}
	}
#endif /* NIC_SEND_COMBINING */

	
	if (adapter->hw.mac.type == e1000_82547 &&
	    adapter->link_duplex == HALF_DUPLEX)
		lem_82547_move_tail(adapter, prev_pidx);
	else {
		if (adapter->hw.mac.type == e1000_82547)
			lem_82547_update_fifo_head(adapter,
									   m_head->m_pkthdr.len);
	}
	
	return (0);
}

/*********************************************************************
 *
 *  Advanced Context Descriptor setup for VLAN, CSUM 
 *
 **********************************************************************/
static int
lem_tx_checksum_setup(struct adapter *adapter, struct mbuf *m_head, int i, u32 *txd_upper, u32 *txd_lower)
{
	struct e1000_context_desc *TXD = NULL;
	struct ether_vlan_header *eh;
	struct ip *ip = NULL;
	struct ip6_hdr *ip6;
	int curr_txd, ehdrlen;
	u32 cmd, hdr_len, ip_hlen;
	u16 etype;
	u8 ipproto;

    cmd = hdr_len = ipproto = 0; 	
    txd_upper = 0;
	txd_lower = 0; 
	curr_txd = i; 

	/*
	 * Determine where frame payload starts.
	 * Jump over vlan headers if already present,
	 * helpful for QinQ too.
	 */
	eh = mtod(m_head, struct ether_vlan_header *);
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN)) {
		etype = ntohs(eh->evl_proto);
		ehdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
	} else {
		etype = ntohs(eh->evl_encap_proto);
		ehdrlen = ETHER_HDR_LEN;
	}

	/*
	 * We only support TCP/UDP for IPv4 and IPv6 for the moment.
	 * TODO: Support SCTP too when it hits the tree.
	 */
	switch (etype) {
	case ETHERTYPE_IP:
		ip = (struct ip *)(m_head->m_data + ehdrlen);
		ip_hlen = ip->ip_hl << 2;

		/* Setup of IP header checksum. */
		if (m_head->m_pkthdr.csum_flags & CSUM_IP) {
			/*
			 * Start offset for header checksum calculation.
			 * End offset for header checksum calculation.
			 * Offset of place to put the checksum.
			 */
			TXD = (struct e1000_context_desc *)
			    &adapter->tx_desc_base[curr_txd];
			TXD->lower_setup.ip_fields.ipcss = ehdrlen;
			TXD->lower_setup.ip_fields.ipcse =
			    htole16(ehdrlen + ip_hlen);
			TXD->lower_setup.ip_fields.ipcso =
			    ehdrlen + offsetof(struct ip, ip_sum);
			cmd |= E1000_TXD_CMD_IP;
			*txd_upper |= E1000_TXD_POPTS_IXSM << 8;
		}

		hdr_len = ehdrlen + ip_hlen;
		ipproto = ip->ip_p;

		break;
	case ETHERTYPE_IPV6:
		ip6 = (struct ip6_hdr *)(m_head->m_data + ehdrlen);
		ip_hlen = sizeof(struct ip6_hdr); /* XXX: No header stacking. */

		/* IPv6 doesn't have a header checksum. */

		hdr_len = ehdrlen + ip_hlen;
		ipproto = ip6->ip6_nxt;
		break;

	default:
		return 0;
	}
	
		switch (ipproto) {
	case IPPROTO_TCP:
		if (m_head->m_pkthdr.csum_flags & CSUM_TCP) {
			*txd_lower = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
			*txd_upper |= E1000_TXD_POPTS_TXSM << 8;
			/* no need for context if already set */
			if (adapter->last_hw_offload == CSUM_TCP)
				return 0;
			adapter->last_hw_offload = CSUM_TCP;
			/*
			 * Start offset for payload checksum calculation.
			 * End offset for payload checksum calculation.
			 * Offset of place to put the checksum.
			 */
			TXD = (struct e1000_context_desc *)
			    &adapter->tx_desc_base[curr_txd];
			TXD->upper_setup.tcp_fields.tucss = hdr_len;
			TXD->upper_setup.tcp_fields.tucse = htole16(0);
			TXD->upper_setup.tcp_fields.tucso =
			    hdr_len + offsetof(struct tcphdr, th_sum);
			cmd |= E1000_TXD_CMD_TCP;
		}
		break;
			case IPPROTO_UDP:
	{
		if (m_head->m_pkthdr.csum_flags & CSUM_UDP) {
			*txd_lower = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
			*txd_upper |= E1000_TXD_POPTS_TXSM << 8;
			/* no need for context if already set */
			if (adapter->last_hw_offload == CSUM_UDP)
				return 0;
			adapter->last_hw_offload = CSUM_UDP;
			/*
			 * Start offset for header checksum calculation.
			 * End offset for header checksum calculation.
			 * Offset of place to put the checksum.
			 */
			TXD = (struct e1000_context_desc *)
			    &adapter->tx_desc_base[curr_txd];
			TXD->upper_setup.tcp_fields.tucss = hdr_len;
			TXD->upper_setup.tcp_fields.tucse = htole16(0);
			TXD->upper_setup.tcp_fields.tucso =
			    hdr_len + offsetof(struct udphdr, uh_sum);
		}
		/* Fall Thru */
	}
		default:
			break;
		}
		if (TXD == NULL)
			return 0;
		TXD->tcp_seg_setup.data = htole32(0);
	TXD->cmd_and_length =
	    htole32(adapter->txd_cmd | E1000_TXD_CMD_DEXT | cmd);
	
	return 0; 
}

/*********************************************************************
 *
 * 82547 workaround to avoid controller hang in half-duplex environment.
 * The workaround is to avoid queuing a large packet that would span
 * the internal Tx FIFO ring boundary. We need to reset the FIFO pointers
 * in this case. We do that only when FIFO is quiescent.
 *
 **********************************************************************/
static void
lem_82547_move_tail(void *arg, int i)
{
	struct adapter *adapter = arg;
	struct e1000_tx_desc *tx_desc;
	u16	hw_tdt, sw_tdt, length = 0;
	bool	eop = 0;

	hw_tdt = E1000_READ_REG(&adapter->hw, E1000_TDT(0));
	sw_tdt = i; 
	
	while (hw_tdt != sw_tdt) {
		tx_desc = &adapter->tx_desc_base[hw_tdt];
		length += tx_desc->lower.flags.length;
		eop = tx_desc->lower.data & E1000_TXD_CMD_EOP;
		if (++hw_tdt == lem_sctx->isc_ntxd)
			hw_tdt = 0;

		if (eop) {
			if (lem_82547_fifo_workaround(adapter, length)) {
				adapter->tx_fifo_wrk_cnt++;
				break;
			}
			E1000_WRITE_REG(&adapter->hw, E1000_TDT(0), hw_tdt);
			lem_82547_update_fifo_head(adapter, length);
			length = 0;
		}
	}	
}

static int
lem_82547_fifo_workaround(struct adapter *adapter, int len)
{	
	int fifo_space, fifo_pkt_len;

	fifo_pkt_len = roundup2(len + EM_FIFO_HDR, EM_FIFO_HDR);

	if (adapter->link_duplex == HALF_DUPLEX) {
		fifo_space = adapter->tx_fifo_size - adapter->tx_fifo_head;

		if (fifo_pkt_len >= (EM_82547_PKT_THRESH + fifo_space)) {
			if (lem_82547_tx_fifo_reset(adapter))
				return (0);
			else
				return (1);
		}
	}

	return (0);
}

static void
lem_82547_update_fifo_head(struct adapter *adapter, int len)
{
	int fifo_pkt_len = roundup2(len + EM_FIFO_HDR, EM_FIFO_HDR);
	
	/* tx_fifo_head is always 16 byte aligned */
	adapter->tx_fifo_head += fifo_pkt_len;
	if (adapter->tx_fifo_head >= adapter->tx_fifo_size) {
		adapter->tx_fifo_head -= adapter->tx_fifo_size;
	}
}

static int
lem_82547_tx_fifo_reset(struct adapter *adapter)
{
	u32 tctl;

	if ((E1000_READ_REG(&adapter->hw, E1000_TDT(0)) ==
	    E1000_READ_REG(&adapter->hw, E1000_TDH(0))) &&
	    (E1000_READ_REG(&adapter->hw, E1000_TDFT) == 
	    E1000_READ_REG(&adapter->hw, E1000_TDFH)) &&
	    (E1000_READ_REG(&adapter->hw, E1000_TDFTS) ==
	    E1000_READ_REG(&adapter->hw, E1000_TDFHS)) &&
	    (E1000_READ_REG(&adapter->hw, E1000_TDFPC) == 0)) {
		/* Disable TX unit */
		tctl = E1000_READ_REG(&adapter->hw, E1000_TCTL);
		E1000_WRITE_REG(&adapter->hw, E1000_TCTL,
		    tctl & ~E1000_TCTL_EN);

		/* Reset FIFO pointers */
		E1000_WRITE_REG(&adapter->hw, E1000_TDFT,
		    adapter->tx_head_addr);
		E1000_WRITE_REG(&adapter->hw, E1000_TDFH,
		    adapter->tx_head_addr);
		E1000_WRITE_REG(&adapter->hw, E1000_TDFTS,
		    adapter->tx_head_addr);
		E1000_WRITE_REG(&adapter->hw, E1000_TDFHS,
		    adapter->tx_head_addr);

		/* Re-enable TX unit */
		E1000_WRITE_REG(&adapter->hw, E1000_TCTL, tctl);
		E1000_WRITE_FLUSH(&adapter->hw);

		adapter->tx_fifo_head = 0;
		adapter->tx_fifo_reset_cnt++;

		return (TRUE);
	}
	else {
		return (FALSE);
	}
}

static void
lem_isc_txd_flush(void *arg, uint16_t txqid, uint32_t pidx)
{
	struct adapter *adapter = arg;
	
	E1000_WRITE_REG(&adapter->hw, E1000_TDT(0), pidx);
}

static int
lem_isc_txd_credits_update(void *arg, uint16_t qid, uint32_t cidx)
{
	struct adapter *adapter = arg;
	u32 first, last, done, processed;
	struct em_buffer *tx_buffer;
	struct e1000_tx_desc *tx_desc, *eop_desc;

	processed = 0;
	first = cidx;
	tx_desc = &adapter->tx_desc_base[first];
	tx_buffer = &adapter->tx_buffer_area[first];
	last = tx_buffer->next_eop;
    if (last == -1)
		return (0);

	eop_desc = &adapter->tx_desc_base[last];

	/*
	 * What this does is get the index of the
	 * first descriptor AFTER the EOP of the 
	 * first packet, that way we can do the
	 * simple comparison on the inner while loop.
	 */
	if (++last == lem_sctx->isc_ntxd)
 		last = 0;
	done = last;

	while (eop_desc->upper.fields.status & E1000_TXD_STAT_DD) {
		/* We clean the range of the packet */
		while (first != done) {
			tx_desc->upper.data = 0;
			tx_desc->lower.data = 0;
			tx_desc->buffer_addr = 0;
			++processed;

			tx_buffer->next_eop = -1; 
			if (++first == lem_sctx->isc_ntxd)
				first = 0;
			tx_buffer = &adapter->tx_buffer_area[first];
			tx_desc = &adapter->tx_desc_base[first]; 
		}
		/* See if we can continue to the next packet */
		last = tx_buffer->next_eop;
		if (last != -1) {
			eop_desc = &adapter->tx_desc_base[last];
			/* Get new done point */
			if (++last == lem_sctx->isc_ntxd) last = 0;
			done = last;
		} else
			break;
	}

    return (processed); 	
}

static void lem_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, uint32_t pidx)
{
    struct adapter *adapter = arg;

	E1000_WRITE_REG(&adapter->hw, E1000_RDT(0), pidx);
}

static int lem_isc_rxd_available(void *arg, uint16_t rxqid, uint32_t idx)
{
	struct adapter *adapter = arg;
	struct e1000_rx_desc *current_desc;
    int i, cnt; 
    u8 status; 
	
	for (cnt = 0, i = idx; cnt < lem_sctx->isc_nrxd;) {
		current_desc = &adapter->rx_desc_base[i];
		status = current_desc->status;
		
		if ((status & E1000_RXD_STAT_DD) == 0)
			break;
		cnt++;
		
		if (++i == lem_sctx->isc_nrxd)
			i = 0; 
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
lem_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{
	struct adapter        *adapter = arg;
    struct e1000_rx_desc  *current_desc;
    u16 len, eop;
	u16 vtag = 0; 
    u8 status = 0; 
	
	ri->iri_qidx = 0;
    current_desc = &adapter->rx_desc_base[ri->iri_cidx];
    status = current_desc->status;
	len = le16toh(current_desc->length); 
 
    /* Error Checkng then decrement count */
	MPASS((status & E1000_RXD_STAT_DD) == 0);

    ri->iri_len = len;
    current_desc->errors = 0;

	eop = ((status & E1000_RXD_STAT_EOP) != 0);
	if (status & E1000_RXD_STAT_VP) {
		vtag = le16toh(current_desc->special); 
	} else {
		vtag = 0; 
	}

     /* Make sure bad packets are discarded */
    if (eop && (status & E1000_RXD_ERR_FRAME_ERR_MASK) != 0) {
		  adapter->dropped_pkts++; 
		  return (EBADMSG);
    }
	
	/* Prefetch the next buffer */
	if (!eop) {
		ri->iri_next_offset = 1; 
	} else {
        lem_receive_checksum(adapter, current_desc, ri); 
		
		if (vtag) {
			ri->iri_vtag = vtag;
			ri->iri_flags |= M_VLANTAG;
		} 
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
lem_receive_checksum(struct adapter *adapter,
	    struct e1000_rx_desc *rx_desc, if_rxd_info_t ri)
{
	/* 82543 or newer only */
	if ((adapter->hw.mac.type < e1000_82543) ||
	    /* Ignore Checksum bit is set */
	    (rx_desc->status & E1000_RXD_STAT_IXSM)) {
		ri->iri_csum_flags = 0;
		return;
	}

	if (rx_desc->status & E1000_RXD_STAT_IPCS) {
		/* Did it pass? */
		if (!(rx_desc->errors & E1000_RXD_ERR_IPE)) {
			/* IP Checksum Good */
			ri->iri_csum_flags = CSUM_IP_CHECKED;
			ri->iri_csum_flags |= CSUM_IP_VALID;

		} else {
			ri->iri_csum_flags = 0;
		}
	}

	if (rx_desc->status & E1000_RXD_STAT_TCPCS) {
		/* Did it pass? */
		if (!(rx_desc->errors & E1000_RXD_ERR_TCPE)) {
			ri->iri_csum_flags |=
			(CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
			ri->iri_csum_data = htons(0xffff);
		}
	}
}

static void lem_isc_rxd_refill(void *arg, uint16_t rxqid, uint8_t flid __unused,
				   uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs __unused, uint16_t count)
{
	struct adapter *adapter       = arg;
	int			i;
	uint32_t next_pidx;
	
	for (i = 0, next_pidx = pidx; i < count; i++)
	{
	  adapter->rx_desc_base[next_pidx].buffer_addr = paddrs[i]; 
	  if (++next_pidx == lem_sctx->isc_nrxd)
		  next_pidx = 0;
	}
}
