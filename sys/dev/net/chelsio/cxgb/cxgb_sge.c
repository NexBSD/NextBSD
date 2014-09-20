/**************************************************************************

Copyright (c) 2007-2009, Chelsio Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Neither the name of the Chelsio Corporation nor the names of its
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

***************************************************************************/

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_inet6.h"
#include "opt_inet.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus_dma.h>
#include <sys/rman.h>
#include <sys/queue.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <sys/proc.h>
#include <sys/sbuf.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/systm.h>
#include <sys/syslog.h>
#include <sys/socket.h>
#include <sys/sglist.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/bpf.h>	
#include <net/ethernet.h>
#include <net/if_vlan_var.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <netinet/tcp.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <cxgb_include.h>
#include <sys/mvec.h>

int	txq_fills = 0;
int	multiq_tx_enable = 1;

#ifdef TCP_OFFLOAD
CTASSERT(NUM_CPL_HANDLERS >= NUM_CPL_CMDS);
#endif

extern struct sysctl_oid_list sysctl__hw_cxgb_children;
int cxgb_txq_buf_ring_size = TX_ETH_Q_SIZE;
SYSCTL_INT(_hw_cxgb, OID_AUTO, txq_mr_size, CTLFLAG_RDTUN, &cxgb_txq_buf_ring_size, 0,
    "size of per-queue mbuf ring");

static int cxgb_tx_coalesce_force = 0;
SYSCTL_INT(_hw_cxgb, OID_AUTO, tx_coalesce_force, CTLFLAG_RWTUN,
    &cxgb_tx_coalesce_force, 0,
    "coalesce small packets into a single work request regardless of ring state");

#define	COALESCE_START_DEFAULT		TX_ETH_Q_SIZE>>1
#define	COALESCE_START_MAX		(TX_ETH_Q_SIZE-(TX_ETH_Q_SIZE>>3))
#define	COALESCE_STOP_DEFAULT		TX_ETH_Q_SIZE>>2
#define	COALESCE_STOP_MIN		TX_ETH_Q_SIZE>>5
#define	TX_RECLAIM_DEFAULT		TX_ETH_Q_SIZE>>5
#define	TX_RECLAIM_MAX			TX_ETH_Q_SIZE>>2
#define	TX_RECLAIM_MIN			TX_ETH_Q_SIZE>>6


static int cxgb_tx_coalesce_enable_start = COALESCE_START_DEFAULT;
SYSCTL_INT(_hw_cxgb, OID_AUTO, tx_coalesce_enable_start, CTLFLAG_RWTUN,
    &cxgb_tx_coalesce_enable_start, 0,
    "coalesce enable threshold");
static int cxgb_tx_coalesce_enable_stop = COALESCE_STOP_DEFAULT;
SYSCTL_INT(_hw_cxgb, OID_AUTO, tx_coalesce_enable_stop, CTLFLAG_RWTUN,
    &cxgb_tx_coalesce_enable_stop, 0,
    "coalesce disable threshold");
static int cxgb_tx_reclaim_threshold = TX_RECLAIM_DEFAULT;
SYSCTL_INT(_hw_cxgb, OID_AUTO, tx_reclaim_threshold, CTLFLAG_RWTUN,
    &cxgb_tx_reclaim_threshold, 0,
    "tx cleaning minimum threshold");

/*
 * XXX don't re-enable this until TOE stops assuming
 * we have an m_ext
 */
static int recycle_enable = 0;

extern int cxgb_use_16k_clusters;
extern int nmbjumbop;
extern int nmbjumbo9;
extern int nmbjumbo16;

#define USE_GTS 0

#define SGE_RX_SM_BUF_SIZE	1536
#define SGE_RX_DROP_THRES	16
#define SGE_RX_COPY_THRES	128

/*
 * Period of the Tx buffer reclaim timer.  This timer does not need to run
 * frequently as Tx buffers are usually reclaimed by new Tx packets.
 */
#define TX_RECLAIM_PERIOD       (hz >> 1)

/* 
 * Values for sge_txq.flags
 */
enum {
	TXQ_RUNNING	= 1 << 0,  /* fetch engine is running */
	TXQ_LAST_PKT_DB = 1 << 1,  /* last packet rang the doorbell */
};

struct tx_desc {
	uint64_t	flit[TX_DESC_FLITS];
} __packed;

struct rx_desc {
	uint32_t	addr_lo;
	uint32_t	len_gen;
	uint32_t	gen2;
	uint32_t	addr_hi;
} __packed;

struct rsp_desc {               /* response queue descriptor */
	struct rss_header	rss_hdr;
	uint32_t		flags;
	uint32_t		len_cq;
	uint8_t			imm_data[47];
	uint8_t			intr_gen;
} __packed;

#define RSPQ_NSOP_NEOP           G_RSPD_SOP_EOP(0)
#define RSPQ_EOP                 G_RSPD_SOP_EOP(F_RSPD_EOP)
#define RSPQ_SOP                 G_RSPD_SOP_EOP(F_RSPD_SOP)
#define RSPQ_SOP_EOP             G_RSPD_SOP_EOP(F_RSPD_SOP|F_RSPD_EOP)

struct rx_sw_desc {                /* SW state per Rx descriptor */
	caddr_t		rxsd_cl;
};

struct txq_state {
	unsigned int	compl;
	unsigned int	gen;
	unsigned int	pidx;
};

/*
 * Maps a number of flits to the number of Tx descriptors that can hold them.
 * The formula is
 *
 * desc = 1 + (flits - 2) / (WR_FLITS - 1).
 *
 * HW allows up to 4 descriptors to be combined into a WR.
 */
static uint8_t flit_desc_map[] = {
	0,
#if SGE_NUM_GENBITS == 1
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4
#elif SGE_NUM_GENBITS == 2
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
#else
# error "SGE_NUM_GENBITS must be 1 or 2"
#endif
};

#define	TXQ_LOCK_ASSERT(qs)	mtx_assert(&(qs)->lock, MA_OWNED)
#define	TXQ_TRYLOCK(qs)		mtx_trylock(&(qs)->lock)	
#define	TXQ_LOCK(qs)		mtx_lock(&(qs)->lock)	
#define	TXQ_UNLOCK(qs)		mtx_unlock(&(qs)->lock)	
#define	TXQ_RING_EMPTY(qs)	drbr_empty((qs)->port->ifp, (qs)->txq[TXQ_ETH].txq_mr)
#define	TXQ_RING_NEEDS_ENQUEUE(qs)					\
	drbr_needs_enqueue((qs)->port->ifp, (qs)->txq[TXQ_ETH].txq_mr)
#define	TXQ_RING_FLUSH(qs)	drbr_flush((qs)->port->ifp, (qs)->txq[TXQ_ETH].txq_mr)
#define	TXQ_RING_DEQUEUE_COND(qs, func, arg)				\
	drbr_dequeue_cond((qs)->port->ifp, (qs)->txq[TXQ_ETH].txq_mr, func, arg)
#define	TXQ_RING_DEQUEUE(qs) \
	drbr_dequeue((qs)->port->ifp, (qs)->txq[TXQ_ETH].txq_mr)

int cxgb_debug = 0;

static void sge_timer_cb(void *arg);
static void sge_timer_reclaim(void *arg, int ncount);
static void sge_txq_reclaim_handler(void *arg, int ncount);

/*
 * XXX need to cope with bursty scheduling by looking at a wider
 * window than we are now for determining the need for coalescing
 *
 */
static __inline uint64_t
check_pkt_coalesce(struct sge_qset *qs) 
{ 
        struct adapter *sc; 
        struct sge_txq *txq; 
	uint8_t *fill;

	if (__predict_false(cxgb_tx_coalesce_force))
		return (1);
	txq = &qs->txq[TXQ_ETH]; 
        sc = qs->port->adapter; 
	fill = &sc->tunq_fill[qs->idx];

	if (cxgb_tx_coalesce_enable_start > COALESCE_START_MAX)
		cxgb_tx_coalesce_enable_start = COALESCE_START_MAX;
	if (cxgb_tx_coalesce_enable_stop < COALESCE_STOP_MIN)
		cxgb_tx_coalesce_enable_start = COALESCE_STOP_MIN;
	/*
	 * if the hardware transmit queue is more than 1/8 full
	 * we mark it as coalescing - we drop back from coalescing
	 * when we go below 1/32 full and there are no packets enqueued, 
	 * this provides us with some degree of hysteresis
	 */
        if (*fill != 0 && (txq->in_use <= cxgb_tx_coalesce_enable_stop) &&
	    TXQ_RING_EMPTY(qs) && (qs->coalescing == 0))
                *fill = 0; 
        else if (*fill == 0 && (txq->in_use >= cxgb_tx_coalesce_enable_start))
                *fill = 1; 

	return (sc->tunq_coalesce);
} 

#ifdef __LP64__
static void
set_wr_hdr(struct work_request_hdr *wrp, uint32_t wr_hi, uint32_t wr_lo)
{
	uint64_t wr_hilo;
#if _BYTE_ORDER == _LITTLE_ENDIAN
	wr_hilo = wr_hi;
	wr_hilo |= (((uint64_t)wr_lo)<<32);
#else
	wr_hilo = wr_lo;
	wr_hilo |= (((uint64_t)wr_hi)<<32);
#endif	
	wrp->wrh_hilo = wr_hilo;
}
#else
static void
set_wr_hdr(struct work_request_hdr *wrp, uint32_t wr_hi, uint32_t wr_lo)
{

	wrp->wrh_hi = wr_hi;
	wmb();
	wrp->wrh_lo = wr_lo;
}
#endif

struct coalesce_info {
	int count;
	int nbytes;
};

static int
coalesce_check(struct mbuf *m, void *arg)
{
	struct coalesce_info *ci = arg;
	int *count = &ci->count;
	int *nbytes = &ci->nbytes;

	if ((*nbytes == 0) || ((*nbytes + m->m_len <= 10500) &&
		(*count < 7) && (m->m_next == NULL))) {
		*count += 1;
		*nbytes += m->m_len;
		return (1);
	}
	return (0);
}

/**
 *	should_restart_tx - are there enough resources to restart a Tx queue?
 *	@q: the Tx queue
 *
 *	Checks if there are enough descriptors to restart a suspended Tx queue.
 */
static __inline int
should_restart_tx(const struct sge_txq *q)
{
	unsigned int r = q->processed - q->cleaned;

	return q->in_use - r < (q->size >> 1);
}

/**
 *	t3_sge_init - initialize SGE
 *	@adap: the adapter
 *	@p: the SGE parameters
 *
 *	Performs SGE initialization needed every time after a chip reset.
 *	We do not initialize any of the queue sets here, instead the driver
 *	top-level must request those individually.  We also do not enable DMA
 *	here, that should be done after the queues have been set up.
 */
void
t3_sge_init(adapter_t *adap, struct sge_params *p)
{
	u_int ctrl, ups;

	ups = 0; /* = ffs(pci_resource_len(adap->pdev, 2) >> 12); */

	ctrl = F_DROPPKT | V_PKTSHIFT(2) | F_FLMODE | F_AVOIDCQOVFL |
	       F_CQCRDTCTRL | F_CONGMODE | F_TNLFLMODE | F_FATLPERREN |
	       V_HOSTPAGESIZE(PAGE_SHIFT - 11) | F_BIGENDIANINGRESS |
	       V_USERSPACESIZE(ups ? ups - 1 : 0) | F_ISCSICOALESCING;
#if SGE_NUM_GENBITS == 1
	ctrl |= F_EGRGENCTRL;
#endif
	if (adap->params.rev > 0) {
		if (!(adap->flags & (USING_MSIX | USING_MSI)))
			ctrl |= F_ONEINTMULTQ | F_OPTONEINTMULTQ;
	}
	t3_write_reg(adap, A_SG_CONTROL, ctrl);
	t3_write_reg(adap, A_SG_EGR_RCQ_DRB_THRSH, V_HIRCQDRBTHRSH(512) |
		     V_LORCQDRBTHRSH(512));
	t3_write_reg(adap, A_SG_TIMER_TICK, core_ticks_per_usec(adap) / 10);
	t3_write_reg(adap, A_SG_CMDQ_CREDIT_TH, V_THRESHOLD(32) |
		     V_TIMEOUT(200 * core_ticks_per_usec(adap)));
	t3_write_reg(adap, A_SG_HI_DRB_HI_THRSH,
		     adap->params.rev < T3_REV_C ? 1000 : 500);
	t3_write_reg(adap, A_SG_HI_DRB_LO_THRSH, 256);
	t3_write_reg(adap, A_SG_LO_DRB_HI_THRSH, 1000);
	t3_write_reg(adap, A_SG_LO_DRB_LO_THRSH, 256);
	t3_write_reg(adap, A_SG_OCO_BASE, V_BASE1(0xfff));
	t3_write_reg(adap, A_SG_DRB_PRI_THRESH, 63 * 1024);
}


/**
 *	sgl_len - calculates the size of an SGL of the given capacity
 *	@n: the number of SGL entries
 *
 *	Calculates the number of flits needed for a scatter/gather list that
 *	can hold the given number of entries.
 */
static __inline unsigned int
sgl_len(unsigned int n)
{
	return ((3 * n) / 2 + (n & 1));
}

/**
 *	get_imm_packet - return the next ingress packet buffer from a response
 *	@resp: the response descriptor containing the packet data
 *
 *	Return a packet containing the immediate data of the given response.
 */
static int
get_imm_packet(const struct rsp_desc *resp, struct mbuf *m)
{

	if (resp->rss_hdr.opcode == CPL_RX_DATA) {
		const struct cpl_rx_data *cpl = (const void *)&resp->imm_data[0];
		m->m_len = sizeof(*cpl) + ntohs(cpl->len);
	} else if (resp->rss_hdr.opcode == CPL_RX_PKT) {
		const struct cpl_rx_pkt *cpl = (const void *)&resp->imm_data[0];
		m->m_len = sizeof(*cpl) + ntohs(cpl->len);
	} else
		m->m_len = IMMED_PKT_SIZE;
	memcpy(mtod(m, uint8_t *), resp->imm_data, m->m_len); 

	return (0);	
}

static __inline u_int
flits_to_desc(u_int n)
{
	return (flit_desc_map[n]);
}

#define SGE_PARERR (F_CPPARITYERROR | F_OCPARITYERROR | F_RCPARITYERROR | \
		    F_IRPARITYERROR | V_ITPARITYERROR(M_ITPARITYERROR) | \
		    V_FLPARITYERROR(M_FLPARITYERROR) | F_LODRBPARITYERROR | \
		    F_HIDRBPARITYERROR | F_LORCQPARITYERROR | \
		    F_HIRCQPARITYERROR)
#define SGE_FRAMINGERR (F_UC_REQ_FRAMINGERROR | F_R_REQ_FRAMINGERROR)
#define SGE_FATALERR (SGE_PARERR | SGE_FRAMINGERR | F_RSPQCREDITOVERFOW | \
		      F_RSPQDISABLED)

/**
 *	t3_sge_err_intr_handler - SGE async event interrupt handler
 *	@adapter: the adapter
 *
 *	Interrupt handler for SGE asynchronous (non-data) events.
 */
void
t3_sge_err_intr_handler(adapter_t *adapter)
{
	unsigned int v, status;

	status = t3_read_reg(adapter, A_SG_INT_CAUSE);
	if (status & SGE_PARERR)
		CH_ALERT(adapter, "SGE parity error (0x%x)\n",
			 status & SGE_PARERR);
	if (status & SGE_FRAMINGERR)
		CH_ALERT(adapter, "SGE framing error (0x%x)\n",
			 status & SGE_FRAMINGERR);
	if (status & F_RSPQCREDITOVERFOW)
		CH_ALERT(adapter, "SGE response queue credit overflow\n");

	if (status & F_RSPQDISABLED) {
		v = t3_read_reg(adapter, A_SG_RSPQ_FL_STATUS);

		CH_ALERT(adapter,
			 "packet delivered to disabled response queue (0x%x)\n",
			 (v >> S_RSPQ0DISABLED) & 0xff);
	}

	t3_write_reg(adapter, A_SG_INT_CAUSE, status);
	if (status & SGE_FATALERR)
		t3_fatal_err(adapter);
}

void
t3_sge_prep(adapter_t *adap, struct sge_params *p)
{
	int i, nqsets, fl_q_size, jumbo_q_size, use_16k, jumbo_buf_size;

	nqsets = min(SGE_QSETS / adap->params.nports, mp_ncpus);
	nqsets *= adap->params.nports;

	fl_q_size = min(nmbclusters/(3*nqsets), FL_Q_SIZE);

	while (!powerof2(fl_q_size))
		fl_q_size--;

	use_16k = cxgb_use_16k_clusters != -1 ? cxgb_use_16k_clusters :
	    is_offload(adap);

#if __FreeBSD_version >= 700111
	if (use_16k) {
		jumbo_q_size = min(nmbjumbo16/(3*nqsets), JUMBO_Q_SIZE);
		jumbo_buf_size = MJUM16BYTES;
	} else {
		jumbo_q_size = min(nmbjumbo9/(3*nqsets), JUMBO_Q_SIZE);
		jumbo_buf_size = MJUM9BYTES;
	}
#else
	jumbo_q_size = min(nmbjumbop/(3*nqsets), JUMBO_Q_SIZE);
	jumbo_buf_size = MJUMPAGESIZE;
#endif
	while (!powerof2(jumbo_q_size))
		jumbo_q_size--;

	if (fl_q_size < (FL_Q_SIZE / 4) || jumbo_q_size < (JUMBO_Q_SIZE / 2))
		device_printf(adap->dev,
		    "Insufficient clusters and/or jumbo buffers.\n");

	p->max_pkt_size = jumbo_buf_size - sizeof(struct cpl_rx_data);

	for (i = 0; i < SGE_QSETS; ++i) {
		struct qset_params *q = p->qset + i;

		if (adap->params.nports > 2) {
			q->coalesce_usecs = 50;
		} else {
#ifdef INVARIANTS			
			q->coalesce_usecs = 10;
#else
			q->coalesce_usecs = 5;
#endif			
		}
		q->polling = 0;
		q->rspq_size = RSPQ_Q_SIZE;
		q->fl_size = fl_q_size;
		q->jumbo_size = jumbo_q_size;
		q->jumbo_buf_size = jumbo_buf_size;
		q->txq_size[TXQ_ETH] = TX_ETH_Q_SIZE;
		q->txq_size[TXQ_OFLD] = is_offload(adap) ? TX_OFLD_Q_SIZE : 16;
		q->txq_size[TXQ_CTRL] = TX_CTRL_Q_SIZE;
		q->cong_thres = 0;
	}
}

void
t3_update_qset_coalesce(struct sge_qset *qs, const struct qset_params *p)
{

	qs->rspq.holdoff_tmr = max(p->coalesce_usecs * 10, 1U);
	qs->rspq.polling = 0 /* p->polling */;
}

/**
 *	recycle_rx_buf - recycle a receive buffer
 *	@adapter: the adapter
 *	@q: the SGE free list
 *	@idx: index of buffer to recycle
 *
 *	Recycles the specified buffer on the given free list by adding it at
 *	the next available slot on the list.
 */
static void
recycle_rx_buf(adapter_t *adap, struct sge_fl *q, unsigned int idx)
{
	struct rx_desc *from = &q->desc[idx];
	struct rx_desc *to   = &q->desc[q->pidx];

	q->sdesc[q->pidx] = q->sdesc[idx];
	to->addr_lo = from->addr_lo;        // already big endian
	to->addr_hi = from->addr_hi;        // likewise
	wmb();	/* necessary ? */
	to->len_gen = htobe32(V_FLD_GEN1(q->gen));
	to->gen2 = htobe32(V_FLD_GEN2(q->gen));
	q->credits++;

	if (++q->pidx == q->size) {
		q->pidx = 0;
		q->gen ^= 1;
	}
	t3_write_reg(adap, A_SG_KDOORBELL, V_EGRCNTX(q->cntxt_id));
}

static int
alloc_sw_ring(adapter_t *sc, size_t nelem,  size_t sw_size, void *sdesc)
{
	size_t len = nelem * elem_size;
	void *s = NULL;
	int err;

	if (sw_size) {
		len = nelem * sw_size;
		s = malloc(len, M_DEVBUF, M_WAITOK|M_ZERO);
		*(void **)sdesc = s;
	}
	return (0);
}

static void
sge_slow_intr_handler(void *arg, int ncount)
{
	adapter_t *sc = arg;

	t3_slow_intr_handler(sc);
	t3_write_reg(sc, A_PL_INT_ENABLE0, sc->slow_intr_mask);
	(void) t3_read_reg(sc, A_PL_INT_ENABLE0);
}

/**
 *	sge_timer_cb - perform periodic maintenance of an SGE qset
 *	@data: the SGE queue set to maintain
 *
 *	Runs periodically from a timer to perform maintenance of an SGE queue
 *	set.  It performs two tasks:
 *
 *	a) Cleans up any completed Tx descriptors that may still be pending.
 *	Normal descriptor cleanup happens when new packets are added to a Tx
 *	queue so this timer is relatively infrequent and does any cleanup only
 *	if the Tx queue has not seen any new packets in a while.  We make a
 *	best effort attempt to reclaim descriptors, in that we don't wait
 *	around if we cannot get a queue's lock (which most likely is because
 *	someone else is queueing new packets and so will also handle the clean
 *	up).  Since control queues use immediate data exclusively we don't
 *	bother cleaning them up here.
 *
 *	b) Replenishes Rx queues that have run out due to memory shortage.
 *	Normally new Rx buffers are added when existing ones are consumed but
 *	when out of memory a queue can become empty.  We try to add only a few
 *	buffers here, the queue will be replenished fully as these new buffers
 *	are used up if memory shortage has subsided.
 *	
 *	c) Return coalesced response queue credits in case a response queue is
 *	starved.
 *
 *	d) Ring doorbells for T304 tunnel queues since we have seen doorbell 
 *	fifo overflows and the FW doesn't implement any recovery scheme yet.
 */
static void
sge_timer_cb(void *arg)
{
	adapter_t *sc = arg;
	if ((sc->flags & USING_MSIX) == 0) {
		
		struct port_info *pi;
		struct sge_qset *qs;
		struct sge_txq  *txq;
		int i, j;
		int reclaim_ofl, refill_rx;

		if (sc->open_device_map == 0) 
			return;

		for (i = 0; i < sc->params.nports; i++) {
			pi = &sc->port[i];
			for (j = 0; j < pi->nqsets; j++) {
				qs = &sc->sge.qs[pi->first_qset + j];
				txq = &qs->txq[0];
				reclaim_ofl = txq[TXQ_OFLD].processed - txq[TXQ_OFLD].cleaned;
				refill_rx = ((qs->fl[0].credits < qs->fl[0].size) || 
				    (qs->fl[1].credits < qs->fl[1].size));
				if (reclaim_ofl || refill_rx) {
					taskqueue_enqueue(sc->tq, &pi->timer_reclaim_task);
					break;
				}
			}
		}
	}
	
	if (sc->params.nports > 2) {
		int i;

		for_each_port(sc, i) {
			struct port_info *pi = &sc->port[i];

			t3_write_reg(sc, A_SG_KDOORBELL, 
				     F_SELEGRCNTX | 
				     (FW_TUNNEL_SGEEC_START + pi->first_qset));
		}
	}	
	if (((sc->flags & USING_MSIX) == 0 || sc->params.nports > 2) &&
	    sc->open_device_map != 0)
		callout_reset(&sc->sge_timer_ch, TX_RECLAIM_PERIOD, sge_timer_cb, sc);
}

/*
 * This is meant to be a catch-all function to keep sge state private
 * to sge.c
 *
 */
int
t3_sge_init_adapter(adapter_t *sc)
{
	callout_init(&sc->sge_timer_ch, CALLOUT_MPSAFE);
	callout_reset(&sc->sge_timer_ch, TX_RECLAIM_PERIOD, sge_timer_cb, sc);
	TASK_INIT(&sc->slow_intr_task, 0, sge_slow_intr_handler, sc);
	return (0);
}

int
t3_sge_reset_adapter(adapter_t *sc)
{
	callout_reset(&sc->sge_timer_ch, TX_RECLAIM_PERIOD, sge_timer_cb, sc);
	return (0);
}

int
t3_sge_init_port(struct port_info *pi)
{
	TASK_INIT(&pi->timer_reclaim_task, 0, sge_timer_reclaim, pi);
	return (0);
}

/**
 *	refill_rspq - replenish an SGE response queue
 *	@adapter: the adapter
 *	@q: the response queue to replenish
 *	@credits: how many new responses to make available
 *
 *	Replenishes a response queue by making the supplied number of responses
 *	available to HW.
 */
static __inline void
refill_rspq(adapter_t *sc, const struct sge_rspq *q, u_int credits)
{

	/* mbufs are allocated on demand when a rspq entry is processed. */
	t3_write_reg(sc, A_SG_RSPQ_CREDIT_RETURN,
		     V_RSPQ(q->cntxt_id) | V_CREDITS(credits));
}

static void
sge_txq_reclaim_handler(void *arg, int ncount)
{
	struct sge_qset *qs = arg;
	int i;

	for (i = 0; i < 3; i++)
		reclaim_completed_tx(qs, 16, i);
}

static void
sge_timer_reclaim(void *arg, int ncount)
{
	struct port_info *pi = arg;
	int i, nqsets = pi->nqsets;
	adapter_t *sc = pi->adapter;
	struct sge_qset *qs;
	struct mtx *lock;
	
	KASSERT((sc->flags & USING_MSIX) == 0,
	    ("can't call timer reclaim for msi-x"));

	for (i = 0; i < nqsets; i++) {
		qs = &sc->sge.qs[pi->first_qset + i];

		reclaim_completed_tx(qs, 16, TXQ_OFLD);
		lock = (sc->flags & USING_MSIX) ? &qs->rspq.lock :
			    &sc->sge.qs[0].rspq.lock;

		if (mtx_trylock(lock)) {
			/* XXX currently assume that we are *NOT* polling */
			uint32_t status = t3_read_reg(sc, A_SG_RSPQ_FL_STATUS);

			if (status & (1 << qs->rspq.cntxt_id)) {
				if (qs->rspq.credits) {
					refill_rspq(sc, &qs->rspq, 1);
					qs->rspq.credits--;
					t3_write_reg(sc, A_SG_RSPQ_FL_STATUS, 
					    1 << qs->rspq.cntxt_id);
				}
			}
			mtx_unlock(lock);
		}
	}
}

/**
 *	init_qset_cntxt - initialize an SGE queue set context info
 *	@qs: the queue set
 *	@id: the queue set id
 *
 *	Initializes the TIDs and context ids for the queues of a queue set.
 */
static void
init_qset_cntxt(struct sge_qset *qs, u_int id)
{

	qs->rspq.cntxt_id = id;
	qs->fl[0].cntxt_id = 2 * id;
	qs->fl[1].cntxt_id = 2 * id + 1;
	qs->txq[TXQ_ETH].cntxt_id = FW_TUNNEL_SGEEC_START + id;
	qs->txq[TXQ_ETH].token = FW_TUNNEL_TID_START + id;
	qs->txq[TXQ_OFLD].cntxt_id = FW_OFLD_SGEEC_START + id;
	qs->txq[TXQ_CTRL].cntxt_id = FW_CTRL_SGEEC_START + id;
	qs->txq[TXQ_CTRL].token = FW_CTRL_TID_START + id;

	mbufq_init(&qs->txq[TXQ_ETH].sendq);
	mbufq_init(&qs->txq[TXQ_OFLD].sendq);
	mbufq_init(&qs->txq[TXQ_CTRL].sendq);
}


static void
txq_prod(struct sge_txq *txq, unsigned int ndesc, struct txq_state *txqs)
{
	txq->in_use += ndesc;
	/*
	 * XXX we don't handle stopping of queue
	 * presumably start handles this when we bump against the end
	 */
	txqs->gen = txq->gen;
	txq->unacked += ndesc;
	txqs->compl = (txq->unacked & 32) << (S_WR_COMPL - 5);
	txq->unacked &= 31;
	txqs->pidx = txq->pidx;
	txq->pidx += ndesc;
#ifdef INVARIANTS
	if (((txqs->pidx > txq->cidx) &&
		(txq->pidx < txqs->pidx) &&
		(txq->pidx >= txq->cidx)) ||
	    ((txqs->pidx < txq->cidx) &&
		(txq->pidx >= txq-> cidx)) ||
	    ((txqs->pidx < txq->cidx) &&
		(txq->cidx < txqs->pidx)))
		panic("txqs->pidx=%d txq->pidx=%d txq->cidx=%d",
		    txqs->pidx, txq->pidx, txq->cidx);
#endif
	if (txq->pidx >= txq->size) {
		txq->pidx -= txq->size;
		txq->gen ^= 1;
	}

}

/**
 *	calc_tx_descs - calculate the number of Tx descriptors for a packet
 *	@m: the packet mbufs
 *      @nsegs: the number of segments 
 *
 * 	Returns the number of Tx descriptors needed for the given Ethernet
 * 	packet.  Ethernet packets require addition of WR and CPL headers.
 */
static __inline unsigned int
calc_tx_descs(const struct mbuf *m, int nsegs)
{
	unsigned int flits;

	if (m->m_pkthdr.len <= PIO_LEN)
		return 1;

	flits = sgl_len(nsegs) + 2;
	if (m->m_pkthdr.csum_flags & CSUM_TSO)
		flits++;

	return flits_to_desc(flits);
}

/**
 *	make_sgl - populate a scatter/gather list for a packet
 *	@sgp: the SGL to populate
 *	@segs: the packet dma segments
 *	@nsegs: the number of segments
 *
 *	Generates a scatter/gather list for the buffers that make up a packet
 *	and returns the SGL size in 8-byte words.  The caller must size the SGL
 *	appropriately.
 */
static __inline void
make_sgl(struct sg_ent *sgp, bus_dma_segment_t *segs, int nsegs)
{
	int i, idx;
	
	for (idx = 0, i = 0; i < nsegs; i++) {
		/*
		 * firmware doesn't like empty segments
		 */
		if (segs[i].ds_len == 0)
			continue;
		if (i && idx == 0) 
			++sgp;
		
		sgp->len[idx] = htobe32(segs[i].ds_len);
		sgp->addr[idx] = htobe64(segs[i].ds_addr);
		idx ^= 1;
	}
	
	if (idx) {
		sgp->len[idx] = 0;
		sgp->addr[idx] = 0;
	}
}
	
/**
 *	check_ring_tx_db - check and potentially ring a Tx queue's doorbell
 *	@adap: the adapter
 *	@q: the Tx queue
 *
 *	Ring the doorbell if a Tx queue is asleep.  There is a natural race,
 *	where the HW is going to sleep just after we checked, however,
 *	then the interrupt handler will detect the outstanding TX packet
 *	and ring the doorbell for us.
 *
 *	When GTS is disabled we unconditionally ring the doorbell.
 */
static __inline void
check_ring_tx_db(adapter_t *adap, struct sge_txq *q, int mustring)
{
#if USE_GTS
	clear_bit(TXQ_LAST_PKT_DB, &q->flags);
	if (test_and_set_bit(TXQ_RUNNING, &q->flags) == 0) {
		set_bit(TXQ_LAST_PKT_DB, &q->flags);
#ifdef T3_TRACE
		T3_TRACE1(adap->tb[q->cntxt_id & 7], "doorbell Tx, cntxt %d",
			  q->cntxt_id);
#endif
		t3_write_reg(adap, A_SG_KDOORBELL,
			     F_SELEGRCNTX | V_EGRCNTX(q->cntxt_id));
	}
#else
	if (mustring || ++q->db_pending >= 32) {
		wmb();            /* write descriptors before telling HW */
		t3_write_reg(adap, A_SG_KDOORBELL,
		    F_SELEGRCNTX | V_EGRCNTX(q->cntxt_id));
		q->db_pending = 0;
	}
#endif
}

static __inline void
wr_gen2(struct tx_desc *d, unsigned int gen)
{
#if SGE_NUM_GENBITS == 2
	d->flit[TX_DESC_FLITS - 1] = htobe64(gen);
#endif
}

/**
 *	write_wr_hdr_sgl - write a WR header and, optionally, SGL
 *	@ndesc: number of Tx descriptors spanned by the SGL
 *	@txd: first Tx descriptor to be written
 *	@txqs: txq state (generation and producer index)
 *	@txq: the SGE Tx queue
 *	@sgl: the SGL
 *	@flits: number of flits to the start of the SGL in the first descriptor
 *	@sgl_flits: the SGL size in flits
 *	@wr_hi: top 32 bits of WR header based on WR type (big endian)
 *	@wr_lo: low 32 bits of WR header based on WR type (big endian)
 *
 *	Write a work request header and an associated SGL.  If the SGL is
 *	small enough to fit into one Tx descriptor it has already been written
 *	and we just need to write the WR header.  Otherwise we distribute the
 *	SGL across the number of descriptors it spans.
 */
static void
write_wr_hdr_sgl(unsigned int ndesc, struct tx_desc *txd, struct txq_state *txqs,
    const struct sge_txq *txq, const struct sg_ent *sgl, unsigned int flits,
    unsigned int sgl_flits, unsigned int wr_hi, unsigned int wr_lo)
{

	struct work_request_hdr *wrp = (struct work_request_hdr *)txd;
	
	if (__predict_true(ndesc == 1)) {
		set_wr_hdr(wrp, htonl(F_WR_SOP | F_WR_EOP | V_WR_DATATYPE(1) |
		    V_WR_SGLSFLT(flits)) | wr_hi,
		    htonl(V_WR_LEN(flits + sgl_flits) | V_WR_GEN(txqs->gen)) |
		    wr_lo);

		wr_gen2(txd, txqs->gen);
		
	} else {
		unsigned int ogen = txqs->gen;
		const uint64_t *fp = (const uint64_t *)sgl;
		struct work_request_hdr *wp = wrp;
		
		wrp->wrh_hi = htonl(F_WR_SOP | V_WR_DATATYPE(1) |
		    V_WR_SGLSFLT(flits)) | wr_hi;
		
		while (sgl_flits) {
			unsigned int avail = WR_FLITS - flits;

			if (avail > sgl_flits)
				avail = sgl_flits;
			memcpy(&txd->flit[flits], fp, avail * sizeof(*fp));
			sgl_flits -= avail;
			ndesc--;
			if (!sgl_flits)
				break;
			
			fp += avail;
			txd++;
			if (++txqs->pidx == txq->size) {
				txqs->pidx = 0;
				txqs->gen ^= 1;
				txd = txq->desc;
			}

			/*
			 * when the head of the mbuf chain
			 * is freed all clusters will be freed
			 * with it
			 */
			wrp = (struct work_request_hdr *)txd;
			wrp->wrh_hi = htonl(V_WR_DATATYPE(1) |
			    V_WR_SGLSFLT(1)) | wr_hi;
			wrp->wrh_lo = htonl(V_WR_LEN(min(WR_FLITS,
				    sgl_flits + 1)) |
			    V_WR_GEN(txqs->gen)) | wr_lo;
			wr_gen2(txd, txqs->gen);
			flits = 1;
		}
		wrp->wrh_hi |= htonl(F_WR_EOP);
		wmb();
		wp->wrh_lo = htonl(V_WR_LEN(WR_FLITS) | V_WR_GEN(ogen)) | wr_lo;
		wr_gen2((struct tx_desc *)wp, ogen);
	}
}

/* sizeof(*eh) + sizeof(*ip) + sizeof(*tcp) */
#define TCPPKTHDRSIZE (ETHER_HDR_LEN + 20 + 20)

#define GET_VTAG(cntrl, m) \
do { \
	if ((m)->m_flags & M_VLANTAG)					            \
		cntrl |= F_TXPKT_VLAN_VLD | V_TXPKT_VLAN((m)->m_pkthdr.ether_vtag); \
} while (0)

static int
cxgb_isc_txd_encap(if_shared_ctx_t sctx, if_pkt_info_t pkt)
{
	adapter_t *sc = DOWNCAST(sctx);
	struct sge_qset *qs = sc->qs[pkt->ipi_qsidx];
	struct mbuf *m0;
	struct sge_txq *txq;
	struct txq_state txqs;
	struct port_info *pi;
	unsigned int ndesc, flits, cntrl, mlen;
	int err, nsegs, tso_info = 0, coalescing;

	struct work_request_hdr *wrp;
	struct sg_ent *sgp, *sgl;
	uint32_t wr_hi, wr_lo, sgl_flits; 
	bus_dma_segment_t segs;
	struct tx_desc *txd;
		
	pi = qs->port;
	sc = pi->adapter;
	txq = &qs->txq[TXQ_ETH];
	txd = &txq->desc[pkt->ipi_pidx];
	prefetch(txd);

	if (pkt->ipi_m == NULL && txq->tx_coalesce_count == 0)
		return (0);
	if (pkt->ipi_m == NULL && txq->tx_coalesce_count > 0)
		goto flush;

coalesce_retry:
	m0 = pkt->ipi_m;
	nsegs = pkt->ipi_nsegs;
	segs = pkt->ipi_segs;
	sgl = txq->txq_sgl;
	coalescing = txq->tx_coalesce_count || check_pkt_coalesce(qs);
	if (coalescing && txq->tx_coalesce_count < 7 &&
		nsegs == 1 && txq->tx_coalesce_bytes + segs[0].ds_len <= 10500) {
		txq->tx_coalesce_segs[txq->tx_coalesce_count++] = &segs[0];
		txq->tx_coalesce_bytes += segs[0].ds_len;
		pi->ipi_new_pidx = pi->ipi_pidx; /* unchanged */
		pi->ipi_ndescs = 0;
		return (0);
	}
	cntrl = V_TXPKT_INTF(pi->txpkt_intf);
	KASSERT(m0->m_flags & M_PKTHDR, ("not packet header\n"));

	/*
	 * We either hit our limit or we couldn't coalesce the
	 * current packet - flush and try to coalesce the current
	 * packet
	 */
	if (txq->coalesce_count) {
	flush:
		segs = txq->tx_coalesce_segs;
		nsegs = txq->coalesce_count;
		ndesc = 1;
		mlen = 0;
		txq->tx_coalesce_count = 0;
		txq->tx_coalesce_bytes = 0;
	} else if  (m0->m_next != NULL &&
	    m0->m_pkthdr.csum_flags & (CSUM_TSO))
		tso_info = V_LSO_MSS(m0->m_pkthdr.tso_segsz);
	else {
		mlen = m0->m_pkthdr.len;
		ndesc = calc_tx_descs(m0, nsegs);
	}

	txq_prod(txq, ndesc, &txqs);

	KASSERT(m0->m_pkthdr.len, ("empty packet nsegs=%d", nsegs));

	if (txq->coalesce_count) {
		struct cpl_tx_pkt_batch *cpl_batch = (struct cpl_tx_pkt_batch *)txd;
		int i, fidx;

		if (nsegs > 7)
			panic("trying to coalesce %d packets in to one WR", nsegs);
		txq->txq_coalesced += nsegs;
		wrp = (struct work_request_hdr *)txd;
		flits = nsegs*2 + 1;

		for (fidx = 1, i = 0; i < nsegs; i++, fidx += 2) {
			struct cpl_tx_pkt_batch_entry *cbe;
			uint64_t flit;
			uint32_t *hflit = (uint32_t *)&flit;
			int cflags = m0->m_pkthdr.csum_flags;

			cntrl = V_TXPKT_INTF(pi->txpkt_intf);
			GET_VTAG(cntrl, m0);
			cntrl |= V_TXPKT_OPCODE(CPL_TX_PKT);
			if (__predict_false(!(cflags & CSUM_IP)))
				cntrl |= F_TXPKT_IPCSUM_DIS;
			if (__predict_false(!(cflags & (CSUM_TCP | CSUM_UDP |
			    CSUM_UDP_IPV6 | CSUM_TCP_IPV6))))
				cntrl |= F_TXPKT_L4CSUM_DIS;

			hflit[0] = htonl(cntrl);
			hflit[1] = htonl(segs[i].ds_len | 0x80000000);
			flit |= htobe64(1 << 24);
			cbe = &cpl_batch->pkt_entry[i];
			cbe->cntrl = hflit[0];
			cbe->len = hflit[1];
			cbe->addr = htobe64(segs[i].ds_addr);
		}

		wr_hi = htonl(F_WR_SOP | F_WR_EOP | V_WR_DATATYPE(1) |
		    V_WR_SGLSFLT(flits)) |
		    htonl(V_WR_OP(FW_WROPCODE_TUNNEL_TX_PKT) | txqs.compl);
		wr_lo = htonl(V_WR_LEN(flits) |
		    V_WR_GEN(txqs.gen)) | htonl(V_WR_TID(txq->token));
		set_wr_hdr(wrp, wr_hi, wr_lo);
		wmb();
		wr_gen2(txd, txqs.gen);
		if (pkt->ipi_m != NULL) {
			pkt->ipi_pidx = (pkt->ipi_pidx + 1)&(txq->size-1);
			txd = &txq->desc[pkt->ipi_pidx];
			prefetch(txd);
			goto coalesce_retry;
		}
	} else if (tso_info) {
		uint16_t eth_type;
		struct cpl_tx_pkt_lso *hdr = (struct cpl_tx_pkt_lso *)txd;
		struct ether_header *eh;
		void *l3hdr;
		struct tcphdr *tcp;

		txd->flit[2] = 0;
		GET_VTAG(cntrl, m0);
		cntrl |= V_TXPKT_OPCODE(CPL_TX_PKT_LSO);
		hdr->cntrl = htonl(cntrl);
		hdr->len = htonl(mlen | 0x80000000);

		if (__predict_false(mlen < TCPPKTHDRSIZE)) {
			printf("mbuf=%p,len=%d,tso_segsz=%d,csum_flags=%b,flags=%#x",
			    m0, mlen, m0->m_pkthdr.tso_segsz,
			    (int)m0->m_pkthdr.csum_flags, CSUM_BITS, m0->m_flags);
			panic("tx tso packet too small");
		}

		/* Make sure that ether, ip, tcp headers are all in m0 */
		if (__predict_false(m0->m_len < TCPPKTHDRSIZE)) {
			m0 = m_pullup(m0, TCPPKTHDRSIZE);
			if (__predict_false(m0 == NULL)) {
				/* XXX panic probably an overreaction */
				panic("couldn't fit header into mbuf");
			}
		}

		eh = mtod(m0, struct ether_header *);
		eth_type = eh->ether_type;
		if (eth_type == htons(ETHERTYPE_VLAN)) {
			struct ether_vlan_header *evh = (void *)eh;

			tso_info |= V_LSO_ETH_TYPE(CPL_ETH_II_VLAN);
			l3hdr = evh + 1;
			eth_type = evh->evl_proto;
		} else {
			tso_info |= V_LSO_ETH_TYPE(CPL_ETH_II);
			l3hdr = eh + 1;
		}

		if (eth_type == htons(ETHERTYPE_IP)) {
			struct ip *ip = l3hdr;

			tso_info |= V_LSO_IPHDR_WORDS(ip->ip_hl);
			tcp = (struct tcphdr *)(ip + 1);
		} else if (eth_type == htons(ETHERTYPE_IPV6)) {
			struct ip6_hdr *ip6 = l3hdr;

			KASSERT(ip6->ip6_nxt == IPPROTO_TCP,
			    ("%s: CSUM_TSO with ip6_nxt %d",
			    __func__, ip6->ip6_nxt));

			tso_info |= F_LSO_IPV6;
			tso_info |= V_LSO_IPHDR_WORDS(sizeof(*ip6) >> 2);
			tcp = (struct tcphdr *)(ip6 + 1);
		} else
			panic("%s: CSUM_TSO but neither ip nor ip6", __func__);

		tso_info |= V_LSO_TCPHDR_WORDS(tcp->th_off);
		hdr->lso_info = htonl(tso_info);

		if (__predict_false(mlen <= PIO_LEN)) {
			/*
			 * pkt not undersized but fits in PIO_LEN
			 * Indicates a TSO bug at the higher levels.
			 */
			m_copydata(m0, 0, mlen, (caddr_t)&txd->flit[3]);
			flits = (mlen + 7) / 8 + 3;
			wr_hi = htonl(V_WR_BCNTLFLT(mlen & 7) |
					  V_WR_OP(FW_WROPCODE_TUNNEL_TX_PKT) |
					  F_WR_SOP | F_WR_EOP | txqs.compl);
			wr_lo = htonl(V_WR_LEN(flits) |
			    V_WR_GEN(txqs.gen) | V_WR_TID(txq->token));
			set_wr_hdr(&hdr->wr, wr_hi, wr_lo);
			wmb();
			ETHER_BPF_MTAP(pi->ifp, m0);
			wr_gen2(txd, txqs.gen);
			m_freem(m0);
			/* tell caller it's been freed */
			pkt->ipi_m = NULL;
			pkt->ipi_ndescs = 0;
			pkt->ipi_new_pidx = (pkt->ipi_pidx + 1) & (txq->count-1);
			return (0);
		}
		flits = 3;	
	} else {
		struct cpl_tx_pkt *cpl = (struct cpl_tx_pkt *)txd;
		
		GET_VTAG(cntrl, m0);
		cntrl |= V_TXPKT_OPCODE(CPL_TX_PKT);
		if (__predict_false(!(m0->m_pkthdr.csum_flags & CSUM_IP)))
			cntrl |= F_TXPKT_IPCSUM_DIS;
		if (__predict_false(!(m0->m_pkthdr.csum_flags & (CSUM_TCP |
		    CSUM_UDP | CSUM_UDP_IPV6 | CSUM_TCP_IPV6))))
			cntrl |= F_TXPKT_L4CSUM_DIS;
		cpl->cntrl = htonl(cntrl);
		cpl->len = htonl(mlen | 0x80000000);

		if (mlen <= PIO_LEN) {
			m_copydata(m0, 0, mlen, (caddr_t)&txd->flit[2]);
			flits = (mlen + 7) / 8 + 2;
			
			wr_hi = htonl(V_WR_BCNTLFLT(mlen & 7) |
			    V_WR_OP(FW_WROPCODE_TUNNEL_TX_PKT) |
					  F_WR_SOP | F_WR_EOP | txqs.compl);
			wr_lo = htonl(V_WR_LEN(flits) |
			    V_WR_GEN(txqs.gen) | V_WR_TID(txq->token));
			set_wr_hdr(&cpl->wr, wr_hi, wr_lo);
			wmb();
			ETHER_BPF_MTAP(pi->ifp, m0);
			wr_gen2(txd, txqs.gen);
			m_freem(m0);
			/* tell caller it's been freed */
			pkt->ipi_m = NULL;
			pkt->ipi_new_pidx = (pkt->ipi_pidx + 1) & (txq->count-1);
			pkt->ipi_ndescs = 0;
			return (0);
		}
		flits = 2;
	}
	wrp = (struct work_request_hdr *)txd;
	sgp = (ndesc == 1) ? (struct sg_ent *)&txd->flit[flits] : sgl;
	make_sgl(sgp, segs, nsegs);

	sgl_flits = sgl_len(nsegs);

	KASSERT(ndesc <= 4, ("ndesc too large %d", ndesc));
	wr_hi = htonl(V_WR_OP(FW_WROPCODE_TUNNEL_TX_PKT) | txqs.compl);
	wr_lo = htonl(V_WR_TID(txq->token));
	write_wr_hdr_sgl(ndesc, txd, &txqs, txq, sgl, flits,
	    sgl_flits, wr_hi, wr_lo);

	return (0);
}

static void
cxgb_txd_flush(if_shared_ctx_t sctx, uint16_t qsidx, uint32_t pidx __unused)
{
	adapter_t *adap = DOWNCAST(sctx);
	struct sge_txq *txq = &adap->qs[qsidx].txq[TXQ_ETH];

	check_ring_tx_db(adap, txq, 1);
}

void
cxgb_tx_watchdog(void *arg)
{
	struct sge_qset *qs = arg;
	struct sge_txq *txq = &qs->txq[TXQ_ETH];

	if (qs->coalescing != 0 &&
	    (txq->in_use <= cxgb_tx_coalesce_enable_stop) &&
	    TXQ_RING_EMPTY(qs))
		qs->coalescing = 0;
	else if (qs->coalescing == 0 &&
			 (txq->in_use >= cxgb_tx_coalesce_enable_start))
		qs->coalescing = 1;
}

static void
cxgb_tx_timeout(void *arg)
{
	struct sge_qset *qs = arg;
	struct sge_txq *txq = &qs->txq[TXQ_ETH];

	if (qs->coalescing == 0 && (txq->in_use >= (txq->size>>3)))
                qs->coalescing = 1;	
	if (TXQ_TRYLOCK(qs)) {
		qs->qs_flags |= QS_TIMEOUT;
		cxgb_start_locked(qs);
		qs->qs_flags &= ~QS_TIMEOUT;
		TXQ_UNLOCK(qs);
	}
}

/**
 *	write_imm - write a packet into a Tx descriptor as immediate data
 *	@d: the Tx descriptor to write
 *	@m: the packet
 *	@len: the length of packet data to write as immediate data
 *	@gen: the generation bit value to write
 *
 *	Writes a packet as immediate data into a Tx descriptor.  The packet
 *	contains a work request at its beginning.  We must write the packet
 *	carefully so the SGE doesn't read accidentally before it's written in
 *	its entirety.
 */
static __inline void
write_imm(struct tx_desc *d, caddr_t src,
	  unsigned int len, unsigned int gen)
{
	struct work_request_hdr *from = (struct work_request_hdr *)src;
	struct work_request_hdr *to = (struct work_request_hdr *)d;
	uint32_t wr_hi, wr_lo;

	KASSERT(len <= WR_LEN && len >= sizeof(*from),
	    ("%s: invalid len %d", __func__, len));
	
	memcpy(&to[1], &from[1], len - sizeof(*from));
	wr_hi = from->wrh_hi | htonl(F_WR_SOP | F_WR_EOP |
	    V_WR_BCNTLFLT(len & 7));
	wr_lo = from->wrh_lo | htonl(V_WR_GEN(gen) | V_WR_LEN((len + 7) / 8));
	set_wr_hdr(to, wr_hi, wr_lo);
	wmb();
	wr_gen2(d, gen);
}

/**
 *	check_desc_avail - check descriptor availability on a send queue
 *	@adap: the adapter
 *	@q: the TX queue
 *	@m: the packet needing the descriptors
 *	@ndesc: the number of Tx descriptors needed
 *	@qid: the Tx queue number in its queue set (TXQ_OFLD or TXQ_CTRL)
 *
 *	Checks if the requested number of Tx descriptors is available on an
 *	SGE send queue.  If the queue is already suspended or not enough
 *	descriptors are available the packet is queued for later transmission.
 *	Must be called with the Tx queue locked.
 *
 *	Returns 0 if enough descriptors are available, 1 if there aren't
 *	enough descriptors and the packet has been queued, and 2 if the caller
 *	needs to retry because there weren't enough descriptors at the
 *	beginning of the call but some freed up in the mean time.
 */
static __inline int
check_desc_avail(adapter_t *adap, struct sge_txq *q,
		 struct mbuf *m, unsigned int ndesc,
		 unsigned int qid)
{
	/* 
	 * XXX We currently only use this for checking the control queue
	 * the control queue is only used for binding qsets which happens
	 * at init time so we are guaranteed enough descriptors
	 */
	if (__predict_false(!mbufq_empty(&q->sendq))) {
addq_exit:	mbufq_tail(&q->sendq, m);
		return 1;
	}
	if (__predict_false(q->size - q->in_use < ndesc)) {

		struct sge_qset *qs = txq_to_qset(q, qid);

		setbit(&qs->txq_stopped, qid);
		if (should_restart_tx(q) &&
		    test_and_clear_bit(qid, &qs->txq_stopped))
			return 2;

		q->stops++;
		goto addq_exit;
	}
	return 0;
}


/**
 *	reclaim_completed_tx_imm - reclaim completed control-queue Tx descs
 *	@q: the SGE control Tx queue
 *
 *	This is a variant of reclaim_completed_tx() that is used for Tx queues
 *	that send only immediate data (presently just the control queues) and
 *	thus do not have any mbufs
 */
static __inline void
reclaim_completed_tx_imm(struct sge_txq *q)
{
	unsigned int reclaim = q->processed - q->cleaned;

	q->in_use -= reclaim;
	q->cleaned += reclaim;
}

/**
 *	ctrl_xmit - send a packet through an SGE control Tx queue
 *	@adap: the adapter
 *	@q: the control queue
 *	@m: the packet
 *
 *	Send a packet through an SGE control Tx queue.  Packets sent through
 *	a control queue must fit entirely as immediate data in a single Tx
 *	descriptor and have no page fragments.
 */
static int
ctrl_xmit(adapter_t *adap, struct sge_qset *qs, struct mbuf *m)
{
	int ret;
	struct work_request_hdr *wrp = mtod(m, struct work_request_hdr *);
	struct sge_txq *q = &qs->txq[TXQ_CTRL];
	
	KASSERT(m->m_len <= WR_LEN, ("%s: bad tx data", __func__));

	wrp->wrh_hi |= htonl(F_WR_SOP | F_WR_EOP);
	wrp->wrh_lo = htonl(V_WR_TID(q->token));

	TXQ_LOCK(qs);
again:	reclaim_completed_tx_imm(q);

	ret = check_desc_avail(adap, q, m, 1, TXQ_CTRL);
	if (__predict_false(ret)) {
		if (ret == 1) {
			TXQ_UNLOCK(qs);
			return (ENOSPC);
		}
		goto again;
	}
	write_imm(&q->desc[q->pidx], m->m_data, m->m_len, q->gen);
	
	q->in_use++;
	if (++q->pidx >= q->size) {
		q->pidx = 0;
		q->gen ^= 1;
	}
	TXQ_UNLOCK(qs);
	wmb();
	t3_write_reg(adap, A_SG_KDOORBELL,
	    F_SELEGRCNTX | V_EGRCNTX(q->cntxt_id));

	m_free(m);
	return (0);
}


/**
 *	restart_ctrlq - restart a suspended control queue
 *	@qs: the queue set cotaining the control queue
 *
 *	Resumes transmission on a suspended Tx control queue.
 */
static void
restart_ctrlq(void *data, int npending)
{
	struct mbuf *m;
	struct sge_qset *qs = (struct sge_qset *)data;
	struct sge_txq *q = &qs->txq[TXQ_CTRL];
	adapter_t *adap = qs->port->adapter;

	TXQ_LOCK(qs);
again:	reclaim_completed_tx_imm(q);

	while (q->in_use < q->size &&
	       (m = mbufq_dequeue(&q->sendq)) != NULL) {

		write_imm(&q->desc[q->pidx], m->m_data, m->m_len, q->gen);
		m_free(m);

		if (++q->pidx >= q->size) {
			q->pidx = 0;
			q->gen ^= 1;
		}
		q->in_use++;
	}
	if (!mbufq_empty(&q->sendq)) {
		setbit(&qs->txq_stopped, TXQ_CTRL);

		if (should_restart_tx(q) &&
		    test_and_clear_bit(TXQ_CTRL, &qs->txq_stopped))
			goto again;
		q->stops++;
	}
	TXQ_UNLOCK(qs);
	t3_write_reg(adap, A_SG_KDOORBELL,
		     F_SELEGRCNTX | V_EGRCNTX(q->cntxt_id));
}


/*
 * Send a management message through control queue 0
 */
int
t3_mgmt_tx(struct adapter *adap, struct mbuf *m)
{
	return ctrl_xmit(adap, &adap->sge.qs[0], m);
}

/**
 *	free_qset - free the resources of an SGE queue set
 *	@sc: the controller owning the queue set
 *	@q: the queue set
 *
 *	Release the HW and SW resources associated with an SGE queue set, such
 *	as HW contexts, packet buffers, and descriptor rings.  Traffic to the
 *	queue set must be quiesced prior to calling this.
 */
static void
t3_free_qset(adapter_t *sc, struct sge_qset *q)
{
	int i;
	
	reclaim_completed_tx(q, 0, TXQ_ETH);
	if (q->txq[TXQ_ETH].txq_mr != NULL) 
		buf_ring_free(q->txq[TXQ_ETH].txq_mr, M_DEVBUF);
	if (q->txq[TXQ_ETH].txq_ifq != NULL) {
		ifq_delete(q->txq[TXQ_ETH].txq_ifq);
		free(q->txq[TXQ_ETH].txq_ifq, M_DEVBUF);
	}

	for (i = 0; i < SGE_RXQ_PER_SET; ++i) {
		if (q->fl[i].desc) {
			mtx_lock_spin(&sc->sge.reg_lock);
			t3_sge_disable_fl(sc, q->fl[i].cntxt_id);
			mtx_unlock_spin(&sc->sge.reg_lock);
			bus_dmamap_unload(q->fl[i].desc_tag, q->fl[i].desc_map);
			bus_dmamem_free(q->fl[i].desc_tag, q->fl[i].desc,
					q->fl[i].desc_map);
			bus_dma_tag_destroy(q->fl[i].desc_tag);
			bus_dma_tag_destroy(q->fl[i].entry_tag);
		}
		if (q->fl[i].sdesc) {
			free_rx_bufs(sc, &q->fl[i]);
			free(q->fl[i].sdesc, M_DEVBUF);
		}
	}

	mtx_unlock(&q->lock);
	MTX_DESTROY(&q->lock);
	for (i = 0; i < SGE_TXQ_PER_SET; i++) {
		if (q->txq[i].desc) {
			mtx_lock_spin(&sc->sge.reg_lock);
			t3_sge_enable_ecntxt(sc, q->txq[i].cntxt_id, 0);
			mtx_unlock_spin(&sc->sge.reg_lock);
			bus_dmamap_unload(q->txq[i].desc_tag,
					q->txq[i].desc_map);
			bus_dmamem_free(q->txq[i].desc_tag, q->txq[i].desc,
					q->txq[i].desc_map);
			bus_dma_tag_destroy(q->txq[i].desc_tag);
			bus_dma_tag_destroy(q->txq[i].entry_tag);
		}
		if (q->txq[i].sdesc) {
			free(q->txq[i].sdesc, M_DEVBUF);
		}
	}

	if (q->rspq.desc) {
		mtx_lock_spin(&sc->sge.reg_lock);
		t3_sge_disable_rspcntxt(sc, q->rspq.cntxt_id);
		mtx_unlock_spin(&sc->sge.reg_lock);
		
		bus_dmamap_unload(q->rspq.desc_tag, q->rspq.desc_map);
		bus_dmamem_free(q->rspq.desc_tag, q->rspq.desc,
			        q->rspq.desc_map);
		bus_dma_tag_destroy(q->rspq.desc_tag);
		MTX_DESTROY(&q->rspq.lock);
	}

	bzero(q, sizeof(*q));
}

/**
 *	t3_free_sge_resources - free SGE resources
 *	@sc: the adapter softc
 *
 *	Frees resources used by the SGE queue sets.
 */
void
t3_free_sge_resources(adapter_t *sc, int nqsets)
{
	int i;

	for (i = 0; i < nqsets; ++i) {
		TXQ_LOCK(&sc->sge.qs[i]);
		t3_free_qset(sc, &sc->sge.qs[i]);
	}
}


void
t3_free_port_sge_resources(struct port_info *pi)
{
	struct sge_qset *qs = pi->qs;
	int i;

	for (i = 0; i < pi->nqsets; ++i, qs++) {
		t3_free_qset(pi->adapter, qs);
	}
}

/**
 *	t3_sge_start - enable SGE
 *	@sc: the controller softc
 *
 *	Enables the SGE for DMAs.  This is the last step in starting packet
 *	transfers.
 */
void
t3_sge_start(adapter_t *sc)
{
	t3_set_reg_field(sc, A_SG_CONTROL, F_GLOBALENABLE, F_GLOBALENABLE);
}

/**
 *	t3_sge_stop - disable SGE operation
 *	@sc: the adapter
 *
 *	Disables the DMA engine.  This can be called in emeregencies (e.g.,
 *	from error interrupts) or from normal process context.  In the latter
 *	case it also disables any pending queue restart tasklets.  Note that
 *	if it is called in interrupt context it cannot disable the restart
 *	tasklets as it cannot wait, however the tasklets will have no effect
 *	since the doorbells are disabled and the driver will call this again
 *	later from process context, at which time the tasklets will be stopped
 *	if they are still running.
 */
void
t3_sge_stop(adapter_t *sc)
{
	int i, nqsets;
	
	t3_set_reg_field(sc, A_SG_CONTROL, F_GLOBALENABLE, 0);

	if (sc->tq == NULL)
		return;
	
	for (nqsets = i = 0; i < (sc)->params.nports; i++) 
		nqsets += sc->port[i].nqsets;
#ifdef notyet
	/*
	 * 
	 * XXX
	 */
	for (i = 0; i < nqsets; ++i) {
		struct sge_qset *qs = &sc->sge.qs[i];
		
		taskqueue_drain(sc->tq, &qs->txq[TXQ_OFLD].qresume_task);
		taskqueue_drain(sc->tq, &qs->txq[TXQ_CTRL].qresume_task);
	}
#endif
}

#define RSPD_GTS_MASK  (F_RSPD_TXQ0_GTS | F_RSPD_TXQ1_GTS)
#define RSPD_CTRL_MASK (RSPD_GTS_MASK | \
			V_RSPD_TXQ0_CR(M_RSPD_TXQ0_CR) | \
			V_RSPD_TXQ1_CR(M_RSPD_TXQ1_CR) | \
			V_RSPD_TXQ2_CR(M_RSPD_TXQ2_CR))

/* How long to delay the next interrupt in case of memory shortage, in 0.1us. */
#define NOMEM_INTR_DELAY 2500

#ifdef TCP_OFFLOAD
/**
 *	write_ofld_wr - write an offload work request
 *	@adap: the adapter
 *	@m: the packet to send
 *	@q: the Tx queue
 *	@pidx: index of the first Tx descriptor to write
 *	@gen: the generation value to use
 *	@ndesc: number of descriptors the packet will occupy
 *
 *	Write an offload work request to send the supplied packet.  The packet
 *	data already carry the work request with most fields populated.
 */
static void
write_ofld_wr(adapter_t *adap, struct mbuf *m, struct sge_txq *q,
    unsigned int pidx, unsigned int gen, unsigned int ndesc)
{
	unsigned int sgl_flits, flits;
	int i, idx, nsegs, wrlen;
	struct work_request_hdr *from;
	struct sg_ent *sgp, t3sgl[TX_MAX_SEGS / 2 + 1];
	struct tx_desc *d = &q->desc[pidx];
	struct txq_state txqs;
	struct sglist_seg *segs;
	struct ofld_hdr *oh = mtod(m, struct ofld_hdr *);
	struct sglist *sgl;

	from = (void *)(oh + 1);	/* Start of WR within mbuf */
	wrlen = m->m_len - sizeof(*oh);

	if (!(oh->flags & F_HDR_SGL)) {
		write_imm(d, (caddr_t)from, wrlen, gen);

		/*
		 * mbuf with "real" immediate tx data will be enqueue_wr'd by
		 * t3_push_frames and freed in wr_ack.  Others, like those sent
		 * down by close_conn, t3_send_reset, etc. should be freed here.
		 */
		if (!(oh->flags & F_HDR_DF))
			m_free(m);
		return;
	}

	memcpy(&d->flit[1], &from[1], wrlen - sizeof(*from));

	sgl = oh->sgl;
	flits = wrlen / 8;
	sgp = (ndesc == 1) ? (struct sg_ent *)&d->flit[flits] : t3sgl;

	nsegs = sgl->sg_nseg;
	segs = sgl->sg_segs;
	for (idx = 0, i = 0; i < nsegs; i++) {
		KASSERT(segs[i].ss_len, ("%s: 0 len in sgl", __func__));
		if (i && idx == 0) 
			++sgp;
		sgp->len[idx] = htobe32(segs[i].ss_len);
		sgp->addr[idx] = htobe64(segs[i].ss_paddr);
		idx ^= 1;
	}
	if (idx) {
		sgp->len[idx] = 0;
		sgp->addr[idx] = 0;
	}

	sgl_flits = sgl_len(nsegs);
	txqs.gen = gen;
	txqs.pidx = pidx;
	txqs.compl = 0;

	write_wr_hdr_sgl(ndesc, d, &txqs, q, t3sgl, flits, sgl_flits,
	    from->wrh_hi, from->wrh_lo);
}

/**
 *	ofld_xmit - send a packet through an offload queue
 *	@adap: the adapter
 *	@q: the Tx offload queue
 *	@m: the packet
 *
 *	Send an offload packet through an SGE offload queue.
 */
static int
ofld_xmit(adapter_t *adap, struct sge_qset *qs, struct mbuf *m)
{
	int ret;
	unsigned int ndesc;
	unsigned int pidx, gen;
	struct sge_txq *q = &qs->txq[TXQ_OFLD];
	struct ofld_hdr *oh = mtod(m, struct ofld_hdr *);

	ndesc = G_HDR_NDESC(oh->flags);

	TXQ_LOCK(qs);
again:	reclaim_completed_tx(qs, 16, TXQ_OFLD);
	ret = check_desc_avail(adap, q, m, ndesc, TXQ_OFLD);
	if (__predict_false(ret)) {
		if (ret == 1) {
			TXQ_UNLOCK(qs);
			return (EINTR);
		}
		goto again;
	}

	gen = q->gen;
	q->in_use += ndesc;
	pidx = q->pidx;
	q->pidx += ndesc;
	if (q->pidx >= q->size) {
		q->pidx -= q->size;
		q->gen ^= 1;
	}

	write_ofld_wr(adap, m, q, pidx, gen, ndesc);
	check_ring_tx_db(adap, q, 1);
	TXQ_UNLOCK(qs);

	return (0);
}

/**
 *	restart_offloadq - restart a suspended offload queue
 *	@qs: the queue set cotaining the offload queue
 *
 *	Resumes transmission on a suspended Tx offload queue.
 */
static void
restart_offloadq(void *data, int npending)
{
	struct mbuf *m;
	struct sge_qset *qs = data;
	struct sge_txq *q = &qs->txq[TXQ_OFLD];
	adapter_t *adap = qs->port->adapter;
	int cleaned;
		
	TXQ_LOCK(qs);
again:	cleaned = reclaim_completed_tx(qs, 16, TXQ_OFLD);

	while ((m = mbufq_peek(&q->sendq)) != NULL) {
		unsigned int gen, pidx;
		struct ofld_hdr *oh = mtod(m, struct ofld_hdr *);
		unsigned int ndesc = G_HDR_NDESC(oh->flags);

		if (__predict_false(q->size - q->in_use < ndesc)) {
			setbit(&qs->txq_stopped, TXQ_OFLD);
			if (should_restart_tx(q) &&
			    test_and_clear_bit(TXQ_OFLD, &qs->txq_stopped))
				goto again;
			q->stops++;
			break;
		}

		gen = q->gen;
		q->in_use += ndesc;
		pidx = q->pidx;
		q->pidx += ndesc;
		if (q->pidx >= q->size) {
			q->pidx -= q->size;
			q->gen ^= 1;
		}
		
		(void)mbufq_dequeue(&q->sendq);
		TXQ_UNLOCK(qs);
		write_ofld_wr(adap, m, q, pidx, gen, ndesc);
		TXQ_LOCK(qs);
	}
#if USE_GTS
	set_bit(TXQ_RUNNING, &q->flags);
	set_bit(TXQ_LAST_PKT_DB, &q->flags);
#endif
	TXQ_UNLOCK(qs);
	wmb();
	t3_write_reg(adap, A_SG_KDOORBELL,
		     F_SELEGRCNTX | V_EGRCNTX(q->cntxt_id));
}

/**
 *	t3_offload_tx - send an offload packet
 *	@m: the packet
 *
 *	Sends an offload packet.  We use the packet priority to select the
 *	appropriate Tx queue as follows: bit 0 indicates whether the packet
 *	should be sent as regular or control, bits 1-3 select the queue set.
 */
int
t3_offload_tx(struct adapter *sc, struct mbuf *m)
{
	struct ofld_hdr *oh = mtod(m, struct ofld_hdr *);
	struct sge_qset *qs = &sc->sge.qs[G_HDR_QSET(oh->flags)];

	if (oh->flags & F_HDR_CTRL) {
		m_adj(m, sizeof (*oh));	/* trim ofld_hdr off */
		return (ctrl_xmit(sc, qs, m));
	} else
		return (ofld_xmit(sc, qs, m));
}
#endif

static void
restart_tx(struct sge_qset *qs)
{
	struct adapter *sc = qs->port->adapter;

	if (isset(&qs->txq_stopped, TXQ_OFLD) &&
	    should_restart_tx(&qs->txq[TXQ_OFLD]) &&
	    test_and_clear_bit(TXQ_OFLD, &qs->txq_stopped)) {
		qs->txq[TXQ_OFLD].restarts++;
		taskqueue_enqueue(sc->tq, &qs->txq[TXQ_OFLD].qresume_task);
	}

	if (isset(&qs->txq_stopped, TXQ_CTRL) &&
	    should_restart_tx(&qs->txq[TXQ_CTRL]) &&
	    test_and_clear_bit(TXQ_CTRL, &qs->txq_stopped)) {
		qs->txq[TXQ_CTRL].restarts++;
		taskqueue_enqueue(sc->tq, &qs->txq[TXQ_CTRL].qresume_task);
	}
}

/**
 *	t3_sge_alloc_qset - initialize an SGE queue set
 *	@sc: the controller softc
 *	@id: the queue set id
 *	@nports: how many Ethernet ports will be using this queue set
 *	@irq_vec_idx: the IRQ vector index for response queue interrupts
 *	@p: configuration parameters for this queue set
 *	@ntxq: number of Tx queues for the queue set
 *	@pi: port info for queue set
 *
 *	Allocate resources and initialize an SGE queue set.  A queue set
 *	comprises a response queue, two Rx free-buffer queues, and up to 3
 *	Tx queues.  The Tx queues are assigned roles in the order Ethernet
 *	queue, offload queue, and control queue.
 */
int
t3_sge_alloc_qset(adapter_t *sc, struct sge_qset *qs, uint8_t id, int nports, int irq_vec_idx,
		  const struct qset_params *p, int ntxq, struct port_info *pi)
{
	int i, addridx, ret = 0;
	caddr_t vaddrs[6];
	uint64_t paddrs[6];

	q->port = pi;
	q->adap = sc;

	init_qset_cntxt(q, pi->first_qset + id);
	q->idx = id;
	if ((ret = iflib_qset_addr_get(sctx, id, vaddrs, paddrs, ntxq + 3)))
		goto err;

	/*
   * Iflib expects the txq and rx completion queue to come first
   *
   */
	q->txq[0],phys_addr = paddrs[0];
	q->txq[0].desc = vaddrs[0];
	q->rspq.phys_addr = paddrs[1];
	q->rspq.desc = vaddrs[1];
	addridx = 2
	q->fl[0].phys_addr = paddrs[addridx];
	q->fl[0].desc = vaddrs[addridx];
	addridx++;
	q->fl[1].phys_addr = paddrs[addridx];
	q->fl[1].desc = vaddrs[addridx];
	addridx++;
	/* do ULP last */
	for (i = 1; i < ntxq ; i++, addridx++) {
		q->txq[i],phys_addr = paddrs[addridx];
		q->txq[i].desc = vaddrs[addridx];
	}

	if ((ret = alloc_sw_ring(sc, p->fl_size, &q->fl[0].sdesc)) != 0) {
		printf("error %d from alloc ring fl0\n", ret);
		goto err;
	}

	if ((ret = alloc_sw_ring(sc, p->jumbo_size, sizeof(struct rx_sw_desc),  &q->fl[1].sdesc))) {
		printf("error %d from alloc ring fl1\n", ret);
		goto err;
	}

	snprintf(q->rspq.lockbuf, RSPQ_NAME_LEN, "t3 rspq lock %d:%d",
	    device_get_unit(sc->dev), irq_vec_idx);
	MTX_INIT(&q->rspq.lock, q->rspq.lockbuf, NULL, MTX_DEF);

	for (i = 0; i < ntxq; ++i) {
		mbufq_init(&q->txq[i].sendq);
		q->txq[i].gen = 1;
		q->txq[i].size = p->txq_size[i];
	}

#ifdef TCP_OFFLOAD
	TASK_INIT(&q->txq[TXQ_OFLD].qresume_task, 0, restart_offloadq, q);
#endif
	TASK_INIT(&q->txq[TXQ_CTRL].qresume_task, 0, restart_ctrlq, q);
	TASK_INIT(&q->txq[TXQ_ETH].qreclaim_task, 0, sge_txq_reclaim_handler, q);
	TASK_INIT(&q->txq[TXQ_OFLD].qreclaim_task, 0, sge_txq_reclaim_handler, q);

	q->fl[0].gen = q->fl[1].gen = 1;
	q->fl[0].size = p->fl_size;
	q->fl[1].size = p->jumbo_size;

	q->rspq.gen = 1;
	q->rspq.cidx = 0;
	q->rspq.size = p->rspq_size;

	q->txq[TXQ_ETH].stop_thres = nports *
	    flits_to_desc(sgl_len(TX_MAX_SEGS + 1) + 3);

	q->fl[0].buf_size = MCLBYTES;
	q->fl[1].buf_size = p->jumbo_buf_size;
	/*
   * XXX Still need to communicate multiple rx buffer sizes to iflib
   */

	mtx_lock_spin(&sc->sge.reg_lock);
	ret = -t3_sge_init_rspcntxt(sc, q->rspq.cntxt_id, irq_vec_idx,
				   q->rspq.phys_addr, q->rspq.size,
				   q->fl[0].buf_size, 1, 0);
	if (ret) {
		printf("error %d from t3_sge_init_rspcntxt\n", ret);
		goto err_unlock;
	}

	for (i = 0; i < SGE_RXQ_PER_SET; ++i) {
		ret = -t3_sge_init_flcntxt(sc, q->fl[i].cntxt_id, 0,
					  q->fl[i].phys_addr, q->fl[i].size,
					  q->fl[i].buf_size, p->cong_thres, 1,
					  0);
		if (ret) {
			printf("error %d from t3_sge_init_flcntxt for index i=%d\n", ret, i);
			goto err_unlock;
		}
	}

	ret = -t3_sge_init_ecntxt(sc, q->txq[TXQ_ETH].cntxt_id, USE_GTS,
				 SGE_CNTXT_ETH, id, q->txq[TXQ_ETH].phys_addr,
				 q->txq[TXQ_ETH].size, q->txq[TXQ_ETH].token,
				 1, 0);
	if (ret) {
		printf("error %d from t3_sge_init_ecntxt\n", ret);
		goto err_unlock;
	}

	if (ntxq > 1) {
		ret = -t3_sge_init_ecntxt(sc, q->txq[TXQ_OFLD].cntxt_id,
					 USE_GTS, SGE_CNTXT_OFLD, id,
					 q->txq[TXQ_OFLD].phys_addr,
					 q->txq[TXQ_OFLD].size, 0, 1, 0);
		if (ret) {
			printf("error %d from t3_sge_init_ecntxt\n", ret);
			goto err_unlock;
		}
	}

	if (ntxq > 2) {
		ret = -t3_sge_init_ecntxt(sc, q->txq[TXQ_CTRL].cntxt_id, 0,
					 SGE_CNTXT_CTRL, id,
					 q->txq[TXQ_CTRL].phys_addr,
					 q->txq[TXQ_CTRL].size,
					 q->txq[TXQ_CTRL].token, 1, 0);
		if (ret) {
			printf("error %d from t3_sge_init_ecntxt\n", ret);
			goto err_unlock;
		}
	}

	mtx_unlock_spin(&sc->sge.reg_lock);
	t3_update_qset_coalesce(q, p);
	refill_rspq(sc, &q->rspq, q->rspq.size - 1);

	t3_write_reg(sc, A_SG_GTS, V_RSPQ(q->rspq.cntxt_id) |
		     V_NEWTIMER(q->rspq.holdoff_tmr));

	return (0);

err_unlock:
	mtx_unlock_spin(&sc->sge.reg_lock);
err:	
	TXQ_LOCK(q);
	t3_free_qset(sc, q);

	return (ret);
}

/*
 * Remove CPL_RX_PKT headers from the mbuf and reduce it to a regular mbuf with
 * ethernet data.  Hardware assistance with various checksums and any vlan tag
 * will also be taken into account here.
 */
static void
t3_rx_eth(struct adapter *adap, if_rxd_info_t ri, void *data)
{
	struct cpl_rx_pkt *cpl = (struct cpl_rx_pkt *)((uint8_t *)data + ri->iri_pad);
	struct port_info *pi = &adap->port[adap->rxpkt_map[cpl->iff]];
	struct ifnet *ifp = pi->ifp;

	if (cpl->vlan_valid) {
		ri.iri_vtag = ntohs(cpl->vlan);
		ri.iri_flags |= M_VLANTAG;
	}

	ri.iri_ifp = ifp;
	ri.iri_pad += sizeof(*cpl);
	/*
	 * adjust after conversion to mbuf chain
	 */

	if (!cpl->fragment && cpl->csum_valid && cpl->csum == 0xffff) {
		struct ether_header *eh = mtod(m, void *);
		uint16_t eh_type;

		if (eh->ether_type == htons(ETHERTYPE_VLAN)) {
			struct ether_vlan_header *evh = mtod(m, void *);

			eh_type = evh->evl_proto;
		} else
			eh_type = eh->ether_type;

		if (ifp->if_capenable & IFCAP_RXCSUM &&
		    eh_type == htons(ETHERTYPE_IP)) {
			ri->iri_csum_flags = (CSUM_IP_CHECKED |
			    CSUM_IP_VALID | CSUM_DATA_VALID | CSUM_PSEUDO_HDR);
			ri->iri_csum_data = 0xffff;
		} else if (ifp->if_capenable & IFCAP_RXCSUM_IPV6 &&
		    eh_type == htons(ETHERTYPE_IPV6)) {
			ri->iri_csum_flags = (CSUM_DATA_VALID_IPV6 |
			    CSUM_PSEUDO_HDR);
			ri->iri_csum_data = 0xffff;
		}
	}
}

/**
 *	get_packet - return the next ingress packet buffer from a free list
 *	@adap: the adapter that received the packet
 *	@drop_thres: # of remaining buffers before we start dropping packets
 *	@qs: the qset that the SGE free list holding the packet belongs to
 *      @mh: the mbuf header, contains a pointer to the head and tail of the mbuf chain
 *      @r: response descriptor 
 *
 *	Get the next packet from a free list and complete setup of the
 *	sk_buff.  If the packet is small we make a copy and recycle the
 *	original buffer, otherwise we use the original buffer itself.  If a
 *	positive drop threshold is supplied packets are dropped and their
 *	buffers recycled if (a) the number of remaining buffers is under the
 *	threshold and the packet is too big to copy, or (b) the packet should
 *	be copied but there is no memory for the copy.
 */
static int
get_fragment(struct sge_qset *qs, if_rxd_info_t ri, caddr_t *data, struct rsp_desc *r)
{
	unsigned int len_cq =  ntohl(r->len_cq);
	uint32_t len = G_RSPD_LEN(len_cq);
	uint8_t sopeop = G_RSPD_SOP_EOP(ntohl(r->flags));
	struct sge_fl *fl = (len_cq & F_RSPD_FLQ) ? &qs->fl[1] : &qs->fl[0];
	int mask, cidx = fl->cidx;
	struct rx_sw_desc *sd = &fl->sdesc[cidx];

	mask = fl->size - 1;
	prefetch(fl->sdesc[(cidx + 1) & mask].rxsd_cl);
	prefetch(fl->sdesc[(cidx + 2) & mask].rxsd_cl);

	ri->iri_len = len;
	ri->iri_flidx = fl->idx
	ri->iri_next_offset = 0;
	if ((sopeop == RSPQ_NSOP_NEOP) || (sopeop == RSPQ_SOP))
		ri->iri_next_offset = 1;

	 if (++fl->cidx == fl->size)
		 fl->cidx = 0;

	 *data = sd->rxsd_cl;
	return (ri->iri_next_offset == 0);
}

/**
 *	handle_rsp_cntrl_info - handles control information in a response
 *	@qs: the queue set corresponding to the response
 *	@flags: the response control flags
 *
 *	Handles the control information of an SGE response, such as GTS
 *	indications and completion credits for the queue set's Tx queues.
 *	HW coalesces credits, we don't do any extra SW coalescing.
 */
static __inline void
handle_rsp_cntrl_info(struct sge_qset *qs, uint32_t flags)
{
	unsigned int credits;

#if USE_GTS
	if (flags & F_RSPD_TXQ0_GTS)
		clear_bit(TXQ_RUNNING, &qs->txq[TXQ_ETH].flags);
#endif
	credits = G_RSPD_TXQ0_CR(flags);
	if (credits) {
		qs->txq[TXQ_ETH].processed += credits;
		iflib_tx_credits_update(UPCAST(qs->adap), qs->idx, credits);
	}

	credits = G_RSPD_TXQ2_CR(flags);
	if (credits)
		qs->txq[TXQ_CTRL].processed += credits;

# if USE_GTS
	if (flags & F_RSPD_TXQ1_GTS)
		clear_bit(TXQ_RUNNING, &qs->txq[TXQ_OFLD].flags);
# endif
	credits = G_RSPD_TXQ1_CR(flags);
	if (credits)
		qs->txq[TXQ_OFLD].processed += credits;

}

static void
check_ring_db(adapter_t *adap, struct sge_qset *qs,
    unsigned int sleeping)
{
	;
}

static int
cxgb_isc_rxd_available(if_shared_ctx_t sctx, uint16_t qsidx, uint32_t cidx)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct sge_qset *qs = adapter->qs[qsidx];
	struct sge_rspq *rspq = qs->rspq;
	struct rsp_desc *r;
	int i, ncidx;

	/*
	 * arbitrarily limit ourselves to polling 10 descriptors
	 */
	for (ncidx = cidx, i = 0; i < 10; i++) {
		r = rspq->desc[ncidx];
		if ((r->intr_gen & F_RSPD_GEN2) != rspq->gen)
			break;
		ncidx = (ncidx + 1) & (rspq->size - 1);
	}
	return (i);
}

/**
 *	process_responses - process responses from an SGE response queue
 *	@adap: the adapter
 *	@qs: the queue set to which the response queue belongs
 *	@budget: how many responses can be processed in this round
 *
 *	Process responses from an SGE response queue up to the supplied budget.
 *	Responses include received packets as well as credits and other events
 *	for the queues that belong to the response queue's queue set.
 *	A negative budget is effectively unlimited.
 *
 *	Additionally choose the interrupt holdoff time for the next interrupt
 *	on this queue.  If the system is under memory shortage use a fairly
 *	long delay to help recovery.
 */
static int
cxgb_isc_rxd_pkt_get(if_shared_ctx_t sctx, if_rxd_info_t ri)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct sge_qset *qs = adapter->qs[ri->iri_qsidx];
	struct sge_rspq *rspq = &qs->rspq;
	struct rsp_desc *r = &rspq->desc[ri->iri_cidx];
	unsigned int sleeping = 0;
	struct mbuf *m;
	int eth, eop = 0;
	uint32_t flags = ntohl(r->flags);
	uint32_t rss_hash = be32toh(r->rss_hdr.rss_hash_val);
	uint8_t opcode = r->rss_hdr.opcode;
	caddr_t data = NULL;

#ifdef DEBUG
	static int last_holdoff = 0;
	if (cxgb_debug && rspq->holdoff_tmr != last_holdoff) {
		printf("next_holdoff=%d\n", rspq->holdoff_tmr);
		last_holdoff = rspq->holdoff_tmr;
	}
#endif
	rspq->next_holdoff = rspq->holdoff_tmr;

	eth = (opcode == CPL_RX_PKT);

	if (__predict_false(flags & F_RSPD_ASYNC_NOTIF)) {
		if (cxgb_debug)
			printf("async notification\n");

		if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL)
			goto no_mem;

		memcpy(mtod(m, char *), r, AN_PKT_SIZE);
		m->m_len = m->m_pkthdr.len = AN_PKT_SIZE;
		*mtod(m, char *) = CPL_ASYNC_NOTIF;
		opcode = CPL_ASYNC_NOTIF;
		eop = 1;
		rspq->async_notif++;
		ri->iri_qidx = -1;
		goto skip;
	} else if  (flags & F_RSPD_IMM_DATA_VALID) {
		m = m_gethdr(M_NOWAIT, MT_DATA);

		if (m == NULL) {
		no_mem:
			rspq->next_holdoff = NOMEM_INTR_DELAY;
			return (ENOMEM);
		}

		get_imm_packet(adap, r, m);
		if (eth) {
			t3_rx_eth(adap, ri, mtod(m, uint8_t *), 0);
			ri->iri_pad = 0;
			ri->iri_m = m;
		} else
			ri->iri_qidx = -1;
		eop = 1;
		rspq->imm_data++;
	} else if (r->len_cq) {
		eop = get_fragment(ri, &data, r);
		ri->iri_pad = 2;
	} else {
		rspq->pure_rsps++;
		ri->iri_qidx = -1;
	}
skip:
	if (flags & RSPD_CTRL_MASK) {
		rspq->sleeping |= flags & RSPD_GTS_MASK;
			handle_rsp_cntrl_info(qs, flags);
	}

	if (!eth && eop) {
		rspq->offload_pkts++;
#ifdef TCP_OFFLOAD
		adap->cpl_handler[opcode](qs, r, m);
#else
		m_freem(m);
#endif
	} else if (eth && eop) {
		if (r->rss_hdr.hash_type && !adap->timestamp) {
			ri->iri_flags |= M_FLOWID;
			ri->iri_flowid = rss_hash;
		}
		t3_rx_eth(adap, ri, data);
	}
	if (__predict_false(++cidx == rspq->size)) {
		rspq->gen ^= 1;
		cidx = 0;
	}
	rspq->cidx = cidx;

	if (++rspq->credits >= 64) {
		refill_rspq(adap, rspq, rspq->credits);
		rspq->credits = 0;
	}
}

static void
cxgb_isc_rxd_refill(if_shared_ctx_t sctx, uint16_t qsidx, uint8_t flidx,
					uint32_t pidx, uint64_t *paddrs, caddr_t *vaddrs,
					uint8_t count)
{
	struct adapter *adapter = DOWNCAST(sctx);
	struct sge_qset *qs = adapter->qs[qsidx];
	struct sge_fl *fl = qs->fl[flidx];
	struct rx_desc *d;
	int i, npidx;

	for (npidx = pidx, i = 0; i < count; i++) {
		fl->sdesc[npidx].rxsd_cl = vaddrs[i];
		d = fl->desc[npidx];
		d->addr_lo = htobe32(paddrs[i] & 0xffffffff);
		d->addr_hi = htobe32(((uint64_t)paddrs[i] >>32) & 0xffffffff);
		d->len_gen = htobe32(V_FLD_GEN1(fl->gen));
		d->gen2 = htobe32(V_FLD_GEN2(fl->gen));
		if (++npidx == fl->size) {
			npidx = 0;
			fl->gen ^= 1;
		}
	}
}

static void
cxgb_isc_rxd_flush(if_shared_ctx_t sctx, uint16_t qsidx, uint8_t flidx,
				   uint32_t pidx __unused)
{
	struct adapter *sc = DOWNCAST(sctx);
	struct sge_qset *qs = adapter->qs[qsidx];
	struct sge_fl *fl = qs->fl[flidx];

	if (qs->rspq->sleeping)
		check_ring_db(sc, qs, sleeping);

	t3_write_reg(sc, A_SG_KDOORBELL, V_EGRCNTX(fl->cntxt_id));
}

/*
 * A helper function that processes responses and issues GTS.
 */
static __inline int
process_responses_gts(adapter_t *adap, struct sge_rspq *rq)
{
	int work;
	static int last_holdoff = 0;
	
	work = process_responses(adap, rspq_to_qset(rq), -1);

	if (cxgb_debug && (rq->next_holdoff != last_holdoff)) {
		printf("next_holdoff=%d\n", rq->next_holdoff);
		last_holdoff = rq->next_holdoff;
	}
	t3_write_reg(adap, A_SG_GTS, V_RSPQ(rq->cntxt_id) |
	    V_NEWTIMER(rq->next_holdoff) | V_NEWINDEX(rq->cidx));
	
	return (work);
}


/*
 * Interrupt handler for legacy INTx interrupts for T3B-based cards.
 * Handles data events from SGE response queues as well as error and other
 * async events as they all use the same interrupt pin.  We use one SGE
 * response queue per port in this mode and protect all response queues with
 * queue 0's lock.
 */
void
t3b_intr(void *data)
{
	uint32_t i, map;
	adapter_t *adap = data;
	struct sge_rspq *q0 = &adap->sge.qs[0].rspq;
	
	t3_write_reg(adap, A_PL_CLI, 0);
	map = t3_read_reg(adap, A_SG_DATA_INTR);

	if (!map) 
		return;

	if (__predict_false(map & F_ERRINTR)) {
		t3_write_reg(adap, A_PL_INT_ENABLE0, 0);
		(void) t3_read_reg(adap, A_PL_INT_ENABLE0);
		taskqueue_enqueue(adap->tq, &adap->slow_intr_task);
	}

	mtx_lock(&q0->lock);
	for_each_port(adap, i)
	    if (map & (1 << i))
			process_responses_gts(adap, &adap->sge.qs[i].rspq);
	mtx_unlock(&q0->lock);
}

/*
 * The MSI interrupt handler.  This needs to handle data events from SGE
 * response queues as well as error and other async events as they all use
 * the same MSI vector.  We use one SGE response queue per port in this mode
 * and protect all response queues with queue 0's lock.
 */
void
t3_intr_msi(void *data)
{
	adapter_t *adap = data;
	struct sge_rspq *q0 = &adap->sge.qs[0].rspq;
	int i, new_packets = 0;

	mtx_lock(&q0->lock);

	for_each_port(adap, i)
	    if (process_responses_gts(adap, &adap->sge.qs[i].rspq)) 
		    new_packets = 1;
	mtx_unlock(&q0->lock);
	if (new_packets == 0) {
		t3_write_reg(adap, A_PL_INT_ENABLE0, 0);
		(void) t3_read_reg(adap, A_PL_INT_ENABLE0);
		taskqueue_enqueue(adap->tq, &adap->slow_intr_task);
	}
}

void
t3_intr_msix(void *data)
{
	struct sge_qset *qs = data;
	adapter_t *adap = qs->port->adapter;
	struct sge_rspq *rspq = &qs->rspq;

	if (process_responses_gts(adap, rspq) == 0)
		rspq->unhandled_irqs++;
}

#define QDUMP_SBUF_SIZE		32 * 400
static int
t3_dump_rspq(SYSCTL_HANDLER_ARGS)
{
	struct sge_rspq *rspq;
	struct sge_qset *qs;
	int i, err, dump_end, idx;
	struct sbuf *sb;
	struct rsp_desc *rspd;
	uint32_t data[4];
	
	rspq = arg1;
	qs = rspq_to_qset(rspq);
	if (rspq->rspq_dump_count == 0) 
		return (0);
	if (rspq->rspq_dump_count > RSPQ_Q_SIZE) {
		log(LOG_WARNING,
		    "dump count is too large %d\n", rspq->rspq_dump_count);
		rspq->rspq_dump_count = 0;
		return (EINVAL);
	}
	if (rspq->rspq_dump_start > (RSPQ_Q_SIZE-1)) {
		log(LOG_WARNING,
		    "dump start of %d is greater than queue size\n",
		    rspq->rspq_dump_start);
		rspq->rspq_dump_start = 0;
		return (EINVAL);
	}
	err = t3_sge_read_rspq(qs->port->adapter, rspq->cntxt_id, data);
	if (err)
		return (err);
	err = sysctl_wire_old_buffer(req, 0);
	if (err)
		return (err);
	sb = sbuf_new_for_sysctl(NULL, NULL, QDUMP_SBUF_SIZE, req);

	sbuf_printf(sb, " \n index=%u size=%u MSI-X/RspQ=%u intr enable=%u intr armed=%u\n",
	    (data[0] & 0xffff), data[0] >> 16, ((data[2] >> 20) & 0x3f),
	    ((data[2] >> 26) & 1), ((data[2] >> 27) & 1));
	sbuf_printf(sb, " generation=%u CQ mode=%u FL threshold=%u\n",
	    ((data[2] >> 28) & 1), ((data[2] >> 31) & 1), data[3]);
	
	sbuf_printf(sb, " start=%d -> end=%d\n", rspq->rspq_dump_start,
	    (rspq->rspq_dump_start + rspq->rspq_dump_count) & (RSPQ_Q_SIZE-1));
	
	dump_end = rspq->rspq_dump_start + rspq->rspq_dump_count;
	for (i = rspq->rspq_dump_start; i < dump_end; i++) {
		idx = i & (RSPQ_Q_SIZE-1);
		
		rspd = &rspq->desc[idx];
		sbuf_printf(sb, "\tidx=%04d opcode=%02x cpu_idx=%x hash_type=%x cq_idx=%x\n",
		    idx, rspd->rss_hdr.opcode, rspd->rss_hdr.cpu_idx,
		    rspd->rss_hdr.hash_type, be16toh(rspd->rss_hdr.cq_idx));
		sbuf_printf(sb, "\trss_hash_val=%x flags=%08x len_cq=%x intr_gen=%x\n",
		    rspd->rss_hdr.rss_hash_val, be32toh(rspd->flags),
		    be32toh(rspd->len_cq), rspd->intr_gen);
	}

	err = sbuf_finish(sb);
	/* Output a trailing NUL. */
	if (err == 0)
		err = SYSCTL_OUT(req, "", 1);
	sbuf_delete(sb);
	return (err);
}	

static int
t3_dump_txq_eth(SYSCTL_HANDLER_ARGS)
{
	struct sge_txq *txq;
	struct sge_qset *qs;
	int i, j, err, dump_end;
	struct sbuf *sb;
	struct tx_desc *txd;
	uint32_t *WR, wr_hi, wr_lo, gen;
	uint32_t data[4];
	
	txq = arg1;
	qs = txq_to_qset(txq, TXQ_ETH);
	if (txq->txq_dump_count == 0) {
		return (0);
	}
	if (txq->txq_dump_count > TX_ETH_Q_SIZE) {
		log(LOG_WARNING,
		    "dump count is too large %d\n", txq->txq_dump_count);
		txq->txq_dump_count = 1;
		return (EINVAL);
	}
	if (txq->txq_dump_start > (TX_ETH_Q_SIZE-1)) {
		log(LOG_WARNING,
		    "dump start of %d is greater than queue size\n",
		    txq->txq_dump_start);
		txq->txq_dump_start = 0;
		return (EINVAL);
	}
	err = t3_sge_read_ecntxt(qs->port->adapter, qs->rspq.cntxt_id, data);
	if (err)
		return (err);
	err = sysctl_wire_old_buffer(req, 0);
	if (err)
		return (err);
	sb = sbuf_new_for_sysctl(NULL, NULL, QDUMP_SBUF_SIZE, req);

	sbuf_printf(sb, " \n credits=%u GTS=%u index=%u size=%u rspq#=%u cmdq#=%u\n",
	    (data[0] & 0x7fff), ((data[0] >> 15) & 1), (data[0] >> 16), 
	    (data[1] & 0xffff), ((data[3] >> 4) & 7), ((data[3] >> 7) & 1));
	sbuf_printf(sb, " TUN=%u TOE=%u generation%u uP token=%u valid=%u\n",
	    ((data[3] >> 8) & 1), ((data[3] >> 9) & 1), ((data[3] >> 10) & 1),
	    ((data[3] >> 11) & 0xfffff), ((data[3] >> 31) & 1));
	sbuf_printf(sb, " qid=%d start=%d -> end=%d\n", qs->idx,
	    txq->txq_dump_start,
	    (txq->txq_dump_start + txq->txq_dump_count) & (TX_ETH_Q_SIZE-1));

	dump_end = txq->txq_dump_start + txq->txq_dump_count;
	for (i = txq->txq_dump_start; i < dump_end; i++) {
		txd = &txq->desc[i & (TX_ETH_Q_SIZE-1)];
		WR = (uint32_t *)txd->flit;
		wr_hi = ntohl(WR[0]);
		wr_lo = ntohl(WR[1]);		
		gen = G_WR_GEN(wr_lo);
		
		sbuf_printf(sb," wr_hi %08x wr_lo %08x gen %d\n",
		    wr_hi, wr_lo, gen);
		for (j = 2; j < 30; j += 4) 
			sbuf_printf(sb, "\t%08x %08x %08x %08x \n",
			    WR[j], WR[j + 1], WR[j + 2], WR[j + 3]);

	}
	err = sbuf_finish(sb);
	/* Output a trailing NUL. */
	if (err == 0)
		err = SYSCTL_OUT(req, "", 1);
	sbuf_delete(sb);
	return (err);
}

static int
t3_dump_txq_ctrl(SYSCTL_HANDLER_ARGS)
{
	struct sge_txq *txq;
	struct sge_qset *qs;
	int i, j, err, dump_end;
	struct sbuf *sb;
	struct tx_desc *txd;
	uint32_t *WR, wr_hi, wr_lo, gen;
	
	txq = arg1;
	qs = txq_to_qset(txq, TXQ_CTRL);
	if (txq->txq_dump_count == 0) {
		return (0);
	}
	if (txq->txq_dump_count > 256) {
		log(LOG_WARNING,
		    "dump count is too large %d\n", txq->txq_dump_count);
		txq->txq_dump_count = 1;
		return (EINVAL);
	}
	if (txq->txq_dump_start > 255) {
		log(LOG_WARNING,
		    "dump start of %d is greater than queue size\n",
		    txq->txq_dump_start);
		txq->txq_dump_start = 0;
		return (EINVAL);
	}

	err = sysctl_wire_old_buffer(req, 0);
	if (err != 0)
		return (err);
	sb = sbuf_new_for_sysctl(NULL, NULL, QDUMP_SBUF_SIZE, req);
	sbuf_printf(sb, " qid=%d start=%d -> end=%d\n", qs->idx,
	    txq->txq_dump_start,
	    (txq->txq_dump_start + txq->txq_dump_count) & 255);

	dump_end = txq->txq_dump_start + txq->txq_dump_count;
	for (i = txq->txq_dump_start; i < dump_end; i++) {
		txd = &txq->desc[i & (255)];
		WR = (uint32_t *)txd->flit;
		wr_hi = ntohl(WR[0]);
		wr_lo = ntohl(WR[1]);		
		gen = G_WR_GEN(wr_lo);
		
		sbuf_printf(sb," wr_hi %08x wr_lo %08x gen %d\n",
		    wr_hi, wr_lo, gen);
		for (j = 2; j < 30; j += 4) 
			sbuf_printf(sb, "\t%08x %08x %08x %08x \n",
			    WR[j], WR[j + 1], WR[j + 2], WR[j + 3]);

	}
	err = sbuf_finish(sb);
	/* Output a trailing NUL. */
	if (err == 0)
		err = SYSCTL_OUT(req, "", 1);
	sbuf_delete(sb);
	return (err);
}

static int
t3_set_coalesce_usecs(SYSCTL_HANDLER_ARGS)
{
	adapter_t *sc = arg1;
	struct qset_params *qsp = &sc->params.sge.qset[0]; 
	int coalesce_usecs;	
	struct sge_qset *qs;
	int i, j, err, nqsets = 0;
	struct mtx *lock;

	if ((sc->flags & FULL_INIT_DONE) == 0)
		return (ENXIO);
		
	coalesce_usecs = qsp->coalesce_usecs;
        err = sysctl_handle_int(oidp, &coalesce_usecs, arg2, req);

	if (err != 0) {
		return (err);
	}
	if (coalesce_usecs == qsp->coalesce_usecs)
		return (0);

	for (i = 0; i < sc->params.nports; i++) 
		for (j = 0; j < sc->port[i].nqsets; j++)
			nqsets++;

	coalesce_usecs = max(1, coalesce_usecs);

	for (i = 0; i < nqsets; i++) {
		qs = &sc->sge.qs[i];
		qsp = &sc->params.sge.qset[i];
		qsp->coalesce_usecs = coalesce_usecs;
		
		lock = (sc->flags & USING_MSIX) ? &qs->rspq.lock :
			    &sc->sge.qs[0].rspq.lock;

		mtx_lock(lock);
		t3_update_qset_coalesce(qs, qsp);
		t3_write_reg(sc, A_SG_GTS, V_RSPQ(qs->rspq.cntxt_id) |
		    V_NEWTIMER(qs->rspq.holdoff_tmr));
		mtx_unlock(lock);
	}

	return (0);
}

static int
t3_pkt_timestamp(SYSCTL_HANDLER_ARGS)
{
	adapter_t *sc = arg1;
	int rc, timestamp;

	if ((sc->flags & FULL_INIT_DONE) == 0)
		return (ENXIO);

	timestamp = sc->timestamp;
	rc = sysctl_handle_int(oidp, &timestamp, arg2, req);

	if (rc != 0)
		return (rc);

	if (timestamp != sc->timestamp) {
		t3_set_reg_field(sc, A_TP_PC_CONFIG2, F_ENABLERXPKTTMSTPRSS,
		    timestamp ? F_ENABLERXPKTTMSTPRSS : 0);
		sc->timestamp = timestamp;
	}

	return (0);
}

void
t3_add_attach_sysctls(adapter_t *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *children;

	ctx = device_get_sysctl_ctx(sc->dev);
	children = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	/* random information */
	SYSCTL_ADD_STRING(ctx, children, OID_AUTO, 
	    "firmware_version",
	    CTLFLAG_RD, &sc->fw_version,
	    0, "firmware version");
	SYSCTL_ADD_UINT(ctx, children, OID_AUTO,
	    "hw_revision",
	    CTLFLAG_RD, &sc->params.rev,
	    0, "chip model");
	SYSCTL_ADD_STRING(ctx, children, OID_AUTO, 
	    "port_types",
	    CTLFLAG_RD, &sc->port_types,
	    0, "type of ports");
	SYSCTL_ADD_INT(ctx, children, OID_AUTO, 
	    "enable_debug",
	    CTLFLAG_RW, &cxgb_debug,
	    0, "enable verbose debugging output");
	SYSCTL_ADD_UQUAD(ctx, children, OID_AUTO, "tunq_coalesce",
	    CTLFLAG_RD, &sc->tunq_coalesce,
	    "#tunneled packets freed");
	SYSCTL_ADD_INT(ctx, children, OID_AUTO, 
	    "txq_overrun",
	    CTLFLAG_RD, &txq_fills,
	    0, "#times txq overrun");
	SYSCTL_ADD_UINT(ctx, children, OID_AUTO,
	    "core_clock",
	    CTLFLAG_RD, &sc->params.vpd.cclk,
	    0, "core clock frequency (in KHz)");
}


static const char *rspq_name = "rspq";
static const char *txq_names[] =
{
	"txq_eth",
	"txq_ofld",
	"txq_ctrl"	
};

static int
sysctl_handle_macstat(SYSCTL_HANDLER_ARGS)
{
	struct port_info *p = arg1;
	uint64_t *parg;

	if (!p)
		return (EINVAL);

	parg = (uint64_t *) ((uint8_t *)&p->mac.stats + arg2);
	PORT_LOCK(p);
	t3_mac_update_stats(&p->mac);
	PORT_UNLOCK(p);

	return (sysctl_handle_64(oidp, parg, 0, req));
}

void
t3_add_configured_sysctls(adapter_t *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *children;
	int i, j;
	
	ctx = device_get_sysctl_ctx(sc->dev);
	children = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, 
	    "intr_coal",
	    CTLTYPE_INT|CTLFLAG_RW, sc,
	    0, t3_set_coalesce_usecs,
	    "I", "interrupt coalescing timer (us)");

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, 
	    "pkt_timestamp",
	    CTLTYPE_INT | CTLFLAG_RW, sc,
	    0, t3_pkt_timestamp,
	    "I", "provide packet timestamp instead of connection hash");

	for (i = 0; i < sc->params.nports; i++) {
		struct port_info *pi = &sc->port[i];
		struct sysctl_oid *poid;
		struct sysctl_oid_list *poidlist;
		struct mac_stats *mstats = &pi->mac.stats;
		
		snprintf(pi->namebuf, PORT_NAME_LEN, "port%d", i);
		poid = SYSCTL_ADD_NODE(ctx, children, OID_AUTO, 
		    pi->namebuf, CTLFLAG_RD, NULL, "port statistics");
		poidlist = SYSCTL_CHILDREN(poid);
		SYSCTL_ADD_UINT(ctx, poidlist, OID_AUTO,
		    "nqsets", CTLFLAG_RD, &pi->nqsets,
		    0, "#queue sets");

		for (j = 0; j < pi->nqsets; j++) {
			struct sge_qset *qs = &sc->sge.qs[pi->first_qset + j];
			struct sysctl_oid *qspoid, *rspqpoid, *txqpoid,
					  *ctrlqpoid, *lropoid;
			struct sysctl_oid_list *qspoidlist, *rspqpoidlist,
					       *txqpoidlist, *ctrlqpoidlist,
					       *lropoidlist;
			struct sge_txq *txq = &qs->txq[TXQ_ETH];
			
			snprintf(qs->namebuf, QS_NAME_LEN, "qs%d", j);
			
			qspoid = SYSCTL_ADD_NODE(ctx, poidlist, OID_AUTO, 
			    qs->namebuf, CTLFLAG_RD, NULL, "qset statistics");
			qspoidlist = SYSCTL_CHILDREN(qspoid);

			SYSCTL_ADD_UINT(ctx, qspoidlist, OID_AUTO, "fl0_empty",
					CTLFLAG_RD, &qs->fl[0].empty, 0,
					"freelist #0 empty");
			SYSCTL_ADD_UINT(ctx, qspoidlist, OID_AUTO, "fl1_empty",
					CTLFLAG_RD, &qs->fl[1].empty, 0,
					"freelist #1 empty");

			rspqpoid = SYSCTL_ADD_NODE(ctx, qspoidlist, OID_AUTO, 
			    rspq_name, CTLFLAG_RD, NULL, "rspq statistics");
			rspqpoidlist = SYSCTL_CHILDREN(rspqpoid);

			txqpoid = SYSCTL_ADD_NODE(ctx, qspoidlist, OID_AUTO, 
			    txq_names[0], CTLFLAG_RD, NULL, "txq statistics");
			txqpoidlist = SYSCTL_CHILDREN(txqpoid);

			ctrlqpoid = SYSCTL_ADD_NODE(ctx, qspoidlist, OID_AUTO, 
			    txq_names[2], CTLFLAG_RD, NULL, "ctrlq statistics");
			ctrlqpoidlist = SYSCTL_CHILDREN(ctrlqpoid);

			lropoid = SYSCTL_ADD_NODE(ctx, qspoidlist, OID_AUTO, 
			    "lro_stats", CTLFLAG_RD, NULL, "LRO statistics");
			lropoidlist = SYSCTL_CHILDREN(lropoid);

			SYSCTL_ADD_UINT(ctx, rspqpoidlist, OID_AUTO, "size",
			    CTLFLAG_RD, &qs->rspq.size,
			    0, "#entries in response queue");
			SYSCTL_ADD_UINT(ctx, rspqpoidlist, OID_AUTO, "cidx",
			    CTLFLAG_RD, &qs->rspq.cidx,
			    0, "consumer index");
			SYSCTL_ADD_UINT(ctx, rspqpoidlist, OID_AUTO, "credits",
			    CTLFLAG_RD, &qs->rspq.credits,
			    0, "#credits");
			SYSCTL_ADD_UINT(ctx, rspqpoidlist, OID_AUTO, "starved",
			    CTLFLAG_RD, &qs->rspq.starved,
			    0, "#times starved");
			SYSCTL_ADD_ULONG(ctx, rspqpoidlist, OID_AUTO, "phys_addr",
			    CTLFLAG_RD, &qs->rspq.phys_addr,
			    "physical_address_of the queue");
			SYSCTL_ADD_UINT(ctx, rspqpoidlist, OID_AUTO, "dump_start",
			    CTLFLAG_RW, &qs->rspq.rspq_dump_start,
			    0, "start rspq dump entry");
			SYSCTL_ADD_UINT(ctx, rspqpoidlist, OID_AUTO, "dump_count",
			    CTLFLAG_RW, &qs->rspq.rspq_dump_count,
			    0, "#rspq entries to dump");
			SYSCTL_ADD_PROC(ctx, rspqpoidlist, OID_AUTO, "qdump",
			    CTLTYPE_STRING | CTLFLAG_RD, &qs->rspq,
			    0, t3_dump_rspq, "A", "dump of the response queue");

			SYSCTL_ADD_UQUAD(ctx, txqpoidlist, OID_AUTO, "dropped",
			    CTLFLAG_RD, &qs->txq[TXQ_ETH].txq_mr->br_drops,
			    "#tunneled packets dropped");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "sendqlen",
			    CTLFLAG_RD, &qs->txq[TXQ_ETH].sendq.qlen,
			    0, "#tunneled packets waiting to be sent");
#if 0			
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "queue_pidx",
			    CTLFLAG_RD, (uint32_t *)(uintptr_t)&qs->txq[TXQ_ETH].txq_mr.br_prod,
			    0, "#tunneled packets queue producer index");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "queue_cidx",
			    CTLFLAG_RD, (uint32_t *)(uintptr_t)&qs->txq[TXQ_ETH].txq_mr.br_cons,
			    0, "#tunneled packets queue consumer index");
#endif			
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "processed",
			    CTLFLAG_RD, &qs->txq[TXQ_ETH].processed,
			    0, "#tunneled packets processed by the card");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "cleaned",
			    CTLFLAG_RD, &txq->cleaned,
			    0, "#tunneled packets cleaned");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "in_use",
			    CTLFLAG_RD, &txq->in_use,
			    0, "#tunneled packet slots in use");
			SYSCTL_ADD_ULONG(ctx, txqpoidlist, OID_AUTO, "frees",
			    CTLFLAG_RD, &txq->txq_frees,
			    "#tunneled packets freed");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "skipped",
			    CTLFLAG_RD, &txq->txq_skipped,
			    0, "#tunneled packet descriptors skipped");
			SYSCTL_ADD_UQUAD(ctx, txqpoidlist, OID_AUTO, "coalesced",
			    CTLFLAG_RD, &txq->txq_coalesced,
			    "#tunneled packets coalesced");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "enqueued",
			    CTLFLAG_RD, &txq->txq_enqueued,
			    0, "#tunneled packets enqueued to hardware");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "stopped_flags",
			    CTLFLAG_RD, &qs->txq_stopped,
			    0, "tx queues stopped");
			SYSCTL_ADD_ULONG(ctx, txqpoidlist, OID_AUTO, "phys_addr",
			    CTLFLAG_RD, &txq->phys_addr,
			    "physical_address_of the queue");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "qgen",
			    CTLFLAG_RW, &qs->txq[TXQ_ETH].gen,
			    0, "txq generation");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "hw_cidx",
			    CTLFLAG_RD, &txq->cidx,
			    0, "hardware queue cidx");			
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "hw_pidx",
			    CTLFLAG_RD, &txq->pidx,
			    0, "hardware queue pidx");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "dump_start",
			    CTLFLAG_RW, &qs->txq[TXQ_ETH].txq_dump_start,
			    0, "txq start idx for dump");
			SYSCTL_ADD_UINT(ctx, txqpoidlist, OID_AUTO, "dump_count",
			    CTLFLAG_RW, &qs->txq[TXQ_ETH].txq_dump_count,
			    0, "txq #entries to dump");			
			SYSCTL_ADD_PROC(ctx, txqpoidlist, OID_AUTO, "qdump",
			    CTLTYPE_STRING | CTLFLAG_RD, &qs->txq[TXQ_ETH],
			    0, t3_dump_txq_eth, "A", "dump of the transmit queue");

			SYSCTL_ADD_UINT(ctx, ctrlqpoidlist, OID_AUTO, "dump_start",
			    CTLFLAG_RW, &qs->txq[TXQ_CTRL].txq_dump_start,
			    0, "ctrlq start idx for dump");
			SYSCTL_ADD_UINT(ctx, ctrlqpoidlist, OID_AUTO, "dump_count",
			    CTLFLAG_RW, &qs->txq[TXQ_CTRL].txq_dump_count,
			    0, "ctrl #entries to dump");			
			SYSCTL_ADD_PROC(ctx, ctrlqpoidlist, OID_AUTO, "qdump",
			    CTLTYPE_STRING | CTLFLAG_RD, &qs->txq[TXQ_CTRL],
			    0, t3_dump_txq_ctrl, "A", "dump of the transmit queue");
#if 0
			SYSCTL_ADD_INT(ctx, lropoidlist, OID_AUTO, "lro_queued",
			    CTLFLAG_RD, &qs->lro.ctrl.lro_queued, 0, NULL);
			SYSCTL_ADD_INT(ctx, lropoidlist, OID_AUTO, "lro_flushed",
			    CTLFLAG_RD, &qs->lro.ctrl.lro_flushed, 0, NULL);
			SYSCTL_ADD_INT(ctx, lropoidlist, OID_AUTO, "lro_bad_csum",
			    CTLFLAG_RD, &qs->lro.ctrl.lro_bad_csum, 0, NULL);
			SYSCTL_ADD_INT(ctx, lropoidlist, OID_AUTO, "lro_cnt",
						   CTLFLAG_RD, &qs->lro.ctrl.lro_cnt, 0, NULL);
#endif
		}

		/* Now add a node for mac stats. */
		poid = SYSCTL_ADD_NODE(ctx, poidlist, OID_AUTO, "mac_stats",
		    CTLFLAG_RD, NULL, "MAC statistics");
		poidlist = SYSCTL_CHILDREN(poid);

		/*
		 * We (ab)use the length argument (arg2) to pass on the offset
		 * of the data that we are interested in.  This is only required
		 * for the quad counters that are updated from the hardware (we
		 * make sure that we return the latest value).
		 * sysctl_handle_macstat first updates *all* the counters from
		 * the hardware, and then returns the latest value of the
		 * requested counter.  Best would be to update only the
		 * requested counter from hardware, but t3_mac_update_stats()
		 * hides all the register details and we don't want to dive into
		 * all that here.
		 */
#define CXGB_SYSCTL_ADD_QUAD(a)	SYSCTL_ADD_OID(ctx, poidlist, OID_AUTO, #a, \
    (CTLTYPE_U64 | CTLFLAG_RD), pi, offsetof(struct mac_stats, a), \
    sysctl_handle_macstat, "QU", 0)
		CXGB_SYSCTL_ADD_QUAD(tx_octets);
		CXGB_SYSCTL_ADD_QUAD(tx_octets_bad);
		CXGB_SYSCTL_ADD_QUAD(tx_frames);
		CXGB_SYSCTL_ADD_QUAD(tx_mcast_frames);
		CXGB_SYSCTL_ADD_QUAD(tx_bcast_frames);
		CXGB_SYSCTL_ADD_QUAD(tx_pause);
		CXGB_SYSCTL_ADD_QUAD(tx_deferred);
		CXGB_SYSCTL_ADD_QUAD(tx_late_collisions);
		CXGB_SYSCTL_ADD_QUAD(tx_total_collisions);
		CXGB_SYSCTL_ADD_QUAD(tx_excess_collisions);
		CXGB_SYSCTL_ADD_QUAD(tx_underrun);
		CXGB_SYSCTL_ADD_QUAD(tx_len_errs);
		CXGB_SYSCTL_ADD_QUAD(tx_mac_internal_errs);
		CXGB_SYSCTL_ADD_QUAD(tx_excess_deferral);
		CXGB_SYSCTL_ADD_QUAD(tx_fcs_errs);
		CXGB_SYSCTL_ADD_QUAD(tx_frames_64);
		CXGB_SYSCTL_ADD_QUAD(tx_frames_65_127);
		CXGB_SYSCTL_ADD_QUAD(tx_frames_128_255);
		CXGB_SYSCTL_ADD_QUAD(tx_frames_256_511);
		CXGB_SYSCTL_ADD_QUAD(tx_frames_512_1023);
		CXGB_SYSCTL_ADD_QUAD(tx_frames_1024_1518);
		CXGB_SYSCTL_ADD_QUAD(tx_frames_1519_max);
		CXGB_SYSCTL_ADD_QUAD(rx_octets);
		CXGB_SYSCTL_ADD_QUAD(rx_octets_bad);
		CXGB_SYSCTL_ADD_QUAD(rx_frames);
		CXGB_SYSCTL_ADD_QUAD(rx_mcast_frames);
		CXGB_SYSCTL_ADD_QUAD(rx_bcast_frames);
		CXGB_SYSCTL_ADD_QUAD(rx_pause);
		CXGB_SYSCTL_ADD_QUAD(rx_fcs_errs);
		CXGB_SYSCTL_ADD_QUAD(rx_align_errs);
		CXGB_SYSCTL_ADD_QUAD(rx_symbol_errs);
		CXGB_SYSCTL_ADD_QUAD(rx_data_errs);
		CXGB_SYSCTL_ADD_QUAD(rx_sequence_errs);
		CXGB_SYSCTL_ADD_QUAD(rx_runt);
		CXGB_SYSCTL_ADD_QUAD(rx_jabber);
		CXGB_SYSCTL_ADD_QUAD(rx_short);
		CXGB_SYSCTL_ADD_QUAD(rx_too_long);
		CXGB_SYSCTL_ADD_QUAD(rx_mac_internal_errs);
		CXGB_SYSCTL_ADD_QUAD(rx_cong_drops);
		CXGB_SYSCTL_ADD_QUAD(rx_frames_64);
		CXGB_SYSCTL_ADD_QUAD(rx_frames_65_127);
		CXGB_SYSCTL_ADD_QUAD(rx_frames_128_255);
		CXGB_SYSCTL_ADD_QUAD(rx_frames_256_511);
		CXGB_SYSCTL_ADD_QUAD(rx_frames_512_1023);
		CXGB_SYSCTL_ADD_QUAD(rx_frames_1024_1518);
		CXGB_SYSCTL_ADD_QUAD(rx_frames_1519_max);
#undef CXGB_SYSCTL_ADD_QUAD

#define CXGB_SYSCTL_ADD_ULONG(a) SYSCTL_ADD_ULONG(ctx, poidlist, OID_AUTO, #a, \
    CTLFLAG_RD, &mstats->a, 0)
		CXGB_SYSCTL_ADD_ULONG(tx_fifo_parity_err);
		CXGB_SYSCTL_ADD_ULONG(rx_fifo_parity_err);
		CXGB_SYSCTL_ADD_ULONG(tx_fifo_urun);
		CXGB_SYSCTL_ADD_ULONG(rx_fifo_ovfl);
		CXGB_SYSCTL_ADD_ULONG(serdes_signal_loss);
		CXGB_SYSCTL_ADD_ULONG(xaui_pcs_ctc_err);
		CXGB_SYSCTL_ADD_ULONG(xaui_pcs_align_change);
		CXGB_SYSCTL_ADD_ULONG(num_toggled);
		CXGB_SYSCTL_ADD_ULONG(num_resets);
		CXGB_SYSCTL_ADD_ULONG(link_faults);
#undef CXGB_SYSCTL_ADD_ULONG
	}
}
	
/**
 *	t3_get_desc - dump an SGE descriptor for debugging purposes
 *	@qs: the queue set
 *	@qnum: identifies the specific queue (0..2: Tx, 3:response, 4..5: Rx)
 *	@idx: the descriptor index in the queue
 *	@data: where to dump the descriptor contents
 *
 *	Dumps the contents of a HW descriptor of an SGE queue.  Returns the
 *	size of the descriptor.
 */
int
t3_get_desc(const struct sge_qset *qs, unsigned int qnum, unsigned int idx,
		unsigned char *data)
{
	if (qnum >= 6)
		return (EINVAL);

	if (qnum < 3) {
		if (!qs->txq[qnum].desc || idx >= qs->txq[qnum].size)
			return -EINVAL;
		memcpy(data, &qs->txq[qnum].desc[idx], sizeof(struct tx_desc));
		return sizeof(struct tx_desc);
	}

	if (qnum == 3) {
		if (!qs->rspq.desc || idx >= qs->rspq.size)
			return (EINVAL);
		memcpy(data, &qs->rspq.desc[idx], sizeof(struct rsp_desc));
		return sizeof(struct rsp_desc);
	}

	qnum -= 4;
	if (!qs->fl[qnum].desc || idx >= qs->fl[qnum].size)
		return (EINVAL);
	memcpy(data, &qs->fl[qnum].desc[idx], sizeof(struct rx_desc));
	return sizeof(struct rx_desc);
}
