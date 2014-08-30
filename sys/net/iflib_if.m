#includes <sys/types.h>
#include <sys/systm.h>
#include <sys/bus.h>

INTERFACE ifc;


METHOD int detach {
	if_shared_ctx_t _ctx;
};

METHOD int suspend {
	if_shared_ctx_t _ctx;
};

METHOD int resume {
	if_shared_ctx_t _ctx;
};

METHOD void init {
	if_shared_ctx_t _ctx;
};

METHOD void stop {
	if_shared_ctx_t _ctx;
};

METHOD void intr_enable {
	if_shared_ctx_t _ctx;
};

METHOD void intr_disable {
	if_shared_ctx_t _ctx;
};

METHOD void tx_intr_enable {
	if_shared_ctx_t _ctx;
	void *_txq;
};

METHOD void rx_intr_enable {
	if_shared_ctx_t _ctx;
	void *_rxq;
};

METHOD void link_intr_enable {
	if_shared_ctx_t _ctx;
};

METHOD void multi_set {
	if_shared_ctx_t _ctx;
};

METHOD int queues_alloc {
	if_shared_ctx_t _ctx;
};

METHOD void update_link_status {
	if_shared_ctx_t _ctx;
};

METHOD int mtu_set {
	if_shared_ctx_t _ctx;
	uint32_t _mtu;
};

METHOD void media_set{
	if_shared_ctx_t _ctx;
};

METHOD void media_status {
	if_shared_ctx_t _ctx;
	struct ifmediareq *_ifm;
};

METHOD int media_change {
	if_shared_ctx_t _ctx;
};

METHOD int timer {
	if_shared_ctx_t _ctx;
};

METHOD int tx {
	if_shared_ctx_t _ctx;
	void *_txr;
	pkt_info_t _pi;
};

METHOD void tx_flush {
	if_shared_ctx_t _ctx;
	void *_txr;
	int _pidx;
};

METHOD void txeof {
	if_shared_ctx_t _ctx;
	void *_txr;
};

METHOD int packet_get {
	if_shared_ctx_t _ctx;
	void *_txr;
	int _idx;
	rx_info_t _ri;
};

METHOD void led_func {
	if_shared_ctx_t _ctx;
	int _onoff;
};

METHOD void watchdog_reset {
	if_shared_ctx_t _ctx;
};

METHOD void promisc_config {
	if_shared_ctx_t _ctx;
	int _flags;
};

METHOD void promisc_disable {
	if_shared_ctx_t _ctx;
	int _flags;
};

METHOD void vlan_register {
	if_shared_ctx_t _ctx;
	u16 _vtag;
};

METHOD void vlan_unregister {
	if_shared_ctx_t _ctx;
	u16 _vtag;
};	
	
METHOD void txq_setup {
	if_shared_ctx_t _ctx;
	void *_arg;
};

METHOD void rxd_refill {
	if_shared_ctx_t _ctx;
	int _rxqid;
	int _pidx;
	uint64_t paddr;
};

METHOD void rxd_refill_flush {
	if_shared_ctx_t _ctx;
	int _rxqid;
	int _pidx;
};

METHOD int is_new {
	if_shared_ctx_t _ctx;
	void *_rxr;
	int _idx;
};
