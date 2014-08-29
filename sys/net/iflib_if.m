#includes <sys/types.h>
#include <sys/systm.h>
#include <sys/bus.h>

INTERFACE iflib;


METHOD int detach {
	iflib_ctx_t _ctx;
};

METHOD int suspend {
	iflib_ctx_t _ctx;
};

METHOD int resume {
	iflib_ctx_t _ctx;
};

METHOD void init {
	iflib_ctx_t _ctx;
};

METHOD void stop {
	iflib_ctx_t _ctx;
};

METHOD void intr_enable {
	iflib_ctx_t _ctx;
};

METHOD void intr_disable {
	iflib_ctx_t _ctx;
};

METHOD void tx_intr_enable {
	iflib_ctx_t _ctx;
	void *_txq;
};

METHOD void rx_intr_enable {
	iflib_ctx_t _ctx;
	void *_rxq;
};

METHOD void link_intr_enable {
	iflib_ctx_t _ctx;
};

METHOD void multi_set {
	iflib_ctx_t _ctx;
};

METHOD int queues_alloc {
	iflib_ctx_t _ctx;
};

METHOD void update_link_status {
	iflib_ctx_t _ctx;
};

METHOD int mtu_set {
	iflib_ctx_t _ctx;
	uint32_t _mtu;
};

METHOD void media_set{
	iflib_ctx_t _ctx;
};

METHOD void media_status {
	iflib_ctx_t _ctx;
	struct ifmediareq *_ifm;
};

METHOD int media_change {
	iflib_ctx_t _ctx;
};

METHOD int timer {
	iflib_ctx_t _ctx;
};

METHOD int tx {
	iflib_ctx_t _ctx;
	void *_txr;
	pkt_info_t _pi;
};

METHOD void txeof {
	iflib_ctx_t _ctx;
	void *_txr;
};

METHOD int packet_get {
	iflib_ctx_t _ctx;
	void *_txr;
	int _idx;
	rx_info_t _ri;
};

METHOD void led_func {
	iflib_ctx_t _ctx;
	int _onoff;
};

METHOD void watchdog_reset {
	iflib_ctx_t _ctx;
};

METHOD void promisc_config {
	iflib_ctx_t _ctx;
	int _flags;
};

METHOD void promisc_disable {
	iflib_ctx_t _ctx;
	int _flags;
};

METHOD void vlan_register {
	iflib_ctx_t _ctx;
	u16 _vtag;
};

METHOD void vlan_unregister {
	iflib_ctx_t _ctx;
	u16 _vtag;
};	
	
METHOD void txq_setup {
	iflib_ctx_t _ctx;
	void *_arg;
};
