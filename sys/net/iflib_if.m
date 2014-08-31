#include <sys/types.h>
#include <sys/systm.h>
#include <sys/bus.h>

INTERFACE ifc;

CODE {
	void
	null_op(if_shared_ctx_t _ctx __unused)
	{
	}

	void
	null_led_func(if_shared_ctx_t _ctx __unused, int _onoff __unused)
	{
	}

	void
	null_vlan_register_op(if_shared_ctx_t _ctx __unused, uint16_t vtag __unused)
	{
	}

	void
	null_q_setup(if_shared_ctx_t _ctx __unused, int _qid __unused)
	{
	}
};

/*
 * driver bus methods
 */

METHOD int detach {
	if_shared_ctx_t _ctx;
};

METHOD int suspend {
	if_shared_ctx_t _ctx;
};

METHOD int resume {
	if_shared_ctx_t _ctx;
};

/*
 * downcall to driver to allocate its
 * own queue state and tie it to the parent
 */

METHOD int queues_alloc {
	if_shared_ctx_t _ctx;
};

/*
 * interface reset / stop
 */

METHOD void init {
	if_shared_ctx_t _ctx;
};

METHOD void stop {
	if_shared_ctx_t _ctx;
};

/*
 * interrupt manipulation
 */

METHOD void intr_enable {
	if_shared_ctx_t _ctx;
};

METHOD void intr_disable {
	if_shared_ctx_t _ctx;
};

METHOD void tx_intr_enable {
	if_shared_ctx_t _ctx;
	int _txqid;
};

METHOD void rx_intr_enable {
	if_shared_ctx_t _ctx;
	int _rxqid;
};

METHOD void link_intr_enable {
	if_shared_ctx_t _ctx;
} DEFAULT null_op;

/*
 * interface configuration
 */

METHOD void multi_set {
	if_shared_ctx_t _ctx;
};

METHOD int mtu_set {
	if_shared_ctx_t _ctx;
	uint32_t _mtu;
};

METHOD void media_set{
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

/*
 * tx methods
 */

METHOD int tx {
	if_shared_ctx_t _ctx;
	int _txqid;
	pkt_info_t _pi;
};

METHOD void tx_flush {
	if_shared_ctx_t _ctx;
	int _txqid;
	int _pidx;
};

METHOD void txeof {
	if_shared_ctx_t _ctx;
	int _txqid;
};

/*
 * rx methods
 */

METHOD int is_new {
	if_shared_ctx_t _ctx;
	int _rxqid;
	int _idx;
};

METHOD int packet_get {
	if_shared_ctx_t _ctx;
	int _txqid;
	int _idx;
	rx_info_t _ri;
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

/*
 * Device status
 */

METHOD void update_link_status {
	if_shared_ctx_t _ctx;
};

METHOD void media_status {
	if_shared_ctx_t _ctx;
	struct ifmediareq *_ifm;
};

METHOD int media_change {
	if_shared_ctx_t _ctx;
};


/*
 * optional methods
 */

METHOD void txq_setup {
	if_shared_ctx_t _ctx;
	int _txqid;
} DEFAULT null_q_setup;

METHOD void rxq_setup {
	if_shared_ctx_t _ctx;
	int _txqid;
} DEFAULT null_q_setup;

METHOD int timer {
	if_shared_ctx_t _ctx;
} DEFAULT null_op;

METHOD void watchdog_reset {
	if_shared_ctx_t _ctx;
} DEFAULT null_op;

METHOD void led_func {
	if_shared_ctx_t _ctx;
	int _onoff;
} DEFAULT null_led_func;

METHOD void vlan_register {
	if_shared_ctx_t _ctx;
	u16 _vtag;
} DEFAULT null_vlan_register_op;

METHOD void vlan_unregister {
	if_shared_ctx_t _ctx;
	u16 _vtag;
} DEFAULT null_vlan_register_op;


