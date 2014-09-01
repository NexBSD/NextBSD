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

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/bus.h>

INTERFACE ifc;

CODE {
	extern int iflib_recycle_enable;

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

	int
	null_recycle_rx_buf(if_shared_ctx_t _ctx __unused, int _qid __unused, int _idx __unused)
	{
		iflib_recycle_enable = 0;
		return (ENOTSUP);
	}
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

METHOD void recycle_rx_buf {
	if_shared_ctx_t _ctx;
	int qidx;
	int idx;
} DEFAULT null_recycle_rx_buf;

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


