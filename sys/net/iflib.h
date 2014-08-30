struct iflib_ctx;
typedef struct iflib_ctx *iflib_ctx_t;

struct iflib_irq;
typedef struct iflib_irq *iflib_irq_t;

#define RXD_SOP       (1 << 0)
#define RXD_SOP_EOP   (1 << 1)
#define RXD_EOP       (1 << 2)
#define RXD_NSOP_NEOP (1 << 3)
#define RXD_VLAN      (1 << 4)

typedef struct rxd_info {
	uint16_t ri_qidx;
	uint16_t ri_vtag;
	int      ri_flags;
	int      ri_cidx;
	uint32_t ri_len;
	struct mbuf *ri_m;
	uint64_t ri_csum_flags;
	uint64_t ri_csum_data;
} *rxd_info_t;

typedef struct iflib_shared_ctx {
	iflib_ctx_t isc_ctx;
	/*
	 * Shared stats or whatever
	 */
} iflib_shared_ctx_t;
