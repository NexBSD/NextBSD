/*-
 * Copyright (c) 2014-2015, Matthew Macy (mmacy@nextbsd.org)
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

#ifndef __MVEC_H_
#define __MVEC_H_
#include <sys/mbuf.h>

typedef struct m_clref {
	uint16_t mc_flags;	/* external storage mbuf flags */
	uint8_t mc_type;	/* type of external storage */
	uint8_t mc_pad;
	uint32_t mc_size;
	caddr_t mc_buf;
	volatile u_int	*mc_cnt;	/* pointer to ref count info */
} *m_clref_t;

/*
 * Represent offsets in to a cluster
 */
typedef struct m_seg {
	uint16_t ms_off;
	uint16_t ms_len;
} *m_seg_t;

/*
 * In the exceptional event of a > 64k cluster
 * this allows to handle corresponding offsets
 */
typedef struct m_wide_seg {
	uint32_t mws_off;
	uint32_t mws_len;
} *m_wide_seg_t;

/*
 * References non-refcounted intermediate data
 * where the data may not be at the beginning (mbufs)
 */
typedef struct m_mbuf_meta {
	uint8_t	mds_type;
	uint8_t mds_flags;
} *m_meta_t;

typedef struct m_segmap_ent {
	uint8_t mse_type  : 2,
		mse_index : 5,
		mse_eop : 1;
} *m_segmap_ent_t;


struct mbuf_lite {
	/*
	 * Header present at the beginning of every intermediate mbuf.
	 * Size ILP32: 12
	 *      LP64: 16
	 * Compile-time assertions in uipc_mbuf.c test these values to ensure
	 * that they are correct.
	 */
	union {	/* next buffer in chain */
		struct mbuf		*m_next;
		SLIST_ENTRY(mbuf)	m_slist;
		STAILQ_ENTRY(mbuf)	m_stailq;
	};
	int32_t		 m_len;		/* amount of data in this mbuf */
	uint32_t	 m_type:8,	/* type of data in this mbuf */
			 m_flags:24;	/* flags; see below */
};


/*
 * pkthdr cache line offsets   - 24 bytes ((56 + 32)%64)
 * mbuf cache line offset      - 32 bytes
 * mbuf_lite cache line offest - 16 bytes
 * Offsets are in multiples of 8 bytes
 * LP64: 8 bytes
 */
typedef struct mvec_hdr {
	/* offset value should be ignored if count is zero */
	uint8_t	mh_ncl:4, 		/* number of distinct clusters */
		mh_ncliref:4;		/* number of clusters with internal references*/
	uint8_t	mh_cl_off;		/* offset of m_cl array */
	uint8_t mh_cl_size;		/* size of cluster array in (/8) */

	uint8_t	mh_nmdata;		/* number of data segments (mbufs) */
	uint8_t	mh_mdata_off;		/* offset of mbuf pointer array - followed by flags/type array */

	uint8_t	mh_ndataptr;		/* number of data pointers (malloc) */

	uint8_t	mh_nsegs;		/* number of total segments */
	uint8_t mh_segoff;		/* offset of m_seg array - followed by type/index array */	
} *mvec_hdr_t;



typedef struct mvec_ext {
	/*
	 * the flags array will indicate whether or not a cluster
	 * uses this ext_free, the default free - if two clusters
	 * in a chain have different non-default free routines
	 * they will be stored in separate vectors - I believe that
	 * this is rare enough to not warrant further optimization
	 */
	void	(*me_free)	/* free routine if not the usual */
			(struct mbuf *, void *, void *);
	void	*me_arg1;	/* optional argument pointer */
	void	*me_arg2;	/* optional argument pointer */

} *mvec_ext_t;	


typedef struct mvec_toc {
	mvec_hdr_t mt_mh;
	uint32_t *mt_iref;
	mvec_ext_t mt_mext;
	m_clref_t mt_cl;
	struct mbuf *mt_mdata;
	m_meta_t mt_mflags;
	caddr_t mt_dataptr;
	m_seg_t mt_segs;
	m_segmap_ent_t mt_segmap;
	int mt_nsegs;
} *mvec_toc_t;



/*
 * layout:
 * | mvec_hdr | m_clrefs (ptr/ref) | mbuf ptrs | mbuf flags/types | malloced ptrs | m_segs (off/len) | segmap |
 *
 * Worst case (all separate clusters M_PKTHDR): 4 clusters can be managed within the data area
 * Worst case (all separate clusters !M_PKTHDR): 8 clusters can be managed within the data area
 * TSO best case (1 cluster, separate headers M_PKTHDR) 40 bytes / 2 segment packet -> 3 packets
 * TSO best case (1 cluster, separate headers !M_PKTHDR) 40 bytes / 2 segment packet -> 5 packets
 * Best case (1 cluster, many segments, M_PKTHDR): 28 segments can be managed within an mbuf's data area
 * Best case (1 cluster, many segments, !M_PKTHDR): 50 segments can be managed within an mbuf's data area
 */


struct mbuf *mvec_serialize(struct mbuf *);
struct mbuf *_mvec_deserialize(struct mbuf *);
struct mbuf *mvec_defrag(struct mbuf *);
void mvec_free(struct mbuf *);

static inline struct mbuf *
mvec_deserialize(struct mbuf *m)
{
	/* no point in converting if there is only one */
	if (m->m_next == NULL)
		return (m);
	/* no point in converting if there are only two and neither has free data space */ 
	if ((((m->m_flags & M_EXT) | (m->m_next->m_flags & M_EXT)) == 0) &&
	    m->m_next->m_next == NULL)
		return (m);
	return (_mvec_deserialize(m));
}


#endif
