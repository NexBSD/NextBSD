
#include <sys/mvec.h>

/*
 * LP64: 16 bytes
 */
typedef struct mbuf_data {
	uint32_t md_type: 8,
		md_flags: 24;	
	uint16_t md_off;
	uint16_t md_len;
	caddr_t md_data;
} *mdata_t;

/*
 * LP64: 40 bytes
 */
typedef struct clref_data {
	uint32_t cd_type: 8,
		cd_flags: 24;
	uint16_t cd_off;
	uint16_t cd_len;
	caddr_t cd_data;
	caddr_t cd_mbuf;
	uint32_t cd_size;
	/* pad */
	volatile u_int	*cd_cnt;	/* pointer to ref count info */
} *clrefdata_t;

/*
 * LP64: 32 bytes
 */
typedef struct cl_data {
	uint32_t cd_type: 8,
		cd_flags: 24;
	uint16_t cd_off;
	uint16_t cd_len;
	caddr_t cd_data;
	caddr_t cd_mbuf;
	uint32_t cd_size;
	/* pad */
} *cldata_t;

struct mchain_hdr {
	struct m_ext mch_ext;
	uint8_t mch_mbufcnt;
	uint8_t mch_recyclecnt;
	uint8_t mch_clcnt;
	uint8_t mch_datacnt;
	uint8_t mch_embcnt;
	uint8_t mch_total;
	uint16_t mch_used;
	caddr_t *mch_recyclebufs;
} *mch_hdr_t;


#define MCHAIN_CLUSTER		0x0100	/* entry is a cluster */
#define MCHAIN_MBUF		0x0200	/* entry is an mbuf */
#define MCHAIN_HASREF		0x0400	/* entry has reference ptr */
#define MCHAIN_NOFREE		0x0800	/* mbuf can't be freed */
#define MCHAIN_MBUF_EXTREF	0x1000	/* refcnt points to re-usable mbuf */

static int
mbuf_chain_encode(struct mbuf *m, caddr_t scratch, int scratch_size)
{
	caddr_t scratch_tmp, last_data;
	int avail, used, error;
	mch_hdr_t mh;
	mdata_t mdp;
	crefdata_t crdp;
	volatile u_int *refcnt, *last_ref;
	

	mh = scratch;
	scratch_tmp = scratch + sizeof(struct mchain_hdr);
	m_tmp = m;
	bzero(me, sizeof(me));
	avail = scratch_size;
	last_data = NULL;
	last_ref = NULL;

	while (m != NULL) {
		if (avail < sizeof(*cdp))
			return (ENOSPC);
		if (m->m_flags & M_EXT) {
			if (m->m_ext.ext_free != NULL) {
				if (__predict_false(mh->mch_ext.ext_free != NULL &&  mh->mch_ext.ext_free != m->m_ext.ext_free))
					return (ENXIO);
				mh.mch_ext.ext_free = m->m_ext.ext_free;
				mh.mch_ext.ext_arg1 = m->m_ext.ext_arg1;
				mh.mch_ext.ext_arg2 = m->m_ext.ext_arg2;
			}
			mh->mch_ext.ext_size += m->m_ext.ext_size;
			crdp = scratch_tmp;
			crdp->cd_flags = MCHAIN_CLUSTER;
			if (m->m_flags & M_NOFREE)
				crdp->cd_flags |= MCHAIN_NOFREE;
			crdp->cd_off = m->m_data - m->m_ext.ext_buf;
			crdp->cd_len = m->m_len;
			crdp->cd_data = m->m_ext.ext_buf;
			crdp->cd_mbuf = m;
			crdp->cd_size = m->m_ext.ext_size;
			if (m->m_flags & EXT_FLAG_EMBREF) {
				if (m->m_ext.ext_count > 1) {
					crdp->cd_flags |= MCHAIN_HAS_REF|MCHAIN_NOFREE;
				}
				mch->mch_embcnt++;
			} else {
				crdp->cd_flags |= MCHAIN_HAS_REF;
				crdp->cd_cnt = m_ext.ext_cnt;
				if (*refcnt == 1) {
					crdp->cd_flags |= MCHAIN_MBUF_EXTREF;
					mh->mch_recyclecnt++;
					mch->mch_embcnt++;
				}
			}
			if (crdp->cd_flags & MCHAIN_HAS_REF) {
				used = sizeof(struct clref_data);
			} else
				used = sizeod(struct cl_data);
			if ((crdp->cd_flags & MCHAIN_NOFREE) == 0)
				mh->mch_recyclecnt++;
			mh->mch_clcount++;
			
		} else 	{
			mh->mch_mbufcnt++;
			mdp = scratch_tmp;
			used = sizeof(*mdp);
			mdp->md_flags = MCHAIN_MBUF;
			if (m->m_flags & M_NOFREE)
				crdp->cd_flags |= MCHAIN_NOFREE;
			mdp->md_off = m->m_data - ((caddr_t)m);
			mdp->md_len = m->m_len;
			mdp->md_data = m;
		}
		mch->mch_total++;
		scratch_tmp += used;
		avail -= used;
		m = m->m_next;
	}
	if (avail < mch->mch_recyclecnt*sizeof(caddr_t))
		return (ENOSPC);
	if (mch->mch_recyclecnt == 0)
		goto done;

	avail -= mch->mch_recyclecnt*sizeof(caddr_t);
	mch->mch_recyclebufs = scratch_tmp;
	scratch_tmp = scratch + sizeof(struct mchain_hdr);
	for (i = 0, j = 0; i < mch->mch_total; i++) {
		crdp = scratch_tmp;

		if (crdp->cd_flags & MCHAIN_MBUF) {
			scratch_tmp += sizeof(*mdp);
			continue;
		}
		if (crdp->cd_flags & MCHAIN_HAS_REF)
			scratch_tmp += sizeof(struct clref_data);
		else
			scratch_tmp += sizeof(struct cl_data);
		if (crdp->cd_flags & MCHAIN_MBUF_EXTREF) {
			MPASS(crdp->cd_flags & MCHAIN_HAS_REF);
			mch->mch_recyclebufs[j] = __containerof(crdp->cd_cnt, struct mbuf, m_ext.ext_count);
			j++;
		}
		if ((crdp->cd_flags & MCHAIN_NOFREE) == 0) {
			mch->mch_recyclebufs[j] = crdp->cd_mbuf;
			j++;			
		}
	}
	MPASS(j == mch->mch_recyclecnt);
done:
	mch->mch_used = scratch_size - avail;
	return (0);
}

void
mvec_unpack(struct mbuf *m, mvec_toc_t toc)
{
	caddr_t cursor;
	int cnt;
	mvec_hdr_t mh;

	if (m->m_flags & M_PKTHDR)
		mh = (mvec_hdr_t)&m->m_pktdat;
	else
		mh = (mvec_hdr_t)(((caddr_t)m) + sizeof(struct mbuf_lite));
	cursor = mh;
	*((uint64_t*)toc->mt_mh) = *((uint64_t *)mh);
	if (mh->mh_ncliref > 0) {
		toc->mt_iref = (uint32_t *)(mh + 1);
		/* round up to 8 byte alignment */
		toc->mt_mext = (mvec_ext_t)(toc->mt_iref + ((mh->mh_ncliref + 2) & ~0x1));
	} else
		toc->mt_mext = (mvec_ext_t)(mh + 1);
	if (mh->mh_ncl > 0) 
		toc->mt_cl = cursor + (mh->mh_cl_off << 3);
	if ((cnt = mh->mh_nmdata) > 0) {
		toc->mt_mdata = (struct mbuf *)(cursor + (mh->mh_mdata_off << 3));
		toc->mt_mflags = (m_meta_t)(cursor + (mh->mh_mdata_off << 3) + (cnt * sizeof(struct mbuf *)));
	}
	if (mh->mh_ndataptr > 0) {
		/* round up to multiple of 8 */
		int moff = ((cnt * (sizeof(struct mbuf *) + sizeof(struct m_mbuf_meta))) + 8) & ~0x8;
		toc->mt_dataptr = (cursor + (mh->mh_mdata_off << 3) + moff);
	}
	MPASS(mh->mh_nsegs > 0);
	cnt = mh->mh_nsegs;
	toc->mt_segs = (m_seg_t)(cursor + (mh->mh_segoff << 3));
	toc->mt_segmap = (m_segmap_ent_t)(cursor + (mh->mh_segoff << 3) + (cnt * sizeof(*m_seg_t)));
}

static inline struct mbuf *
mbuf_alloc(caddr_t *bufs, int *bufcnt, int *idx)
{
	if (*bufcnt > 0) {
		mp = bufs[*idx];
		(*bufcnt)--;
		(*idx)++;
	} else
		mp = m_gethdr(M_NOWAIT, MT_NOINIT);
	return (mp);
}

static int
mvec_deserialize_one(struct mbuf *m, struct mchain_hdr *mch, int remaining,
		     caddr_t *curptr)
{
	struct mvec_toc enc_toc;
	caddr_t chain;
	mvec_hdr_t mh;
	int maxcl, maxmb, iref, newcl;
	int segcount, mcount, clcount, irefcount;

	if (m->m_flags & M_PKTHDR)
		mh = (mvec_hdr_t)&m->m_pktdat;
	else
		mh = (mvec_hdr_t)(((caddr_t)m) + sizeof(struct mbuf_lite));
	*((uint64_t*)mh) = 0;

	if (m->m_flags & M_PKTHDR)
		avail = MHLEN - sizeof(*mh);
	else
		avail = MSIZE - sizeof(*mh) - sizeof(struct mbuf_lite);

	chain = *curptr;

	 /* Determine if we need additional mbufs */
	ext_buf_last = NULL;
	mbuf_segsize = sizeof(struct mbuf *) + sizeof(*m_meta_t) + sizeof(*m_seg_t) + sizeof(*m_segmap_ent);
	segcount = mcount = clcount = irefcount = 0;
	newcl = iref = 0;
	for (i = 0; i < remaining; i++) {
		crdp = (crefdata_t)chain;
		mdp = (mdata_t)mdp;
		if (crdp->cd_flags & MCHAIN_MBUF) {
			if (avail < mbuf_segsize)
				break;
			avail -= mbuf_segsize;
			chain = (caddr_t)(mdp + 1);
			mh->mh_nmdata++;
			mh->mh_nsegs++;
			continue;
		}

		if (crdp->cd_data != ext_buf_last) {
			used += sizeof(*m_clref_t);
			ext_buf_last = crdp->cd_data;
			newcl = 1;
			/* The cluster has no reference or one with a refcount == 1 */
			if ((crdp->cd_flags & MCHAIN_HAS_REF) == 0 ||
			    (crdp->cd_flags & (MCHAIN_HAS_REF|MCHAIN_MBUF_EXTREF)) == (MCHAIN_HAS_REF|MCHAIN_MBUF_EXTREF)) {
				iref = 1;
				if (mh->mh_ncliref  == 0 || ((mh->mh_ncliref & 0x1) == 1))
					used += 8;
				/* internal references and pointers to them get set in pack */		    
			}
		} 
		/* every segment has at least on off/len pair and a segmap entry */
		used += sizeof(*m_seg_t) + sizeof(*m_segmap_ent); 
		/* set their values */
		if (avail < used)
			break;
		if (crdp->cd_flags & MCHAIN_HAS_REF)
			chain = (caddr_t)(crdp + 1);
		else
			chain += sizeof(struct cl_data);

		if (iref) {
			mh->mh_ncliref++;
			iref = 0;
		}
		if (newcl) {
			mh->mh_ncl++;
			newcl = 0;
		}
		mh->mh_nsegs++;
	}

	/* calculate the offsets */
	mvec_unpack(m, &enc_toc);
	chain = *curptr;
	segcount = irefcount = clcount = mcount = 0;
	if (mh->mh_ncliref) {
		/* first refcount is for the mvec itself */
		enc_toc.mt_iref[0] = mh->mh_ncliref;
		for (i = 0; i < mh->mh_ncliref; i++)
			enc_toc.mt_iref[irefcount+1] = 1;

	}
	for (i = 0; i < mh->mh_nsegs; i++) {
		crdp = (crefdata_t)chain;
		mdp = (mdata_t)mdp;
		if (crdp->cd_flags & MCHAIN_MBUF) {
			if (avail < mbuf_segsize)
				break;
			avail -= mbuf_segsize;
			chain = (caddr_t)(mdp + 1);
			/* update mbuf values, segs, and segment map */
			enc_toc.mt_mdata[mcount] = mdp->md_data;
			enc_toc.mt_mflags[mcount].mds_type = mdp->md_type;
			enc_toc.mt_mflags[mcount].mds_flags = (mdp->md_flags & 0xff);
			enc_toc.mt_segs[segcount].ms_off = mdp->md_off;
			enc_toc.mt_segs[segcount].ms_len = mdp->md_len;
			enc_toc.mt_segmap[segcount].mse_type = MVEC_TYPE_MBUF;
			enc_toc.mt_segmap[segcount].mse_index = mcount;
			mcount++;
			segcount++;
			continue;
		}
		enc_toc.mt_segs[segcount].ms_off = crdp->cd_off;
		enc_toc.mt_segs[segcount].ms_len = crdp->cd_len;
		enc_toc.mt_segmap[segcount].mse_type = MVEC_TYPE_CLUSTER;
		enc_toc.mt_segmap[segcount].mse_index = clcount;
		segcount++;	
		if (crdp->cd_data != ext_buf_last) {
			ext_buf_last = crdp->cd_data;
			/* copy new cluster entry */

			enc_toc.mt_cl[clcount].mc_flags = crdp->cd_flags;
			enc_toc.mt_cl[clcount].mc_type = crdp->cd_type;
			enc_toc.mt_cl[clcount].mc_size = crdp->cd_size;
			enc_toc.mt_cl[clcount].mc_buf = crdp->cd_data;

			/* The cluster has no reference or one with a refcount == 1 */
			if ((crdp->cd_flags & MCHAIN_HAS_REF) == 0 ||
			    (crdp->cd_flags & (MCHAIN_HAS_REF|MCHAIN_MBUF_EXTREF)) == (MCHAIN_HAS_REF|MCHAIN_MBUF_EXTREF)) {
				enc_toc.mt_cl[clcount].mc_cnt = &enc_toc.mt_iref[irefcount+1];
				irefcount++;
				last_ref = NULL;
				enc_toc.mt_cl[clcount].mc_flags |= MVEC_FLAG_EMBREF;
				/* internal references and pointers to them get set in pack */
			} else {
				last_ref = enc_toc.mt_cl[clcount].mc_cnt = crdp->cd_cnt;
			}

			clcount++;
		} else if (last_ref != NULL) {
			atomic_add_int(last_ref, -1);
		}
		if (crdp->cd_flags & MCHAIN_HAS_REF)
			chain += sizeof(*crdp);
		else
			chain += sizeof(struct cl_data);
	}
	
	*curptr = chain;
	return (remaining - mh->mh_nsegs);
}

struct mbuf *
mvec_deserialize_(struct mbuf *m, caddr_t scratch, int scratch_size)
{
	int i, midx, err, avail, mbuf_avail, irefcount, used;
	int mbuf_segsize, remaining;
	struct mchain_hdr *mch;
	struct mbuf *mp, *mhp, *mt;
	caddr_t new_scratch, chain, recycle_offset, *bufs;
	void *ext_buf_last;
	mdata_t mdp;
	crefdata_t crdp;

	if ((err = mbuf_chain_encode(m, scratch, scratch_size) != 0))
		return (m);

	mch = scratch;
	chain = scratch + sizeof(*mch);
	new_scratch = scratch + mch->mch_used;
	avail = MHLEN - sizeof(struct mvec_hdr);

	mbuf_avail = mch->mch_recyclecnt;
	midx = 0;
	bufs = mch->mch_recyclebufs;

	if ((mp = mbuf_alloc(bufs, &mbuf_avail, &midx)) == NULL)
		return (m);
	
	if (mp != m)
		memcpy(mp, m, MPKTHSIZE);
	mt = mhp = mp;
	remaining = mch->mch_total;
	do {
		remaining = mvec_deserialize_one(mp, mch, remaining, &chain,
				new_scratch, scratch_size - (new_scratch - scratch));
		if (mt != mp) {
			mt->mp_next = mp;
			mt = mp;
		}
		if (remaining && (mp = mbuf_alloc(bufs, &mbuf_avail, &midx)) == NULL)
			goto err;
	} while (remaining);
	while (mbuf_avail) {
		uma_zfree_arg(zone_mbuf, bufs[midx], MB_DTOR_SKIP);
		mbuf_avail--;
	}
	
	return (mhp);
err:
	mvec_free(mhp);
	crdp = (crefdata_t)chain;
	mdp = (mdata_t)chain;
	if (mdp->md_flags & MCHAIN_MBUF) {
		m_freem((struct mbuf *)mdp->md_data);
	} else {
		m_freem((struct mbuf *)crdp->cd_mbuf);
	}

	return (NULL);
}

static void
mvec_serialize_ext_init(struct mbuf *mp, struct mvec_toc *enc_toc, int segidx)
{
	m_clref_t crp;
	int clidx;

	clidx = enc_toc.mt_segmap[segidx].mse_index;

	/* initialize m_ext */
	crp = &enc_toc.mt_cl[clidx];
	mp->m_flags |= M_EXT;
	if (crp->mc_flags & MVEC_CLUSTER_M_NOFREE)
		mp->m_flags |= M_NOFREE;

	mp->m_data = crp->mc_buf + enc_toc.mt_segs[segidx].ms_off;
	mp->m_ext.ext_buf = crp->mc_buf;
	mp->m_ext.ext_size = crp->mc_size;
	if (*crp->mc_cnt == 1) {
		mp->m_ext.ext_flags = EXT_FLAG_EMBREF;
		if ((crp->mc_flags & MVEC_CLUSTER_EMBREF) == 0)
			uma_zfree_arg(zone_mbuf, __containerof(crp->mc_cnt, struct mbuf, m_ext.ext_count), MB_DTOR_SKIP);
		else
			atomic_add_int(&enc_toc->mt_iref[0], -1);
		mp->m_ext.ext_count = 1;

	} else {
		mp->m_ext.ext_cnt = crp->mc_cnt;
		/* tell new owner that it's refcount ptr is holding an mvec */
		if (crp->mc_flags & MVEC_CLUSTER_EMBREF)
			mp->m_ext.ext_flags = EXT_FLAG_MVEC_EMBREF;
	}
	if (crp->mc_flags & MVEC_CLUSTER_EXT_FREE) {
		mp->m_ext.ext_free = enc_toc.mt_mext.me_free;
		mp->m_ext.ext_free = enc_toc.mt_mext.me_arg1;
		mp->m_ext.ext_free = enc_toc.mt_mext.me_arg2;
	} else {
		mp->m_ext.ext_free = NULL;
		mp->m_ext.ext_arg1 = NULL;
		mp->m_ext.ext_arg2 = NULL;	
	}
}

static struct mbuf *
mvec_serialize_mbuf_init(struct mvec_toc *enc_toc, int segidx)
{
	int midx;
	struct mbuf *mp;

	midx = enc_toc.mt_segmap[segidx].mse_index;
	mp = enc_toc.mt_mdata[midx];
	mp->m_data = enc_toc.mt_mdata[midx] + enc_toc.mt_segs[segidx].ms_off;
	return (mp);
}
	
static void
mvec_free_idx(struct mvec_toc *enc_toc)
{
	m_clref_t crp;
	int idx;

	idx = enc_toc.mt_segmap[i].mse_index;

	if (enc_toc.mt_segmap[i].mse_type == MVEC_TYPE_MBUF)
		m_free(enc_toc.mt_mdata[idx]);
	if (enc_toc.mt_segmap[i].mse_type == MVEC_TYPE_CLUSTER) {

}
	
/* convert an mvec to a legacy mbuf chain */
struct mbuf *
mvec_serialize_one(struct mbuf *m) {
	struct mvec_toc enc_toc;
	struct mbuf *mh, *mt, *mp;
	m_clref_t crp;
	caddr_t datap;
	int i;

	mh = mt = NULL;
	mvec_unpack(m, &enc_toc);

	/*
	 * XXX this leading logic is a bit convoluted
	 * we special case handling of the first mbuf in the chain
	 */
	if (m->m_flags & M_PKTHDR) {
		if ((mp = m_gethdr(M_NOWAIT, MT_DATA)) == NULL)
			goto cleanup;
		memcpy(mp, m, MPKTHSIZE);
		mp->m_data = &mp->m_pktdat[0];
		if (enc_toc.mt_segmap[0].mse_type == MVEC_TYPE_MBUF) {
			datap = enc_toc.mt_mdata[0] + enc_toc.mt_segs[0].ms_off;
			MPASS(avail >= enc_toc.mt_segs[0].ms_len);
			/*
			 * copy data to first mbuf so as not to confuse drivers assuming
			 * the ip packet header should be contained in the first mbuf
			 */
			memcpy(m->m_data, datap, enc_toc.mt_segs[0].ms_len);
		}
	} else if (enc_toc.mt_segmap[0].mse_type == MVEC_TYPE_MBUF) {
		/* simply re-use the mbuf */
		mp = mvec_serialize_mbuf_init(&enc_toc, 0); 
	} else {
		if ((mp = m_gethdr(M_NOWAIT, MT_DATA)) == NULL)
			goto cleanup;
	}
	mh = mt = mp;
	mp->m_len = enc_toc.mt_segs[0].ms_len;
	if (enc_toc.mt_segmap[0].mse_type == MVEC_TYPE_CLUSTER) {
		mvec_serialize_ext_init(mp, &enc_toc, 0);
	} 
	
	for (i = 1; i < enc_toc.mt_nsegs; i++) {
		if (enc_toc.mt_segmap[i].mse_type == MVEC_TYPE_MBUF) {
			mp = mvec_serialize_mbuf_init(&enc_toc, i); 
		} else if (enc_toc.mt_segmap[0].mse_type == MVEC_TYPE_CLUSTER) {
			if ((mp = m_gethdr(M_NOWAIT, MT_DATA)) == NULL)
				goto cleanup;
			mvec_serialize_ext_init(mp, &enc_toc, idx, i);
		} else
			panic("unexpected type in mvec_serialize_one");
		mt->m_next = mp;
		mt = mp;
	}

	rerturn (h);
cleanup:
	if (mh)
		m_freem(mh);
	if (mp)
		m_free(mp);
	for (; i < enc_toc.mt_nsegs; i++) {
		mvec_free_idx(enc_toc, i);
	}


	return (NULL);
}


#if 0
void
mvec_pack(struct mbuf *m, mvec_toc_t toc)
{
	struct mvec_toc enc_toc;
	mvec_hdr_t mh;
	int i, cnt, count, size;
	uint64_t *cursor64, *enc_cursor64;
	uint32_t *cursor32, *enc_cursor32;
	uint16_t *cursor16, *enc_cursor16;
	uint8_t *cursor8, *enc_cursor8;

	if (m->m_flags & M_PKTHDR)
		mh = (mvec_hdr_t)&m->m_pktdat;
	else
		mh = (mvec_hdr_t)(((caddr_t)m) + sizeof(struct mbuf_lite));
	*((uint64_t*)mh) = *((uint64_t *)toc->mt_mh);

	/* calculate the offsets */
	mvec_unpack(m, &enc_toc);
	/* initialize internal references */
	if (mh->mh_ncliref > 0) {
		/* first is the refcount to the mvec itself */
		enc_toc.mt_iref[0] = mh->mh_ncliref;
		for (i = 1; i < mh->mh_ncliref + 1; i++)
			enc_toc.mt_iref[i] = 1;
	}
	/* copy ext_free function / data */
	enc_toc.mt_mext.me_free = toc->mt_mext.me_free;
	enc_toc.mt_mext.me_arg1 = = toc->mt_mext.me_arg1;
	enc_toc.mt_mext.me_arg2 = toc->mt_mext.me_arg2;
	/* copy clusters */
	if (mh->mh_ncl > 0) {
		cursor64 = (uint64_t *)toc->mt_cl;
		enc_cursor64 = (uint64_t *)enc_toc.mt_cl;
		for (i = 0; i < mh->mh_cl_size; i++, cursor64++, enc_cursor64++)
			*enc_cursor64 = *cursor64;
	}
	/* copy mbufs and their flags */
	if ((cnt = mh->mh_nmdata) > 0) {
		cursor64 = (uint64_t *)toc->mt_mdata;
		enc_cursor64 = (uint64_t *)enc_toc.mt_mdata;
		size = ((cnt * (sizeof(struct mbuf *) + sizeof(struct m_mbuf_meta))) + 8) & ~0x8;
		for (i = 0; i < size; i++, cursor64++, enc_cursor64++)
			*enc_cursor64 = *cursor64;
	}
	/* copy any block data pointers */	
	if (mh->mh_ndataptr > 0) {
		cursor64 = (uint64_t *)toc->mt_dataptr;
		enc_cursor64 = (uint64_t *)enc_toc.mt_dataptr;
		for (i = 0; i < mh->mh_ndataptr; i++, cursor64++, enc_cursor64++)
			*enc_cursor64 = *cursor64;
	}
	/* copy the segments and the segment map */
	size = mh->mh_nsegs * (sizeof(*m_segmap_ent_t) + sizeof(*m_seg_t));
	switch (size & 0x3) {
	case 0:
		count = (size >> 2);
		cursor32 = (uint32_t *)toc->mt_segs;
		enc_cursor32 = (uint32_t *)enc_toc.mt_segs;
		for (i = 0; i < count; i++, cursor32++, enc_cursor32++)
			*enc_cursor32 = *cursor32;
		break;
	case 2:
		count = (size >> 1);
		cursor16 = (uint16_t *)toc->mt_segs;
		enc_cursor16 = (uint16_t *)enc_toc.mt_segs;
		for (i = 0; i < count; i++, cursor16++, enc_cursor16++)
			*enc_cursor16 = *cursor16;
		break;
	case 1:
	case 3:
	default:	
		count = size;
		cursor8 = (uint8_t *)toc->mt_segs;
		enc_cursor8 = (uint8_t *)enc_toc.mt_segs;
		for (i = 0; i < count; i++, cursor8++, enc_cursor8++)
			*enc_cursor8 = *cursor8;
		break;
	}
}
#endif
