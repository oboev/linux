// SPDX-License-Identifier: GPL-2.0
/*
 * VCPM per-subgraph persistent calibration for the hostless voice graphs.
 *
 * The vendor's voice path registers a VCPM_PARAM_ID_CAL_TABLE payload for
 * each voice subgraph immediately after that graph's GRAPH_OPEN, out of band,
 * with APM_CMD_REGISTER_CFG addressed to APM while the payload names VCPM
 * (audioreach-graphservices gsl_graph_send_persist_cal()), and deregisters it
 * at close. The tables for the two transmit stream subgraphs carry, among
 * the vendor's own tuning, the output media format of every transmit MFC
 * once per SAMPLING_RATE key - 8, 16, 32 and 48 kHz - and the modem's voice
 * service supplies that key on every call. With them registered the transmit
 * chain follows the vocoder the network negotiated; the AP writes nothing to
 * those MFCs (audioreach_voice_media_format()). Without them a narrowband
 * call's encoder refuses its input and the uplink is silent (experiment
 * 0038).
 *
 * The tables are firmware files built from the handset ACDB's own blobs,
 * filtered to the modules the shipped topology carries and pinned here by
 * size and SHA-256. Everything is fail-closed: a table is only registered if
 * its firmware matches its digest, its wrapper agrees with its content, its
 * embedded subgraph is the expected one and is open exactly once, every
 * module instance it addresses is VCPM's own or a module of that subgraph,
 * and no other registration or shared-memory command is in flight. A second
 * registration that fails takes the first one down again while both graphs
 * are still unstarted.
 */
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/unaligned.h>
#include <crypto/sha2.h>
#include "q6apm.h"

#define Q6APM_VCAL_PORT_SG_TX_A		0x400c	/* vendor 0xb000003e */
#define Q6APM_VCAL_PORT_SG_TX_B		0x400d	/* vendor 0xb000003f */

/*
 * Module instances the vendor's TX stream blobs address that the shipped
 * topology does not carry: SMECNS_V2 (0x41a0), which this ADSP refuses to
 * instantiate, and its neighbour 0x41a4. Generated from the topology's
 * manifest; a table addressing either has not been filtered against the
 * deployed topology and must not be registered.
 */
static const u32 q6apm_vcal_absent_iids[] = {
	0x41a0, 0x41a4,
};

struct q6apm_vcal_fw {
	const char *name;
	u32 sg_id;
	size_t size;			/* the wrapped file */
	u8 sha256[SHA256_DIGEST_SIZE];
};

/*
 * The two tables, pinned by size and digest (voice-audio/vcpm-tables in the
 * port's project builds them; the device package ships them). The kernel
 * refuses anything else, so a stale or mis-built file cannot reach the DSP.
 */
static const struct q6apm_vcal_fw q6apm_vcal_fw_400c = {
	.name = "qcom/sm8650/oneplus/giulia/vcpm-cal-400c.bin",
	.sg_id = Q6APM_VCAL_PORT_SG_TX_A, .size = 10632,
	.sha256 = { 0x67, 0x1a, 0xa5, 0x1e, 0x74, 0x9d, 0xec, 0x2b,
		    0xac, 0xa8, 0xed, 0x29, 0x91, 0x95, 0x3c, 0x04,
		    0xd7, 0x9f, 0x11, 0x12, 0x82, 0x65, 0x7e, 0xb9,
		    0xe0, 0xb9, 0x08, 0x47, 0x78, 0x72, 0x45, 0x6a },
};

static const struct q6apm_vcal_fw q6apm_vcal_fw_400d = {
	.name = "qcom/sm8650/oneplus/giulia/vcpm-cal-400d.bin",
	.sg_id = Q6APM_VCAL_PORT_SG_TX_B, .size = 3768,
	.sha256 = { 0x6f, 0x86, 0xda, 0x21, 0x63, 0x4a, 0x18, 0xef,
		    0x47, 0x1a, 0xb8, 0x6c, 0x1e, 0xa2, 0x35, 0x0f,
		    0x3a, 0x06, 0x90, 0x93, 0xb9, 0x41, 0xd2, 0x3b,
		    0x3e, 0xed, 0xb6, 0x55, 0x5c, 0xf1, 0xf3, 0x7f },
};

/*
 * Registration order matters: the vendor registers sg3e before sg3f, and a
 * partial set is never left standing.
 */
static const struct q6apm_vcal_fw *const q6apm_vcal_tables[Q6APM_VCAL_MAX_TABLES] = {
	&q6apm_vcal_fw_400c, &q6apm_vcal_fw_400d,
};

/* ------------------------------------------------------------------ blob */

/*
 * The emitted VCPM_PARAM_ID_CAL_TABLE layout, from BuildVcpmBlob. Only what
 * the qualification needs is decoded: the identity of the table, and every
 * module instance any reachable data-pool record addresses.
 *
 *   apm_module_param_data {iid, pid, param_size, error_code}
 *   u32 num_subgraphs (1)
 *   {subgraph_id, table_size, major, minor, master_key_offset (0), num_ckv}
 *     per ckv table:  {table_size, key_table_offset, cal_dot_size, num_obj}
 *       per object:   {lut_offset, num_data_offsets}
 *                     num_data_offsets x {data_pool_offset, is_persistent}
 *   u32 size; master keys
 *   u32 size; cal key ids
 *   u32 size; cal key values
 *   u32 size; data pool: {iid, pid, param_size, error_code, payload, pad8}
 */
#define VCAL_SGID_PERSIST_HDR	8
#define VCAL_MODULE_HDR		16

struct vcal_reader {
	const u8 *buf;
	size_t len;
	size_t pos;
};

static int vcal_u32(struct vcal_reader *r, u32 *v)
{
	if (r->pos > r->len || r->len - r->pos < sizeof(*v))
		return -EBADMSG;
	*v = get_unaligned_le32(r->buf + r->pos);
	r->pos += sizeof(*v);
	return 0;
}

#define VCAL_RD(r, v) do { if (vcal_u32((r), (v))) return -EBADMSG; } while (0)

/* Does @iid belong to @sg in the graph that owns that subgraph? */
static bool vcal_iid_in_subgraph(const struct audioreach_graph_info *info,
				 u32 sg_id, u32 iid)
{
	struct audioreach_sub_graph *sg;
	struct audioreach_container *cont;
	struct audioreach_module *mod;

	list_for_each_entry(sg, &info->sg_list, node) {
		if (sg->sub_graph_id != sg_id)
			continue;
		list_for_each_entry(cont, &sg->container_list, node)
			list_for_each_entry(mod, &cont->modules_list, node)
				if (mod->instance_id == iid)
					return true;
	}
	return false;
}

/*
 * Walk the table and prove it addresses only this subgraph. Returns the number
 * of data-pool records on success. Nothing is written; a malformed table is
 * rejected rather than repaired.
 */
static int vcal_qualify_blob(struct device *dev,
			     const struct audioreach_graph_info *info,
			     const u8 *plain, size_t plain_len, u32 want_sg)
{
	struct vcal_reader r = { plain, plain_len, 0 };
	u32 iid, pid, param_size, err, num_sg, sg_id, tbl_size;
	u32 major, minor, mk_off, num_ckv, i, j, k;
	u32 chunk[4], dp_size, dp_base, cur, records = 0;
	u32 *dp_offsets;
	size_t sub_info_end;
	int ret = 0;

	VCAL_RD(&r, &iid);
	VCAL_RD(&r, &pid);
	VCAL_RD(&r, &param_size);
	VCAL_RD(&r, &err);
	if (iid != VCPM_MODULE_INSTANCE_ID || pid != VCPM_PARAM_ID_CAL_TABLE) {
		dev_err(dev, "vcpm-cal: table addresses <%#x, %#x>, not VCPM's cal table\n",
			iid, pid);
		return -EBADMSG;
	}
	if (err) {
		dev_err(dev, "vcpm-cal: table error_code is %#x\n", err);
		return -EBADMSG;
	}
	if (param_size > plain_len - VCAL_MODULE_HDR ||
	    VCAL_MODULE_HDR + ALIGN(param_size, 8) != plain_len) {
		dev_err(dev, "vcpm-cal: param_size %u against a %zu-byte table\n",
			param_size, plain_len);
		return -EBADMSG;
	}

	VCAL_RD(&r, &num_sg);
	if (num_sg != 1) {
		dev_err(dev, "vcpm-cal: num_subgraphs is %u\n", num_sg);
		return -EBADMSG;
	}
	VCAL_RD(&r, &sg_id);
	VCAL_RD(&r, &tbl_size);
	VCAL_RD(&r, &major);
	VCAL_RD(&r, &minor);
	VCAL_RD(&r, &mk_off);
	VCAL_RD(&r, &num_ckv);
	if (sg_id != want_sg) {
		dev_err(dev, "vcpm-cal: table owns subgraph %#x, the arm names %#x\n",
			sg_id, want_sg);
		return -EBADMSG;
	}
	if (mk_off) {
		dev_err(dev, "vcpm-cal: master-key-table offset is %#x, the vendor zeroes it\n",
			mk_off);
		return -EBADMSG;
	}
	if (!num_ckv || num_ckv > 64)
		return -EBADMSG;

	/* pass 1: how many records, so the offset list can be sized */
	{
		struct vcal_reader s = r;

		for (i = 0; i < num_ckv; i++) {
			u32 t_size, key_off, dot_size, num_obj;

			VCAL_RD(&s, &t_size);
			VCAL_RD(&s, &key_off);
			VCAL_RD(&s, &dot_size);
			VCAL_RD(&s, &num_obj);
			if (t_size != 12 + dot_size || !num_obj || num_obj > 4096)
				return -EBADMSG;
			for (j = 0; j < num_obj; j++) {
				u32 lut_off, nparams;

				VCAL_RD(&s, &lut_off);
				VCAL_RD(&s, &nparams);
				if (!nparams || nparams > 4096)
					return -EBADMSG;
				for (k = 0; k < nparams; k++) {
					u32 tmp;

					VCAL_RD(&s, &tmp);
					VCAL_RD(&s, &tmp);
				}
				records += nparams;
			}
		}
		sub_info_end = s.pos;
	}
	if (!records || records > 8192)
		return -EBADMSG;

	dp_offsets = kcalloc(records, sizeof(*dp_offsets), GFP_KERNEL);
	if (!dp_offsets)
		return -ENOMEM;

	/* pass 2: collect the data-pool offsets in the order they are named */
	records = 0;
	for (i = 0; i < num_ckv; i++) {
		u32 t_size, key_off, dot_size, num_obj;

		if (vcal_u32(&r, &t_size) || vcal_u32(&r, &key_off) ||
		    vcal_u32(&r, &dot_size) || vcal_u32(&r, &num_obj)) {
			ret = -EBADMSG;
			goto out;
		}
		for (j = 0; j < num_obj; j++) {
			u32 lut_off, nparams;

			if (vcal_u32(&r, &lut_off) || vcal_u32(&r, &nparams)) {
				ret = -EBADMSG;
				goto out;
			}
			for (k = 0; k < nparams; k++) {
				u32 off, persist;

				if (vcal_u32(&r, &off) || vcal_u32(&r, &persist)) {
					ret = -EBADMSG;
					goto out;
				}
				dp_offsets[records++] = off;
			}
		}
	}
	if (r.pos != sub_info_end) {
		ret = -EBADMSG;
		goto out;
	}

	/* the four sized chunks; only the data pool is walked */
	for (i = 0; i < 4; i++) {
		if (vcal_u32(&r, &chunk[i])) {
			ret = -EBADMSG;
			goto out;
		}
		if (chunk[i] > r.len - r.pos) {
			ret = -EBADMSG;
			goto out;
		}
		if (i < 3)
			r.pos += chunk[i];
	}
	dp_size = chunk[3];
	dp_base = r.pos;
	if (dp_base + dp_size != VCAL_MODULE_HDR + param_size) {
		dev_err(dev, "vcpm-cal: chunks end at %u, param_size ends at %u\n",
			dp_base + dp_size, VCAL_MODULE_HDR + param_size);
		ret = -EBADMSG;
		goto out;
	}

	/* every record the table names: in range, in this subgraph, contiguous */
	cur = 0;
	for (i = 0; i < records; i++) {
		u32 rec_iid, rec_pid, rec_size, rec_err;
		struct vcal_reader s = { plain, dp_base + dp_size, dp_base + dp_offsets[i] };

		if (dp_offsets[i] != cur) {
			dev_err(dev, "vcpm-cal: record %u names data-pool offset %u, the next unread record is at %u\n",
				i, dp_offsets[i], cur);
			ret = -EBADMSG;
			goto out;
		}
		if (vcal_u32(&s, &rec_iid) || vcal_u32(&s, &rec_pid) ||
		    vcal_u32(&s, &rec_size) || vcal_u32(&s, &rec_err)) {
			ret = -EBADMSG;
			goto out;
		}
		if (rec_err || cur + 16 + ALIGN(rec_size, 8) > dp_size) {
			ret = -EBADMSG;
			goto out;
		}
		if (rec_iid != VCPM_MODULE_INSTANCE_ID &&
		    !vcal_iid_in_subgraph(info, want_sg, rec_iid)) {
			dev_err(dev, "vcpm-cal: record %u addresses iid %#x, which is not a module of subgraph %#x\n",
				i, rec_iid, want_sg);
			ret = -EBADMSG;
			goto out;
		}
		for (j = 0; j < ARRAY_SIZE(q6apm_vcal_absent_iids); j++) {
			if (rec_iid == q6apm_vcal_absent_iids[j]) {
				dev_err(dev, "vcpm-cal: record %u addresses iid %#x, declared absent from this topology\n",
					i, rec_iid);
				ret = -EBADMSG;
				goto out;
			}
		}
		cur += 16 + ALIGN(rec_size, 8);
	}
	if (cur != dp_size) {
		dev_err(dev, "vcpm-cal: %u of %u data-pool bytes reachable\n", cur, dp_size);
		ret = -EBADMSG;
		goto out;
	}
	ret = records;
out:
	kfree(dp_offsets);
	return ret;
}

/* ------------------------------------------------------- graph qualification */

/*
 * Every instance declared absent must really be absent from every open graph.
 */
static int vcal_qualify_absent(struct device *dev, struct q6apm *apm)
{
	struct audioreach_graph *ar_graph;
	struct audioreach_sub_graph *sg;
	struct audioreach_container *cont;
	struct audioreach_module *mod;
	int id, i;

	guard(mutex)(&apm->lock);
	idr_for_each_entry(&apm->graph_idr, ar_graph, id)
		list_for_each_entry(sg, &ar_graph->info->sg_list, node)
			list_for_each_entry(cont, &sg->container_list, node)
				list_for_each_entry(mod, &cont->modules_list, node)
					for (i = 0; i < ARRAY_SIZE(q6apm_vcal_absent_iids); i++)
						if (mod->instance_id == q6apm_vcal_absent_iids[i]) {
							dev_err(dev, "vcpm-cal: iid %#x is present in graph %d but declared absent\n",
								mod->instance_id, id);
							return -EINVAL;
						}
	return 0;
}

/*
 * The one open graph that owns @sg_id. The DSP files a CAL_TABLE by the
 * subgraph id in its payload, whatever port it arrives on, so a table is
 * qualified against the graph that really carries its subgraph - the RX
 * graph for 0x400b although the TX graph sends it. The subgraph must be open
 * exactly once; the returned info belongs to the topology and outlives the
 * graph.
 */
static const struct audioreach_graph_info *
vcal_find_owner(struct device *dev, struct q6apm *apm, u32 sg_id)
{
	const struct audioreach_graph_info *owner = NULL;
	struct audioreach_graph *ar_graph;
	struct audioreach_sub_graph *sg;
	int id, owner_id = -1, seen = 0;

	scoped_guard(mutex, &apm->lock) {
		idr_for_each_entry(&apm->graph_idr, ar_graph, id)
			list_for_each_entry(sg, &ar_graph->info->sg_list, node)
				if (sg->sub_graph_id == sg_id) {
					owner = ar_graph->info;
					owner_id = id;
					seen++;
				}
	}
	if (seen != 1) {
		dev_err(dev, "vcpm-cal: subgraph %#x is open %d times across the open graphs; it must be open exactly once\n",
			sg_id, seen);
		return NULL;
	}
	dev_dbg(dev, "vcpm-cal: subgraph %#x is owned by open graph %d\n",
		 sg_id, owner_id);
	return owner;
}

/* --------------------------------------------------------------- ownership */

static void vcal_free(struct device *dev, struct q6apm_vcal_table *t)
{
	if (t->virt) {
		dma_free_coherent(dev, t->size, t->virt, t->phys);
		t->virt = NULL;
	}
	t->allocated = false;
}

/*
 * The DSP is told an address in its own stream: the buffer's IOVA in the
 * q6apm-dais IOMMU domain with the SMMU stream id in the high word, exactly
 * what q6apm-dai does for a PCM buffer. Allocating against any other device,
 * or handing over the bare IOVA, gives the DSP an address it cannot resolve.
 */
static u64 vcal_dsp_addr(const struct q6apm_vcal_table *t)
{
	if (t->sid < 0)
		return t->phys;
	return (u64)t->phys | ((u64)t->sid << 32);
}

static int vcal_load(struct q6apm_graph *graph, struct q6apm_vcal_table *t,
		     const struct q6apm_vcal_fw *fw, long long sid)
{
	struct device *dev = graph->dev;
	const struct firmware *blob;
	u8 digest[SHA256_DIGEST_SIZE];
	int ret;

	t->fw = fw;
	ret = request_firmware(&blob, fw->name, dev);
	if (ret) {
		dev_err(dev, "vcpm-cal: %s: not available (%d)\n", fw->name, ret);
		t->build_rc = ret;
		return ret;
	}
	if (blob->size != fw->size) {
		dev_err(dev, "vcpm-cal: %s: %zu bytes, expected %zu\n",
			fw->name, blob->size, fw->size);
		ret = -EINVAL;
		goto out;
	}
	sha256(blob->data, blob->size, digest);
	if (memcmp(digest, fw->sha256, sizeof(digest))) {
		dev_err(dev, "vcpm-cal: %s: sha256 does not match the built table\n",
			fw->name);
		ret = -EINVAL;
		goto out;
	}

	/* the AcdbSgIdPersistData wrapper GSL allocates against */
	if (blob->size < VCAL_SGID_PERSIST_HDR + VCAL_MODULE_HDR) {
		ret = -EINVAL;
		goto out;
	}
	t->sg_id = get_unaligned_le32(blob->data);
	t->payload_size = get_unaligned_le32(blob->data + 4);
	if (t->sg_id != fw->sg_id ||
	    t->payload_size != blob->size - VCAL_SGID_PERSIST_HDR) {
		dev_err(dev, "vcpm-cal: %s: wrapper {%#x, %u} against %zu bytes for subgraph %#x\n",
			fw->name, t->sg_id, t->payload_size, blob->size, fw->sg_id);
		t->wrapper_rc = -EINVAL;
		ret = -EINVAL;
		goto out;
	}

	t->size = blob->size;
	t->sid = sid;
	t->virt = dma_alloc_coherent(dev, t->size, &t->phys, GFP_KERNEL);
	if (!t->virt) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy(t->virt, blob->data, t->size);
	t->allocated = true;
	memcpy(t->sha256, digest, sizeof(digest));
	dev_dbg(dev, "vcpm-cal: %s loaded: %zu bytes, subgraph %#x, payload %u, sid %lld, sha256 %*phN\n",
		 fw->name, t->size, t->sg_id, t->payload_size, t->sid,
		 SHA256_DIGEST_SIZE, t->sha256);
out:
	t->build_rc = ret;
	release_firmware(blob);
	return ret;
}

/* ---------------------------------------------------------------- the wire */

static int vcal_send(struct q6apm_graph *graph, struct q6apm_vcal_table *t,
		     u32 opcode)
{
	struct device *dev = graph->dev;
	struct apm_cmd_header *hdr;
	u64 payload_addr = vcal_dsp_addr(t) + VCAL_SGID_PERSIST_HDR;
	int ret;
	struct gpr_pkt *pkt __free(kfree) =
		audioreach_alloc_cmd_pkt(0, opcode, 0, graph->port->id,
					 APM_MODULE_INSTANCE_ID);

	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	hdr = (void *)pkt + GPR_HDR_SIZE;
	hdr->payload_address_lsw = lower_32_bits(payload_addr);
	hdr->payload_address_msw = upper_32_bits(payload_addr);
	hdr->mem_map_handle = t->mem_map_handle;
	hdr->payload_size = t->payload_size;

	/* the whole outbound packet and the exact bytes it points at, before the send */
	dev_dbg(dev, "vcpm-cal: send seq %u opcode %#x src_port %#x dst_port %#x\n",
		 ++t->seq, opcode, graph->port->id, APM_MODULE_INSTANCE_ID);
	dev_dbg(dev, "vcpm-cal:   gpr %*phN\n",
		 (int)(GPR_HDR_SIZE + APM_CMD_HDR_SIZE), (u8 *)pkt);
	dev_dbg(dev, "vcpm-cal:   oob dsp_addr %#llx (iova %pad) +%u size %u handle %#x sha256 %*phN\n",
		 payload_addr - VCAL_SGID_PERSIST_HDR, &t->phys,
		 VCAL_SGID_PERSIST_HDR, t->payload_size,
		 t->mem_map_handle, SHA256_DIGEST_SIZE, t->sha256);
	if (t->payload_size <= 256)
		dev_dbg(dev, "vcpm-cal:   oob %*phN\n",
			 (int)t->payload_size, (u8 *)t->virt + VCAL_SGID_PERSIST_HDR);

	t->send_called = true;
	t->acknowledged_opcode = 0;
	t->raw_dsp_status_valid = false;
	t->raw_dsp_status = 0;
	t->response_seen = false;

	ret = audioreach_graph_send_cmd_sync(graph, pkt, 0);
	t->send_rc = ret;
	t->wait_completed = ret != -ETIMEDOUT;
	t->response_seen = t->acknowledged_opcode == opcode;

	dev_dbg(dev, "vcpm-cal: seq %u opcode %#x: send_rc %d wait_completed %d response_seen %d acknowledged_opcode %#x raw_dsp_status_valid %d raw_dsp_status %u\n",
		 t->seq, opcode, t->send_rc, t->wait_completed, t->response_seen,
		 t->acknowledged_opcode, t->raw_dsp_status_valid, t->raw_dsp_status);

	if (ret == -ETIMEDOUT)
		dev_warn(dev, "vcpm-cal: seq %u opcode %#x: no correlated response - this is a timeout, not a refusal; no DSP status exists\n",
			 t->seq, opcode);
	return ret;
}

/* ------------------------------------------------------------------- arms */

/**
 * q6apm_voice_cal_register() - register the arm's tables for a TX voice graph
 * @graph: the live TX voice graph, already opened
 * @sid: the SMMU stream id q6apm-dai read from its iommus property, or -1
 *
 * Runs after the TX graph's GRAPH_OPEN and before its VCPM session config,
 * media format, prepare or start - the position gsl_graph_send_persist_cal()
 * occupies. Every table of the arm is sent from this graph's port and
 * qualified against the open graph that owns its subgraph. On any
 * qualification failure nothing is sent, local resources are released, and
 * the graph continues unaffected: a preflight refusal is a recorded outcome,
 * not an error the session has to survive.
 *
 * Returns 0 whether or not tables were registered; only a partial arm that
 * could not be unwound is fatal.
 */
int q6apm_voice_cal_register(struct q6apm_graph *graph, long long sid)
{
	struct q6apm *apm = graph->apm;
	struct device *dev = graph->dev;
	struct q6apm_vcal *vc = &apm->vcal;
	const struct audioreach_graph_info *owner[Q6APM_VCAL_MAX_TABLES] = { };
	unsigned int i;
	int ret, records;

	guard(mutex)(&vc->lock);

	if (vc->ntables) {
		dev_err(dev, "vcpm-cal: tables are still registered from an earlier session; registering nothing\n");
		return 0;
	}
	if (vc->in_flight) {
		dev_err(dev, "vcpm-cal: a registration is already in flight; registering nothing\n");
		return 0;
	}
	vc->ntables = 0;

	if (apm->voice_open_count < 2) {
		dev_err(dev, "vcpm-cal: only %u voice graph(s) open; the vendor registers after both have opened. Registering nothing.\n",
			apm->voice_open_count);
		return 0;
	}
	if (vcal_qualify_absent(dev, apm))
		return 0;
	/* every subgraph the arm names must be open, once, before anything is sent */
	for (i = 0; i < Q6APM_VCAL_MAX_TABLES; i++) {
		owner[i] = vcal_find_owner(dev, apm, q6apm_vcal_tables[i]->sg_id);
		if (!owner[i])
			return 0;
	}

	vc->in_flight = true;
	for (i = 0; i < Q6APM_VCAL_MAX_TABLES; i++) {
		struct q6apm_vcal_table *t = &vc->tbl[i];

		memset(t, 0, sizeof(*t));
		if (vcal_load(graph, t, q6apm_vcal_tables[i], sid))
			goto unwind;

		records = vcal_qualify_blob(dev, owner[i],
					    (u8 *)t->virt + VCAL_SGID_PERSIST_HDR,
					    t->payload_size, t->sg_id);
		if (records < 0) {
			dev_err(dev, "vcpm-cal: %s failed qualification (%d)\n",
				t->fw->name, records);
			vcal_free(dev, t);
			goto unwind;
		}
		dev_dbg(dev, "vcpm-cal: %s qualified: %d record(s), all inside subgraph %#x\n",
			 t->fw->name, records, t->sg_id);

		ret = q6apm_map_cal_region(apm, i, vcal_dsp_addr(t), t->size,
					   &t->mem_map_handle);
		t->map_rc = ret;
		if (ret) {
			dev_err(dev, "vcpm-cal: %s: shared-memory map failed %d\n",
				t->fw->name, ret);
			vcal_free(dev, t);
			goto unwind;
		}
		t->mapped = true;
		dev_dbg(dev, "vcpm-cal: %s: mapped, handle %#x\n",
			 t->fw->name, t->mem_map_handle);

		if (vcal_send(graph, t, APM_CMD_REGISTER_CFG))
			goto unwind;
		t->registered = true;
		vc->ntables = i + 1;
		dev_dbg(dev, "vcpm-cal: %s: REGISTERED for subgraph %#x\n",
			 t->fw->name, t->sg_id);
	}
	vc->in_flight = false;
	vc->graph = graph;
	return 0;

unwind:
	/*
	 * Never continue with an accidental partial arm: drop what is already
	 * registered while both graphs are still unstarted, then release
	 * everything this attempt owns.
	 */
	dev_err(dev, "vcpm-cal: registration incomplete - unwinding %u registered table(s); this call's uplink will be silent\n",
		vc->ntables);
	while (vc->ntables) {
		struct q6apm_vcal_table *t = &vc->tbl[--vc->ntables];

		if (t->registered) {
			if (vcal_send(graph, t, APM_CMD_DEREGISTER_CFG)) {
				dev_err(dev, "vcpm-cal: %s: deregistration failed; its mapping is deliberately leaked for this boot and cleanup is invalid\n",
					t->fw->name);
				t->mapped = false;
				t->virt = NULL;
				t->allocated = false;
				continue;
			}
			t->registered = false;
			t->deregistered = true;
		}
		if (t->mapped) {
			q6apm_unmap_cal_region(apm, vc->ntables, t->mem_map_handle);
			t->mapped = false;
		}
		vcal_free(dev, t);
	}
	for (i = 0; i < Q6APM_VCAL_MAX_TABLES; i++) {
		struct q6apm_vcal_table *t = &vc->tbl[i];

		if (t->mapped) {
			q6apm_unmap_cal_region(apm, i, t->mem_map_handle);
			t->mapped = false;
		}
		vcal_free(dev, t);
	}
	vc->ntables = 0;
	vc->in_flight = false;
	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_voice_cal_register);

/**
 * q6apm_voice_cal_deregister() - drop the arm's tables
 * @graph: the TX voice graph, stopped but not yet closed
 *
 * Reverse order, while the graph port is still live, mirroring GSL's
 * close-time ordering. A mapping is unmapped and freed only after its
 * deregistration has been acknowledged with status zero; a table that could
 * not be dropped keeps its buffer for the rest of the boot rather than handing
 * the DSP memory back underneath it.
 */
void q6apm_voice_cal_deregister(struct q6apm_graph *graph)
{
	struct q6apm *apm = graph->apm;
	struct device *dev = graph->dev;
	struct q6apm_vcal *vc = &apm->vcal;

	guard(mutex)(&vc->lock);

	if (!vc->ntables)
		return;
	if (vc->graph != graph) {
		dev_err(dev, "vcpm-cal: teardown on a graph that did not register; leaving the tables alone\n");
		return;
	}

	while (vc->ntables) {
		struct q6apm_vcal_table *t = &vc->tbl[--vc->ntables];

		if (!t->registered)
			continue;
		if (vcal_send(graph, t, APM_CMD_DEREGISTER_CFG)) {
			dev_err(dev, "vcpm-cal: %s: deregistration failed - the mapping is not reused or freed this boot, cleanup is INVALID, power-cycle before another arm\n",
				t->fw->name);
			t->mapped = false;
			t->virt = NULL;
			t->allocated = false;
			continue;
		}
		t->registered = false;
		t->deregistered = true;
		dev_dbg(dev, "vcpm-cal: %s: DEREGISTERED for subgraph %#x\n",
			 t->fw->name, t->sg_id);
		q6apm_unmap_cal_region(apm, vc->ntables, t->mem_map_handle);
		t->mapped = false;
		vcal_free(dev, t);
	}
	vc->graph = NULL;
}
EXPORT_SYMBOL_GPL(q6apm_voice_cal_deregister);
