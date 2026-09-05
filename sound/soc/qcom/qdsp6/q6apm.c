// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Linaro Limited

#include <dt-bindings/soc/qcom,gpr.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/soc/qcom/apr.h>
#include <linux/wait.h>
#include <sound/soc.h>
#include <sound/soc-card.h>
#include <sound/soc-dapm.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include "audioreach.h"
#include "q6apm.h"

/* Graph Management */
struct apm_graph_mgmt_cmd {
	struct apm_module_param_data param_data;
	uint32_t num_sub_graphs;
	uint32_t sub_graph_id_list[];
} __packed;

#define APM_GRAPH_MGMT_PSIZE(p, n) ALIGN(struct_size(p, sub_graph_id_list, n), 8)

static struct q6apm *g_apm;

int q6apm_send_cmd_sync(struct q6apm *apm, const struct gpr_pkt *pkt,
			uint32_t rsp_opcode)
{
	gpr_device_t *gdev = apm->gdev;

	return audioreach_send_cmd_sync(&gdev->dev, gdev, &apm->result, &apm->lock,
					NULL, &apm->wait, pkt, rsp_opcode);
}

static bool q6apm_info_is_voice(struct audioreach_graph_info *info, int dir);

static bool audioreach_info_has_module(const struct audioreach_graph_info *info, uint32_t iid)
{
	struct audioreach_container *container;
	struct audioreach_sub_graph *sgs;
	struct audioreach_module *module;

	list_for_each_entry(sgs, &info->sg_list, node)
		list_for_each_entry(container, &sgs->container_list, node)
			list_for_each_entry(module, &container->modules_list, node)
				if (module->instance_id == iid)
					return true;
	return false;
}

/*
 * VCPM is told about a voice graph exactly once, at the
 * front end's GRAPH_OPEN; the RX backend (the I2S sink) opens later, at the
 * backend DAI's prepare, in an open that carries the FE->BE link and that APM
 * does not forward to any proxy. The vendor opens stream, post-processing and
 * device sub-graphs in one command. So: for a voice RX graph, find the
 * backend it is routed to (the graph info that holds a link FROM one of this
 * graph's modules - audioreach_connect_sub_graphs() put it there) and, if that
 * backend is not open yet, open it in the same GRAPH_OPEN. Returns NULL for
 * every other graph, and for a backend that is already open (today's path).
 * Caller holds no apm lock.
 */
static struct audioreach_graph_info *q6apm_voice_rx_backend_info(struct q6apm *apm,
								 struct audioreach_graph_info *info)
{
	struct audioreach_graph_info *cand;
	int id;

	if (!q6apm_info_is_voice(info, SNDRV_PCM_STREAM_PLAYBACK))
		return NULL;

	mutex_lock(&apm->lock);
	idr_for_each_entry(&apm->graph_info_idr, cand, id) {
		if (cand == info || !cand->src_mod_inst_id || !cand->dst_mod_inst_id)
			continue;
		if (!audioreach_info_has_module(info, cand->src_mod_inst_id))
			continue;
		if (idr_find(&apm->graph_idr, id))
			cand = NULL;	/* already open: nothing to carry */
		break;
	}
	mutex_unlock(&apm->lock);

	return cand;
}

static struct audioreach_graph *q6apm_get_audioreach_graph(struct q6apm *apm, uint32_t graph_id)
{
	struct audioreach_graph_info *info, *be_info;
	struct audioreach_graph *graph, *be = NULL;
	int id;

	mutex_lock(&apm->lock);
	graph = idr_find(&apm->graph_idr, graph_id);
	mutex_unlock(&apm->lock);

	if (graph) {
		kref_get(&graph->refcount);
		return graph;
	}

	info = idr_find(&apm->graph_info_idr, graph_id);

	if (!info)
		return ERR_PTR(-ENODEV);

	graph = kzalloc_obj(*graph);
	if (!graph)
		return ERR_PTR(-ENOMEM);

	graph->apm = apm;
	graph->info = info;
	graph->id = graph_id;

	be_info = q6apm_voice_rx_backend_info(apm, info);
	if (be_info) {
		be = kzalloc_obj(*be);
		if (!be) {
			kfree(graph);
			return ERR_PTR(-ENOMEM);
		}
		be->apm = apm;
		be->info = be_info;
		be->id = be_info->id;
		graph->adopted = be;
		graph->graph = audioreach_alloc_graph_pkt_merged(apm, info, be_info);
	} else {
		graph->graph = audioreach_alloc_graph_pkt(apm, info);
	}
	if (IS_ERR(graph->graph)) {
		void *err = graph->graph;

		kfree(be);
		kfree(graph);
		return ERR_CAST(err);
	}

	mutex_lock(&apm->lock);
	id = idr_alloc(&apm->graph_idr, graph, graph_id, graph_id + 1, GFP_KERNEL);
	if (id < 0) {
		dev_err(apm->dev, "Unable to allocate graph id (%d)\n", graph_id);
		kfree(graph->graph);
		kfree(graph);
		mutex_unlock(&apm->lock);
		return ERR_PTR(id);
	}
	mutex_unlock(&apm->lock);

	kref_init(&graph->refcount);

	if (be) {
		mutex_lock(&apm->lock);
		id = idr_alloc(&apm->graph_idr, be, be->id, be->id + 1, GFP_KERNEL);
		if (id < 0)
			idr_remove(&apm->graph_idr, graph_id);
		mutex_unlock(&apm->lock);
		if (id < 0) {
			dev_err(apm->dev, "Unable to register backend graph %d beside graph %d (%d)\n",
				be->id, graph_id, id);
			kfree(graph->graph);
			kfree(graph);
			kfree(be);
			return ERR_PTR(id);
		}
		kref_init(&be->refcount);
	}

	graph->open_result.rc = q6apm_send_cmd_sync(apm, graph->graph, 0);
	graph->open_result.opcode = apm->result.opcode;
	graph->open_result.status = apm->result.status;
	graph->open_result.seen = true;

	if (be) {
		be->open_result = graph->open_result;
		dev_dbg(apm->dev, "GRAPH_OPEN graph %d opened backend graph %d with it: rc %d opcode 0x%x status %u\n",
			 graph_id, be->id, graph->open_result.rc,
			 graph->open_result.opcode, graph->open_result.status);
	}

	return graph;
}

static int audioreach_graph_mgmt_cmd(struct audioreach_graph *graph, uint32_t opcode)
{
	struct audioreach_graph_info *info = graph->info;
	int num_sub_graphs = info->num_sub_graphs;
	struct apm_module_param_data *param_data;
	struct apm_graph_mgmt_cmd *mgmt_cmd;
	struct audioreach_sub_graph *sg;
	struct q6apm *apm = graph->apm;
	int i = 0, payload_size = APM_GRAPH_MGMT_PSIZE(mgmt_cmd, num_sub_graphs);

	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(payload_size, opcode, 0);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	mgmt_cmd = (void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;

	mgmt_cmd->num_sub_graphs = num_sub_graphs;

	param_data = &mgmt_cmd->param_data;
	param_data->module_instance_id = APM_MODULE_INSTANCE_ID;
	param_data->param_id = APM_PARAM_ID_SUB_GRAPH_LIST;
	param_data->param_size = payload_size - APM_MODULE_PARAM_DATA_SIZE;

	list_for_each_entry(sg, &info->sg_list, node)
		mgmt_cmd->sub_graph_id_list[i++] = sg->sub_graph_id;

	return q6apm_send_cmd_sync(apm, pkt, 0);
}

static void q6apm_put_audioreach_graph(struct kref *ref)
{
	struct audioreach_graph *graph, *adopted;
	struct q6apm *apm;

	graph = container_of(ref, struct audioreach_graph, refcount);
	apm = graph->apm;

	audioreach_graph_mgmt_cmd(graph, APM_CMD_GRAPH_CLOSE);

	mutex_lock(&apm->lock);
	graph = idr_remove(&apm->graph_idr, graph->id);
	mutex_unlock(&apm->lock);

	adopted = graph->adopted;
	kfree(graph->graph);
	kfree(graph);

	/* the backend this graph opened in its own GRAPH_OPEN */
	if (adopted)
		kref_put(&adopted->refcount, q6apm_put_audioreach_graph);
}


static int q6apm_get_apm_state(struct q6apm *apm)
{
	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(0,
								APM_CMD_GET_SPF_STATE, 0);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	q6apm_send_cmd_sync(apm, pkt, APM_CMD_RSP_GET_SPF_STATE);

	return apm->state;
}

bool q6apm_is_adsp_ready(void)
{
	if (g_apm)
		return q6apm_get_apm_state(g_apm);

	return false;
}
EXPORT_SYMBOL_GPL(q6apm_is_adsp_ready);

static struct audioreach_module *__q6apm_find_module_by_mid(struct q6apm *apm,
						    struct audioreach_graph_info *info,
						    uint32_t mid)
{
	struct audioreach_container *container;
	struct audioreach_sub_graph *sgs;
	struct audioreach_module *module;

	list_for_each_entry(sgs, &info->sg_list, node) {
		list_for_each_entry(container, &sgs->container_list, node) {
			list_for_each_entry(module, &container->modules_list, node) {
				if (mid == module->module_id)
					return module;
			}
		}
	}

	return NULL;
}

int q6apm_graph_media_format_shmem(struct q6apm_graph *graph,
				   struct audioreach_module_config *cfg)
{
	struct audioreach_module *module;

	if (cfg->direction == SNDRV_PCM_STREAM_CAPTURE) {
		module = q6apm_find_module_by_mid(graph, MODULE_ID_SH_MEM_PUSH_MODE);
		if (!module)
			module = q6apm_find_module_by_mid(graph, MODULE_ID_RD_SHARED_MEM_EP);
	} else {
		module = q6apm_find_module_by_mid(graph, MODULE_ID_SH_MEM_PULL_MODE);
		if (!module)
			module = q6apm_find_module_by_mid(graph, MODULE_ID_WR_SHARED_MEM_EP);
	}

	if (!module) {
		dev_err(graph->dev, "No SHMEM module found in graph\n");
		return -ENODEV;
	}

	return audioreach_set_media_format(graph, module, cfg);
}
EXPORT_SYMBOL_GPL(q6apm_graph_media_format_shmem);

static int __q6apm_map_memory_fixed_region(struct device *dev, unsigned int graph_id,
					   phys_addr_t phys, size_t sz, bool is_pos_buf)
{
	struct audioreach_graph_info *info;
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct apm_shared_map_region_payload *mregions;
	struct apm_cmd_shared_mem_map_regions *cmd;
	int payload_size = sizeof(*cmd) + (sizeof(*mregions));
	uint32_t buf_sz;
	void *p;
	uint32_t pos_mask = is_pos_buf ? APM_MMAP_TOKEN_MAP_TYPE_POS_BUF : 0;
	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(payload_size,
					APM_CMD_SHARED_MEM_MAP_REGIONS, (graph_id | pos_mask));

	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	info = idr_find(&apm->graph_info_idr, graph_id);
	if (!info)
		return -ENODEV;

	if (is_pos_buf) {
		if (info->pos_buf_mem_map_handle)
			return 0;
	} else {
		if (info->mem_map_handle)
			return 0;
	}

	/* DSP expects size should be aligned to 4K */
	buf_sz = ALIGN(sz, 4096);

	p = (void *)pkt + GPR_HDR_SIZE;
	cmd = p;
	cmd->mem_pool_id = APM_MEMORY_MAP_SHMEM8_4K_POOL;
	cmd->num_regions = 1;
	if (is_pos_buf)
		cmd->property_flag = 0x2;
	else
		cmd->property_flag = 0x0;

	mregions = p + sizeof(*cmd);

	mregions->shm_addr_lsw = lower_32_bits(phys);
	mregions->shm_addr_msw = upper_32_bits(phys);
	mregions->mem_size_bytes = buf_sz;

	return q6apm_send_cmd_sync(apm, pkt, APM_CMD_RSP_SHARED_MEM_MAP_REGIONS);
}

/*
 * A dedicated mapping for one VCPM persistent-calibration table. It is neither
 * a graph data buffer nor a position buffer, so it carries its own token type
 * and slot and its handle is returned to the caller rather than stored on a
 * graph: two tables can be live at once, and each must be unmapped against its
 * own handle.
 */
int q6apm_map_cal_region(struct q6apm *apm, unsigned int slot, u64 dsp_addr,
			 size_t sz, uint32_t *handle)
{
	struct apm_shared_map_region_payload *mregions;
	struct apm_cmd_shared_mem_map_regions *cmd;
	int payload_size = sizeof(*cmd) + sizeof(*mregions);
	uint32_t token;
	void *p;
	int ret;

	if (slot >= Q6APM_VCAL_MAX_TABLES)
		return -EINVAL;

	token = APM_MMAP_TOKEN_MAP_TYPE_CAL |
		(slot ? APM_MMAP_TOKEN_CAL_SLOT : 0);

	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(payload_size,
					APM_CMD_SHARED_MEM_MAP_REGIONS, token);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	p = (void *)pkt + GPR_HDR_SIZE;
	cmd = p;
	cmd->mem_pool_id = APM_MEMORY_MAP_SHMEM8_4K_POOL;
	cmd->num_regions = 1;
	cmd->property_flag = 0x0;

	mregions = p + sizeof(*cmd);
	mregions->shm_addr_lsw = lower_32_bits(dsp_addr);
	mregions->shm_addr_msw = upper_32_bits(dsp_addr);
	mregions->mem_size_bytes = ALIGN(sz, 4096);

	apm->vcal.tbl[slot].mem_map_handle = 0;
	ret = q6apm_send_cmd_sync(apm, pkt, APM_CMD_RSP_SHARED_MEM_MAP_REGIONS);
	if (ret)
		return ret;
	if (!apm->vcal.tbl[slot].mem_map_handle)
		return -ENODEV;

	*handle = apm->vcal.tbl[slot].mem_map_handle;
	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_map_cal_region);

int q6apm_unmap_cal_region(struct q6apm *apm, unsigned int slot, uint32_t handle)
{
	struct apm_cmd_shared_mem_unmap_regions *cmd;
	uint32_t token;
	int ret;

	if (slot >= Q6APM_VCAL_MAX_TABLES || !handle)
		return -EINVAL;

	/*
	 * The map type has to survive into the response: the shared unmap
	 * handler resolves an ordinary token through the graph idr, which a
	 * calibration mapping is not in.
	 */
	token = APM_MMAP_TOKEN_MAP_TYPE_CAL |
		(slot ? APM_MMAP_TOKEN_CAL_SLOT : 0);

	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(sizeof(*cmd),
					APM_CMD_SHARED_MEM_UNMAP_REGIONS, token);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	cmd = (void *)pkt + GPR_HDR_SIZE;
	cmd->mem_map_handle = handle;

	ret = q6apm_send_cmd_sync(apm, pkt, APM_CMD_SHARED_MEM_UNMAP_REGIONS);
	apm->vcal.tbl[slot].mem_map_handle = 0;
	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_unmap_cal_region);

int q6apm_map_pos_buffer(struct device *dev, unsigned int graph_id, phys_addr_t phys, size_t sz)
{
	return __q6apm_map_memory_fixed_region(dev, graph_id, phys, sz, true);
}
EXPORT_SYMBOL_GPL(q6apm_map_pos_buffer);

int q6apm_map_memory_fixed_region(struct device *dev, unsigned int graph_id,
				  phys_addr_t phys, size_t sz)
{
	return __q6apm_map_memory_fixed_region(dev, graph_id, phys, sz, false);
}
EXPORT_SYMBOL_GPL(q6apm_map_memory_fixed_region);

int q6apm_alloc_fragments(struct q6apm_graph *graph, unsigned int dir, phys_addr_t phys,
				size_t period_sz, unsigned int periods)
{
	struct audioreach_graph_data *data;
	struct audio_buffer *buf;
	int cnt;

	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		data = &graph->rx_data;
	else
		data = &graph->tx_data;

	mutex_lock(&graph->lock);

	data->dsp_buf = 0;

	if (data->buf) {
		mutex_unlock(&graph->lock);
		return 0;
	}

	buf = kzalloc_objs(struct audio_buffer, periods);
	if (!buf) {
		mutex_unlock(&graph->lock);
		return -ENOMEM;
	}

	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		data = &graph->rx_data;
	else
		data = &graph->tx_data;

	data->buf = buf;

	buf[0].phys = phys;
	buf[0].size = period_sz;

	for (cnt = 1; cnt < periods; cnt++) {
		if (period_sz > 0) {
			buf[cnt].phys = buf[0].phys + (cnt * period_sz);
			buf[cnt].size = period_sz;
		}
	}
	data->num_periods = periods;

	mutex_unlock(&graph->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_alloc_fragments);

static int __q6apm_unmap_memory_fixed_region(struct device *dev, unsigned int graph_id,
					     bool is_pos_buf)
{
	struct apm_cmd_shared_mem_unmap_regions *cmd;
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct audioreach_graph_info *info;
	uint32_t mem_map_handle;
	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_apm_cmd_pkt(sizeof(*cmd),
						APM_CMD_SHARED_MEM_UNMAP_REGIONS, graph_id);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	info = idr_find(&apm->graph_info_idr, graph_id);
	if (!info)
		return -ENODEV;

	if (is_pos_buf) {
		if (!info->pos_buf_mem_map_handle)
			return 0;
		mem_map_handle = info->pos_buf_mem_map_handle;
	} else {

		if (!info->mem_map_handle)
			return 0;
		mem_map_handle = info->mem_map_handle;
	}

	cmd = (void *)pkt + GPR_HDR_SIZE;
	cmd->mem_map_handle = mem_map_handle;

	return q6apm_send_cmd_sync(apm, pkt, APM_CMD_SHARED_MEM_UNMAP_REGIONS);
}

int q6apm_unmap_memory_fixed_region(struct device *dev, unsigned int graph_id)
{
	return __q6apm_unmap_memory_fixed_region(dev, graph_id, false);
}
EXPORT_SYMBOL_GPL(q6apm_unmap_memory_fixed_region);

int q6apm_unmap_pos_buffer(struct device *dev, unsigned int graph_id)
{
	return __q6apm_unmap_memory_fixed_region(dev, graph_id, true);
}
EXPORT_SYMBOL_GPL(q6apm_unmap_pos_buffer);

int q6apm_free_fragments(struct q6apm_graph *graph, unsigned int dir)
{
	audioreach_graph_free_buf(graph);

	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_free_fragments);

int q6apm_remove_initial_silence(struct device *dev, struct q6apm_graph *graph, uint32_t samples)
{
	struct audioreach_module *module;

	module = q6apm_find_module_by_mid(graph, MODULE_ID_PLACEHOLDER_DECODER);
	if (!module)
		return -ENODEV;

	return audioreach_send_u32_param(graph, module, PARAM_ID_REMOVE_INITIAL_SILENCE, samples);
}
EXPORT_SYMBOL_GPL(q6apm_remove_initial_silence);

int q6apm_remove_trailing_silence(struct device *dev, struct q6apm_graph *graph, uint32_t samples)
{
	struct audioreach_module *module;

	module = q6apm_find_module_by_mid(graph, MODULE_ID_PLACEHOLDER_DECODER);
	if (!module)
		return -ENODEV;

	return audioreach_send_u32_param(graph, module, PARAM_ID_REMOVE_TRAILING_SILENCE, samples);
}
EXPORT_SYMBOL_GPL(q6apm_remove_trailing_silence);

int q6apm_enable_compress_module(struct device *dev, struct q6apm_graph *graph, bool en)
{
	struct audioreach_module *module;

	module = q6apm_find_module_by_mid(graph, MODULE_ID_PLACEHOLDER_DECODER);
	if (!module)
		return -ENODEV;

	return audioreach_send_u32_param(graph, module, PARAM_ID_MODULE_ENABLE, en);
}
EXPORT_SYMBOL_GPL(q6apm_enable_compress_module);

int q6apm_set_real_module_id(struct device *dev, struct q6apm_graph *graph,
			     uint32_t codec_id)
{
	struct audioreach_module *module;
	uint32_t module_id;

	module = q6apm_find_module_by_mid(graph, MODULE_ID_PLACEHOLDER_DECODER);
	if (!module)
		return -ENODEV;

	switch (codec_id) {
	case SND_AUDIOCODEC_MP3:
		module_id = MODULE_ID_MP3_DECODE;
		break;
	case SND_AUDIOCODEC_AAC:
		module_id = MODULE_ID_AAC_DEC;
		break;
	case SND_AUDIOCODEC_FLAC:
		module_id = MODULE_ID_FLAC_DEC;
		break;
	case SND_AUDIOCODEC_OPUS_RAW:
		module_id = MODULE_ID_OPUS_DEC;
		break;
	default:
		return -EINVAL;
	}

	return audioreach_send_u32_param(graph, module, PARAM_ID_REAL_MODULE_ID,
					 module_id);
}
EXPORT_SYMBOL_GPL(q6apm_set_real_module_id);

int q6apm_graph_media_format_pcm(struct q6apm_graph *graph, struct audioreach_module_config *cfg)
{
	struct audioreach_graph_info *info = graph->info;
	struct audioreach_sub_graph *sgs;
	struct audioreach_container *container;
	struct audioreach_module *module;
	int ret;

	list_for_each_entry(sgs, &info->sg_list, node) {
		list_for_each_entry(container, &sgs->container_list, node) {
			list_for_each_entry(module, &container->modules_list, node) {
				if ((module->module_id == MODULE_ID_WR_SHARED_MEM_EP) ||
					(module->module_id == MODULE_ID_RD_SHARED_MEM_EP) ||
					(module->module_id == MODULE_ID_SH_MEM_PULL_MODE) ||
					(module->module_id == MODULE_ID_SH_MEM_PUSH_MODE))
					continue;

				ret = audioreach_set_media_format(graph, module, cfg);
				if (ret)
					return ret;
			}
		}
	}

	return 0;

}
EXPORT_SYMBOL_GPL(q6apm_graph_media_format_pcm);

int q6apm_write_async(struct q6apm_graph *graph, uint32_t len, uint32_t msw_ts,
		      uint32_t lsw_ts, uint32_t wflags)
{
	struct apm_data_cmd_wr_sh_mem_ep_data_buffer_v2 *write_buffer;
	struct audio_buffer *ab;

	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_pkt(sizeof(*write_buffer),
					DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2,
					graph->rx_data.dsp_buf | (len << APM_WRITE_TOKEN_LEN_SHIFT),
					graph->port->id, graph->shm_iid);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	write_buffer = (void *)pkt + GPR_HDR_SIZE;

	mutex_lock(&graph->lock);
	ab = &graph->rx_data.buf[graph->rx_data.dsp_buf];

	write_buffer->buf_addr_lsw = lower_32_bits(ab->phys);
	write_buffer->buf_addr_msw = upper_32_bits(ab->phys);
	write_buffer->buf_size = len;
	write_buffer->timestamp_lsw = lsw_ts;
	write_buffer->timestamp_msw = msw_ts;
	write_buffer->mem_map_handle = graph->info->mem_map_handle;
	write_buffer->flags = wflags;

	graph->rx_data.dsp_buf++;

	if (graph->rx_data.dsp_buf >= graph->rx_data.num_periods)
		graph->rx_data.dsp_buf = 0;

	mutex_unlock(&graph->lock);

	return gpr_send_port_pkt(graph->port, pkt);
}
EXPORT_SYMBOL_GPL(q6apm_write_async);

int q6apm_read(struct q6apm_graph *graph)
{
	struct data_cmd_rd_sh_mem_ep_data_buffer_v2 *read_buffer;
	struct audioreach_graph_data *port;
	struct audio_buffer *ab;

	struct gpr_pkt *pkt __free(kfree) = audioreach_alloc_pkt(sizeof(*read_buffer),
					DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2,
					graph->tx_data.dsp_buf, graph->port->id, graph->shm_iid);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	read_buffer = (void *)pkt + GPR_HDR_SIZE;

	mutex_lock(&graph->lock);
	port = &graph->tx_data;
	ab = &port->buf[port->dsp_buf];

	read_buffer->buf_addr_lsw = lower_32_bits(ab->phys);
	read_buffer->buf_addr_msw = upper_32_bits(ab->phys);
	read_buffer->mem_map_handle = graph->info->mem_map_handle;
	read_buffer->buf_size = ab->size;

	port->dsp_buf++;

	if (port->dsp_buf >= port->num_periods)
		port->dsp_buf = 0;

	mutex_unlock(&graph->lock);

	return gpr_send_port_pkt(graph->port, pkt);
}
EXPORT_SYMBOL_GPL(q6apm_read);

int q6apm_get_hw_pointer(struct q6apm_graph *graph, int dir)
{
	struct audioreach_graph_data *data;

	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		data = &graph->rx_data;
	else
		data = &graph->tx_data;

	return (int)atomic_read(&data->hw_ptr);
}
EXPORT_SYMBOL_GPL(q6apm_get_hw_pointer);

static int graph_callback(const struct gpr_resp_pkt *data, void *priv, int op)
{
	struct data_cmd_rsp_rd_sh_mem_ep_data_buffer_done_v2 *rd_done;
	struct data_cmd_rsp_wr_sh_mem_ep_data_buffer_done_v2 *done;
	struct apm_module_event *event;
	const struct gpr_ibasic_rsp_result_t *result;
	struct q6apm_graph *graph = priv;
	const struct gpr_hdr *hdr = &data->hdr;
	struct device *dev = graph->dev;
	uint32_t client_event;
	phys_addr_t phys;
	int token;

	result = data->payload;

	switch (hdr->opcode) {
	case APM_EVENT_MODULE_TO_CLIENT:
		event = data->payload;
		switch (event->event_id) {
		case EVENT_ID_SH_MEM_PULL_PUSH_MODE_WATERMARK:
			client_event = APM_CLIENT_EVENT_WATERMARK_EVENT;
			graph->cb(client_event, hdr->token, data->payload, graph->priv);
			break;
		}

		break;
	case DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2:
		if (!graph->ar_graph)
			break;
		client_event = APM_CLIENT_EVENT_DATA_WRITE_DONE;
		mutex_lock(&graph->lock);
		token = hdr->token & APM_WRITE_TOKEN_MASK;

		done = data->payload;
		if (!graph->rx_data.buf) {
			mutex_unlock(&graph->lock);
			break;
		}
		phys = graph->rx_data.buf[token].phys;
		mutex_unlock(&graph->lock);
		/* token numbering starts at 0 */
		atomic_set(&graph->rx_data.hw_ptr, token + 1);
		if (lower_32_bits(phys) == done->buf_addr_lsw &&
		    upper_32_bits(phys) == done->buf_addr_msw) {
			graph->result.opcode = hdr->opcode;
			graph->result.status = done->status;
			if (graph->cb)
				graph->cb(client_event, hdr->token, data->payload, graph->priv);
		} else {
			dev_err(dev, "WR BUFF Unexpected addr %08x-%08x\n", done->buf_addr_lsw,
				done->buf_addr_msw);
		}

		break;
	case DATA_CMD_RSP_RD_SH_MEM_EP_DATA_BUFFER_V2:
		if (!graph->ar_graph)
			break;
		client_event = APM_CLIENT_EVENT_DATA_READ_DONE;
		mutex_lock(&graph->lock);
		rd_done = data->payload;
		if (!graph->tx_data.buf) {
			mutex_unlock(&graph->lock);
			break;
		}
		phys = graph->tx_data.buf[hdr->token].phys;
		mutex_unlock(&graph->lock);
		/* token numbering starts at 0 */
		atomic_set(&graph->tx_data.hw_ptr, hdr->token + 1);

		if (upper_32_bits(phys) == rd_done->buf_addr_msw &&
		    lower_32_bits(phys) == rd_done->buf_addr_lsw) {
			graph->result.opcode = hdr->opcode;
			graph->result.status = rd_done->status;
			if (graph->cb)
				graph->cb(client_event, hdr->token, data->payload, graph->priv);
		} else {
			dev_err(dev, "RD BUFF Unexpected addr %08x-%08x\n", rd_done->buf_addr_lsw,
				rd_done->buf_addr_msw);
		}
		break;
	case DATA_CMD_WR_SH_MEM_EP_EOS_RENDERED:
		client_event = APM_CLIENT_EVENT_CMD_EOS_DONE;
		if (graph->cb)
			graph->cb(client_event, hdr->token, data->payload, graph->priv);
		break;
	case GPR_BASIC_RSP_RESULT:
		switch (result->opcode) {
		case APM_CMD_SHARED_MEM_MAP_REGIONS:
		case DATA_CMD_WR_SH_MEM_EP_MEDIA_FORMAT:
		case APM_CMD_REGISTER_MODULE_EVENTS:
		case APM_CMD_SET_CFG:
		case APM_CMD_REGISTER_CFG:
		case APM_CMD_DEREGISTER_CFG:
			if (result->opcode == APM_CMD_REGISTER_CFG ||
			    result->opcode == APM_CMD_DEREGISTER_CFG) {
				struct q6apm_vcal *vc = &graph->apm->vcal;
				unsigned int i;

				/*
				 * Keep the DSP's own status word beside the
				 * wrapper's return code: a timeout has no
				 * status at all and must never be reported as
				 * a refusal.
				 */
				for (i = 0; i < Q6APM_VCAL_MAX_TABLES; i++) {
					if (!vc->tbl[i].send_called)
						continue;
					vc->tbl[i].acknowledged_opcode = result->opcode;
					vc->tbl[i].raw_dsp_status = result->status;
					vc->tbl[i].raw_dsp_status_valid = true;
				}
			}
			graph->result.opcode = result->opcode;
			graph->result.status = result->status;
			if (result->status)
				dev_err(dev, "Error (%d) Processing 0x%08x cmd\n",
					result->status, result->opcode);
			wake_up(&graph->cmd_wait);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return 0;
}

int q6apm_register_watermark_event(struct q6apm_graph *graph, int water_mark_level_bytes,
				   int num_levels)
{
	return audioreach_shmem_register_event(graph, water_mark_level_bytes, num_levels);
}
EXPORT_SYMBOL_GPL(q6apm_register_watermark_event);

int q6apm_push_pull_config(struct q6apm_graph *graph, phys_addr_t bphys,
			   phys_addr_t pphys, uint32_t size)
{
	struct audioreach_graph_info *info = graph->info;

	return audioreach_setup_push_pull(graph, bphys, pphys, info->mem_map_handle,
					  info->pos_buf_mem_map_handle, size);
}
EXPORT_SYMBOL_GPL(q6apm_push_pull_config);

bool q6apm_is_graph_in_push_pull_mode_from_id(struct device *dev, unsigned int graph_id, int dir)
{
	struct audioreach_graph_info *info;
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct audioreach_module *module;

	info = idr_find(&apm->graph_info_idr, graph_id);
	if (!info)
		return false;

	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		module = __q6apm_find_module_by_mid(apm, info, MODULE_ID_SH_MEM_PULL_MODE);
	else
		module = __q6apm_find_module_by_mid(apm, info, MODULE_ID_SH_MEM_PUSH_MODE);

	return !!module;

}
EXPORT_SYMBOL_GPL(q6apm_is_graph_in_push_pull_mode_from_id);

bool q6apm_is_graph_in_push_pull_mode(struct q6apm_graph *graph)
{
	return graph->info->is_push_pull_mode;
}
EXPORT_SYMBOL_GPL(q6apm_is_graph_in_push_pull_mode);

/*
 * A voice (hostless) graph carries exactly one mailbox for its direction and
 * no shared-memory endpoint of any kind. The positive predicate keeps a
 * malformed conventional graph from silently classifying as hostless.
 */
static bool q6apm_info_is_voice(struct audioreach_graph_info *info, int dir)
{
	struct audioreach_container *container;
	struct audioreach_sub_graph *sgs;
	struct audioreach_module *module;
	int mailbox_rx = 0, mailbox_tx = 0, shmem = 0;

	list_for_each_entry(sgs, &info->sg_list, node) {
		list_for_each_entry(container, &sgs->container_list, node) {
			list_for_each_entry(module, &container->modules_list, node) {
				switch (module->module_id) {
				case MODULE_ID_MAILBOX_RX:
					mailbox_rx++;
					break;
				case MODULE_ID_MAILBOX_TX:
					mailbox_tx++;
					break;
				case MODULE_ID_WR_SHARED_MEM_EP:
				case MODULE_ID_RD_SHARED_MEM_EP:
				case MODULE_ID_SH_MEM_PULL_MODE:
				case MODULE_ID_SH_MEM_PUSH_MODE:
					shmem++;
					break;
				}
			}
		}
	}

	if (shmem)
		return false;
	if (dir == SNDRV_PCM_STREAM_PLAYBACK)
		return mailbox_rx == 1 && !mailbox_tx;

	return mailbox_tx == 1 && !mailbox_rx;
}

bool q6apm_is_voice_graph(struct q6apm_graph *graph, int dir)
{
	return q6apm_info_is_voice(graph->info, dir);
}
EXPORT_SYMBOL_GPL(q6apm_is_voice_graph);

bool q6apm_is_voice_graph_from_id(struct device *dev, unsigned int graph_id, int dir)
{
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct audioreach_graph_info *info;

	info = idr_find(&apm->graph_info_idr, graph_id);
	if (!info)
		return false;

	return q6apm_info_is_voice(info, dir);
}
EXPORT_SYMBOL_GPL(q6apm_is_voice_graph_from_id);

static const uint32_t q6apm_voice_vsids[] = {
	VOICE_VSID_SUB1,
	VOICE_VSID_SUB2,
	VOICE_VSID_LB_SUB1,
	VOICE_VSID_LB_SUB2,
};

int q6apm_voice_acquire(struct device *dev, uint32_t *vsid)
{
	struct q6apm *apm = dev_get_drvdata(dev->parent);

	mutex_lock(&apm->voice_lock);
	if (apm->voice_open_count == 0)
		apm->voice_vsid = q6apm_voice_vsids[apm->voice_vsid_idx];
	apm->voice_open_count++;
	*vsid = apm->voice_vsid;
	mutex_unlock(&apm->voice_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_voice_acquire);

void q6apm_voice_release(struct device *dev)
{
	struct q6apm *apm = dev_get_drvdata(dev->parent);

	mutex_lock(&apm->voice_lock);
	if (!WARN_ON(apm->voice_open_count == 0))
		apm->voice_open_count--;
	mutex_unlock(&apm->voice_lock);
}
EXPORT_SYMBOL_GPL(q6apm_voice_release);

static int q6apm_graph_get_module_iid(struct q6apm_graph *graph, uint32_t mid)
{
	struct audioreach_module *module;

	module = q6apm_find_module_by_mid(graph, mid);
	if (!module)
		return -ENODEV;

	return module->instance_id;
}

struct q6apm_graph *q6apm_graph_open(struct device *dev, q6apm_cb cb,
				     void *priv, int graph_id, int dir)
{
	struct q6apm *apm = dev_get_drvdata(dev->parent);
	struct audioreach_graph *ar_graph;
	struct q6apm_graph *graph;
	int ret, iid = 0;

	ar_graph = q6apm_get_audioreach_graph(apm, graph_id);
	if (IS_ERR(ar_graph)) {
		dev_err(dev, "No graph found with id %d\n", graph_id);
		return ERR_CAST(ar_graph);
	}

	graph = kzalloc_obj(*graph);
	if (!graph) {
		ret = -ENOMEM;
		goto put_ar_graph;
	}

	graph->apm = apm;
	graph->priv = priv;
	graph->cb = cb;
	graph->info = ar_graph->info;
	graph->ar_graph = ar_graph;
	graph->id = ar_graph->id;
	graph->dev = dev;

	if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
		iid = q6apm_graph_get_module_iid(graph, MODULE_ID_SH_MEM_PULL_MODE);
		if (iid < 0)
			iid = q6apm_graph_get_module_iid(graph, MODULE_ID_WR_SHARED_MEM_EP);
		else
			graph->info->is_push_pull_mode = true;

	} else {
		iid = q6apm_graph_get_module_iid(graph, MODULE_ID_SH_MEM_PUSH_MODE);
		if (iid < 0)
			iid = q6apm_graph_get_module_iid(graph, MODULE_ID_RD_SHARED_MEM_EP);
		else
			graph->info->is_push_pull_mode = true;
	}

	if (iid > 0)
		graph->shm_iid = iid;

	mutex_init(&graph->lock);
	init_waitqueue_head(&graph->cmd_wait);

	graph->port = gpr_alloc_port(apm->gdev, dev, graph_callback, graph);
	if (IS_ERR(graph->port)) {
		ret = PTR_ERR(graph->port);
		goto free_graph;
	}

	return graph;

free_graph:
	kfree(graph);
put_ar_graph:
	kref_put(&ar_graph->refcount, q6apm_put_audioreach_graph);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(q6apm_graph_open);

int q6apm_graph_close(struct q6apm_graph *graph)
{
	struct audioreach_graph *ar_graph = graph->ar_graph;

	graph->ar_graph = NULL;
	kref_put(&ar_graph->refcount, q6apm_put_audioreach_graph);
	gpr_free_port(graph->port);
	kfree(graph);

	return 0;
}
EXPORT_SYMBOL_GPL(q6apm_graph_close);

int q6apm_graph_prepare(struct q6apm_graph *graph)
{
	return audioreach_graph_mgmt_cmd(graph->ar_graph, APM_CMD_GRAPH_PREPARE);
}
EXPORT_SYMBOL_GPL(q6apm_graph_prepare);

int q6apm_graph_start(struct q6apm_graph *graph)
{
	struct audioreach_graph *ar_graph = graph->ar_graph;
	int ret = 0;

	if (ar_graph->start_count == 0) {
		ret = audioreach_graph_mgmt_cmd(ar_graph, APM_CMD_GRAPH_START);
		if (ret)
			return ret;
	}

	ar_graph->start_count++;

	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_graph_start);

int q6apm_graph_stop(struct q6apm_graph *graph)
{
	struct audioreach_graph *ar_graph = graph->ar_graph;

	if (--ar_graph->start_count > 0)
		return 0;

	return audioreach_graph_mgmt_cmd(ar_graph, APM_CMD_GRAPH_STOP);
}
EXPORT_SYMBOL_GPL(q6apm_graph_stop);

int q6apm_graph_flush(struct q6apm_graph *graph)
{
	return audioreach_graph_mgmt_cmd(graph->ar_graph, APM_CMD_GRAPH_FLUSH);
}
EXPORT_SYMBOL_GPL(q6apm_graph_flush);

static int q6apm_audio_probe(struct snd_soc_component *component)
{
	return audioreach_tplg_init(component);
}

static void q6apm_audio_remove(struct snd_soc_component *component)
{
	/* remove topology */
	snd_soc_tplg_component_remove(component);
}

#define APM_AUDIO_DRV_NAME "q6apm-audio"

static const char * const q6apm_voice_vsid_texts[] = {
	"Voice1", "Voice2", "Loopback1", "Loopback2",
};

static SOC_ENUM_SINGLE_EXT_DECL(q6apm_voice_vsid_enum, q6apm_voice_vsid_texts);

static int q6apm_voice_vsid_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct q6apm *apm = dev_get_drvdata(c->dev);

	ucontrol->value.enumerated.item[0] = apm->voice_vsid_idx;

	return 0;
}

static int q6apm_voice_vsid_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct q6apm *apm = dev_get_drvdata(c->dev);
	unsigned int idx = ucontrol->value.enumerated.item[0];
	int ret = 0;

	if (idx >= ARRAY_SIZE(q6apm_voice_vsids))
		return -EINVAL;

	mutex_lock(&apm->voice_lock);
	if (apm->voice_open_count)
		ret = -EBUSY;
	else if (apm->voice_vsid_idx != idx) {
		apm->voice_vsid_idx = idx;
		ret = 1;
	}
	mutex_unlock(&apm->voice_lock);

	return ret;
}

static const struct snd_kcontrol_new q6apm_controls[] = {
	SOC_ENUM_EXT("Voice VSID", q6apm_voice_vsid_enum,
		     q6apm_voice_vsid_get, q6apm_voice_vsid_put),
};

/* Right-slot gates, one per playback front end (struct q6apm_slot). */
static struct q6apm_slot *q6apm_slot_find(struct q6apm *apm, int graph_id)
{
	struct q6apm_slot *slot;

	list_for_each_entry(slot, &apm->slot_list, node)
		if (slot->graph_id == graph_id)
			return slot;

	return NULL;
}

static void q6apm_slot_notify(struct q6apm_slot *slot)
{
	if (slot->kctl)
		snd_ctl_notify(slot->card, SNDRV_CTL_EVENT_MASK_VALUE, &slot->kctl->id);
}

static int q6apm_slot_get(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct q6apm_slot *slot = (struct q6apm_slot *)kcontrol->private_value;

	ucontrol->value.integer.value[0] = slot->on;

	return 0;
}

/*
 * The gate moves only while its stream is open. A voice RX stream must also
 * be running, and is written on the spot: a value set before a call can
 * then never ride into its first buffer, and the control never reports a
 * route the wire does not carry. Any other stream may be set between open
 * and prepare, where the value waits for prepare to write it. The DSP's
 * answer is the return value - a refused table leaves the control where it
 * was.
 */
static int q6apm_slot_put(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct q6apm_slot *slot = (struct q6apm_slot *)kcontrol->private_value;
	struct q6apm *apm = slot->apm;
	struct audioreach_graph *graph;
	bool on = !!ucontrol->value.integer.value[0];
	int ret = 0;

	mutex_lock(&apm->slot_lock);
	if (on == slot->on)
		goto out;

	mutex_lock(&apm->lock);
	graph = idr_find(&apm->graph_idr, slot->graph_id);
	if (graph && !kref_get_unless_zero(&graph->refcount))
		graph = NULL;
	mutex_unlock(&apm->lock);

	if (!graph) {
		ret = -EBUSY;
		goto out;
	}

	if (graph->start_count && slot->gated)
		ret = audioreach_set_right_slot(apm, graph->info, on);
	else if (graph->start_count || slot->voice)
		ret = -EBUSY;

	kref_put(&graph->refcount, q6apm_put_audioreach_graph);

	if (!ret) {
		slot->on = on;
		ret = 1;
	}
out:
	mutex_unlock(&apm->slot_lock);

	return ret;
}

int q6apm_slot_add(struct snd_soc_component *component, int graph_id,
		   const char *link_name)
{
	struct q6apm *apm = dev_get_drvdata(component->dev);
	struct snd_kcontrol_new kc = {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.info = snd_soc_info_bool_ext,
		.get = q6apm_slot_get,
		.put = q6apm_slot_put,
	};
	struct audioreach_graph_info *info;
	struct q6apm_slot *slot;
	char *name;
	int ret;

	mutex_lock(&apm->lock);
	info = idr_find(&apm->graph_info_idr, graph_id);
	mutex_unlock(&apm->lock);
	if (!info)
		return 0;

	slot = devm_kzalloc(apm->dev, sizeof(*slot), GFP_KERNEL);
	if (!slot)
		return -ENOMEM;

	name = kasprintf(GFP_KERNEL, "%s Right Slot Switch", link_name);
	if (!name)
		return -ENOMEM;

	slot->apm = apm;
	slot->card = component->card->snd_card;
	slot->graph_id = graph_id;
	slot->voice = q6apm_info_is_voice(info, SNDRV_PCM_STREAM_PLAYBACK);
	slot->dflt = !slot->voice;
	slot->on = slot->dflt;

	kc.name = name;
	kc.private_value = (unsigned long)slot;
	ret = snd_soc_add_component_controls(component, &kc, 1);
	if (!ret) {
		slot->kctl = snd_soc_card_get_kcontrol(component->card, name);
		mutex_lock(&apm->slot_lock);
		list_add_tail(&slot->node, &apm->slot_list);
		mutex_unlock(&apm->slot_lock);
	}
	kfree(name);

	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_slot_add);

/*
 * Before the graph is prepared: the converter is written with the gate, so
 * the first buffer already carries it. A voice RX stream goes to its
 * default - closed - on every prepare; any other stream keeps what was set
 * on it since it opened. A stream without a stereo FL/FR output has no
 * right slot to gate; a voice RX stream shaped that way is refused rather
 * than started open.
 */
int q6apm_graph_slot_prepare(struct q6apm_graph *graph, int dir,
			     const struct audioreach_module_config *cfg)
{
	struct q6apm *apm = graph->apm;
	struct q6apm_slot *slot;
	bool changed = false;
	int ret = 0;

	if (dir != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	mutex_lock(&apm->slot_lock);
	slot = q6apm_slot_find(apm, graph->id);
	if (!slot)
		goto out;

	if (slot->voice) {
		changed = slot->on != slot->dflt;
		slot->on = slot->dflt;
	}
	slot->gated = cfg->num_channels == 2 &&
		      cfg->channel_map[0] == PCM_CHANNEL_FL &&
		      cfg->channel_map[1] == PCM_CHANNEL_FR;

	if (slot->gated)
		ret = audioreach_set_right_slot(apm, graph->info, slot->on);
	if (ret == -ENOENT && !slot->voice) {
		/* no converter in this graph: nothing to gate */
		slot->gated = false;
		ret = 0;
	} else if (!slot->gated && slot->voice) {
		ret = -EINVAL;
	}
out:
	mutex_unlock(&apm->slot_lock);
	if (changed)
		q6apm_slot_notify(slot);

	return ret;
}
EXPORT_SYMBOL_GPL(q6apm_graph_slot_prepare);

/*
 * The stream is closing: a gate whose default is closed is shut on the wire
 * first, then the control returns to its default for the next stream.
 */
void q6apm_graph_slot_close(struct q6apm_graph *graph, int dir)
{
	struct q6apm *apm = graph->apm;
	struct q6apm_slot *slot;
	bool changed = false;

	if (dir != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	mutex_lock(&apm->slot_lock);
	slot = q6apm_slot_find(apm, graph->id);
	if (slot) {
		changed = slot->on != slot->dflt;
		if (changed && slot->gated && !slot->dflt)
			audioreach_set_right_slot(apm, graph->info, slot->dflt);
		slot->on = slot->dflt;
		slot->gated = false;
	}
	mutex_unlock(&apm->slot_lock);
	if (changed)
		q6apm_slot_notify(slot);
}
EXPORT_SYMBOL_GPL(q6apm_graph_slot_close);

static const struct snd_soc_component_driver q6apm_audio_component = {
	.name		= APM_AUDIO_DRV_NAME,
	.probe		= q6apm_audio_probe,
	.remove		= q6apm_audio_remove,
	.controls	= q6apm_controls,
	.num_controls	= ARRAY_SIZE(q6apm_controls),
	.remove_order   = SND_SOC_COMP_ORDER_LAST,
};

static int apm_probe(gpr_device_t *gdev)
{
	struct device *dev = &gdev->dev;
	struct q6apm *apm;
	int ret;

	apm = devm_kzalloc(dev, sizeof(*apm), GFP_KERNEL);
	if (!apm)
		return -ENOMEM;

	dev_set_drvdata(dev, apm);

	mutex_init(&apm->lock);
	mutex_init(&apm->voice_lock);
	mutex_init(&apm->slot_lock);
	INIT_LIST_HEAD(&apm->slot_list);
	mutex_init(&apm->vcal.lock);
	apm->dev = dev;
	apm->gdev = gdev;
	init_waitqueue_head(&apm->wait);

	INIT_LIST_HEAD(&apm->widget_list);
	idr_init(&apm->graph_idr);
	idr_init(&apm->graph_info_idr);
	idr_init(&apm->sub_graphs_idr);
	idr_init(&apm->containers_idr);

	idr_init(&apm->modules_idr);

	g_apm = apm;

	q6apm_get_apm_state(apm);

	ret = snd_soc_register_component(dev, &q6apm_audio_component, NULL, 0);
	if (ret < 0) {
		dev_err(dev, "failed to register q6apm: %d\n", ret);
		return ret;
	}

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret)
		snd_soc_unregister_component(dev);

	return ret;
}

static void apm_remove(gpr_device_t *gdev)
{
	of_platform_depopulate(&gdev->dev);
	snd_soc_unregister_component(&gdev->dev);
}

struct audioreach_module *q6apm_find_module_by_mid(struct q6apm_graph *graph, uint32_t mid)
{
	struct audioreach_graph_info *info = graph->info;
	struct q6apm *apm = graph->apm;

	return __q6apm_find_module_by_mid(apm, info, mid);

}

static int apm_callback(const struct gpr_resp_pkt *data, void *priv, int op)
{
	gpr_device_t *gdev = priv;
	struct audioreach_graph_info *info;
	struct q6apm *apm = dev_get_drvdata(&gdev->dev);
	struct apm_cmd_rsp_shared_mem_map_regions *rsp;
	struct device *dev = &gdev->dev;
	struct gpr_ibasic_rsp_result_t *result;
	const struct gpr_hdr *hdr = &data->hdr;
	int graph_id, is_pos_buf;

	result = data->payload;

	switch (hdr->opcode) {
	case APM_CMD_RSP_GET_SPF_STATE:
		apm->result.opcode = hdr->opcode;
		apm->result.status = 0;
		/* First word of result it state */
		apm->state = result->opcode;
		wake_up(&apm->wait);
		break;
	case GPR_BASIC_RSP_RESULT:
		switch (result->opcode) {
		case APM_CMD_SHARED_MEM_MAP_REGIONS:
		case APM_CMD_GRAPH_START:
		case APM_CMD_GRAPH_OPEN:
		case APM_CMD_GRAPH_PREPARE:
		case APM_CMD_GRAPH_CLOSE:
		case APM_CMD_GRAPH_FLUSH:
		case APM_CMD_GRAPH_STOP:
		case APM_CMD_SET_CFG:
			apm->result.opcode = result->opcode;
			apm->result.status = result->status;
			if (result->status)
				dev_err(dev, "Error (%d) Processing 0x%08x cmd\n", result->status,
					result->opcode);
			wake_up(&apm->wait);
			break;
		case APM_CMD_SHARED_MEM_UNMAP_REGIONS:
			if (hdr->token & APM_MMAP_TOKEN_MAP_TYPE_CAL) {
				unsigned int slot = !!(hdr->token & APM_MMAP_TOKEN_CAL_SLOT);

				apm->vcal.tbl[slot].mem_map_handle = 0;
				apm->result.opcode = result->opcode;
				apm->result.status = result->status;
				wake_up(&apm->wait);
				break;
			}
			apm->result.opcode = hdr->opcode;
			apm->result.status = 0;
			rsp = data->payload;

			info = idr_find(&apm->graph_info_idr, hdr->token);
			if (info)
				info->mem_map_handle = 0;
			else
				dev_err(dev, "Error (%d) Processing 0x%08x cmd\n", result->status,
					result->opcode);

			wake_up(&apm->wait);
			break;
		default:
			break;
		}
		break;
	case APM_CMD_RSP_SHARED_MEM_MAP_REGIONS:
		apm->result.opcode = hdr->opcode;
		apm->result.status = 0;
		rsp = data->payload;
		if (hdr->token & APM_MMAP_TOKEN_MAP_TYPE_CAL) {
			unsigned int slot = !!(hdr->token & APM_MMAP_TOKEN_CAL_SLOT);

			apm->vcal.tbl[slot].mem_map_handle = rsp->mem_map_handle;
			wake_up(&apm->wait);
			break;
		}
		graph_id = hdr->token & APM_MMAP_TOKEN_GID_MASK;
		is_pos_buf = hdr->token & APM_MMAP_TOKEN_MAP_TYPE_POS_BUF;

		info = idr_find(&apm->graph_info_idr, graph_id);
		if (info) {
			if (is_pos_buf)
				info->pos_buf_mem_map_handle = rsp->mem_map_handle;
			else
				info->mem_map_handle = rsp->mem_map_handle;
		} else {
			dev_err(dev, "Error (%d) Processing 0x%08x cmd\n", result->status,
				result->opcode);
		}

		wake_up(&apm->wait);
		break;
	default:
		break;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id apm_device_id[]  = {
	{ .compatible = "qcom,q6apm" },
	{},
};
MODULE_DEVICE_TABLE(of, apm_device_id);
#endif

static gpr_driver_t apm_driver = {
	.probe = apm_probe,
	.remove = apm_remove,
	.gpr_callback = apm_callback,
	.driver = {
		.name = "qcom-apm",
		.of_match_table = of_match_ptr(apm_device_id),
	},
};

module_gpr_driver(apm_driver);
MODULE_DESCRIPTION("Audio Process Manager");
MODULE_LICENSE("GPL");
