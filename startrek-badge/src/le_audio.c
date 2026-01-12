#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/buf.h>

#include "le_audio.h"

LOG_MODULE_REGISTER(le_audio);

static struct bt_bap_stream streams[2]; // 0: Sink, 1: Source
static le_audio_recv_cb_t recv_cb = NULL;

/*
 * Pool for ISO transmission.
 * Sized to hold raw PCM samples (320 bytes) for testing, plus headers.
 */
NET_BUF_POOL_FIXED_DEFINE(tx_pool, 2, 350, 8, NULL);

/*
 * LC3 Codec Capabilities (PACS)
 * 16kHz, 10ms frame duration, 40-120 octets per frame (Standard Quality)
 */
static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
    BT_AUDIO_CODEC_CAP_FREQ_16KHZ,
    BT_AUDIO_CODEC_CAP_DURATION_10,
    BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1),
    40u, 120u, 1u,
    (BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA)
);

/* Capability wrappers for registration */
static struct bt_pacs_cap cap_sink = { .codec_cap = &lc3_codec_cap };
static struct bt_pacs_cap cap_source = { .codec_cap = &lc3_codec_cap };

/*
 * Stream Callbacks
 */

static void stream_started_cb(struct bt_bap_stream *stream)
{
    LOG_INF("Stream started");
}

static void stream_stopped_cb(struct bt_bap_stream *stream, uint8_t reason)
{
    LOG_INF("Stream stopped, reason 0x%02x", reason);
}

static void stream_recv_cb(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
                           struct net_buf *buf)
{
    if (recv_cb && buf && buf->len > 0) {
        recv_cb(buf->data, buf->len);
    }
}

static struct bt_bap_stream_ops stream_ops = {
    .started = stream_started_cb,
    .stopped = stream_stopped_cb,
    .recv = stream_recv_cb,
};

/*
 * BAP Unicast Server Callbacks
 */

static int unicast_server_config(struct bt_conn *conn,
                                 const struct bt_bap_ep *ep,
                                 enum bt_audio_dir dir,
                                 const struct bt_audio_codec_cfg *codec_cfg,
                                 struct bt_bap_stream **stream,
                                 struct bt_bap_qos_cfg_pref *const pref,
                                 struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("Unicast Server Config Request: dir %d", dir);

    // Check if codec is LC3 (id 0x06)
    if (codec_cfg->id != BT_HCI_CODING_FORMAT_LC3) {
        LOG_ERR("Codec not LC3");
        return -EINVAL;
    }

    // Assign a stream context
    *stream = &streams[dir == BT_AUDIO_DIR_SINK ? 0 : 1];
    bt_bap_stream_cb_register(*stream, &stream_ops);

    // Set preferred QoS
    pref->unframed_supported = true;
    pref->phy = BT_GAP_LE_PHY_2M;
    pref->rtn = 2;
    pref->latency = 10;
    pref->pd_min = 10000;
    pref->pd_max = 40000;
    pref->pref_pd_min = 10000;
    pref->pref_pd_max = 40000;

    return 0;
}

static int unicast_server_reconfig(struct bt_bap_stream *stream,
                                   enum bt_audio_dir dir,
                                   const struct bt_audio_codec_cfg *codec_cfg,
                                   struct bt_bap_qos_cfg_pref *const pref,
                                   struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}

static int unicast_server_qos(struct bt_bap_stream *stream,
                              const struct bt_bap_qos_cfg *qos,
                              struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}

static int unicast_server_enable(struct bt_bap_stream *stream,
                                 const struct bt_audio_codec_cfg *meta,
                                 struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}

static int unicast_server_start(struct bt_bap_stream *stream,
                                struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}

static struct bt_bap_unicast_server_cb unicast_server_cbs = {
    .config = unicast_server_config,
    .reconfig = unicast_server_reconfig,
    .qos = unicast_server_qos,
    .enable = unicast_server_enable,
    .start = unicast_server_start,
};

void le_audio_init(void)
{
    int err;

    // Register PACS capabilities
    // Sink (Speaker)
    err = bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);
    if (err) LOG_ERR("Failed to register Sink PAC (err %d)", err);

    // Source (Mic)
    err = bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);
    if (err) LOG_ERR("Failed to register Source PAC (err %d)", err);

    err = bt_bap_unicast_server_register(&unicast_server_cbs);
    if (err) {
        LOG_ERR("Failed to register unicast server (err %d)", err);
        return;
    }

    LOG_INF("LE Audio Unicast Server Initialized with LC3 PACS");
}

int le_audio_send(const uint8_t *data, size_t len)
{
    struct bt_bap_stream *stream = &streams[1]; // Source stream
    struct net_buf *buf;
    int err;

    // Minimal implementation safety check
    // Real implementation should fragment or encode data
    if (len > CONFIG_BT_ISO_TX_MTU) {
        LOG_WRN("Data too large for ISO MTU, truncating");
        len = CONFIG_BT_ISO_TX_MTU;
    }

    buf = net_buf_alloc(&tx_pool, K_NO_WAIT);
    if (!buf) {
        return -ENOMEM;
    }

    net_buf_add_mem(buf, data, len);

    err = bt_bap_stream_send(stream, buf, 0); // Seq num 0 for now
    if (err < 0) {
        net_buf_unref(buf);
        return err;
    }

    return 0;
}

void le_audio_register_recv_cb(le_audio_recv_cb_t cb)
{
    recv_cb = cb;
}
