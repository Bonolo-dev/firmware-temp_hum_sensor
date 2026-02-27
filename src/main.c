/*
 * SHT30-D Temperature/Humidity sensor with threshold interrupt and BLE advertising
 *
 * When temp/humidity breaches thresholds:
 * - Wakes from sleep
 * - Advertises only the breached value(s) over BLE in CBOR format
 * - Stops advertising when values return to normal
 * - Logs events to LittleFS: {recycle}|{uptime}A|T{val} or H{val} (breach),
 *   {recycle}|{uptime}N|T{val} or H{val} (back to normal)
 * - Recycle counter and thresholds persist across power cycles (LittleFS)
 *
 * Button 1 (SW1): Wake from idle, read T/H, log to events.log, advertise (connectable)
 * so user can connect via BLE and fetch events.log (write "00" to command char).
 *
 * Wiring: SHT30-D VDD GND SDA SCL ALERT -> nRF52840 3V3 GND P0.26 P0.27 P1.3
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <zcbor_encode.h>
#include <zcbor_common.h>

/* Default thresholds - can be overridden via BLE (opcode 01) */
#define DEFAULT_TEMP_LO        15
#define DEFAULT_TEMP_HI        35
#define DEFAULT_HUMIDITY_LO    30
#define DEFAULT_HUMIDITY_HI    70
#define CMD_BUF_MAX            64
#define POLL_INTERVAL_MS      2000
#define MANUAL_ADV_TIMEOUT_MS 60000  /* Stay connectable with T/H for 60s after Button 1 */

/* BLE manufacturer data: 2-byte company ID + CBOR payload (max ~27 bytes) */
#define COMPANY_ID           0xFFFF  /* Use your company ID or 0xFFFF for dev */
#define MANUF_DATA_BUF_SIZE  30

/* LittleFS paths (fstab mounts at /lfs1) */
#define PATH_RC         "/lfs1/rc"
#define PATH_EVENTS     "/lfs1/events.log"
#define PATH_THRESHOLDS "/lfs1/thresholds"
#define LOG_LINE_MAX    48

/* Button 1 (SW1 on nRF52840 DK) - wake from idle for manual reading */
#define SW1_NODE     DT_ALIAS(sw0)

/* GATT service for log retrieval - write "00" to command, read events.log */
#define LOG_SVC_UUID  0xd4, 0x86, 0x48, 0x24, 0x54, 0xb3, 0x43, 0xa1, \
		     0xbc, 0x20, 0x97, 0x8f, 0xc3, 0x76, 0xc2, 0x75
#define CMD_CHAR_UUID 0xa6, 0xe8, 0xc4, 0x60, 0x7e, 0xaa, 0x41, 0x6b, \
		     0x95, 0xd4, 0x9d, 0xcc, 0x08, 0x4f, 0xcf, 0x6a
#define EVENTS_CHAR_UUID 0xb7, 0xf9, 0xd5, 0x71, 0x8f, 0xbb, 0x52, 0x7c, \
			 0xa6, 0xe5, 0xae, 0xdd, 0x19, 0x60, 0xd0, 0x7b
#define BT_UUID_LOG_SVC     BT_UUID_DECLARE_128(LOG_SVC_UUID)
#define BT_UUID_CMD_CHAR    BT_UUID_DECLARE_128(CMD_CHAR_UUID)
#define BT_UUID_EVENTS_CHAR BT_UUID_DECLARE_128(EVENTS_CHAR_UUID)

static K_SEM_DEFINE(threshold_sem, 0, 1);
static K_SEM_DEFINE(button_sem, 0, 1);

#define STORAGE_INIT_DELAY_MS  2000
#define REBOOT_DELAY_MS        1000  /* Defer reboot so BLE write completes */

/* Persisted across power cycles */
static uint32_t recycle_count;
static bool storage_available;

/* Runtime thresholds (configurable via BLE opcode 01) - protected by threshold_mutex */
static struct sensor_value alert_temp_lo_val = { .val1 = DEFAULT_TEMP_LO, .val2 = 0 };
static struct sensor_value alert_temp_hi_val = { .val1 = DEFAULT_TEMP_HI, .val2 = 0 };
static int alert_humidity_lo = DEFAULT_HUMIDITY_LO;
static int alert_humidity_hi = DEFAULT_HUMIDITY_HI;
static K_MUTEX_DEFINE(threshold_mutex);

/* Sensor device - set in main() */
static const struct device *sensor_dev;

/* Forward decl */
static void thresholds_save(void);

/* Save thresholds + reboot - must run in work queue, not BLE callback.
 * BLE callback has limited stack and FS ops can block/deadlock there. */
static void save_and_reboot_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	thresholds_save();
	printk("BLE: saved, rebooting to apply thresholds\n");
	sys_reboot(SYS_REBOOT_COLD);
}
static K_WORK_DELAYABLE_DEFINE(save_and_reboot_work, save_and_reboot_work_handler);

static uint8_t manuf_data[MANUF_DATA_BUF_SIZE];
static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, 0),
};
static bool ad_active;
static bool ad_connectable;
static bool log_requested;

/* Connectable advertising for GATT (Button 1 manual - name in scan response to fit 31-byte limit) */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
static struct bt_data ad_conn[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, 0),
};
static const struct bt_data sd_conn[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, sizeof(DEVICE_NAME) - 1),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, LOG_SVC_UUID),
};

static bool in_range(const struct sensor_value *temp, const struct sensor_value *hum)
{
	float t_float, t_lo, t_hi;
	int h_lo, h_hi;

	k_mutex_lock(&threshold_mutex, K_FOREVER);
	t_lo = sensor_value_to_float(&alert_temp_lo_val);
	t_hi = sensor_value_to_float(&alert_temp_hi_val);
	h_lo = alert_humidity_lo;
	h_hi = alert_humidity_hi;
	k_mutex_unlock(&threshold_mutex);

	t_float = sensor_value_to_float(temp);
	return t_float >= t_lo && t_float <= t_hi &&
	       hum->val1 >= h_lo && hum->val1 <= h_hi;
}

static bool temp_breached(const struct sensor_value *temp)
{
	float t_float, t_lo, t_hi;

	k_mutex_lock(&threshold_mutex, K_FOREVER);
	t_lo = sensor_value_to_float(&alert_temp_lo_val);
	t_hi = sensor_value_to_float(&alert_temp_hi_val);
	k_mutex_unlock(&threshold_mutex);

	t_float = sensor_value_to_float(temp);
	return t_float < t_lo || t_float > t_hi;
}

static bool hum_breached(const struct sensor_value *hum)
{
	int h_lo, h_hi;

	k_mutex_lock(&threshold_mutex, K_FOREVER);
	h_lo = alert_humidity_lo;
	h_hi = alert_humidity_hi;
	k_mutex_unlock(&threshold_mutex);

	return hum->val1 < h_lo || hum->val1 > h_hi;
}

/* Append one log line: {recycle}|{uptime}{A|N|M}|{T|H}{value}
 * A=alert/breach, N=back to normal, M=manual (Button 1) */
static void log_append_type(char type, bool is_temp, int val)
{
	struct fs_file_t f;

	if (!storage_available) {
		return;
	}
	char line[LOG_LINE_MAX];
	uint32_t uptime = k_uptime_get_32() / 1000; /* seconds */
	int n;
	int ret;

	fs_file_t_init(&f);
	n = snprintf(line, sizeof(line), "%u|%u%c|%c%d\n",
		     (unsigned int)recycle_count,
		     (unsigned int)uptime,
		     type,
		     is_temp ? 'T' : 'H',
		     val);
	if (n <= 0 || (size_t)n >= sizeof(line)) {
		return;
	}

	ret = fs_open(&f, PATH_EVENTS, FS_O_CREATE | FS_O_APPEND | FS_O_WRITE);
	if (ret != 0) {
		printk("log_append: fs_open failed %d\n", ret);
		return;
	}
	if (fs_write(&f, line, (size_t)n) != (ssize_t)n) {
		fs_close(&f);
		return;
	}
	fs_sync(&f);
	fs_close(&f);
}

/* Read recycle count, increment, write back. Called once at boot.
 * If any step fails, storage_available stays false and core logic continues.
 */
static void storage_init(void)
{
	struct fs_file_t f;
	char buf[16];
	ssize_t len;
	int n;
	uint32_t val;

	storage_available = false;
	recycle_count = 1;
	fs_file_t_init(&f);

	/* Open existing or create empty */
	if (fs_open(&f, PATH_RC, FS_O_CREATE | FS_O_READ) != 0) {
		return;
	}

	len = fs_read(&f, buf, sizeof(buf) - 1);
	fs_close(&f);
	if (len > 0) {
		buf[len] = '\0';
		n = 0;
		val = 0;
		while (buf[n] >= '0' && buf[n] <= '9') {
			val = val * 10 + (unsigned int)(buf[n] - '0');
			n++;
		}
		recycle_count = val;
	}

	recycle_count++;
	if (fs_open(&f, PATH_RC, FS_O_CREATE | FS_O_WRITE) != 0) {
		return;
	}
	len = snprintf(buf, sizeof(buf), "%u\n", (unsigned int)recycle_count);
	if (len > 0) {
		fs_write(&f, buf, (size_t)len);
	}
	fs_close(&f);
	storage_available = true;
}

/* Forward decl for use in thresholds_load */
static void double_to_sensor_value(double v, struct sensor_value *out);

/* Save current thresholds to LittleFS. Call when thresholds are updated via BLE. */
static void thresholds_save(void)
{
	if (!storage_available) {
		return;
	}
	struct fs_file_t f;
	char buf[48];
	int n;
	int tl_frac, th_frac;

	k_mutex_lock(&threshold_mutex, K_FOREVER);
	tl_frac = (int)((alert_temp_lo_val.val2 < 0 ? -alert_temp_lo_val.val2 :
			 alert_temp_lo_val.val2) * 100 / 1000000);
	th_frac = (int)((alert_temp_hi_val.val2 < 0 ? -alert_temp_hi_val.val2 :
			 alert_temp_hi_val.val2) * 100 / 1000000);
	n = snprintf(buf, sizeof(buf), "%d.%02d %d.%02d %d %d\n",
		     alert_temp_lo_val.val1, tl_frac,
		     alert_temp_hi_val.val1, th_frac,
		     alert_humidity_lo, alert_humidity_hi);
	k_mutex_unlock(&threshold_mutex);
	if (n <= 0 || (size_t)n >= sizeof(buf)) {
		return;
	}

	fs_file_t_init(&f);
	if (fs_open(&f, PATH_THRESHOLDS, FS_O_CREATE | FS_O_WRITE) != 0) {
		return;
	}
	if (fs_write(&f, buf, (size_t)n) != (ssize_t)n) {
		fs_close(&f);
		return;
	}
	fs_sync(&f);
	fs_close(&f);
}

/* Load thresholds from LittleFS. Returns true if loaded and applied. */
static bool thresholds_load(void)
{
	struct fs_file_t f;
	char buf[48];
	ssize_t len;
	int tl_val, tl_frac, th_val, th_frac, hl, hh;
	int n;

	fs_file_t_init(&f);
	if (fs_open(&f, PATH_THRESHOLDS, FS_O_READ) != 0) {
		return false;
	}
	len = fs_read(&f, buf, sizeof(buf) - 1);
	fs_close(&f);
	if (len <= 0) {
		return false;
	}
	buf[len] = '\0';
	n = 0;
	if (sscanf(buf, "%d.%d %d.%d %d %d%n",
		   &tl_val, &tl_frac, &th_val, &th_frac, &hl, &hh, &n) < 6) {
		return false;
	}
	/* Basic validation: low < high */
	if (tl_val > th_val || (tl_val == th_val && tl_frac >= th_frac) ||
	    hl >= hh || hl < 0 || hh > 100) {
		return false;
	}

	k_mutex_lock(&threshold_mutex, K_FOREVER);
	alert_temp_lo_val.val1 = tl_val;
	alert_temp_lo_val.val2 = (int32_t)tl_frac * 1000000 / 100;
	alert_temp_hi_val.val1 = th_val;
	alert_temp_hi_val.val2 = (int32_t)th_frac * 1000000 / 100;
	alert_humidity_lo = hl;
	alert_humidity_hi = hh;
	k_mutex_unlock(&threshold_mutex);

	return true;
}

static void storage_init_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	storage_init();
	if (storage_available) {
		printk("Storage: ready (recycle count: %u)\n", (unsigned int)recycle_count);
		/* Load persisted thresholds and apply to sensor if available */
		if (thresholds_load() && sensor_dev && device_is_ready(sensor_dev)) {
			struct sensor_value t_lo, t_hi, h_lo, h_hi;

			k_mutex_lock(&threshold_mutex, K_FOREVER);
			t_lo = alert_temp_lo_val;
			t_hi = alert_temp_hi_val;
			h_lo.val1 = alert_humidity_lo;
			h_lo.val2 = 0;
			h_hi.val1 = alert_humidity_hi;
			h_hi.val2 = 0;
			k_mutex_unlock(&threshold_mutex);

			if (sensor_attr_set(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP,
					    SENSOR_ATTR_LOWER_THRESH, &t_lo) == 0 &&
			    sensor_attr_set(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP,
					    SENSOR_ATTR_UPPER_THRESH, &t_hi) == 0 &&
			    sensor_attr_set(sensor_dev, SENSOR_CHAN_HUMIDITY,
					    SENSOR_ATTR_LOWER_THRESH, &h_lo) == 0 &&
			    sensor_attr_set(sensor_dev, SENSOR_CHAN_HUMIDITY,
					    SENSOR_ATTR_UPPER_THRESH, &h_hi) == 0) {
				printk("Thresholds: loaded from storage (T %d.%d-%d.%d C, H %d-%d %%RH)\n",
				       t_lo.val1,
				       (int)((t_lo.val2 < 0 ? -t_lo.val2 : t_lo.val2) * 10 / 1000000),
				       t_hi.val1,
				       (int)((t_hi.val2 < 0 ? -t_hi.val2 : t_hi.val2) * 10 / 1000000),
				       h_lo.val1, h_hi.val1);
			}
		}
	} else {
		printk("Storage: init failed, continuing without logging\n");
	}
}

K_WORK_DELAYABLE_DEFINE(storage_init_work, storage_init_work_handler);

/* Encode breach data as CBOR map: {"t": temp} and/or {"h": humidity} - only breached */
static size_t cbor_encode_breach(const struct sensor_value *temp,
				 const struct sensor_value *hum,
				 uint8_t *buf, size_t buf_len)
{
	bool inc_temp = temp_breached(temp);
	bool inc_hum = hum_breached(hum);
	size_t map_entries = (inc_temp ? 1 : 0) + (inc_hum ? 1 : 0);

	if (map_entries == 0 || buf_len < 4) {
		return 0;
	}

	ZCBOR_STATE_E(state, 1, buf, buf_len, 0);
	bool ok = zcbor_map_start_encode(state, map_entries);

	if (inc_temp) {
		float t = sensor_value_to_float(temp);
		ok = ok && zcbor_tstr_put_lit(state, "t") && zcbor_float32_put(state, t);
	}
	if (inc_hum) {
		float h = sensor_value_to_float(hum);
		ok = ok && zcbor_tstr_put_lit(state, "h") && zcbor_float32_put(state, h);
	}
	ok = ok && zcbor_map_end_encode(state, map_entries);

	if (!ok) {
		return 0;
	}
	return (size_t)(state->payload_mut - buf);
}

/* Encode both T and H (for manual Button 1 reading) */
static size_t cbor_encode_full(const struct sensor_value *temp,
			       const struct sensor_value *hum,
			       uint8_t *buf, size_t buf_len)
{
	if (buf_len < 8) {
		return 0;
	}
	ZCBOR_STATE_E(state, 1, buf, buf_len, 0);
	bool ok = zcbor_map_start_encode(state, 2);
	float t = sensor_value_to_float(temp);
	float h = sensor_value_to_float(hum);
	ok = ok && zcbor_tstr_put_lit(state, "t") && zcbor_float32_put(state, t);
	ok = ok && zcbor_tstr_put_lit(state, "h") && zcbor_float32_put(state, h);
	ok = ok && zcbor_map_end_encode(state, 2);
	return ok ? (size_t)(state->payload_mut - buf) : 0;
}

static int ble_start_advertise_connectable(const struct sensor_value *temp,
					   const struct sensor_value *hum)
{
	ad_conn[1].data_len = 0;
	if (temp && hum) {
		manuf_data[0] = COMPANY_ID & 0xff;
		manuf_data[1] = (COMPANY_ID >> 8) & 0xff;
		size_t cbor_len = cbor_encode_full(temp, hum, &manuf_data[2],
						    MANUF_DATA_BUF_SIZE - 2);
		ad_conn[1].data_len = 2 + cbor_len;
	}

	if (ad_active && ad_connectable) {
		return bt_le_adv_update_data(ad_conn, ARRAY_SIZE(ad_conn),
					     sd_conn, ARRAY_SIZE(sd_conn));
	}
	if (ad_active) {
		(void)bt_le_adv_stop();
		ad_active = false;
	}
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad_conn, ARRAY_SIZE(ad_conn),
				 sd_conn, ARRAY_SIZE(sd_conn));
	if (!err) {
		ad_active = true;
		ad_connectable = true;
	}
	return err;
}

static int ble_start_advertise_breach(const struct sensor_value *temp,
				      const struct sensor_value *hum)
{
	manuf_data[0] = COMPANY_ID & 0xff;
	manuf_data[1] = (COMPANY_ID >> 8) & 0xff;

	size_t cbor_len = cbor_encode_breach(temp, hum, &manuf_data[2],
					     MANUF_DATA_BUF_SIZE - 2);
	if (cbor_len == 0) {
		return -EINVAL;
	}

	ad[1].data_len = 2 + cbor_len;

	if (ad_active && !ad_connectable) {
		return bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
	}
	if (ad_active) {
		(void)bt_le_adv_stop();
		ad_active = false;
	}

	int err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (!err) {
		ad_active = true;
		ad_connectable = false;
	}
	return err;
}

static int ble_stop_advertise(void)
{
	if (!ad_active) {
		return 0;
	}
	int err = bt_le_adv_stop();
	if (!err) {
		ad_active = false;
		ad_connectable = false;
	}
	return err;
}

static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trig);
	k_sem_give(&threshold_sem);
}

/* Convert double to sensor_value (val1=integer, val2=frac in micro-units) */
static void double_to_sensor_value(double v, struct sensor_value *out)
{
	out->val1 = (int32_t)v;
	out->val2 = (int32_t)((v - (double)out->val1) * 1000000.0);
}

/* --- GATT: write commands to command char ---
 * Opcode 00: request events.log (write "00", then read events characteristic)
 * Opcode 01: set thresholds - format 01|TL10.5|TH30|HL30|HH90 (any subset of TL,TH,HL,HH)
 */
static ssize_t cmd_char_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset > 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len < 2) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	/* Opcode 00: request events.log */
	if (((const char *)buf)[0] == '0' && ((const char *)buf)[1] == '0') {
		log_requested = true;
		printk("BLE: user requested events.log (code 00)\n");
		return len;
	}

	/* Opcode 01: set thresholds - 01|TL10.5|TH30|HL30|HH90 or 01|RS to reset to defaults */
	if (((const char *)buf)[0] == '0' && ((const char *)buf)[1] == '1') {
		char cmd_buf[CMD_BUF_MAX];
		char *tok;
		bool has_tl = false, has_th = false, has_hl = false, has_hh = false;
		bool has_rs = false;
		struct sensor_value new_tl;
		struct sensor_value new_th;
		int new_hl;
		int new_hh;

		if (len >= CMD_BUF_MAX) {
			return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
		}
		memcpy(cmd_buf, buf, len);
		cmd_buf[len] = '\0';

		k_mutex_lock(&threshold_mutex, K_FOREVER);
		new_tl = alert_temp_lo_val;
		new_th = alert_temp_hi_val;
		new_hl = alert_humidity_lo;
		new_hh = alert_humidity_hi;
		k_mutex_unlock(&threshold_mutex);

		tok = strtok(cmd_buf, "|");
		if (!tok || strlen(tok) != 2 || tok[0] != '0' || tok[1] != '1') {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}

		while ((tok = strtok(NULL, "|")) != NULL) {
			if (strlen(tok) == 2 && tok[0] == 'R' && tok[1] == 'S') {
				has_rs = true;
				break;
			}
			if (strlen(tok) < 3) {
				continue;
			}
			double fval;
			char *end;
			fval = strtod(tok + 2, &end);
			if (end == tok + 2) {
				return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
			}

			if (tok[0] == 'T' && tok[1] == 'L') {
				has_tl = true;
				double_to_sensor_value(fval, &new_tl);
			} else if (tok[0] == 'T' && tok[1] == 'H') {
				has_th = true;
				double_to_sensor_value(fval, &new_th);
			} else if (tok[0] == 'H' && tok[1] == 'L') {
				has_hl = true;
				new_hl = (int)fval;
			} else if (tok[0] == 'H' && tok[1] == 'H') {
				has_hh = true;
				new_hh = (int)fval;
			}
		}

		if (has_rs) {
			new_tl.val1 = DEFAULT_TEMP_LO;
			new_tl.val2 = 0;
			new_th.val1 = DEFAULT_TEMP_HI;
			new_th.val2 = 0;
			new_hl = DEFAULT_HUMIDITY_LO;
			new_hh = DEFAULT_HUMIDITY_HI;
			has_tl = has_th = has_hl = has_hh = true;
		}

		/* Validation: low must be < high for each pair */
		{
			float tl_f = sensor_value_to_float(&new_tl);
			float th_f = sensor_value_to_float(&new_th);
			if (has_tl && has_th && tl_f >= th_f) {
				printk("BLE: threshold error: TL (%d.%d) must be < TH (%d.%d)\n",
				       new_tl.val1,
				       (int)((new_tl.val2 < 0 ? -new_tl.val2 : new_tl.val2) * 10 / 1000000),
				       new_th.val1,
				       (int)((new_th.val2 < 0 ? -new_th.val2 : new_th.val2) * 10 / 1000000));
				return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
			}
			if (has_hl && has_hh && new_hl >= new_hh) {
				printk("BLE: threshold error: HL (%d) must be < HH (%d)\n",
				       new_hl, new_hh);
				return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
			}
			/* Validate new values against existing if only one of pair is updated */
			if (has_tl && !has_th) {
				float th_f_cur = sensor_value_to_float(&alert_temp_hi_val);
				if (tl_f >= th_f_cur) {
					return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
				}
			}
			if (has_th && !has_tl) {
				float tl_f_cur = sensor_value_to_float(&alert_temp_lo_val);
				if (tl_f_cur >= th_f) {
					return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
				}
			}
			if (has_hl && !has_hh && new_hl >= alert_humidity_hi) {
				return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
			}
			if (has_hh && !has_hl && alert_humidity_lo >= new_hh) {
				return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
			}
		}

		k_mutex_lock(&threshold_mutex, K_FOREVER);
		if (has_tl) {
			alert_temp_lo_val = new_tl;
		}
		if (has_th) {
			alert_temp_hi_val = new_th;
		}
		if (has_hl) {
			alert_humidity_lo = new_hl;
		}
		if (has_hh) {
			alert_humidity_hi = new_hh;
		}
		k_mutex_unlock(&threshold_mutex);

		printk("BLE: thresholds set (T %d.%d-%d.%d C, H %d-%d %%RH)\n",
		       alert_temp_lo_val.val1,
		       (int)((alert_temp_lo_val.val2 < 0 ? -alert_temp_lo_val.val2 :
			      alert_temp_lo_val.val2) * 10 / 1000000),
		       alert_temp_hi_val.val1,
		       (int)((alert_temp_hi_val.val2 < 0 ? -alert_temp_hi_val.val2 :
			      alert_temp_hi_val.val2) * 10 / 1000000),
		       alert_humidity_lo, alert_humidity_hi);
		printk("BLE: scheduling save & reboot in %d ms\n", REBOOT_DELAY_MS);
		k_work_schedule(&save_and_reboot_work, K_MSEC(REBOOT_DELAY_MS));
		return len;
	}

	return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
}

static ssize_t events_char_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);

	if (!log_requested) {
		return BT_GATT_ERR(BT_ATT_ERR_READ_NOT_PERMITTED);
	}
	if (!storage_available) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	struct fs_file_t f;
	fs_file_t_init(&f);
	int ret = fs_open(&f, PATH_EVENTS, FS_O_READ);
	if (ret != 0) {
		return 0; /* File may not exist yet */
	}

	if (offset > 0) {
		ret = fs_seek(&f, offset, FS_SEEK_SET);
		if (ret != 0) {
			fs_close(&f);
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}
	}

	ssize_t n = fs_read(&f, buf, len);
	fs_close(&f);
	return (n >= 0) ? (ssize_t)n : BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
}

BT_GATT_SERVICE_DEFINE(log_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_LOG_SVC),
	BT_GATT_CHARACTERISTIC(BT_UUID_CMD_CHAR,
			      BT_GATT_CHRC_WRITE,
			      BT_GATT_PERM_WRITE,
			      NULL, cmd_char_write, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_EVENTS_CHAR,
			      BT_GATT_CHRC_READ,
			      BT_GATT_PERM_READ,
			      events_char_read, NULL, NULL),
);

static void conn_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("BLE: connected %s\n", addr);
}

static void conn_disconnected(struct bt_conn *conn, uint8_t reason)
{
	log_requested = false;
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("BLE: disconnected %s (reason %u)\n", addr, reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = conn_connected,
	.disconnected = conn_disconnected,
};

/* Button 1 (SW1) interrupt - wake for manual reading */
#if DT_NODE_HAS_PROP(SW1_NODE, gpios)
static void button_pressed(const struct device *gpio_dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	ARG_UNUSED(gpio_dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_sem_give(&button_sem);
}
#endif

int main(void)
{
	const struct device *dev = DEVICE_DT_GET_ONE(sensirion_sht3xd);
	struct sensor_value temp_lo, temp_hi, hum_lo, hum_hi;
	struct sensor_value temp, hum;
	struct sensor_trigger trig;
	int rc;

	if (!device_is_ready(dev)) {
		printk("SHT30-D: device not ready\n");
		return 0;
	}

	printk("SHT30-D: initializing...\n");

	/* Defer storage init so sensor/trigger/BLE stabilize first (avoids init-order issues) */
	storage_available = false;
	recycle_count = 1;
	k_work_schedule(&storage_init_work, K_MSEC(STORAGE_INIT_DELAY_MS));

	/* Bluetooth */
	rc = bt_enable(NULL);
	if (rc) {
		printk("BLE init failed: %d\n", rc);
		return 0;
	}
	printk("BLE initialized\n");
	/* No advertising when idle - only on threshold breach or Button 1 press */

#if DT_NODE_HAS_PROP(SW1_NODE, gpios)
	/* Button 1 (SW1) - wake for manual T/H reading */
	static struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
	static struct gpio_callback button_cb;

	if (!gpio_is_ready_dt(&button)) {
		printk("Button 1 not ready\n");
	} else {
		rc = gpio_pin_configure_dt(&button, GPIO_INPUT);
		if (rc == 0) {
			rc = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
		}
		if (rc == 0) {
			gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
			gpio_add_callback(button.port, &button_cb);
			printk("Button 1 (SW1): press to read T/H, log & advertise\n");
		}
	}
#endif

	/* Store sensor device for BLE threshold updates */
	sensor_dev = dev;

	/* Temperature thresholds */
	temp_lo = alert_temp_lo_val;
	temp_hi = alert_temp_hi_val;
	rc = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
			     SENSOR_ATTR_LOWER_THRESH, &temp_lo);
	if (rc) {
		printk("SHT30-D: failed to set temp low: %d\n", rc);
		return 0;
	}
	rc = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
			     SENSOR_ATTR_UPPER_THRESH, &temp_hi);
	if (rc) {
		printk("SHT30-D: failed to set temp high: %d\n", rc);
		return 0;
	}

	/* Humidity thresholds */
	hum_lo.val1 = alert_humidity_lo;
	hum_lo.val2 = 0;
	hum_hi.val1 = alert_humidity_hi;
	hum_hi.val2 = 0;
	rc = sensor_attr_set(dev, SENSOR_CHAN_HUMIDITY,
			     SENSOR_ATTR_LOWER_THRESH, &hum_lo);
	if (rc) {
		printk("SHT30-D: failed to set humidity low: %d\n", rc);
		return 0;
	}
	rc = sensor_attr_set(dev, SENSOR_CHAN_HUMIDITY,
			     SENSOR_ATTR_UPPER_THRESH, &hum_hi);
	if (rc) {
		printk("SHT30-D: failed to set humidity high: %d\n", rc);
		return 0;
	}

	trig.type = SENSOR_TRIG_THRESHOLD;
	trig.chan = SENSOR_CHAN_HUMIDITY;
	rc = sensor_trigger_set(dev, &trig, trigger_handler);
	if (rc) {
		printk("SHT30-D: trigger setup failed: %d\n", rc);
		return 0;
	}

	printk("SHT30-D: thresholds (Temp %d-%d C, Hum %d-%d %%RH)\n",
	       DEFAULT_TEMP_LO, DEFAULT_TEMP_HI, DEFAULT_HUMIDITY_LO, DEFAULT_HUMIDITY_HI);
	printk("SHT30-D: sleeping - wake on threshold breach or Button 1 press\n\n");

#if DT_NODE_HAS_PROP(SW1_NODE, gpios)
	struct k_poll_event events[] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&threshold_sem, 0),
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&button_sem, 0),
	};
#define NUM_POLL_EVENTS 2
#else
#define NUM_POLL_EVENTS 1
	struct k_poll_event events[] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&threshold_sem, 0),
	};
#endif

	while (true) {
		bool temp_breach_logged = false;
		bool hum_breach_logged = false;
		bool from_button = false;

		/* Reset poll events and wait for threshold OR button */
		for (int i = 0; i < NUM_POLL_EVENTS; i++) {
			events[i].state = K_POLL_STATE_NOT_READY;
			events[i].unused = 0U;
		}
		rc = k_poll(events, NUM_POLL_EVENTS, K_FOREVER);
		if (rc != 0) {
			continue;
		}

#if DT_NODE_HAS_PROP(SW1_NODE, gpios)
		if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
			(void)k_sem_take(&button_sem, K_NO_WAIT); /* consume */
			from_button = true;
		}
#endif
		if (!from_button && events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
			(void)k_sem_take(&threshold_sem, K_NO_WAIT); /* consume */
		}

		rc = sensor_sample_fetch(dev);
		if (rc) {
			printk("SHT30-D: fetch failed: %d\n", rc);
			continue;
		}

		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum);

		if (from_button) {
			/* Button 1: manual reading - log, advertise connectable (user can connect) */
			printk("*** BUTTON 1: MANUAL READING *** %d.%02d C ; %d.%02d %%RH\n",
			       temp.val1, (int)((temp.val2 < 0 ? -temp.val2 : temp.val2) * 100 / 1000000),
			       hum.val1, (int)((hum.val2 < 0 ? -hum.val2 : hum.val2) * 100 / 1000000));

			log_append_type('M', true, temp.val1);
			log_append_type('M', false, hum.val1);

			rc = ble_start_advertise_connectable(&temp, &hum);
			if (rc) {
				printk("BLE manual advertise failed: %d\n", rc);
			} else {
				printk("BLE: connectable with T/H - connect & write 00 for events.log\n");
			}

			k_sleep(K_MSEC(MANUAL_ADV_TIMEOUT_MS));
			ble_stop_advertise(); /* back to idle connectable */
			printk("BLE: back to idle advertising\n\n");
			continue;
		}

		/* Threshold breach */
		printk("*** THRESHOLD BREACH *** %d.%02d C ; %d.%02d %%RH\n",
		       temp.val1, (int)((temp.val2 < 0 ? -temp.val2 : temp.val2) * 100 / 1000000),
		       hum.val1, (int)((hum.val2 < 0 ? -hum.val2 : hum.val2) * 100 / 1000000));

		/* Log breach on first detection (initial fetch) */
		if (temp_breached(&temp)) {
			log_append_type('A', true, temp.val1);
			temp_breach_logged = true;
		}
		if (hum_breached(&hum)) {
			log_append_type('A', false, hum.val1);
			hum_breach_logged = true;
		}

		/* Advertise only when we have breach data (avoids -EINVAL from empty payload) */
		if (temp_breached(&temp) || hum_breached(&hum)) {
			rc = ble_start_advertise_breach(&temp, &hum);
			if (rc) {
				printk("BLE advertise start failed: %d\n", rc);
			} else {
				printk("BLE: advertising breach (CBOR)\n");
			}
		}

		/* Poll until back to normal (or button press to exit early) */
		while (true) {
#if DT_NODE_HAS_PROP(SW1_NODE, gpios)
			/* Poll for button or timeout */
			struct k_poll_event inner_ev[] = {
				K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
								K_POLL_MODE_NOTIFY_ONLY,
								&button_sem, 0),
			};
			for (int i = 0; i < ARRAY_SIZE(inner_ev); i++) {
				inner_ev[i].state = K_POLL_STATE_NOT_READY;
			}
			rc = k_poll(inner_ev, ARRAY_SIZE(inner_ev), K_MSEC(POLL_INTERVAL_MS));
			if (rc == 0 && inner_ev[0].state == K_POLL_STATE_SEM_AVAILABLE) {
				break;  /* Button - outer loop handles */
			}
#else
			k_sleep(K_MSEC(POLL_INTERVAL_MS));
#endif

			rc = sensor_sample_fetch(dev);
			if (rc) {
				printk("SHT30-D: fetch failed in poll loop: %d\n", rc);
				break;  /* Exit to k_poll so we can respond to button */
			}
			sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum);

			if (in_range(&temp, &hum)) {
				ble_stop_advertise();
				/* Log back-to-normal: separate line per quantity */
				log_append_type('N', true, temp.val1);
				log_append_type('N', false, hum.val1);
				printk("*** BACK TO NORMAL *** %d.%02d C ; %d.%02d %%RH\n",
				       temp.val1, (int)((temp.val2 < 0 ? -temp.val2 : temp.val2) * 100 / 1000000),
				       hum.val1, (int)((hum.val2 < 0 ? -hum.val2 : hum.val2) * 100 / 1000000));
				printk("BLE: stopped advertising\n");
				printk("SHT30-D: sleeping - will wake on next breach\n\n");
				break;
			}
			/* Log breach on first detection during poll (if missed on initial fetch) */
			if (temp_breached(&temp) && !temp_breach_logged) {
				log_append_type('A', true, temp.val1);
				temp_breach_logged = true;
			}
			if (hum_breached(&hum) && !hum_breach_logged) {
				log_append_type('A', false, hum.val1);
				hum_breach_logged = true;
			}
			/* Update advertising when we have breach data */
			if (temp_breached(&temp) || hum_breached(&hum)) {
				ble_start_advertise_breach(&temp, &hum);
			}
		}
	}

	return 0;
}
