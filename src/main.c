/*
 * SHT30-D Temperature/Humidity sensor with threshold interrupt and BLE advertising
 *
 * When temp/humidity breaches thresholds:
 * - Wakes from sleep
 * - Advertises only the breached value(s) over BLE in CBOR format
 * - Stops advertising when values return to normal
 * - Logs events to LittleFS: {recycle}|{uptime}A|T{val} or H{val} (breach),
 *   {recycle}|{uptime}N|T{val} or H{val} (back to normal)
 * - Recycle counter persists across power cycles
 *
 * Wiring: SHT30-D VDD GND SDA SCL ALERT -> nRF52840 3V3 GND P0.26 P0.27 P1.3
 */

#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sys/util.h>
#include <zcbor_encode.h>
#include <zcbor_common.h>

/* Thresholds - adjust for your application */
#define ALERT_TEMP_LO        15
#define ALERT_TEMP_HI        35
#define ALERT_HUMIDITY_LO    30
#define ALERT_HUMIDITY_HI    70
#define POLL_INTERVAL_MS     2000

/* BLE manufacturer data: 2-byte company ID + CBOR payload (max ~27 bytes) */
#define COMPANY_ID           0xFFFF  /* Use your company ID or 0xFFFF for dev */
#define MANUF_DATA_BUF_SIZE  30

/* LittleFS paths (fstab mounts at /lfs1) */
#define PATH_RC       "/lfs1/rc"
#define PATH_EVENTS   "/lfs1/events.log"
#define LOG_LINE_MAX  48

static K_SEM_DEFINE(threshold_sem, 0, 1);

#define STORAGE_INIT_DELAY_MS  2000

/* Persisted across power cycles */
static uint32_t recycle_count;
static bool storage_available;

static uint8_t manuf_data[MANUF_DATA_BUF_SIZE];
static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, 0),
};
static bool ad_active;

static bool in_range(const struct sensor_value *temp, const struct sensor_value *hum)
{
	return temp->val1 >= ALERT_TEMP_LO && temp->val1 <= ALERT_TEMP_HI &&
	       hum->val1 >= ALERT_HUMIDITY_LO && hum->val1 <= ALERT_HUMIDITY_HI;
}

static bool temp_breached(const struct sensor_value *temp)
{
	return temp->val1 < ALERT_TEMP_LO || temp->val1 > ALERT_TEMP_HI;
}

static bool hum_breached(const struct sensor_value *hum)
{
	return hum->val1 < ALERT_HUMIDITY_LO || hum->val1 > ALERT_HUMIDITY_HI;
}

/* Append one log line: {recycle}|{uptime}{A|N}|{T|H}{value} */
static void log_append(bool alert, bool is_temp, int val)
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
	n = snprintf(line, sizeof(line), "%u|%u%s|%c%d\n",
		     (unsigned int)recycle_count,
		     (unsigned int)uptime,
		     alert ? "A" : "N",
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

static void storage_init_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	storage_init();
	if (storage_available) {
		printk("Storage: ready (recycle count: %u)\n", (unsigned int)recycle_count);
	} else {
		printk("Storage: init failed, continuing without logging\n");
	}
}

K_WORK_DELAYABLE_DEFINE(storage_init_work, storage_init_work_handler);

/* Encode breach data as CBOR map: {"t": temp} and/or {"h": humidity} */
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

static int ble_start_advertise_breach(const struct sensor_value *temp,
				      const struct sensor_value *hum)
{
	/* Manufacturer data: company_id (2 bytes LE) + CBOR */
	manuf_data[0] = COMPANY_ID & 0xff;
	manuf_data[1] = (COMPANY_ID >> 8) & 0xff;

	size_t cbor_len = cbor_encode_breach(temp, hum, &manuf_data[2],
					     MANUF_DATA_BUF_SIZE - 2);
	if (cbor_len == 0) {
		return -EINVAL;
	}

	ad[1].data_len = 2 + cbor_len;

	if (ad_active) {
		return bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
	}

	int err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (!err) {
		ad_active = true;
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

	/* Temperature thresholds */
	temp_lo.val1 = ALERT_TEMP_LO;
	temp_lo.val2 = 0;
	temp_hi.val1 = ALERT_TEMP_HI;
	temp_hi.val2 = 0;
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
	hum_lo.val1 = ALERT_HUMIDITY_LO;
	hum_lo.val2 = 0;
	hum_hi.val1 = ALERT_HUMIDITY_HI;
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
	       ALERT_TEMP_LO, ALERT_TEMP_HI, ALERT_HUMIDITY_LO, ALERT_HUMIDITY_HI);
	printk("SHT30-D: sleeping - will advertise breach via BLE on wake\n\n");

	while (true) {
		bool temp_breach_logged = false;
		bool hum_breach_logged = false;

		k_sem_take(&threshold_sem, K_FOREVER);

		rc = sensor_sample_fetch(dev);
		if (rc) {
			printk("SHT30-D: fetch failed: %d\n", rc);
			continue;
		}

		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum);

		printk("*** THRESHOLD BREACH *** %d.%02d C ; %d.%02d %%RH\n",
		       temp.val1, (int)((temp.val2 < 0 ? -temp.val2 : temp.val2) * 100 / 1000000),
		       hum.val1, (int)((hum.val2 < 0 ? -hum.val2 : hum.val2) * 100 / 1000000));

		/* Log breach on first detection (initial fetch) */
		if (temp_breached(&temp)) {
			log_append(true, true, temp.val1);
			temp_breach_logged = true;
		}
		if (hum_breached(&hum)) {
			log_append(true, false, hum.val1);
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

		/* Poll until back to normal */
		while (true) {
			k_sleep(K_MSEC(POLL_INTERVAL_MS));

			rc = sensor_sample_fetch(dev);
			if (rc) {
				continue;
			}
			sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum);

			if (in_range(&temp, &hum)) {
				ble_stop_advertise();
				/* Log back-to-normal: separate line per quantity */
				log_append(false, true, temp.val1);
				log_append(false, false, hum.val1);
				printk("*** BACK TO NORMAL *** %d.%02d C ; %d.%02d %%RH\n",
				       temp.val1, (int)((temp.val2 < 0 ? -temp.val2 : temp.val2) * 100 / 1000000),
				       hum.val1, (int)((hum.val2 < 0 ? -hum.val2 : hum.val2) * 100 / 1000000));
				printk("BLE: stopped advertising\n");
				printk("SHT30-D: sleeping - will wake on next breach\n\n");
				break;
			}
			/* Log breach on first detection during poll (if missed on initial fetch) */
			if (temp_breached(&temp) && !temp_breach_logged) {
				log_append(true, true, temp.val1);
				temp_breach_logged = true;
			}
			if (hum_breached(&hum) && !hum_breach_logged) {
				log_append(true, false, hum.val1);
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
