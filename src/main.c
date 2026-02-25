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
 * Button 1 (SW1): Wake from idle, read T/H, log to events.log, advertise (connectable)
 * so user can connect via BLE and fetch events.log (write "00" to command char).
 *
 * Wiring: SHT30-D VDD GND SDA SCL ALERT -> nRF52840 3V3 GND P0.26 P0.27 P1.3
 */

#include <errno.h>
#include <stdio.h>
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
#include <zephyr/sys/util.h>
#include <zcbor_encode.h>
#include <zcbor_common.h>

/* Thresholds - adjust for your application */
#define ALERT_TEMP_LO        15
#define ALERT_TEMP_HI        35
#define ALERT_HUMIDITY_LO    30
#define ALERT_HUMIDITY_HI    70
#define POLL_INTERVAL_MS      2000
#define MANUAL_ADV_TIMEOUT_MS 60000  /* Stay connectable with T/H for 60s after Button 1 */

/* BLE manufacturer data: 2-byte company ID + CBOR payload (max ~27 bytes) */
#define COMPANY_ID           0xFFFF  /* Use your company ID or 0xFFFF for dev */
#define MANUF_DATA_BUF_SIZE  30

/* LittleFS paths (fstab mounts at /lfs1) */
#define PATH_RC       "/lfs1/rc"
#define PATH_EVENTS   "/lfs1/events.log"
#define LOG_LINE_MAX  48

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

/* Persisted across power cycles */
static uint32_t recycle_count;
static bool storage_available;

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
	int err = bt_le_adv_start(BT_LE_ADV_CONN, ad_conn, ARRAY_SIZE(ad_conn),
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

/* --- GATT: write "00" to command, read events.log from events characteristic --- */
static ssize_t cmd_char_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset > 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len >= 2 && ((const char *)buf)[0] == '0' && ((const char *)buf)[1] == '0') {
		log_requested = true;
		printk("BLE: user requested events.log (code 00)\n");
		return len;
	}
	return len;
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
