/*
 * SHT30-D Temperature/Humidity sensor with threshold interrupt wake-up
 *
 * Connects SHT30-D to nRF52840 DK. When temp/humidity exceeds configured
 * thresholds, the ALERT pin triggers an interrupt and wakes the CPU.
 *
 * Wiring (see boards/nrf52840dk_nrf52840.overlay):
 *   SHT30-D: VDD GND SDA SCL ALERT
 *   nRF52840: 3V3 GND P0.26 P0.27 P1.3
 */

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/* Thresholds - adjust for your application */
#define ALERT_TEMP_LO   15
#define ALERT_TEMP_HI   35
#define ALERT_HUMIDITY_LO  30
#define ALERT_HUMIDITY_HI  70

#define POLL_INTERVAL_MS   2000  /* Check every 2s when waiting for "back to normal" */

static K_SEM_DEFINE(threshold_sem, 0, 1);

static bool in_range(const struct sensor_value *temp, const struct sensor_value *hum)
{
	return temp->val1 >= ALERT_TEMP_LO && temp->val1 <= ALERT_TEMP_HI &&
	       hum->val1 >= ALERT_HUMIDITY_LO && hum->val1 <= ALERT_HUMIDITY_HI;
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

	/* Set temperature thresholds */
	temp_lo.val1 = ALERT_TEMP_LO;
	temp_lo.val2 = 0;
	temp_hi.val1 = ALERT_TEMP_HI;
	temp_hi.val2 = 0;
	rc = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
			     SENSOR_ATTR_LOWER_THRESH, &temp_lo);
	if (rc) {
		printk("SHT30-D: failed to set temp low threshold: %d\n", rc);
		return 0;
	}
	rc = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
			     SENSOR_ATTR_UPPER_THRESH, &temp_hi);
	if (rc) {
		printk("SHT30-D: failed to set temp high threshold: %d\n", rc);
		return 0;
	}

	/* Set humidity thresholds */
	hum_lo.val1 = ALERT_HUMIDITY_LO;
	hum_lo.val2 = 0;
	hum_hi.val1 = ALERT_HUMIDITY_HI;
	hum_hi.val2 = 0;
	rc = sensor_attr_set(dev, SENSOR_CHAN_HUMIDITY,
			     SENSOR_ATTR_LOWER_THRESH, &hum_lo);
	if (rc) {
		printk("SHT30-D: failed to set humidity low threshold: %d\n", rc);
		return 0;
	}
	rc = sensor_attr_set(dev, SENSOR_CHAN_HUMIDITY,
			     SENSOR_ATTR_UPPER_THRESH, &hum_hi);
	if (rc) {
		printk("SHT30-D: failed to set humidity high threshold: %d\n", rc);
		return 0;
	}

	/* Register threshold trigger - fires when ALERT pin activates */
	trig.type = SENSOR_TRIG_THRESHOLD;
	trig.chan = SENSOR_CHAN_HUMIDITY;  /* ALERT fires for temp or humidity */
	rc = sensor_trigger_set(dev, &trig, trigger_handler);
	if (rc) {
		printk("SHT30-D: trigger setup failed: %d\n", rc);
		return 0;
	}

	printk("SHT30-D: thresholds set (Temp: %d-%d C, Humidity: %d-%d %%RH)\n",
	       ALERT_TEMP_LO, ALERT_TEMP_HI,
	       ALERT_HUMIDITY_LO, ALERT_HUMIDITY_HI);
	printk("SHT30-D: sleeping - will wake on threshold breach\n\n");

	while (true) {
		/* Sleep until ALERT interrupt wakes us (breach) */
		k_sem_take(&threshold_sem, K_FOREVER);

		/* Fetch and print breach reading */
		rc = sensor_sample_fetch(dev);
		if (rc) {
			printk("SHT30-D: fetch failed: %d\n", rc);
			continue;
		}

		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum);

		printk("*** THRESHOLD BREACH *** %d.%02d C ; %d.%02d %%RH (out of range)\n",
		       temp.val1, (int)((temp.val2 < 0 ? -temp.val2 : temp.val2) * 100 / 1000000),
		       hum.val1, (int)((hum.val2 < 0 ? -hum.val2 : hum.val2) * 100 / 1000000));

		/* Poll until values return to normal (driver only fires on breach, not on clear) */
		while (true) {
			k_sleep(K_MSEC(POLL_INTERVAL_MS));

			rc = sensor_sample_fetch(dev);
			if (rc) {
				continue;
			}
			sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum);

			if (in_range(&temp, &hum)) {
				printk("*** BACK TO NORMAL *** %d.%02d C ; %d.%02d %%RH (within range)\n",
				       temp.val1, (int)((temp.val2 < 0 ? -temp.val2 : temp.val2) * 100 / 1000000),
				       hum.val1, (int)((hum.val2 < 0 ? -hum.val2 : hum.val2) * 100 / 1000000));
				printk("SHT30-D: sleeping - will wake on next threshold breach\n\n");
				break;
			}
		}
	}

	return 0;
}
