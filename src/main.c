#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
#define SW1_NODE	DT_ALIAS(sw1)
#if !DT_NODE_HAS_STATUS(SW1_NODE, okay)
#error "Unsupported board: sw1 devicetree alias is not defined"
#endif
#define SW2_NODE	DT_ALIAS(sw2)
#if !DT_NODE_HAS_STATUS(SW2_NODE, okay)
#error "Unsupported board: sw2 devicetree alias is not defined"
#endif

static void connect(void);

const char ledble_addr[18] = "78:9C:E7:37:2A:6C";
static struct bt_conn *default_conn;
static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_uuid_16 service_uuid = BT_UUID_INIT_16(0xFFE0);
static struct bt_uuid_16 char_uuid = BT_UUID_INIT_16(0xFFE1);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_write_params write_params;
static char data[] = { 0x7E, 0x07, 0x05, 0x03, 0x00, 0xFF, 0x00, 0x00, 0xEF };
static uint16_t att_handle = 1;

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct k_sem sem0;
static struct k_sem sem1;
static struct k_sem sem2;
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios,
							      {0});
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET_OR(SW2_NODE, gpios,
							      {0});
static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	if (pins & 0x2000)
	{
		k_sem_give(&sem0);
	}
	else if (pins & 0x4000)
	{
		k_sem_give(&sem1);
	}
	else if (pins & 0x8000)
	{
		k_sem_give(&sem2);
	}
	else
	{
		printk("No effect implemented for pins 0x%X\n", pins);
	}
}

void written(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params* params)
{
	printk("Write successful\n");
	printk("[err] 0x%X\n", err);
}

void write(struct bt_conn *conn, uint16_t handle)
{
	printk("[ATTRIBUTE] handle %u\n", handle);
	int err;
	write_params.func = written;
	write_params.handle = handle;
	write_params.data = data;
	write_params.length = sizeof(data);
	write_params.offset = 0;
	err = bt_gatt_write(conn, &write_params);
	if (err)
	{
		printk("Failed to write (err %d)\n", err);
	}
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	// the characteristic is 2 away from the service
	att_handle = attr->handle + 2;
	printk("[ATTRIBUTE] handle %u stored\n", att_handle);
	write(default_conn, att_handle);

	return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;
		return;
	}

	printk("Connected: %s\n", addr);

	if (conn == default_conn) {
		memcpy(&uuid, &service_uuid, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printk("Discover failed(err %d)\n", err);
			return;
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;
}

static void connect(void)
{
	int err;

	bt_addr_le_t addr;
	err = bt_addr_le_from_str(ledble_addr, "public", &addr);
	if (err)
	{
		printk("Failed to parse BTLE address %s\n", ledble_addr);
		return;
	}

	err = bt_conn_le_create(&addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM(BT_GAP_INIT_CONN_INT_MIN, BT_GAP_INIT_CONN_INT_MAX, 0, 2000), &default_conn);
	if (err) {
		printk("Create conn failed (err %d)\n", err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void main(void)
{
	int err;

	k_sem_init(&sem0, 0, 1);
	k_sem_init(&sem1, 0, 1);
	k_sem_init(&sem2, 0, 1);

	if (!device_is_ready(button0.port)) {
		printk("Error: button device %s is not ready\n",
		       button0.port->name);
		return;
	}
	
	if (!device_is_ready(button1.port)) {
		printk("Error: button device %s is not ready\n",
		       button1.port->name);
		return;
	}
	
	if (!device_is_ready(button2.port)) {
		printk("Error: button device %s is not ready\n",
		       button2.port->name);
		return;
	}

	err = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       err, button0.port->name, button0.pin);
		return;
	}

	err = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       err, button1.port->name, button1.pin);
		return;
	}

	err = gpio_pin_configure_dt(&button2, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       err, button2.port->name, button2.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button0,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			err, button0.port->name, button0.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button1,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			err, button1.port->name, button1.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button2,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			err, button2.port->name, button2.pin);
		return;
	}

	gpio_init_callback(&button0_cb_data, button_pressed, BIT(button0.pin));
	gpio_init_callback(&button1_cb_data, button_pressed, BIT(button1.pin));
	gpio_init_callback(&button2_cb_data, button_pressed, BIT(button2.pin));
	gpio_add_callback(button0.port, &button0_cb_data);
	gpio_add_callback(button1.port, &button1_cb_data);
	gpio_add_callback(button2.port, &button2_cb_data);
	printk("Set up button at %s pin %d\n", button0.port->name, button0.pin);
	printk("Set up button at %s pin %d\n", button1.port->name, button1.pin);
	printk("Set up button at %s pin %d\n", button2.port->name, button2.pin);

	if (led.port && !device_is_ready(led.port)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       err, led.port->name);
		led.port = NULL;
	}
	if (led.port) {
		err = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (err != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       err, led.port->name, led.pin);
			led.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}
	
	err = bt_enable(NULL);

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	printk("Press a button\n");
	if (led.port) {
		while (1) {
			if (k_sem_take(&sem0, K_MSEC(50)) == 0) {
				data[4] = 0xFF;
				data[5] = 0x00;
				data[6] = 0x00;
				connect();
			}
			if (k_sem_take(&sem1, K_MSEC(50)) == 0) {
				data[4] = 0x00;
				data[5] = 0xFF;
				data[6] = 0x00;
				connect();
			}
			if (k_sem_take(&sem2, K_MSEC(50)) == 0) {
				data[4] = 0x00;
				data[5] = 0x00;
				data[6] = 0xFF;
				connect();
			}
		}
	}
}