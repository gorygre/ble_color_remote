#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/zephyr.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

static void connect(void);

const char ledble_addr[18] = "78:9C:E7:37:2A:6C";
static struct bt_conn *default_conn;
static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_uuid_16 service_uuid = BT_UUID_INIT_16(0xFFE0);
static struct bt_uuid_16 char_uuid = BT_UUID_INIT_16(0xFFE1);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_write_params write_params;
static char data[] = { 0x7E, 0x07, 0x05, 0x03, 0xCC, 0x88, 0x99, 0x00, 0xEF };
static uint16_t att_handle = 1;

void written(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params* params)
{
	printk("Write successful\n");
	printk("[err] 0x%X\n", err);
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
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

	printk("[ATTRIBUTE] handle %u\n", attr->handle);
	att_handle = attr->handle + 2;
	write(conn, att_handle);

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

		//connect();
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

	//connect();
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

	err = bt_conn_le_create(&addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &default_conn);
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
	err = bt_enable(NULL);

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	connect();
}
