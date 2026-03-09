#include "config.h"
#include "network_manager.h"
#include "console_cmds.h"
#include "nvs.h"

/*
Tested with 3.0.3 esp32 arduino board package by Espressif Systems.
*/

// Hot-path forwarding buffers — local to this file
static uint8_t  gl_pld_buffer[PAYLOAD_BUFFER_SIZE] = {0};
static buffer_t gl_uart_buffer = {
	.buf  = gl_pld_buffer,
	.size = sizeof(gl_pld_buffer),
	.len  = 0
};
static uint8_t udp_pkt_buf[UDP_PKT_BUF_SIZE] = {0};

// ---------------------------------------------------------------------------

static void handle_udp(uint32_t ts)
{
	int len = udp.parsePacket();
	if (len == 0) return;

	len = udp.read(udp_pkt_buf, UDP_PKT_BUF_SIZE - 1);

	int cmp = cmd_match((const char *)udp_pkt_buf, "WHO_GOES_THERE");
	if (cmp > 0)
	{
		int nlen = strlen(gl_prefs.name);
		udp.beginPacket(udp.remoteIP(), udp.remotePort() + gl_prefs.reply_offset);
		udp.write((uint8_t *)gl_prefs.name, nlen);
		udp.endPacket();
	}

	Serial2.write(udp_pkt_buf, len);
	led_state = 1;
	digitalWrite(gl_prefs.led_pin, led_state);
	led_ts = ts;

	for (int i = 0; i < len; i++)
		udp_pkt_buf[i] = 0;
}

static void handle_tcp(uint32_t ts)
{
	// Accept new client
	WiFiClient new_client = tcp_server.available();
	if (new_client)
	{
		if (tcp_client)
			tcp_client.stop();
		tcp_client = new_client;
		tcp_client.setNoDelay(true);
		Serial.printf("TCP client connected: %s\r\n", tcp_client.remoteIP().toString().c_str());
	}

	// Read from TCP client → forward to UART
	if (tcp_client && tcp_client.connected())
	{
		while (tcp_client.available())
		{
			int n = tcp_client.read(udp_pkt_buf, UDP_PKT_BUF_SIZE - 1);
			if (n > 0)
			{
				int cmp = cmd_match((const char *)udp_pkt_buf, "WHO_GOES_THERE");
				if (cmp > 0)
					tcp_client.write((uint8_t *)gl_prefs.name, strlen(gl_prefs.name));
				Serial2.write(udp_pkt_buf, n);
				led_state = 1;
				digitalWrite(gl_prefs.led_pin, led_state);
				led_ts = ts;
			}
		}
	}
	else if (tcp_client)
	{
		tcp_client.stop();
	}
}

static void handle_uart_to_network(uint32_t ts)
{
	if (tcp_client && tcp_client.connected())
	{
		// TCP mode: null-terminated framing
		while (Serial2.available())
		{
			uint8_t new_byte = Serial2.read();
			if (gl_uart_buffer.len >= gl_uart_buffer.size)
			{
				gl_uart_buffer.len = 0;
			}
			gl_pld_buffer[gl_uart_buffer.len++] = new_byte;
			if (new_byte == 0)
			{
				tcp_client.write(gl_uart_buffer.buf, gl_uart_buffer.len);
				gl_uart_buffer.len = 0;
				led_state = 1;
				digitalWrite(gl_prefs.led_pin, led_state);
				led_ts = ts;
			}
		}
	}
	else
	{
		// UDP mode: null-terminated framing
		while (Serial2.available())
		{
			uint8_t new_byte = Serial2.read();
			if (gl_uart_buffer.len >= gl_uart_buffer.size)
			{
				gl_uart_buffer.len = 0;
			}
			gl_pld_buffer[gl_uart_buffer.len++] = new_byte;
			if (new_byte == 0)
			{
				if (gl_prefs.en_fixed_target == 0 && udp.remoteIP() != IPAddress(0, 0, 0, 0))
				{
					udp.beginPacket(udp.remoteIP(), udp.remotePort() + gl_prefs.reply_offset);
				}
				else if (gl_prefs.en_fixed_target == 0 && udp.remoteIP() == IPAddress(0, 0, 0, 0))
				{
					udp.beginPacket(IPAddress(255, 255, 255, 255), gl_prefs.port + gl_prefs.reply_offset);
				}
				else
				{
					IPAddress remote_ip(gl_prefs.remote_target_ip);
					udp.beginPacket(remote_ip, gl_prefs.port + gl_prefs.reply_offset);
				}
				udp.write(gl_uart_buffer.buf, gl_uart_buffer.len);
				udp.endPacket();
				gl_uart_buffer.len = 0;
				led_state = 1;
				digitalWrite(gl_prefs.led_pin, led_state);
				led_ts = ts;
			}
		}
	}
}

// ---------------------------------------------------------------------------

void setup()
{
	pinMode(NRST_PIN, OUTPUT);
	digitalWrite(NRST_PIN, 1);
	pinMode(BOOT_PIN, OUTPUT);
	digitalWrite(BOOT_PIN, 0);
	init_prefs(&preferences, &gl_prefs);
	network_manager_setup(&gl_prefs);
}

void loop()
{
	uint32_t ts = millis();
	handle_i2s_audio();
	handle_udp(ts);
	handle_tcp(ts);
	handle_uart_to_network(ts);
	handle_console_cmds();
	handle_wifi_reconnect(&gl_prefs, ts);
	handle_led_timeout(&gl_prefs, ts);
}
