#include "config.h"
#include "network_manager.h"
#include "console_cmds.h"
#include "nvs.h"

/*
Tested with 3.0.3 esp32 arduino board package by Espressif Systems.
*/

// Hot-path forwarding buffers — local to this file
static uint8_t  gl_pld_mem[PAYLOAD_BUFFER_SIZE] = {0};
static buffer_t gl_uart_buf = 
{
	.buf  = gl_pld_mem,
	.size = sizeof(gl_pld_mem),
	.len  = 0
};
static uint8_t udp_pkt_buf[UDP_PKT_BUF_SIZE] = {0};

static uint32_t spamts = 0;

// ---------------------------------------------------------------------------
static void handle_spam(uint32_t ts)
{
	if(gl_prefs.spam_target)
	{
		if(ts - spamts > 20)
		{
			spamts = ts;
			udp.beginPacket(udp.remoteIP(), udp.remotePort()+gl_prefs.reply_offset);
			udp.write((uint8_t*)"WAZZUP",6);
			udp.endPacket();          
		}
	}
}

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

// ---------------------------------------------------------------------------

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
				{
					tcp_client.write((uint8_t *)gl_prefs.name, strlen(gl_prefs.name));
				}
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


/*helper function to transmit over tcp*/
void send_buffer_tcp(buffer_t * msg, uint32_t ts)
{
	// TCP mode: null-terminated framing
	tcp_client.write(msg->buf, msg->len);
	led_state = 1;
	digitalWrite(gl_prefs.led_pin, led_state);
	led_ts = ts;
}

/*helper function to transmit serial frames over udp*/
void send_buffer_udp(buffer_t * msg, uint32_t ts)
{
	// UDP mode: null-terminated framing
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
	udp.write(msg->buf, msg->len);
	udp.endPacket();
	
	led_state = 1;
	digitalWrite(gl_prefs.led_pin, led_state);
	led_ts = ts;
}


/*
Serial frames -> uart
*/
static void handle_uart_to_network(buffer_t * msg, uint32_t ts)
{
	if(gl_prefs.serial_frame_encoding_mode == COBS_FRAMING)
	{
		while (Serial2.available())
		{
			uint8_t new_byte = Serial2.read();
			if (msg->len >= msg->size)
			{
				msg->len = 0;
			}
			msg->buf[msg->len++] = new_byte;
			if (new_byte == 0 && msg->len > 1)
			{
				if (tcp_client && tcp_client.connected())
				{
					send_buffer_tcp(msg, ts);
				}
				else
				{
					send_buffer_udp(msg,ts);
				}
				msg->len = 0;
			}
		}
	}
	else
	{
		while (Serial2.available())
		{
			uint8_t new_byte = Serial2.read();
			if (msg->len >= msg->size)
			{
				msg->len = 0;
			}
			msg->buf[msg->len++] = new_byte;
			if(new_byte == 0x7E && msg->len > 1)
			{
				if (tcp_client && tcp_client.connected())
				{
					send_buffer_tcp(msg, ts);
				}
				else
				{
					send_buffer_udp(msg,ts);
				}
				msg->len = 0;
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
	handle_spam(ts);
	handle_udp(ts);
	handle_tcp(ts);
	handle_uart_to_network(&gl_uart_buf, ts);
	handle_console_cmds();
	handle_wifi_reconnect(&gl_prefs, ts);
	handle_led_timeout(&gl_prefs, ts);
}
