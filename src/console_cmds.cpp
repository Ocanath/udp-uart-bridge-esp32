#include "console_cmds.h"
#include "parse_console.h"
#include "nvs.h"
#include "network_manager.h"
#include <WiFi.h>
#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

// ---------------------------------------------------------------------------
// HTTP chunk callback — registered by html_console.cpp before each command
// ---------------------------------------------------------------------------
static console_chunk_cb_t http_chunk_cb = NULL;

void console_set_http_chunk_cb(console_chunk_cb_t cb)
{
    http_chunk_cb = cb;
}

// ---------------------------------------------------------------------------
// Variadic output router
// ---------------------------------------------------------------------------
void reply_over_interface(console_iface_t iface, const char *fmt, ...)
{
    char buf[BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n <= 0)
	{
		 return;
	}
    if (n >= (int)sizeof(buf)) 	//this check is >= because of the snprintf null terminator
	{
		n = sizeof(buf) - 1;	//make room for the null terminator
	}

    if (iface == CONSOLE_IFACE_SERIAL)
    {
        Serial.write(buf, n);
    }
    else if (iface == CONSOLE_IFACE_HTTP && http_chunk_cb != NULL)
    {
        http_chunk_cb(buf, (size_t)n);
    }
}

// ---------------------------------------------------------------------------

int cmd_match(const char *in, const char *cmd)
{
    int i = 0;
    for (i = 0; cmd[i] != '\0'; i++)
    {
        if (in[i] == '\0') return -1;
        if (in[i] != cmd[i]) return -1;
    }
    return i;
}

// ---------------------------------------------------------------------------

void handle_console_cmds(console_cmd_t *input, console_iface_t iface)
{
    if (input->parsed != 0)
        return;

    uint8_t match = 0;
    uint8_t save  = 0;
    int cmp = -1;

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setstaticip ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        ip_from_cmd_arg(arg, gl_prefs.static_ip);
        reply_over_interface(iface, "Setting static ip to %s\r\n", gl_prefs.static_ip.toString().c_str());
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setsubnetmask ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        ip_from_cmd_arg(arg, gl_prefs.subnet);
        reply_over_interface(iface, "Setting subnet mask to %s\r\n", gl_prefs.subnet.toString().c_str());
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setgateway ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        ip_from_cmd_arg(arg, gl_prefs.gateway);
        reply_over_interface(iface, "Setting gateway to %s\r\n", gl_prefs.gateway.toString().c_str());
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setdns ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        ip_from_cmd_arg(arg, gl_prefs.dns);
        reply_over_interface(iface, "Setting dns to %s\r\n", gl_prefs.dns.toString().c_str());
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "readstaticipsettings");
    if (cmp > 0)
    {
        match = 1;
        reply_over_interface(iface, "Desired static ip %s\r\n", gl_prefs.static_ip.toString().c_str());
        reply_over_interface(iface, "Subnet mask %s\r\n",       gl_prefs.subnet.toString().c_str());
        reply_over_interface(iface, "Gateway is %s\r\n",        gl_prefs.gateway.toString().c_str());
        reply_over_interface(iface, "DNS is %s\r\n",            gl_prefs.dns.toString().c_str());
    }

    //////////////////////////////////////////////////////////////////////
    cmp = strcmp((const char *)input->buf, "ipconfig\r");
    if (cmp == 0)
    {
        match = 1;
        if (WiFi.status() == WL_CONNECTED)
            reply_over_interface(iface, "Connected to: %s\r\n", gl_prefs.ssid);
        else
            reply_over_interface(iface, "Not connected to: %s\r\n", gl_prefs.ssid);
        reply_over_interface(iface, "UDP server on port: %d\r\n", gl_prefs.port);
        reply_over_interface(iface, "TCP server on port: %d\r\n", gl_prefs.port);
        if (tcp_client && tcp_client.connected())
            reply_over_interface(iface, "TCP client connected: %s\r\n", tcp_client.remoteIP().toString().c_str());
        else
            reply_over_interface(iface, "No TCP client connected\r\n");
        reply_over_interface(iface, "Server Response Offset: %d\r\n", gl_prefs.reply_offset);
        reply_over_interface(iface, "IP address: %s\r\n", WiFi.localIP().toString().c_str());
        if (gl_prefs.static_ip != IPAddress((uint32_t)0))
        {
            reply_over_interface(iface, "Requested Static IP: %s\r\n", gl_prefs.static_ip.toString().c_str());
            reply_over_interface(iface, "Subnet Mask: %s\r\n",         gl_prefs.subnet.toString().c_str());
            reply_over_interface(iface, "Gateway: %s\r\n",             gl_prefs.gateway.toString().c_str());
            reply_over_interface(iface, "DNS: %s\r\n",                 gl_prefs.dns.toString().c_str());
        }
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "udpconfig\r");
    if (cmp > 0)
    {
        match = 1;
        reply_over_interface(iface, "UDP server on port: %d\r\n", gl_prefs.port);
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setssid ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        for (int i = 0; i < WIFI_MAX_SSID_LEN; i++)
            gl_prefs.ssid[i] = '\0';
        for (int i = 0; arg[i] != '\0'; i++)
        {
            if (arg[i] != '\r' && arg[i] != '\n')
                gl_prefs.ssid[i] = arg[i];
        }
        reply_over_interface(iface, "Changing ssid to: %s\r\n", gl_prefs.ssid);
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setname ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        for (int i = 0; i < DEVICE_NAME_LEN; i++)
        {
            gl_prefs.name[i] = '\0';
        }
        for (int i = 0; arg[i] != '\0'; i++)
        {
            if (arg[i] != '\r' && arg[i] != '\n')
                gl_prefs.name[i] = arg[i];
        }
        reply_over_interface(iface, "Changing device name to: %s\r\n", gl_prefs.name);
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "settargetip ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        char copy[15] = {0};
        reply_over_interface(iface, "Raw str arg: ");
        for (int i = 0; arg[i] != 0 && arg[i] != '\r' && arg[i] != '\n'; i++)
        {
            copy[i] = arg[i];
            reply_over_interface(iface, "%0.2X", arg[i]);
        }
        reply_over_interface(iface, ": %s\r\n", copy);
        IPAddress addr;
        if (addr.fromString((const char *)copy) == true)
            reply_over_interface(iface, "Parsed IP address successfully\r\n");
        else
            reply_over_interface(iface, "Invalid IP string entered\r\n");
        gl_prefs.remote_target_ip = (uint32_t)addr;
        reply_over_interface(iface, "%X\r\n", gl_prefs.remote_target_ip);
        IPAddress parseconfirm(gl_prefs.remote_target_ip);
        reply_over_interface(iface, "Target IP: %s\r\n", parseconfirm.toString().c_str());
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "fixedtarget ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        int argcmp = cmd_match(arg, "enable");
        if (argcmp > 0)
        {
            gl_prefs.en_fixed_target = 1;
            IPAddress parseconfirm(gl_prefs.remote_target_ip);
            reply_over_interface(iface, "Enabling Fixed Target: %s\r\n", parseconfirm.toString().c_str());
        }
        argcmp = cmd_match(arg, "disable");
        if (argcmp > 0)
        {
            gl_prefs.en_fixed_target = 0;
            reply_over_interface(iface, "Disabling Fixed Target\r\n");
        }
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setpwd ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        for (int i = 0; i < WIFI_MAX_PWD_LEN; i++)
            gl_prefs.password[i] = '\0';
        for (int i = 0; arg[i] != '\0'; i++)
        {
            if (arg[i] != '\r' && arg[i] != '\n')
                gl_prefs.password[i] = arg[i];
        }
        reply_over_interface(iface, "Changing pwd to: %s\r\n", gl_prefs.password);
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setport ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        char *tmp;
        int port = strtol(arg, &tmp, 10);
        reply_over_interface(iface, "Changing port to: %d\r\n", port);
        gl_prefs.port = port;
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setaudio ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        char *tmp;
        int setting = (strtol(arg, &tmp, 10)) & 1;
        gl_prefs.use_i2s = setting;
        reply_over_interface(iface, "Using audio: %d\r\n", setting);
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "readaudio");
    if (cmp > 0)
    {
        match = 1;
        reply_over_interface(iface, gl_prefs.use_i2s ? "I2S Mic Enabled\r\n" : "I2S Mic Disabled\r\n");
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setledpin ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        char *tmp;
        uint8_t pin = (uint8_t)(strtol(arg, &tmp, 10));
        reply_over_interface(iface, "Changing led pin to: %d\r\n", pin);
        gl_prefs.led_pin = pin;
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "readledpin");
    if (cmp > 0)
    {
        match = 1;
        reply_over_interface(iface, "LED Pin is %d\r\n", gl_prefs.led_pin);
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setbaudrate ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        char *tmp;
        unsigned long baudrate = strtol(arg, &tmp, 10);
        if (baudrate > 2000000) baudrate = 2000000;
        if (baudrate < 9600)    baudrate = 9600;
        reply_over_interface(iface, "Changing baudrate to: %d\r\n", baudrate);
        gl_prefs.baudrate = baudrate;
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "readbaudrate");
    if (cmp > 0)
    {
        match = 1;
        reply_over_interface(iface, "Baudrate is: %d\r\n", gl_prefs.baudrate);
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setTXoff ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        char *tmp;
        int offset = strtol(arg, &tmp, 10);
        reply_over_interface(iface, "Setting port offset to: %d\r\n", offset);
        gl_prefs.reply_offset = offset;
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setencoding ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        char *tmp;
        int mode = strtol(arg, &tmp, 10);
        if (mode >= 0 && mode <= 1)
        {
            if (mode == COBS_FRAMING)
                reply_over_interface(iface, "Setting mode to COBS\n");
            else
                reply_over_interface(iface, "Setting mode to PPP\n");
            gl_prefs.serial_frame_encoding_mode = mode;
        }
        else
        {
            reply_over_interface(iface, "Invalid serial encoding mode selection\n");
        }
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "readencoding");
    if (cmp > 0)
    {
        match = 1;
        if (gl_prefs.serial_frame_encoding_mode == COBS_FRAMING)
            reply_over_interface(iface, "Using mode: COBS\n");
        else
            reply_over_interface(iface, "Using mode: PPP\n");
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "setspam ");
    if (cmp > 0)
    {
        match = 1;
        const char *arg = (const char *)(&input->buf[cmp]);
        char *tmp;
        int mode = strtol(arg, &tmp, 10);
        if (mode >= 0 && mode <= 1)
        {
            if (mode == 0)
                reply_over_interface(iface, "Disabling spam\n");
            else
                reply_over_interface(iface, "Spam mode ON\n");
            gl_prefs.spam_target = mode;
        }
        else
        {
            reply_over_interface(iface, "Invalid spam setting\n");
        }
        save = 1;
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "readspam");
    if (cmp > 0)
    {
        match = 1;
        if (gl_prefs.spam_target == 0)
            reply_over_interface(iface, "Spam disabled\n");
        else
            reply_over_interface(iface, "Spam enabled\n");
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "readcred");
    if (cmp > 0)
    {
        match = 1;
        reply_over_interface(iface, "SSID: '");
        for (int i = 0; gl_prefs.ssid[i] != 0; i++)
        {
            char c = gl_prefs.ssid[i];
            if (c >= 0x1f && c <= 0x7E) reply_over_interface(iface, "%c", c);
            else                         reply_over_interface(iface, "%0.2X", c);
        }
        reply_over_interface(iface, "'\r\n");
        reply_over_interface(iface, "Password: '");
        for (int i = 0; gl_prefs.password[i] != 0; i++)
        {
            char c = gl_prefs.password[i];
            if (c >= 0x1f && c <= 0x7E) reply_over_interface(iface, "%c", c);
            else                         reply_over_interface(iface, "%0.2X", c);
        }
        reply_over_interface(iface, "'\r\n");
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "reconnect\r");
    if (cmp > 0)
    {
        match = 1;
        reply_over_interface(iface, "restarting wifi connection...\r\n");
        WiFi.disconnect();
        WiFi.begin((const char *)gl_prefs.ssid, (const char *)gl_prefs.password);
        network_begin_udp_tcp(&gl_prefs);
    }

    //////////////////////////////////////////////////////////////////////
    cmp = cmd_match((const char *)input->buf, "restart\r");
    if (cmp > 0)
    {
        reply_over_interface(iface, "restarting chip...\r\n");
        ESP.restart();
    }

    /****************************** Cleanup ******************************/
    if (match == 0)
        reply_over_interface(iface, "Failed to parse: %s\r\n", input->buf);

    if (save != 0)
    {
        int nb = preferences.putBytes("settings", &gl_prefs, sizeof(nvs_settings_t));
        reply_over_interface(iface, "Saved %d bytes\r\n", nb);
    }

    for (size_t i = 0; i < input->size; i++)
        input->buf[i] = 0;
    input->len = 0;
    input->parsed = 1;
}

// ---------------------------------------------------------------------------

void handle_console_cmds_serial(void)
{
    get_console_lines();
    handle_console_cmds(&gl_console_cmd, CONSOLE_IFACE_SERIAL);
}
