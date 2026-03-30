#pragma once
#include "parse_console.h"

// Interface selector — passed to handle_console_cmds and reply_over_interface
typedef enum {
    CONSOLE_IFACE_SERIAL = 0,
    CONSOLE_IFACE_HTTP   = 1,
} console_iface_t;

int  cmd_match(const char *in, const char *cmd);

// Callback type for streaming HTTP chunks — registered by html_console.cpp
typedef void (*console_chunk_cb_t)(const char *data, size_t len);
void console_set_http_chunk_cb(console_chunk_cb_t cb);

// Variadic output router — drops in where Serial.printf was called
void reply_over_interface(console_iface_t iface, const char *fmt, ...);

// Core handler — iface selects where output goes
void handle_console_cmds(console_cmd_t *input, console_iface_t iface);

// Serial-path convenience wrapper (replaces old handle_console_cmds() call in loop)
void handle_console_cmds_serial(void);
