#pragma once
#include "parse_console.h"

// Interface selector — passed to handle_console_cmds and reply_over_interface
typedef enum {
    CONSOLE_IFACE_SERIAL = 0,
    CONSOLE_IFACE_HTTP   = 1,
} console_iface_t;

int  cmd_match(const char *in, const char *cmd);

// Variadic output router — drops in where Serial.printf was called
void reply_over_interface(console_iface_t iface, const char *fmt, ...);

// HTTP reply buffer accessors (used by html_console.cpp)
void        http_reply_reset(void);
const char *http_reply_get(void);

// Core handler — iface selects where output goes
void handle_console_cmds(console_cmd_t *input, console_iface_t iface);

// Serial-path convenience wrapper (replaces old handle_console_cmds() call in loop)
void handle_console_cmds_serial(void);
