#ifndef PARSE_CONSOLE_H
#define PARSE_CONSOLE_H

#include <Arduino.h>
#include <stdint.h>
#define BUFFER_SIZE 128

#define CHAR_BUF_SIZE 256

typedef struct console_cmd_t
{
  uint8_t buf[BUFFER_SIZE];
  int len;

  uint8_t parsed; //flag for indicating whether the command has been handled
}console_cmd_t;

extern console_cmd_t gl_console_cmd;

void get_console_lines(void);

#endif
