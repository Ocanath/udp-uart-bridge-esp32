#include "parse_console.h"

static unsigned char gl_console_cmd_buf[BUFFER_SIZE] = {0};
console_cmd_t gl_console_cmd = {
    .buf    = gl_console_cmd_buf,
    .size   = sizeof(gl_console_cmd_buf),
    .len    = 0,
    .parsed = 1
};

void get_console_lines(void)
{
    if (gl_console_cmd.parsed == 0)
        return;  // previous command not yet handled, don't overwrite

    int rv = Serial.read();
    if (rv == -1)
        return;

    unsigned char rchar = (unsigned char)rv;
    Serial.write(rchar);

    if (gl_console_cmd.len < gl_console_cmd.size - 1)
        gl_console_cmd.buf[gl_console_cmd.len++] = rchar;

    if (rchar == '\r' || rchar == '\n')
    {
        gl_console_cmd.parsed = 0;
        Serial.printf("\n");
    }
}
