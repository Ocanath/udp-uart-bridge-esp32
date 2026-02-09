#include "parse_console.h"

static uint8_t cmd_buf[BUFFER_SIZE] = {0};  //local buffer we fill with characters
static int charbuf_loc = 0;

console_cmd_t gl_console_cmd = {0};

void get_console_lines(void)
{
    int rv = Serial.read();
    if(rv != -1)
    {
      /*Record a list of typed characters*/
      uint8_t rchar = (uint8_t)rv;
      Serial.write(rchar);
      if(charbuf_loc < CHAR_BUF_SIZE-1) //always leave the last element as 0
      {
        cmd_buf[charbuf_loc++] = rchar;
      }
      else
      {
        charbuf_loc = 0;  //wraparound. it's fine to do this because we won't send commands more than buflen
      }

      /*If you get a carriage return, copy through the buffer contents into the double buffer*/
      if(rchar == '\r' || rchar == '\n')
      {
        gl_console_cmd.parsed = 0;
        gl_console_cmd.len = charbuf_loc;
        charbuf_loc = 0;
        for(int i = 0; i < gl_console_cmd.len; i++)
        {
          gl_console_cmd.buf[i] = cmd_buf[i]; 
        }
        gl_console_cmd.len = 0;
        Serial.printf("\n");
        //Serial.printf("%s\r\n",gl_console_cmd.buf);
      }
    }
}



