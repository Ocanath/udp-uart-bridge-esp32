#include <WebServer.h>
#include "html_console.h"
#include "console_cmds.h"
#include "parse_console.h"
#include <string.h>

static WebServer http_server(80);

static const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html>
<head><title>Console</title>
<style>
  body{font-family:monospace;background:#111;color:#0f0;margin:8px}
  #log{height:80vh;overflow-y:auto;border:1px solid #0f0;padding:4px;white-space:pre-wrap}
  #inp{width:calc(100% - 60px);background:#222;color:#0f0;border:1px solid #0f0;padding:4px}
  button{background:#222;color:#0f0;border:1px solid #0f0;padding:4px 8px;cursor:pointer}
</style></head>
<body>
<div id="log"></div>
<div style="margin-top:4px">
  <input id="inp" type="text" autofocus>
  <button onclick="send()">Send</button>
</div>
<script>
var log=document.getElementById('log');
var inp=document.getElementById('inp');
inp.onkeydown=function(e){if(e.key==='Enter')send();};
function send(){
  var cmd=inp.value.trim();
  if(!cmd)return;
  inp.value='';
  log.textContent+="> "+cmd+"\n";
  fetch('/cmd',{method:'POST',body:cmd})
    .then(function(r){return r.text();})
    .then(function(t){log.textContent+=t+"\n";log.scrollTop=log.scrollHeight;})
    .catch(function(e){log.textContent+="[error: "+e+"]\n";});
}
</script>
</body></html>
)rawliteral";

static void handle_root(void)
{
    http_server.send_P(200, "text/html", HTML_PAGE);
}

static void send_chunk(const char *data, size_t len)
{
    http_server.sendContent(data, len);
}

static void handle_cmd(void)
{
    if (!http_server.hasArg("plain"))
    {
        http_server.send(400, "text/plain", "no body");
        return;
    }
    String body = http_server.arg("plain");

    // Load command into shared input buffer
    memset(gl_console_cmd.buf, 0, gl_console_cmd.size);
    size_t len = body.length();
    if (len > gl_console_cmd.size - 2) len = gl_console_cmd.size - 2;
    memcpy(gl_console_cmd.buf, body.c_str(), len);
    gl_console_cmd.buf[len] = '\r';   // match serial CR convention
    gl_console_cmd.len    = len + 1;
    gl_console_cmd.parsed = 0;

    console_set_http_chunk_cb(send_chunk);
    http_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    http_server.send(200, "text/plain", "");
    handle_console_cmds(&gl_console_cmd, CONSOLE_IFACE_HTTP);
    http_server.sendContent("");       // end chunked response
    console_set_http_chunk_cb(NULL);
}

void html_console_setup(void)
{
    http_server.on("/",    HTTP_GET,  handle_root);
    http_server.on("/cmd", HTTP_POST, handle_cmd);
    http_server.begin();
}

void html_console_handle(void)
{
    http_server.handleClient();
}
