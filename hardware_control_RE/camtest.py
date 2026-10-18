#!/usr/bin/env python3
"""Low-latency GOAT camera preview: NV12 shm -> downsampled YUV420 -> browser canvas."""

import argparse, mmap, os, socket, struct, sys, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CTRL_SIZE = 0xB0        # mutex, index, counter, 2 rwlocks
HDR_SIZE  = 0x30        # per-frame header
CTL_SOCK  = '/tmp/camerasockt.domain'
CLIENT    = 'camstream'


def camera(on):
    """stream_on/off via cameraProvider's control socket (what sitctl does)."""
    msg = 'camera0 {"stream_on":%d,"name":"%s","type":"socket_default"}' % (int(on), CLIENT)
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(3)
        s.connect(CTL_SOCK)
        s.sendall(msg.encode())
        r = s.recv(4096).decode('utf-8', 'replace').strip()
        s.close()
        return r
    except OSError as e:
        return 'control socket failed: %s' % e


class Camera:
    """One reader per connection: each keeps its own last_ts, so no cross-client races."""

    def __init__(self, path, scale):
        self.fd = os.open(path, os.O_RDONLY)
        self.size = os.fstat(self.fd).st_size
        self.dsize = (self.size - CTRL_SIZE) // 2
        self.m = mmap.mmap(self.fd, self.size, prot=mmap.PROT_READ)
        self.scale = scale
        self.last_ts = 0

    def close(self):
        self.m.close()
        os.close(self.fd)

    def grab(self):
        """Newest unseen frame as (ow, oh, planar YUV420 bytes), else None.

        Slices straight out of the mmap with a C-level step, so there is no
        3 MB copy and no per-pixel Python.
        """
        m, s = self.m, self.scale
        idx, cnt = struct.unpack_from('<ii', m, 0x30)
        base = CTRL_SIZE + idx * self.dsize
        ts, sz, w, h = struct.unpack_from('<8xq16xIII', m, base)
        if ts in (0, self.last_ts) or sz == 0 or sz > self.dsize - HDR_SIZE:
            return None

        ow, oh = (w // s) & ~1, (h // s) & ~1
        y0 = base + HDR_SIZE
        uv0 = y0 + w * h
        out = []
        for r in range(oh):                       # Y: every s'th row and column
            o = y0 + r * s * w
            out.append(m[o:o + w:s][:ow])
        for off in (0, 1):                        # U then V, de-interleaved
            for r in range(oh // 2):
                o = uv0 + r * s * w
                out.append(m[o + off:o + w:2 * s][:ow // 2])

        # The writer always fills the *other* buffer, so one publish mid-read is
        # harmless; two means ours was reused and the frame may be torn.
        _, cnt2 = struct.unpack_from('<ii', m, 0x30)
        if cnt2 - cnt > 1:
            return None
        self.last_ts = ts
        return ow, oh, b''.join(out)


PAGE = b"""<!doctype html><meta charset=utf-8><title>cam</title>
<style>html,body{margin:0;background:#000}canvas{width:100vw;height:100vh;object-fit:contain}</style>
<canvas id=c></canvas>
<script>
const c=document.getElementById('c'),g=c.getContext('2d');let img=null;
function draw(w,h,d){
  if(!img||c.width!=w){c.width=w;c.height=h;img=g.createImageData(w,h);}
  const p=img.data,cw=w>>1,us=w*h,vs=us+cw*(h>>1);
  for(let y=0;y<h;y++)for(let x=0;x<w;x++){
    const Y=1.164*(d[y*w+x]-16),i=(y>>1)*cw+(x>>1),
          U=d[us+i]-128,V=d[vs+i]-128,o=(y*w+x)<<2;
    p[o]=Y+1.596*V;p[o+1]=Y-0.813*V-0.391*U;p[o+2]=Y+2.018*U;p[o+3]=255;}
  g.putImageData(img,0,0);
}
fetch('s').then(r=>{
  const rd=r.body.getReader();let buf=new Uint8Array(0);
  const pump=({done,value})=>{
    if(done)return;
    const n=new Uint8Array(buf.length+value.length);n.set(buf);n.set(value,buf.length);buf=n;
    for(;;){
      if(buf.length<12)break;
      const dv=new DataView(buf.buffer,buf.byteOffset,12),
            w=dv.getUint32(0),h=dv.getUint32(4),len=dv.getUint32(8);
      if(buf.length<12+len)break;
      draw(w,h,buf.subarray(12,12+len));
      buf=buf.slice(12+len);
    }
    return rd.read().then(pump);
  };
  return rd.read().then(pump);
});
</script>"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith('/s'):
            return self.stream()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(PAGE)))
        self.end_headers()
        self.wfile.write(PAGE)

    def stream(self):
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')   # no length: stream till disconnect
        self.end_headers()
        self.close_connection = True
        cam = Camera(self.server.shm, self.server.scale)
        try:
            while True:
                f = cam.grab()
                if f is None:
                    time.sleep(0.005)             # source is 10 fps; poll well under that
                    continue
                ow, oh, data = f
                self.wfile.write(struct.pack('>III', ow, oh, len(data)) + data)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            cam.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--scale', type=int, default=4, help='downsample factor (4 -> 480x270)')
    ap.add_argument('--shm', default='/dev/shm/shm_fisheye2')
    ap.add_argument('--no-stream-on', action='store_true')
    a = ap.parse_args()

    if not a.no_stream_on:
        print('stream_on:', camera(True))
        time.sleep(0.5)

    srv = ThreadingHTTPServer(('0.0.0.0', a.port), Handler)
    srv.daemon_threads = True
    srv.shm, srv.scale = a.shm, a.scale
    print('http://%s:%d/  (scale %d)' % (socket.gethostbyname(socket.gethostname()), a.port, a.scale))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if not a.no_stream_on:
            print('\nstream_off:', camera(False))


if __name__ == '__main__':
    main()
