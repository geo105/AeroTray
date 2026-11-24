import binascii
import io
from pathlib import Path
import random
import sys
import tempfile
from types import SimpleNamespace
import unittest
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"herramientas"))
from ensayo_hover.protocolo import Parser, FRAME, FRAME_V2, FIELDS, decode, row
from ensayo_hover.consola import Console, Recorder


def packet(seq=0, mode=0, flags=13, version=1):
    schema = FRAME if version==1 else FRAME_V2
    v=list(schema.unpack(bytes(schema.size)))
    v[:10]=[0x484d5344,schema.size,version,seq,seq*5,seq*5000,mode,flags,1300,.2]
    v[18]=16384
    v[22:24]=[1,1]
    v[24:32]=[1000]*8
    v[36:39]=[5000,2200,20]
    if version==2:
        v[43:48]=[1.06,1.05,0,0,2]
    data=schema.pack(*v)
    v[-1]=binascii.crc_hqx(data[:-2],0xffff)
    return schema.pack(*v)


class FakeSerial:
    def __init__(self):
        self.pending=b""
        self.writes=[]
    def read(self,n):
        data,self.pending=self.pending,b""
        return data
    def write(self,data):
        self.writes.append(data)
        return len(data)
    def close(self):
        pass


class ProtocolTests(unittest.TestCase):
    def test_schema(self):
        self.assertEqual(FRAME.size,128)
        self.assertEqual(len(FIELDS),len(row(decode(packet()),1.)))
        self.assertEqual(FRAME_V2.size,144)
        self.assertEqual(len(FIELDS),len(row(decode(packet(version=2)),1.)))

    def test_mixed_v1_v2_and_corruption(self):
        data=b"# init\n"+packet(0)+packet(1,version=2)+packet(2,version=2)+packet(3)
        for cut in range(len(data)+1):
            p=Parser();out=p.feed(data[:cut])+p.feed(data[cut:])
            self.assertEqual([v[3] for k,v in out if k=="frame"],[0,1,2,3])
            self.assertEqual(p.errors,0)
        bad=bytearray(packet(1,version=2));bad[130]^=1
        p=Parser();out=p.feed(bytes(bad)+packet(2,version=2))
        self.assertEqual([v[3] for k,v in out if k=="frame"],[2])
        self.assertEqual(p.errors,1)

    def test_every_split_and_single_byte(self):
        data=b"# hello\n"+packet()+packet(1)+b"# bye\n"
        for cut in range(len(data)+1):
            p=Parser()
            out=p.feed(data[:cut])+p.feed(data[cut:])
            self.assertEqual([v[3] for k,v in out if k=="frame"],[0,1])
            self.assertEqual(p.errors,0)
        p=Parser(); out=[]
        for v in data:
            out+=p.feed(bytes([v]))
        self.assertEqual([v[3] for k,v in out if k=="frame"],[0,1])

    def test_corruption_resync(self):
        bad=bytearray(packet(2)); bad[55]^=0xff
        p=Parser()
        stream=packet(1)+bad+packet(3)+b"# text\nDSMH\x00\x00\x00\x00junk"+packet(4)
        rng=random.Random(3); out=[]
        while stream:
            n=rng.randrange(1,67); out+=p.feed(stream[:n]); stream=stream[n:]
        self.assertEqual([v[3] for k,v in out if k=="frame"],[1,3,4])
        self.assertGreaterEqual(p.errors,2)

    def test_console_no_auto_start_and_stop_without_enter(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec=Recorder(Path(tmp)/"s","fake")
            ser=FakeSerial()
            c=Console(SimpleNamespace(rviz=False),ser,rec)
            c.command("armar") # sin identidad del firmware, no se envia
            self.assertEqual(ser.writes,[])
            ser.pending=packet()
            c.receive()
            self.assertEqual(ser.writes,[b"!\n"]) # conexion solicita parada; nunca arranca ni calibra
            c.command("estado")
            self.assertEqual(ser.writes[-1],b"estado\n")
            with patch("ensayo_hover.consola.select.select",return_value=([sys.stdin],[],[])), patch("ensayo_hover.consola.os.read",return_value=b"!"):
                c.input()
            self.assertEqual(ser.writes[-1],b"!\n")
            c.finish()
            self.assertEqual((Path(tmp)/"s/serial.bin").read_bytes(),packet())

    def test_sequence_loss_and_fatal_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec=Recorder(Path(tmp)/"s","fake"); ser=FakeSerial()
            c=Console(SimpleNamespace(rviz=False),ser,rec)
            ser.pending=packet(10)+packet(13)
            c.receive(); self.assertEqual(c.gaps,2)
            # Parada no depende del registro de eventos.
            with patch.object(rec,"event",side_effect=OSError("disco lleno")):
                with self.assertRaises(OSError): c.emergency("test")
            self.assertEqual(ser.writes[-1],b"!\n")
            c.finish()

    def test_stale_command_blocked(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec=Recorder(Path(tmp)/"s","fake"); ser=FakeSerial()
            c=Console(SimpleNamespace(rviz=False),ser,rec)
            ser.pending=packet(); c.receive(); c.last_received-=1
            c.command("arrancar")
            self.assertEqual(ser.writes,[b"!\n"])
            c.command("apagar")
            self.assertEqual(ser.writes[-1],b"!\n")
            c.finish()

    def test_boot_invalidates_old_sample_and_start_waits_for_stability(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec=Recorder(Path(tmp)/"s","fake"); ser=FakeSerial()
            c=Console(SimpleNamespace(rviz=False),ser,rec)
            ser.pending=packet(); c.receive()
            c.command("armar")
            self.assertEqual(ser.writes,[b"!\n"])
            c.stable_since-=.6
            c.command("armar")
            self.assertEqual(ser.writes[-1],b"armar\n")
            c.text="arrancar"
            ser.pending=b"# ENSAYO: INICIO DESARMADO\n"
            c.receive()
            self.assertIsNone(c.latest)
            self.assertIsNone(c.stable_since)
            self.assertEqual(c.text,"")
            count=len(ser.writes)
            c.command("arrancar")
            self.assertEqual(len(ser.writes),count)
            c.finish()

    def test_auto_preview_never_arms_and_emergency_cancels(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec=Recorder(Path(tmp)/"s","fake"); ser=FakeSerial()
            c=Console(SimpleNamespace(rviz=False,calibrar_vista=True),ser,rec)
            ser.pending=packet(999); c.receive()
            ser.pending=b"# ENSAYO: INICIO DESARMADO\n"; c.receive()
            self.assertEqual(c.auto_state,0) # reset esperado al abrir; aun no se ha calibrado
            ser.pending=packet(1000,flags=12); c.receive()
            c.stable_since-=1
            c.auto_preview(time.monotonic(),True)
            self.assertEqual(ser.writes[-1],b"cal\n")
            ser.pending=packet(1001,flags=13); c.receive()
            c.auto_preview(time.monotonic(),True)
            self.assertEqual(ser.writes[-1],b"vista\n")
            self.assertNotIn(b"armar\n",ser.writes)
            self.assertNotIn(b"arrancar\n",ser.writes)
            c.auto_state=0
            c.emergency("test")
            count=len(ser.writes)
            c.auto_preview(time.monotonic(),True)
            self.assertEqual(len(ser.writes),count)
            c.finish()


if __name__=="__main__":
    unittest.main()
