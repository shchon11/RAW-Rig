"""GigE Vision 카메라의 GenICam XML 을 SDK 없이 받아 저장한다 (랩: LUCID TRI054S 설정 탭용).

  python3 gvcp_xml.py <카메라 IP> [--out DIR]

Spinnaker 는 처음 붙을 때 ~/.config/spinnaker/xml 에 XML 을 캐시해 두지만, Arena 는 그런 캐시가 없다. 그래서
부트스트랩 레지스터의 First URL(0x0200, "Local:<이름>;<주소>;<길이>")을 읽고 그 메모리를 READMEM 으로 받는다.
읽기만 한다 — 다른 프로그램이 카메라를 쥐고 스트리밍 중이어도 된다.
"""
import argparse
import random
import socket
import struct
import sys
from pathlib import Path

PORT = 3956
READMEM_CMD, READMEM_ACK = 0x0084, 0x0085
CHUNK = 512


def readmem(sock, ip, address, count, req_id):
    packet = struct.pack(">BBHHH", 0x42, 0x01, READMEM_CMD, 8, req_id) + struct.pack(">IHH", address, 0, count)
    for _ in range(5):
        sock.sendto(packet, (ip, PORT))
        try:
            while True:
                data, _ = sock.recvfrom(2048)
                status, ack, length, ack_id = struct.unpack(">HHHH", data[:8])
                if ack != READMEM_ACK or ack_id != req_id:
                    continue
                if status != 0:
                    raise RuntimeError(f"READMEM 0x{address:08x} status 0x{status:04x}")
                return data[12:12 + length - 4]
        except socket.timeout:
            continue
    raise TimeoutError(f"READMEM 0x{address:08x}: no answer from {ip}")


def fetch(ip):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))
    sock.settimeout(0.5)
    req = random.randint(1, 0x7FFF)
    url = readmem(sock, ip, 0x0200, 512, req).split(b"\0", 1)[0].decode(errors="replace")
    if not url.lower().startswith("local:"):
        raise RuntimeError(f"XML 이 카메라 메모리에 없다 (First URL = {url!r})")
    name, address, length = url[6:].split(";")[:3]
    address, length = int(address, 16), int(length, 16)
    blob = bytearray()
    while len(blob) < length:
        req = req % 0x7FFF + 1
        count = min(CHUNK, length - len(blob))
        count += (-count) % 4                     # READMEM 은 4바이트 단위
        blob += readmem(sock, ip, address + len(blob), count, req)
    return name, bytes(blob[:length])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip")
    ap.add_argument("--out", default=str(Path.home() / ".config" / "dm_clip_gui_raw" / "genicam"))
    a = ap.parse_args()
    name, blob = fetch(a.ip)
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    path = out / name
    path.write_bytes(blob)
    print(f"{path} ({len(blob)} bytes)")


if __name__ == "__main__":
    sys.exit(main())
