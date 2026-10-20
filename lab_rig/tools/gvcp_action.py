"""GigE Vision 액션 커맨드 — 브로드캐스트 패킷 하나로 여러 카메라를 같은 순간에 트리거한다.

카메라 쪽 준비는 드라이버 파라미터 trigger.mode=action (TriggerSource Action0, 아래 키와 같은 값).
PTP 없이 즉시 실행(예약 시각 없음)이라, 두 카메라가 같은 패킷을 받는 시각 차이(스위치 복제, µs 단위)만큼만 어긋난다.

    from gvcp_action import fire
    t_ns, acks = fire("enp5s0")          # acks: [(카메라 IP, status)], status 0 = 받아들임
"""
import random
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path.home() / "jsh/DM_clipGUI/scripts"))
import net_tools  # noqa: E402

DEVICE_KEY = 0x4A53     # 드라이버 trigger.action.* 기본값과 같다
GROUP_KEY = 1
GROUP_MASK = 1
ACTION_CMD, ACTION_ACK = 0x0100, 0x0101


def fire(iface, device_key=DEVICE_KEY, group_key=GROUP_KEY, group_mask=GROUP_MASK, ack=True, timeout=0.25,
         expect=None, unicast=None):
    """액션 커맨드 한 번. (보낸 시각 time.time_ns(), [(IP, status), ...]).

    ack=True 면 카메라가 받아들였다는 ACTION_ACK 를 timeout 동안 (또는 expect 개가 올 때까지) 모은다.
    키가 안 맞는 카메라는 조용히 무시한다 — ACK 가 없으면 그 카메라는 안 찍힌 것이다.
    unicast: 카메라 IP 목록을 주면 브로드캐스트 대신 그 카메라들에만 보낸다 (다른 카메라에 패킷이 안 가게).
    """
    sock, targets = net_tools._gvcp_socket(iface, timeout)
    if sock is None:
        raise OSError(f"{iface} 에 IPv4 주소가 없다")
    if unicast:
        targets = list(unicast)
    req_id = random.randint(1, 0xFFFF)
    payload = struct.pack(">III", device_key, group_key, group_mask)
    packet = struct.pack(">BBHHH", 0x42, 0x01 if ack else 0x00, ACTION_CMD, len(payload), req_id) + payload
    acks = []
    with sock:
        t_ns = time.time_ns()
        for target in targets:
            sock.sendto(packet, (target, net_tools.GVCP_PORT))
        if ack:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and (expect is None or len(acks) < expect):
                sock.settimeout(max(0.001, deadline - time.monotonic()))
                try:
                    data, (ip, _port) = sock.recvfrom(64)
                except socket.timeout:
                    break
                if len(data) >= 8:
                    status, answer, _length, ack_id = struct.unpack(">HHHH", data[:8])
                    if answer == ACTION_ACK and ack_id == req_id:
                        acks.append((ip, status))
    return t_ns, acks


if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else "enp5s0"
    print(fire(iface))
