"""Helper to open UDP socket on the Sonar 3D-15."""

from __future__ import annotations

import socket
import struct

DEFAULT_MCAST_GRP = "224.0.0.96"
DEFAULT_MCAST_PORT = 4747


def open_sonar_udp_unicast_socket(
    *,
    udp_port: int,
    iface_ip: str = "0.0.0.0",
) -> socket.socket:
    """Helper function to open a UDP socket for listening to unicast packets from the Sonar 3D-15.

    It is the callers responsibility to close the socket.

    Args:
        udp_port: UDP port to listen on. Must match configuration on the sonar.
        iface_ip: IP to bind to.

    Returns:
        A socket.socket object configured to receive UDP packets from the Sonar 3D-15.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((iface_ip, int(udp_port)))

    return sock


def open_sonar_udp_multicast_socket(
    *,
    mcast_group: str = DEFAULT_MCAST_GRP,
    udp_port: int = DEFAULT_MCAST_PORT,
    iface_ip: str = "0.0.0.0",
) -> socket.socket:
    """Helper function to open a UDP socket for listening to multicast packets from the Sonar 3D-15.

    It is the callers responsibility to close the socket.

    Args:
        mcast_group: multicast address to join. You should not have to change this.
        udp_port: UDP port to listen on. Default matches default Sonar configuration.
        iface_ip: interface IP for multicast membership.

    Returns:
        A socket.socket object configured to receive UDP packets from the Sonar 3D-15.
    """
    sock = open_sonar_udp_unicast_socket(udp_port=udp_port, iface_ip=iface_ip)

    # Join multicast group on selected interface.
    mreq = struct.pack("=4s4s", socket.inet_aton(mcast_group), socket.inet_aton(iface_ip))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    return sock
