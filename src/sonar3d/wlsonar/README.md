# wlsonar

Package wlsonar is a Python client library for the Water Linked [Sonar 3D-15](https://www.waterlinked.com/shop/wl-21045-2-sonar-3d-15-689).

The key features of this package are:

- `wlsonar.Sonar3D` for configuration and inspection of system state.
- `wlsonar.range_image_protocol` for [Range Image Protocol](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-api/#range-image-protocol-rip2) packets.

## Installation 

The wlsonar package is hosted on pypi and can be installed with pip:

```bash
pip install wlsonar
```

## Quickstart

Following is a snippet showing how to connect to the sonar and receive images.

```python
import wlsonar
import wlsonar.range_image_protocol as rip

# (set to your sonar's IP address
ip = "10.1.2.139"

# connect, enable acoustics, configure to send images over UDP multicast
sonar = wlsonar.Sonar3D(ip)
sonar.set_acoustics_enabled(True)
sonar.set_udp_multicast()

print("Sonar configured, listening for UDP packets...")

# receive UDP packets, parse them into protobuf, and extract voxels
sock = wlsonar.open_sonar_udp_multicast_socket()
try:
    while True:
        packet, addr = sock.recvfrom(wlsonar.UDP_MAX_DATAGRAM_SIZE)
        try:
            msg = rip.unpackb(packet)
        except rip.UnknownProtobufTypeError:
            # silently skip unknown packet types
            continue
        if isinstance(msg, rip.RangeImage):
            xyz = wlsonar.range_image_to_xyz(msg)
            id = msg.header.sequence_id
            print(f"Got range image {id} with {len(xyz)} voxels")
finally:
    sock.close()
```

More elaborate examples can be found in [the examples folder](./examples/).

## Documentation and resources

Documentation for this package is provided in the form of:

- Elaborate examples in [the examples folder](./examples/).
- Tests in [the tests folder](./tests).
- Docstrings in code.

For general documentation about the Sonar 3D-15 see: https://docs.waterlinked.com/sonar-3d/sonar-3d-15/. The integration API that this package interfaces with is documented here: https://docs.waterlinked.com/sonar-3d/sonar-3d-15-api/. See also the replayer: https://sonar.replay.waterlinked.com/.

## Development and testing

[README_dev.md](./README_dev.md) documents development and testing of this package.