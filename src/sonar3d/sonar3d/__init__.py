"""sonar3d package.

Avoid importing protobuf modules at package import time to prevent
descriptor pool collisions when using the wlsonar decoder.
"""

__all__ = []