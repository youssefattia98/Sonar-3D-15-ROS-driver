"""Example use of wlsonar module to trim a .sonar recording by timestamps."""

import argparse
import datetime
from typing import Optional

import wlsonar.range_image_protocol as rip


def human_readable_size(size: int) -> str:
    """Human readable size."""
    x = float(size)
    for unit in ["bytes", "KB", "MB", "GB", "TB"]:
        if x < 1024.0:
            return f"{x:.2f} {unit}"
        x /= 1024.0
    return f"{x:.2f} PB"


def parse_time(timestr: str) -> datetime.time:
    """Parse time from string."""
    return (
        datetime.datetime.strptime(timestr, "%H:%M:%S")
        .replace(tzinfo=datetime.datetime.now().astimezone().tzinfo)
        .timetz()
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Example use of wlsonar module: Trim the a .sonar recording to include only packets"
            "between two given timestamps. Assumes that sonar recording does not span multiple"
            "days."
        ),
    )
    parser.add_argument("--input", required=True, type=str, help="Input .sonar file")
    parser.add_argument("--output", required=True, type=str, help="Output .sonar file")
    parser.add_argument(
        "--start", type=str, help="Start in format HH:MM:SS, or leave empty for start of file"
    )
    parser.add_argument(
        "--end", type=str, help="End in format HH:MM:SS, or leave empty for end of file"
    )
    args = parser.parse_args()

    if not args.start and not args.end:
        raise ValueError("At least one of --start or --end must be specified")

    start_time = parse_time(args.start) if args.start else None
    end_time = parse_time(args.end) if args.end else None

    in_start: Optional[datetime.time] = None
    in_end: Optional[datetime.time] = None
    in_size = 0

    out_start: Optional[datetime.time] = None
    out_end: Optional[datetime.time] = None
    out_size = 0

    with open(args.input, "rb") as infile, open(args.output, "wb") as outfile:
        while True:
            packet_start = infile.tell()
            try:
                msg = rip.unpack(infile)
            except rip.UnknownProtobufTypeError:
                continue  # unknown type, ignore
            except EOFError:
                # end of file
                break
            packet_end = infile.tell()

            header: rip.Header = msg.header  # type: ignore
            ts = header.timestamp.ToDatetime(tzinfo=datetime.timezone.utc).astimezone().timetz()

            # remember start and end timestamps of original file
            if in_start is None:
                in_start = ts
            in_end = ts
            in_size += packet_end - packet_start

            # skip packets outside of time range
            if start_time is not None and ts < start_time:
                continue
            if end_time is not None and ts > end_time:
                continue

            # remember new start and end timestamps of trimmed file
            if out_start is None:
                out_start = ts
            out_end = ts
            out_size += packet_end - packet_start

            # rewind infile and write as-is to output file
            infile.seek(packet_start)
            packet_data = infile.read(packet_end - packet_start)
            outfile.write(packet_data)

    if in_start is None or in_end is None:
        raise RuntimeError("No packets found in input file")
    print(f"Created trimmed recording '{args.output}' from original recording '{args.input}':")
    print(
        "  Input:   "
        f"{in_start.strftime('%H:%M:%S')} - {in_end.strftime('%H:%M:%S')} "
        f"({human_readable_size(in_size):>12} )"
    )

    if out_start is None or out_end is None:
        raise RuntimeError("No packets fell within the specified time range")
    print(
        "  Output:  "
        f"{out_start.strftime('%H:%M:%S')} - {out_end.strftime('%H:%M:%S')} "
        f"({human_readable_size(out_size):>12} )"
    )
