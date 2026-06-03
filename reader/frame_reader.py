"""
read_yuv422_stream.py
=====================
Receive a raw YUV422 (YUYV packed) frame from the EAGLE node over USB-CDC
serial, log statistics, display the frame live with OpenCV, and save each
frame to disk.

Eagle serial protocol
---------------------
1. Human-readable stats + "[EAGLE] Streaming reassembled frame over serial..."
2. Raw binary payload: YUV_WIDTH * YUV_HEIGHT * 2 bytes (YUYV interleaved)
3. "[EAGLE] Stream complete."

Steps 1 and 3 are UTF-8 text lines terminated with \\n.
Step 2 is a raw binary blob — no length prefix, no framing markers.
The host knows the exact size because it is fixed (172 800 bytes).

HOW TO RUN VERY IMPORTANT!!!

install uv

uv run frame_reader.py

"""

import argparse
import queue
import threading
import time
import datetime
import os

import serial
import numpy as np

try:
    import cv2

    HAS_DISPLAY = True
except ImportError:
    HAS_DISPLAY = False

# ------------------------------------------------------------------ #
# Configuration                                                        #
# ------------------------------------------------------------------ #

# PORT = "/dev/cu.usbmodem01"
PORT = "COM4"
BAUD = 115200

YUV_WIDTH = 360
YUV_HEIGHT = 120
IMAGE_SIZE = YUV_WIDTH * YUV_HEIGHT * 2  # 172 800 bytes — YUYV packed

OUTPUT_DIR = "yuv_frames"
OUTPUT_LATEST = "latest_frame.png"  # always overwritten
LOG_FILE = "yuv_stream.log"

PREVIEW_WINDOW = "YUV422 Preview"

READ_CHUNK = 4096
QUEUE_DEPTH = 2

# ------------------------------------------------------------------ #
# Logging helper                                                       #
# ------------------------------------------------------------------ #

_log_file = None


def log(msg: str, also_print: bool = True):
    """Append timestamped message to log file and optionally stdout."""
    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    line = f"[{ts}] {msg}"
    if also_print:
        print(line)
    if _log_file is not None:
        _log_file.write(line + "\n")
        _log_file.flush()


# ------------------------------------------------------------------ #
# YUV422 decode                                                        #
# ------------------------------------------------------------------ #


def decode_yuv422(raw: bytes, width: int, height: int) -> np.ndarray | None:
    """
    Convert a YUYV-packed buffer to a BGR uint8 numpy array.

    YUYV layout (4 bytes → 2 pixels):
      byte 0: Y0   byte 1: Cb   byte 2: Y1   byte 3: Cr

    Returns a (height, width, 3) BGR array, or None on error.
    """
    expected = width * height * 2
    if len(raw) < expected:
        log(f"decode_yuv422: buffer too short ({len(raw)} < {expected})")
        return None

    try:
        arr = np.frombuffer(raw[:expected], dtype=np.uint8).reshape(height, width * 2)

        # Extract luma and chroma planes
        Y = arr[:, 0::2].astype(np.float32)  # every even byte  → Y
        Cb = arr[:, 1::4].astype(
            np.float32
        )  # bytes 1,5,9,…   → Cb (shared by 2 pixels)
        Cr = arr[:, 3::4].astype(
            np.float32
        )  # bytes 3,7,11,…  → Cr (shared by 2 pixels)

        # Upsample Cb/Cr: each sample covers 2 horizontal pixels
        Cb = np.repeat(Cb, 2, axis=1)
        Cr = np.repeat(Cr, 2, axis=1)

        # BT.601 full-range conversion
        Cb -= 128.0
        Cr -= 128.0

        R = np.clip(Y + 1.402 * Cr, 0, 255).astype(np.uint8)
        G = np.clip(Y - 0.344136 * Cb - 0.714136 * Cr, 0, 255).astype(np.uint8)
        B = np.clip(Y + 1.772 * Cb, 0, 255).astype(np.uint8)

        bgr = cv2.merge([B, G, R]) if HAS_DISPLAY else np.stack([B, G, R], axis=2)
        return bgr

    except Exception as e:
        log(f"decode_yuv422 exception: {e}")
        return None


# ------------------------------------------------------------------ #
# Serial worker (runs in its own thread)                              #
# ------------------------------------------------------------------ #


def serial_worker(
    ser: serial.Serial, frame_queue: queue.Queue, stop_event: threading.Event
):
    """
    State-machine reader:

    IDLE      → scan text lines for the streaming-start sentinel
    RECEIVING → read exactly IMAGE_SIZE binary bytes
    DONE      → scan for the stream-complete sentinel, then back to IDLE
    """
    frame_count = 0
    SENTINEL_START = "[EAGLE] Streaming reassembled frame over serial..."
    SENTINEL_DONE = "[EAGLE] Stream complete."

    try:
        while not stop_event.is_set():

            # ── IDLE: read text lines until we see the start sentinel ──
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                log(f"< {line}")

            if SENTINEL_START not in line:
                continue

            # Drain anything that arrived in the same buffer as the sentinel
            # (binary 0x0A bytes would corrupt a readline-based read)
            ser.reset_input_buffer()
            time.sleep(0.05)

            # ── RECEIVING: slurp exactly IMAGE_SIZE bytes ──────────── #
            log(f"Frame incoming — expecting {IMAGE_SIZE} bytes …")
            t_start = time.perf_counter()
            t_deadline = t_start + 10.0  # give up after 10 s

            buffer = bytearray()
            remaining = IMAGE_SIZE

            while remaining > 0 and not stop_event.is_set():
                if time.perf_counter() > t_deadline:
                    log(
                        f"ERROR: frame receive timed out ({len(buffer)}/{IMAGE_SIZE} bytes received)"
                    )
                    buffer.clear()
                    break
                chunk = ser.read(min(remaining, READ_CHUNK))
                if not chunk:
                    continue
                buffer.extend(chunk)
                remaining -= len(chunk)

            elapsed_rx = time.perf_counter() - t_start

            if len(buffer) < IMAGE_SIZE:
                log(
                    f"WARNING: short read — got {len(buffer)} / {IMAGE_SIZE} bytes; discarding frame"
                )
                continue

            frame_count += 1
            log(
                f"Frame {frame_count}: received {len(buffer)} bytes "
                f"in {elapsed_rx*1000:.1f} ms  "
                f"({IMAGE_SIZE*8/elapsed_rx/1000:.1f} kbps effective)"
            )

            # Sanity-check first 8 bytes of YUYV data
            log(
                f"  First 8 bytes (YUYV): "
                f"{' '.join(f'{b:02X}' for b in buffer[:8])}"
            )

            # Expected values from build_yuv422_frame():
            #   Top half  → Y=0xFF, Cb=0x80, Cr=0x80  (white luma, neutral chroma)
            #   Bottom half → Y=0x00, Cb=0x80, Cr=0x80  (black luma, neutral chroma)
            expected_y_top = 0xFF
            expected_y_bottom = 0x00
            y_first = buffer[0]
            y_mid = buffer[IMAGE_SIZE // 2]
            log(
                f"  Y @ row 0   : 0x{y_first:02X}  "
                f"(expected 0x{expected_y_top:02X} — {'OK' if y_first == expected_y_top else 'MISMATCH'})"
            )
            log(
                f"  Y @ row {YUV_HEIGHT//2} : 0x{y_mid:02X}  "
                f"(expected 0x{expected_y_bottom:02X} — {'OK' if y_mid == expected_y_bottom else 'MISMATCH'})"
            )

            raw = bytes(buffer)

            # ── DONE: read until stream-complete sentinel ──────────── #
            while not stop_event.is_set():
                done_line = ser.readline().decode("utf-8", errors="ignore").strip()
                if done_line:
                    log(f"< {done_line}")
                if SENTINEL_DONE in done_line:
                    break

            # Push frame to display thread (drop if full)
            if not frame_queue.full():
                try:
                    frame_queue.put_nowait((frame_count, raw))
                except queue.Full:
                    pass

    except Exception as e:
        log(f"Serial worker error: {e}")
    finally:
        log("Serial worker stopped.")


# ------------------------------------------------------------------ #
# Frame saver                                                          #
# ------------------------------------------------------------------ #


def save_frame(frame_count: int, bgr: np.ndarray):
    """Save frame as PNG — latest.png (overwrite) + timestamped copy."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Always-overwritten latest frame
    cv2.imwrite(OUTPUT_LATEST, bgr)

    # Timestamped archive copy
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    path = os.path.join(OUTPUT_DIR, f"frame_{frame_count:04d}_{ts}.png")
    cv2.imwrite(path, bgr)
    log(f"  Saved → {path}")


# ------------------------------------------------------------------ #
# Main                                                                 #
# ------------------------------------------------------------------ #


def main():
    global _log_file, IMAGE_SIZE, YUV_WIDTH, YUV_HEIGHT

    parser = argparse.ArgumentParser(
        description="Receive YUV422 frame stream from EAGLE over serial"
    )
    parser.add_argument(
        "--port", default=PORT, help="Serial port (default: %(default)s)"
    )
    parser.add_argument(
        "--baud", default=BAUD, type=int, help="Baud rate (default: %(default)s)"
    )
    parser.add_argument(
        "--width",
        default=YUV_WIDTH,
        type=int,
        help="Frame width  (default: %(default)s)",
    )
    parser.add_argument(
        "--height",
        default=YUV_HEIGHT,
        type=int,
        help="Frame height (default: %(default)s)",
    )
    parser.add_argument(
        "--no-save", action="store_true", help="Disable frame saving to disk"
    )
    args = parser.parse_args()

    # Recompute image size from CLI args (in case user overrides dimensions)
    image_size = args.width * args.height * 2
    if image_size != IMAGE_SIZE:
        log(f"Image size overridden to {image_size} bytes ({args.width}x{args.height})")

    _log_file = open(LOG_FILE, "a", encoding="utf-8")
    log("=" * 60)
    log(
        f"read_yuv422_stream.py  port={args.port}  baud={args.baud}  "
        f"res={args.width}×{args.height}  image_size={image_size}"
    )
    log("=" * 60)

    log(f"Opening {args.port} @ {args.baud}")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)

    frame_queue = queue.Queue(maxsize=QUEUE_DEPTH)
    stop_event = threading.Event()

    # Update module-level constants with CLI args
    IMAGE_SIZE = image_size
    YUV_WIDTH = args.width
    YUV_HEIGHT = args.height

    worker = threading.Thread(
        target=serial_worker,
        args=(ser, frame_queue, stop_event),
        daemon=True,
    )
    worker.start()

    if HAS_DISPLAY:
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(PREVIEW_WINDOW, YUV_WIDTH * 2, YUV_HEIGHT * 2)
        log("Press 'q' in the preview window to quit.")
    else:
        log("opencv-python not found — install it for live preview.")

    try:
        last_bgr = None

        while not stop_event.is_set():

            # Drain the queue — process all pending frames
            frame_data = None
            while True:
                try:
                    frame_data = frame_queue.get_nowait()
                except queue.Empty:
                    break

            if frame_data is not None:
                frame_count, raw = frame_data
                bgr = decode_yuv422(raw, YUV_WIDTH, YUV_HEIGHT)

                if bgr is not None:
                    last_bgr = bgr
                    if not args.no_save and HAS_DISPLAY:
                        save_frame(frame_count, bgr)
                else:
                    log(f"  decode_yuv422 returned None for frame {frame_count}")

            if HAS_DISPLAY:
                if last_bgr is not None:
                    cv2.imshow(PREVIEW_WINDOW, last_bgr)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    stop_event.set()
            else:
                time.sleep(0.1)

    except KeyboardInterrupt:
        log("\nStopping …")
    finally:
        stop_event.set()
        worker.join(timeout=3.0)
        ser.close()
        if HAS_DISPLAY:
            cv2.destroyAllWindows()
        if _log_file:
            _log_file.close()
        log("Done.", also_print=True)


if __name__ == "__main__":
    main()
