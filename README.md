# CAM Hardware Test

Bring-up and bench-test firmware for the **CAM-MK3** transmitter board and the **EAGLE**
receiver board.

This repo is the sandbox that feeds [`CAM-Software`](../CAM-Software) (`~/iss/CAM-Software`).
Anything that touches new hardware — a radio chip, a link budget, a packet format, a new
peripheral — gets proven here first as a small standalone sketch. Once it works and the
numbers look right, the driver and protocol get ported into `CAM-Software`, which is the
real flight firmware (video capture → JPEG encode → radio → ground station).

**Rule of thumb:** if you are asking *"does this hardware even work?"* you belong in this
repo. If you are asking *"how does the flight video pipeline behave?"* you belong in
`CAM-Software`.

| | CAM-Hardware-Test (this repo) | CAM-Software |
| --- | --- | --- |
| Purpose | Hardware bring-up, driver bring-up, RF benchmarks | Flight firmware |
| Scope | One test program at a time | Full video pipeline (DVP → JPEG → RF → USB) |
| Framework | PlatformIO + Arduino (esp32-p4) | PlatformIO, hybrid ESP-IDF + Arduino |
| `components/arduino` | Committed in-tree | Git submodule |
| Video | Synthetic test patterns | Real NTSC from Runcam via TVP5151 |

## Hardware

- **CAM-MK3** — ESP32-P4 transmitter board (rocket avionics bay). Built with `-DIS_CAM`.
- **EAGLE** — ESP32-P4 ground-station receiver board. Built with `-DIS_EAGLE`.
- **LR2021** — sub-GHz transceiver on both boards, driven over SPI (RadioLib 7.6.0, vendored
  in [lib/RadioLib/](lib/RadioLib/)).

Both boards use the same `esp32-p4-evboard` PlatformIO board definition and differ only in
pin map and role. All pins for both boards live in [src/pins.h](src/pins.h), guarded by
`IS_CAM` / `IS_EAGLE`.

## Building and flashing

```bash
# CAM (transmitter)
pio run -e CAMmk3 -t upload

# EAGLE (receiver)
pio run -e EAGLE -t upload

# Serial monitor (115200)
pio device monitor -e EAGLE
```

There is **no git submodule step** in this repo — `components/arduino` is committed directly,
unlike `CAM-Software`. A plain clone builds.

Serial output goes over **USB-CDC (TinyUSB)**, not USB-serial-JTAG — see
[sdkconfig.defaults](sdkconfig.defaults). Every test program does the same dance:

```cpp
USBCDC USBSerial;
#undef Serial
#define Serial USBSerial
```

so `Serial` in these sketches means the CDC port (e.g. `/dev/tty.usbmodem01`, `COM4`), which
only enumerates after `USB.begin()`.

## How the test programs work

[src/CMakeLists.txt](src/CMakeLists.txt) globs **everything** in `src/`, so exactly one file
may have live code at a time. Every other test is kept in the tree fully commented out.
To switch tests: comment out the currently active file, uncomment the one you want, rebuild.

Yes, this is crude. It is also why each test file is self-contained and duplicates its own
`setup()`/`loop()`.

Currently active: **[src/fsk_framer_test.cpp](src/fsk_framer_test.cpp)**.

| File | What it proves |
| --- | --- |
| [fsk_framer_test.cpp](src/fsk_framer_test.cpp) | End-to-end frame test: CAM builds a synthetic YUV422 frame, fragments it, bursts it; EAGLE reassembles it and streams the raw frame out over USB for the Python reader. |
| [fsk_tester.cpp](src/fsk_tester.cpp) | Minimal FSK hello-world — single packet TX/RX, sanity check that the link is alive. |
| [fsk_tester_benchmark_single.cpp](src/fsk_tester_benchmark_single.cpp) | Throughput/loss over N single packets, one `transmit()` per packet. |
| [fsk_tester_benchmark_burst.cpp](src/fsk_tester_benchmark_burst.cpp) | Same benchmark using `transmitBurst()` (back-to-back FIFO writes), plus per-packet RSSI/LQI log. |
| [flrc_tester.cpp](src/flrc_tester.cpp) | FLRC mode hello-world. **Deprecated** — FLRC's 511-byte FIFO refill was never made reliable. |
| [flrc_tester_benchmark.cpp](src/flrc_tester_benchmark.cpp) | FLRC throughput benchmark. Deprecated with the above. |
| [blank.cpp](src/blank.cpp) | LED blink + serial print. First thing to flash on a fresh board. |
| [old.cpp](src/old.cpp) | Deprecated raw-RadioLib example, kept for reference on the FIFO opcodes. |

## Radio driver (`lib/LR2021/`)

- [radio.h](lib/LR2021/radio.h) — all RF configuration in one place: 434 MHz, 1000 kbps GFSK,
  250 kHz deviation, 2222.22 kHz RX bandwidth, 22 dBm, 24-bit preamble, 2-byte IBM CRC,
  8-byte sync word, 255-byte fixed packet length. 915 MHz constants are defined alongside.
  Also defines the `LR2021Error` struct — every driver call returns one, and
  `.stageStr()` tells you which init/TX/RX stage failed instead of a bare RadioLib code.
- [fsk.h](lib/LR2021/fsk.h) / [fsk.cpp](lib/LR2021/fsk.cpp) — the driver actually in use.
  `transmit()`, `transmitBurst()`, `receive()` (with RSSI/LQI packet status), and
  `transmitCallSign()` for FCC ID (`KE2CNQ`).
- [flrc.h](lib/LR2021/flrc.h) / [flrc.cpp](lib/LR2021/flrc.cpp) — **deprecated, do not use.**
- [fsk_framer.h](lib/LR2021/fsk_framer.h) — splits a buffer into fixed-size fragments.
- [fsk_reassembler.h](lib/LR2021/fsk_reassembler.h) — puts them back together on EAGLE,
  with frame timeout and drop/reset statistics.

### Fragment format

A 255-byte packet carries a 7-byte header and 248 bytes of payload:

```
byte  0      frame_id
bytes 1-2    fragment index   (big endian)
bytes 3-6    total frame size (big endian)
bytes 7-254  payload, zero-padded on the last fragment
```

The last fragment is zero-padded rather than short, because the radio runs in fixed
packet-length mode. Frames and fragment pools are allocated in SPIRAM
(`heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`), not internal RAM.

## Host tools (`reader/`)

Python side of the bench, managed with [uv](https://docs.astral.sh/uv/).

```bash
cd reader
uv run frame_reader.py   # live YUV422 preview + per-frame PNG dump
uv run reader.py         # plain serial logger, appends to log.txt
```

- [frame_reader.py](reader/frame_reader.py) — reads EAGLE's USB-CDC stream, splits text
  status lines from the raw binary frame blob, converts YUYV → BGR, shows it with OpenCV
  and writes frames to `yuv_frames/`.
- [reader.py](reader/reader.py) — timestamped line logger, for benchmark runs where you only
  care about the printed stats.

**Both scripts hardcode the serial port and `frame_reader.py` hardcodes the frame
dimensions** — edit `PORT` / `YUV_WIDTH` / `YUV_HEIGHT` to match your machine and whatever
`fsk_framer_test.cpp` is currently configured for, or the reader will desync from the stream.

### Captured runs

Raw serial captures from bench sessions are committed for reference:

- [eagle_test_5_18_2026](reader/eagle_test_5_18_2026) — 200/200 packets, 0 loss, ~772 kbps single-packet.
- [eagler_test_burst_5_31_2026](reader/eagler_test_burst_5_31_2026) — burst mode, 200 packets, 0 CRC errors, ~414 kbps.
- [eagle_framer_output_5_31_2026](reader/eagle_framer_output_5_31_2026) — full frame reassembled: 697 fragments, 172800 bytes, 0 CRC drops, ~749 kbps.

Note the framer capture predates the "decreasing image size" commit, so its frame size is
larger than what the current [fsk_framer_test.cpp](src/fsk_framer_test.cpp) sends.

## Gotchas

- **Only one active file in `src/`.** Two files with a live `setup()` will fail to link with
  duplicate-symbol errors.
- **Flash appears to fail but succeeded.** Same known toolchain watchdog bug as in
  `CAM-Software` — if the flashing dialog appeared, it almost certainly took.
- **EAGLE prints nothing.** It waits on `while (!Serial)`. Open the port; also check the CDC
  device re-enumerated after reset.
- **Bring both boards up together.** CAM transmits shortly after boot and EAGLE ends the run
  on a 500 ms RX idle timeout, so a late receiver simply misses the burst.
- **SPIRAM allocation failures** print `FATAL: SPIRAM alloc failed` and halt — check the
  sdkconfig for the environment you built.
