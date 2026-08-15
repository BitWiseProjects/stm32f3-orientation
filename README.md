# The board that knows which way it's pointing

Firmware and a web visualizer for the STM32F3 Discovery, built in eight stages —
from lighting one LED to fusing a gyroscope, an accelerometer and a
magnetometer into a live 3D orientation on screen.

This is the code from the [BitWise](https://youtube.com/@bitwisedex) episode of
the same name. It is meant to be read in order and flashed in order: each stage
is the previous one plus exactly one new idea, and the difference between two
files is the lesson.

## What you need

- **An STM32F3 Discovery board** (`STM32F3DISCOVERY`, board reference MB1035)
  and the USB cable it came with. That is the entire parts list.
- **Rust**, stable. `rust-toolchain.toml` pins the two compilation targets, so
  rustup installs them the first time you build — you do not have to add them
  by hand.
- **Three tools**, none of which come with Rust:

  ```
  cargo install probe-rs-tools    # flashes the board and prints its output
  cargo install flip-link         # makes stack overflows fault instead of corrupt
  cargo install trunk             # builds the web page
  ```

- **Chrome or Edge**, for the viewer. Web Serial does not exist in Firefox or
  Safari, and there is no polyfill for it.

## Layout

Three separate cargo workspaces in one repository:

```
firmware/   the code that runs on the board      (thumbv7em-none-eabihf)
viewer/     the web page                          (wasm32-unknown-unknown)
math/       the parts both of them use            (no target — runs anywhere)
```

They are separate workspaces rather than one because `.cargo/config.toml`
applies per directory, so each gets its own compilation target, linker flags and
runner. Separate lockfiles also make it impossible for a stray
`cargo build --workspace` to try building the firmware for your laptop.

`math/` holds four crates:

- **`attitude`** — sensor fusion. Rates and fields in, orientation out.
- **`magcal`** — hard-iron calibration. Magnetometer samples in, the offset
  they are all displaced by out.
- **`packet`** — the wire format. Where every byte of a packet goes, and the
  checksum over it.
- **`vector`** — geometry and projection, stopping at normalized device
  coordinates.

`packet` is the odd one out — it is not maths. It lives here because what this
directory really holds is source that both targets compile, and a wire format
is exactly the kind of thing that must not be written down twice. The firmware
encodes with it and the browser decodes with it, so the two cannot drift apart.

None of them knows what a sensor or a screen is, so all four run on your
machine:

```
cd math && cargo test
```

That is worth doing before you flash anything. It proves, among other things,
that gravity alone cannot correct a heading — which is the point the whole
second half of the episode turns on — and that a compass calibration done by
spinning the board flat on a desk cannot work however long you spin it for.
Both checked in about a millisecond.

`packet` comes with a tool for the other end of the wire — point it at raw
bytes captured off the serial port and it decodes them with exactly the code
the browser runs:

```
cargo run -p packet --example decode -- board.bin
```

If that says the stream is good, the firmware end is fine and anything wrong is
on the page.

### Fitting a calibration on your laptop

The same tool will extract the raw magnetometer samples out of a capture
instead of reporting on it, and from there two separate programs will fit them:

```
cargo run -p packet --example decode -- board.bin --csv > samples.csv
cargo run -p magcal --example survey -- samples.csv       # the fit the board runs
../firmware/notes/fit_sphere.py samples.csv               # an unrelated one, in Python
```

Both read that one file, and that is the whole point of it existing. They are
independent implementations of the same estimator in different languages, so
when they agree on the same bytes, the agreement is evidence rather than
coincidence. If each captured its own samples instead, it would be neither.

`fit_sphere.py` is stdlib-only — no numpy, no venv — and it also scores
candidate offsets against a capture rather than only fitting new ones, which is
how the constant compiled into the firmware was chosen.

## Flashing, in order

From `firmware/`. Each of these builds, flashes and then streams the board's
output back to your terminal:

```
cargo run --bin 00_ring
```

| Stage | What it adds | You should see |
|---|---|---|
| `00_ring` | nothing — it is the smoke test | a light walks round the ring |
| `01_ring` | the ring as one byte; the 72 MHz clock | one light, then two neighbours, walking |
| `02_compass` | I2C, the magnetometer, a heading | turn the board — and the lit LED does **not** hold still, which is the next stage's problem |
| `03_calibrate` | hard-iron calibration — a least-squares sphere fit | press the button, wave the board, and now the lit LED does stay pointing the same way in the room |
| `04_gyro` | SPI, the gyroscope, integration | the ring follows a turn; a magnet does nothing |
| `05_serial` | packets out of the serial port | the viewer tracks the board — then drifts when you put it down |
| `06_gravity` | the accelerometer as a correction | put it down and it stops tipping over — but spin it flat and the heading still slides |
| `07_fused` | the magnetometer as a second correction | put it down and it holds |

Start at `00_ring`. If it does not work, the problem is in the setup rather than
in anything the later stages do, and that is much easier to find in twenty lines
than in three hundred.

## The viewer

With `05_serial`, `06_gravity` or `07_fused` running on the board:

```
cd viewer && trunk serve
```

Open <http://localhost:8080> and press **Connect**, then pick the ST-LINK's port.

The button is not decoration — a page cannot open a serial port on its own, and
the port picker only appears in response to a real click. That is the permission
model working as intended.

With `07_fused`, pressing the board's blue USER button also switches the page to
its calibration view and back again. There is no control for that on the page:
the board says in its packets that a run has started, and the page follows. So
the only thing to press is the one on the board, and the older stages — which
never send those packets — go on working untouched.

The viewer has a few tests of its own, covering when that switch happens and
what it says when a run ends badly. They need the host target named explicitly,
because this directory otherwise builds everything for the browser, test harness
included:

```
cd viewer && cargo test --target "$(rustc -vV | sed -n 's/^host: //p')"
```

Two things that look like faults and are not:

- **A burst of nonsense on connect.** The board has been sending packets since
  it was powered on and the ST-LINK buffers them while nothing is listening, so
  connecting delivers a lump of stale data before it catches up to live. The
  packet marker exists precisely so the page can find its footing mid-stream.
- **A red exception in the browser console** saying *"Using exceptions for
  control flow, don't mind me."* That is how the windowing library returns from
  `main` on the web. It is not an error.

## Two notes for anyone reading the build closely

**There is no `memory.x` and no `build.rs`.** `stm32f3xx-hal` generates the
linker's memory map from the chip feature in `firmware/Cargo.toml`. Adding a
hand-written one would just be a second copy that can disagree.

**The linker may warn** that `.text` is not a multiple of alignment 8. The
vector table on this chip is 404 bytes — an odd number of words — so whatever
follows it starts four bytes off an eight-byte boundary. The alignment is
requested by `cortex_m::asm::delay`, which aligns its inner loop so its timing
does not shift around; it is a performance hint, not a correctness requirement,
and nothing in `.text` faults for want of it.

## Licence

MIT or Apache-2.0, at your option.
