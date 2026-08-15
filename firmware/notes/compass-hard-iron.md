# The compass points the wrong way — open investigation

**Status: the cable is eliminated.** The ferrite-free cable arrived and the §7 test was run on
2026-08-15. **The ratio did not move: 0.97, against 0.97 and 0.98 on the old one.** The prime
suspect of §5 is dead, and the investigation now points at something fixed to the board — see §8,
which also corrects a piece of reasoning in §5 that turns out not to hold.

This file exists so the investigation can be picked up cold. It is now a **record rather than a
workbench**: the two diagnostic binaries and the four captures they produced have been deleted, so
the numbers quoted here can be read but not re-derived. That was a deliberate trade — the questions
they were built to answer are all answered, and the answers are written down here and in the
firmware. §9 says what took their place and how to capture fresh data.

---

## 1. The symptom

`02_compass` is supposed to hold a lit LED pointing at one place in the room while you turn the
board underneath it. It doesn't. Turning the board 90° moves the heading by roughly 45° over three
of the four quadrants, and by much more over the fourth. The LED wanders instead of staying put.

The heading is not random and it is not noisy. It is *smoothly and repeatably wrong* — the same
board angle gives the same wrong answer every time.

## 2. What produced the data

A throwaway diagnostic binary, `_mag_survey.rs`, since deleted — see *§9, what became of the
tooling*. It configured the LSM303AGR magnetometer at `HighResolution` / 50 Hz and the
accelerometer at `Normal` / 50 Hz / ±2 g, and printed one line every 250 ms:

```
total 50558.68  horiz 16759.773  heading 45.725227  tilt 2.738025
```

| Field | Meaning |
|---|---|
| `total` | \|B\|, the length of the whole field vector, in nT |
| `horiz` | the part of it lying in the board's own XY plane — the part a compass steers by |
| `heading` | `atan2(y, x)` in degrees, 0–360 |
| `tilt` | angle between the board's Z axis and gravity, from the accelerometer |

`total` is the useful invariant. **Rotating a sensor inside a fixed field cannot change the length
of the field vector**, only its direction. So if `total` swings while the board turns in place, the
field is not what the maths assumes.

> This investigation is closed, and both throwaway binaries have been deleted along with the
> captures they produced. The numbers below are the record; they are not reproducible from this
> repo as it now stands. **See §9 for what replaced them, and how to capture fresh data.**

## 3. What the data says

Three continuous captures, since deleted along with the binary that made them. The log names below
identify which capture each row came from; they are no longer files you can open.

Rotating the board in place should trace a circle centred on the origin. Fitting a circle to the
horizontal samples and asking *where its centre is* turns the problem into one number.

| Capture | AGR offset cancellation | n | Offset from origin | Radius | Ratio | Offset bearing |
|---|---|---:|---:|---:|---:|---:|
| `mag_survey.log` | off | 468 | 21,631 nT | 23,674 nT | **0.91** | −68.0° |
| `mag_survey2.log` | **on** | 469 | 23,945 nT | 24,356 nT | **0.98** | −27.3° |
| `mag_parked.log` | **on** | 589 | 23,267 nT | 23,989 nT | **0.97** | −29.8° |
| `mag_newcable.log` | **on** | 500 | 25,515 nT | 26,252 nT | **0.97** | −35.8° |

The radius — about 24,000 nT — is a believable earth horizontal field. The offset is very nearly
the same size.

The last row is the new cable, and it changed nothing. Figures in that row are the `--tilt 4` fit so
they compare like for like with §4's table; the other rows quote their all-sample fits, which differ
in the third significant figure at most.

**That ratio is the whole problem.** When the offset approaches the radius, the circle nearly
touches the origin, and headings crowd together on one side and spread out on the other. A ratio
above 0.5 means some headings become unresolvable. At 0.97 the compass is barely a compass.

### The parked positions

`mag_parked.log` is one continuous run in which the board was set down, held still, turned by hand,
and set down again. Splitting it at the moments it moved gives the stationary readings:

| Park | n | `total` | `horiz` | vertical | `heading` | `tilt` |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 62 | 50,067 | 16,304 | 47,338 | 45.9° | 2.5° |
| 2 | 44 | 53,019 | 18,633 | 49,637 | 261.6° | 2.7° |
| 3 | 104 | 67,602 | 44,538 | 50,857 | 307.6° | 3.3° |
| 4 | 187 | 67,083 | 44,817 | 49,916 | 350.6° | 3.4° |
| 5 | 19 | 51,803 | 20,862 | 47,416 | 36.4° | 4.5° |

`horiz` ranges over 2.7× between orientations of a board sitting on the same table. It cannot do
that in a uniform field.

## 4. What has been ruled out, and how

Each of these was a live hypothesis. None survived.

### Torn samples across an I2C read — **eliminated by reading the driver**

If the three axes came from different instants the vector length would be nonsense. Checked
`lsm303agr` 0.3: `init()` enables block-data-update and a field read is a single burst. Not
possible.

### Tilt leaking vertical field into the horizontal plane — **eliminated by refitting**

The vertical field here is ~48,000 nT, so tilting the board by θ leaks `48000·sin θ` into the
horizontal. That is a real effect and the captures do contain tilt spikes up to 41°. But it cannot
be the cause, because the fit does not care:

| Capture | all samples | tilt < 6° | tilt < 4° |
|---|---:|---:|---:|
| `mag_survey.log` | 21,631 | 21,627 | 21,644 |
| `mag_survey2.log` | 23,945 | 23,921 | 23,896 |
| `mag_parked.log` | 23,267 | 23,252 | 23,194 |

Unchanged to three significant figures. Also, faking a 23,000 nT offset would take about 28° of
*sustained* tilt, and the mean tilt across every capture is 3–4°.

Worth knowing anyway: none of our code is tilt-compensated, and ST's demo is. That is a separate
improvement, not this bug.

### A non-uniform field from the steel screw in the coffee-table leg — **eliminated by moving the board**

The board sits on a coffee table whose leg has a large screw in it. If the field varied across the
table, moving the board would look exactly like a hard-iron offset. So: same orientation, two
places, ~10 s averaged each.

| Spot | n | `total` | `horiz` | `heading` | `tilt` |
|---|---:|---:|---:|---:|---:|
| A — directly over the screw | 41 | 73,156 | 48,207 | 309.9° | 3.4° |
| B — far corner of the table | 41 | 76,906 | 48,223 | 308.4° | 3.3° |

`horiz` differs by 16 nT — **0.03%**. Heading by 1.5°.

The screw is real and it does something: `total` changed 5%, which works out to the vertical
component going 55,027 → 59,908, about 9%. A vertical screw below the board pulling on the vertical
field is exactly what you would expect. **But it barely touches the horizontal**, and the horizontal
is the only part a compass uses.

So position is not the variable. The 2.7× spread in the parked table is orientation.

### Soft iron — **eliminated by the shape of the locus**

Soft iron distorts the circle into an *ellipse*. Hard iron slides it sideways as a *circle*. The
residuals from a pure circle fit are 465–750 nT mean against a 24,000 nT radius — around 3%. It is a
displaced circle.

### The sensor's own zero offset — **eliminated by turning on the part's own correction**

The AGR can measure and subtract its own offset using set/reset pulses through an internal coil.
`enable_mag_offset_cancellation()` was switched on partway through the session (the `off`/`on`
column in the table above marks exactly which captures had it).

**The offset did not go away.** It was 21,631 nT before and 23,945 nT after. The fit got tidier —
residuals dropped from 1,965 to 487 — but the displacement stayed.

That is decisive about *where* the magnet is: not in the die.

## 5. What is left

**Hard iron, external to the sensor, moving with the board.**

The prime suspect is the USB cable. It has a ferrite about 10 cm from the board, it rotates with the
board when the board is turned, and its connector shell is steel that may have been magnetised.

One piece of evidence points quite specifically at it. If the magnet were a component soldered to
the PCB, the offset vector would be *fixed in the board frame* — same magnitude, same bearing,
forever. It isn't:

| | Cancellation | Offset | Bearing |
|---|---|---:|---:|
| `mag_survey.log` (01:25 UTC) | **off** | 21,631 nT | −68.0° |
| `mag_survey2.log` (01:37 UTC) | on | 23,945 nT | −27.3° |
| `mag_parked.log` (01:41 UTC) | on | 23,267 nT | −29.8° |

The magnitude is roughly constant throughout. The **bearing moved 41°** between the first capture
and the second — across an interval when the board was picked up, reflashed, and put back down, so
the cable was certainly disturbed. Between the second and third, four minutes apart with the cable
left alone, the bearing moved 2.4°.

A magnet bolted to the board does not do that. A magnet lying on a cable that flops around does.

> **This argument does not hold, and the cancellation column is why — added 2026-08-15.** The 41°
> jump is between the one capture with offset cancellation *off* and the first one with it *on*.
> Those two measure different quantities: with cancellation off you get the external hard iron
> **plus the die's own offset**, with it on you get the external part alone. Two different vectors
> summed differently will of course point differently, so the 41° is explained by the setting
> change and says nothing about whether the source moved.
>
> Compare only the three cancellation-on captures — now four minutes, then five days and a
> different cable apart — and they agree to **8.5° of bearing and 10% of magnitude**. That is not a
> magnet flopping about on a cable. That is something bolted down. See §8.

### A useful cross-check from ST

ST's own factory demo for this board is in the STM32F3-Discovery firmware package
(`Project/Demonstration/main.c`, with `Utilities/STM32F3_Discovery/stm32f3_discovery_lsm303dlhc.c`).
It reads the magnetometer, tilt-compensates it, takes `atan2`, and lights one of eight LEDs.

**It performs no calibration of any kind** — no offset, no bias, no hard-iron term anywhere in
either file. ST shipped that as the out-of-box experience for this exact board.

So a bare, uncalibrated magnetometer on an STM32F3 Discovery is normally expected to point north
well enough to demo. That argues the offset we are seeing is not a property of the board.

*(Unrelated but worth recording: ST's tilt compensation has a bug. `main.c` line 307 reads
`... - MagBuffer[1]*fSinRoll*fCosPitch`, where the standard formula's third term uses Z —
`MagBuffer[2]`. It hides because the term vanishes when the board is flat, and the demo refuses to
update the LEDs past 40° of tilt anyway.)*

## 6. Why this blocks the episode

Slate #1 claims the board can tell you which way it is pointing, and Act 5 rests on the magnetometer
correcting the gyro's heading drift. A compass with a 0.97 offset ratio cannot do that.

It also decides a script question. If the cause is one particular cable, it is a filming note. If
it is the board, then hard-iron calibration has to go into the firmware, because **every viewer who
clones this repo hits the same wall** — and that changes what the episode has to teach.

## 7. The test to run when the cable arrives

**This test has been run — §8 has the result.** The steps are kept because the reasoning in them
is what §8 is a verdict on, but the two commands below name a binary and a script that no longer
exist. §9 says what to run instead.

Change one thing. Same table, same spot, same everything, new cable.

1. Flash the survey binary:
   ```
   cargo run --bin _mag_survey
   ```
2. Capture a full turn in **one continuous run**, without lifting the board. Set it down, hold it
   still ~10 s, turn it ~90° by sliding rather than lifting, hold still again, and go round.
   Do not pick the board up at any point — that is what invalidated the first comparison.
3. Keep it flat. Tilt spikes are survivable but they widen the residuals.
4. Save the output to `data/mag_newcable.log` and run:
   ```
   python3 fit_circle.py data/mag_newcable.log --parks --tilt 4
   ```

**Read the `ratio` line.**

| Ratio | Meaning | What to do |
|---|---|---|
| < 0.1 | The cable was the whole problem | Note it as a filming requirement, delete the diagnostics, carry on to the gyro |
| 0.1 – 0.5 | Mostly the cable, something else left | Usable compass. Decide whether the remainder is worth a calibration step |
| > 0.5 | Not the cable | See below |

### If it is not the cable

Then something board-fixed is doing it, and the next steps are, in order:

1. **Unplug and run on battery or a different host** to rule out the laptop and its power supply.
2. **Take the board off the table entirely** — hold it in the air in the middle of the room and
   repeat the turn. Rules out the room.
3. If the offset survives both, it is the board, and the episode needs a hard-iron calibration
   step: turn the board through a full circle, fit the circle, subtract the centre. The host-side
   script already does the fitting — it would move into `attitude` or into `02_compass` directly.

   *(This is what happened. The fit became the `magcal` crate and the step became `03_calibrate`,
   and the "full circle" in this sentence turned out to be wrong — a circle cannot locate a
   sphere's centre. It has to be a sweep through many attitudes. See §9.)*

Note that option 3 is not a disaster for the script. "Your compass is lying to you, here is how you
find out by how much, here is how you fix it" is a good five minutes of television. It is just a
*different* five minutes than the one currently written, so it is a decision to make deliberately.

---

## 8. The cable is eliminated — 2026-08-15

The §7 test was run exactly as written: same table, same spot, new ferrite-free cable, one
continuous run of 500 samples over about two minutes, sliding rather than lifting.

```
offset       25515 nT  bearing -35.8 deg
radius       26252 nT
ratio         0.97
residual       718 nT mean
```

**Ratio 0.97, against 0.98 and 0.97 on the old cable.** Nothing moved. Read against §7's table that
is the *"> 0.5 — not the cable"* row, and §5's prime suspect is dead.

### Steps 1 and 2 above are already answered, by geometry

Do not spend a bench session on them. **A field that is fixed in the room cannot displace the
circle at all.** The sensor reads the field in the board's own frame, so turning the board in place
sweeps any static room field — earth, the laptop, its power supply, the screw in the table leg,
the building's steel — around a circle *centred on the origin*. Such a source changes the radius.
It cannot move the centre.

Moving the centre requires a field that is constant **in the board frame**, which means a source
that physically turns with the board. §4 already showed this empirically when the board moved
across the table and `horiz` changed by 0.03%; the geometry says the same thing in one line.

So the candidate list was only ever: the cable, or the board. The cable is now out.

### What this does not settle

**The dip is not measurable from this capture**, so Slate #1's *"Re-measure the dip, once the new
cable is here"* item is *not* closed by it. The fitted radius, about 26,000 nT, is a fair estimate of the true
horizontal field because fitting the circle removes the offset. The vertical component is not
recoverable the same way — it comes out of `total`, which is contaminated, and the per-park
`vert` column spreads from 36,564 to 50,829 nT on a board that never left the table, which is not a
number anyone should put in an animation. The dip stays blocked until the hard iron is resolved or
removed.

---

## 9. What became of the tooling — 2026-08-15

The investigation closed, so its instruments were retired. Both were written to answer one question
each, both questions are answered above, and a diagnostic that has given its answer is a liability
in a repo people are told to clone.

| Deleted | Why it existed | Where its answer lives now |
|---|---|---|
| `src/bin/_i2c_probe.rs` | Which part is this — DLHC or AGR? | The *Facts about the hardware* list below, and the comment on `lsm303agr` in `firmware/Cargo.toml`. It also did not compile: it imported `lsm303dlhc`, a driver for a chip this board does not have, which broke a bare `cargo build` for everyone |
| `src/bin/_mag_survey.rs` | Is the field clean enough to navigate by? | No — §3 through §8. The measurement it performed is now a by-product of `03_calibrate`, which reports the same displacement as part of doing something about it |
| `notes/data/*.log` | The evidence for the tables above | The tables above |
| `notes/fit_circle.py` | Fit a circle to horizontal samples | `notes/fit_sphere.py`, below |

### Why the replacement fits a sphere and the old one fit a circle

Not a refinement — a correction. §7 step 3 above proposed "turn the board through a full circle, fit
the circle, subtract the centre", and that is wrong in a way that took a while to see.

**A circle cannot locate the centre of a sphere.** Spin the board flat and every sample shares a Z,
so nothing in the data separates the centre's Z from the radius; the normal equations go singular.
The old script got away with it only because it never claimed a Z — it fitted the two horizontal
axes and left the third alone, which is fine for *detecting* hard iron and useless for *removing*
it. Correcting a heading needs all three.

So the sweep became a sweep through many attitudes, and the fit became a sphere fit. `magcal` refuses
a flat spin outright rather than returning the confident nonsense a solver will happily produce from
one.

### Capturing fresh data now

Raw `mx, my, mz` never left the board in the old arrangement — `_mag_survey` computed magnitude and
heading on-chip and printed only those, which is the other reason a 3D fit was never possible from
those logs. The calibration packet carries the raw reading, so it is possible now.

```
cd firmware && cargo run --bin 03_calibrate        # or 07_fused
```

Press the blue USER button, then sweep the board through as many attitudes as you can for at least
30 seconds — turn it *and* tilt it. Capture the stream (see the header of `math/packet/examples/decode.rs`
for the `stty`/`dd` incantation and its two traps), then:

```
cd math
cargo run -p packet --example decode -- board.bin --csv > samples.csv
cargo run -p magcal --example survey -- samples.csv        # the fit the board runs
../firmware/notes/fit_sphere.py samples.csv                # an unrelated one, in Python
```

Both read that same file, which is the point. Two implementations of the same estimator, in two
languages, over identical bytes — when they agree the agreement means something. Verified on the
bench captures: they match to the printed digit, spread and all.

`fit_sphere.py` also scores candidate offsets against a capture:

```
./fit_sphere.py samples.csv --compare 8062.6,-30600,-9042.9
```

which is how the constant now compiled into the firmware was chosen — several runs, each scored
against every run's samples, taking the offset with the best *worst* case.

---

## Appendix — what can and cannot be reproduced

Nothing in §3, §4 or §8 can be regenerated from this repo — those captures are gone. The numbers
stand as a record of what the board read on 2026-08-14 and 2026-08-15.

The A/B position readings in §4 were taken live and averaged over 41 samples each; they never had a
log file at all.

What *can* be re-run is the conclusion: fit a fresh capture as §9 describes and the offset should
land near the constant in `03_calibrate.rs`. Expect the centre to wander by a couple of thousand nT
between runs. That is a shallow minimum rather than an unstable measurement — the residual barely
changes across that whole neighbourhood, and the resulting heading moves by well under a degree.

### Facts about the hardware that this investigation established

- The e-compass is an **LSM303AGR**, not the LSM303DLHC that every STM32F3 Discovery tutorial names.
  They share both I2C addresses (0x19 accelerometer, 0x1E magnetometer) and their accelerometers are
  compatible enough that DLHC code drives an AGR accelerometer correctly and silently. Their
  magnetometers are unrelated. Discriminating reads: `0x1E:0x0A` = 0x48 on a DLHC, `0x1E:0x4F` = 0x40
  on an AGR magnetometer, `0x19:0x0F` = 0x33 on an AGR accelerometer.
- The AGR magnetometer has **one fixed full scale**, 150 nT per count. There is no gain register to
  choose, unlike the DLHC. The driver returns nanotesla directly.
- `lsm303agr` **0.3**, not 1.x — 1.x requires embedded-hal 1.0 and `stm32f3xx-hal` 0.10 provides 0.2.
