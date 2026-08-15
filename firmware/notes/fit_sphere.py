#!/usr/bin/env python3
"""Fit a sphere to raw magnetometer samples, as a second opinion on `magcal`.

Rotating a magnetometer through every attitude traces a sphere. If that sphere
is centred on the origin, the only field present is the earth's. If it is
centred somewhere else, something magnetic is riding along with the board, and
the vector from the origin to the centre is the hard-iron offset — the constant
the firmware subtracts.

The input is what `packet`'s decode writes: three raw readings a line, in
nanotesla, uncorrected.

    cargo run -p packet --example decode -- board.bin --csv > samples.csv
    ./fit_sphere.py samples.csv

`magcal`'s survey example reads that identical file:

    cargo run -p magcal --example survey -- samples.csv

Which is the point of this script. The board's fit and this one are separate
implementations of the same estimator, in different languages, over the same
bytes — so when they agree, the agreement means something. Reconstructing the
samples separately for each tool would have thrown that away.

    ./fit_sphere.py samples.csv --compare 8062.6,-30600,-9042.9

`--compare` scores candidate offsets against these samples instead of just
fitting a new one. That is how the baked-in constant was chosen: several
calibration runs, each scored against every run's data, picking the offset with
the best worst case rather than the best average.

Stdlib only — no numpy — so it runs anywhere.
"""

import argparse
import math
import sys


def read(path):
    """Three floats a line. Blank lines and `#` comments are skipped."""
    out = []
    with open(path) as handle:
        for number, line in enumerate(handle, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) != 3:
                sys.exit(f"{path}:{number}: expected three numbers, got {len(parts)}")
            try:
                out.append(tuple(float(p) for p in parts))
            except ValueError:
                sys.exit(f"{path}:{number}: not a number: {line!r}")
    return out


def solve(a, b):
    """Gaussian elimination with partial pivoting. Returns None if singular.

    Deliberately the plain textbook version rather than anything clever. This
    file exists to disagree with the firmware when the firmware is wrong, which
    it can only do if the two share as little as possible — the board
    accumulates 14 running sums in f32 and never stores a sample; this holds
    every sample and works in double precision.
    """
    n = len(b)
    m = [list(row) + [rhs] for row, rhs in zip(a, b)]

    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        for row in range(col + 1, n):
            factor = m[row][col] / m[col][col]
            for k in range(col, n + 1):
                m[row][k] -= factor * m[col][k]

    x = [0.0] * n
    for row in reversed(range(n)):
        total = m[row][n] - sum(m[row][k] * x[k] for k in range(row + 1, n))
        x[row] = total / m[row][row]
    return x


def algebraic(samples):
    """The fit the board runs: linear, one pass, no starting guess needed.

    |m - c|^2 = r^2 expands to

        2·mx·cx + 2·my·cy + 2·mz·cz + k = |m|^2      with  k = r^2 - |c|^2

    which is linear in the four unknowns, so least squares is a single 4x4
    solve. That is why the firmware can do it in a few hundred bytes of state.

    What it minimises is the error in |m|^2, not the error in distance, so
    samples far from the centre pull harder than they should. `geometric`
    below fixes that; on real data the two land within a few nT of each other.
    """
    a = [[0.0] * 4 for _ in range(4)]
    b = [0.0] * 4

    for x, y, z in samples:
        row = [2 * x, 2 * y, 2 * z, 1.0]
        square = x * x + y * y + z * z
        for i in range(4):
            for j in range(4):
                a[i][j] += row[i] * row[j]
            b[i] += row[i] * square

    answer = solve(a, b)
    if answer is None:
        return None

    cx, cy, cz, k = answer
    inside = k + cx * cx + cy * cy + cz * cz
    if inside <= 0.0:
        return None
    return (cx, cy, cz), math.sqrt(inside)


def geometric(samples, centre, radius, rounds=50):
    """Refine by minimising the honest residual, sum of (|m - c| - r)^2.

    Gauss-Newton from the algebraic answer. Each residual has a simple
    derivative: moving the centre one unit toward a sample shortens its
    distance by one unit along the unit vector to it, so the row is
    (-ux, -uy, -uz, -1).

    This is the estimator you would use if you had the whole capture and no
    memory limit. Reporting both is a way of asking what the board's shortcut
    actually costs — and on a well-swept run the answer is: almost nothing.
    """
    cx, cy, cz = centre
    r = radius

    for _ in range(rounds):
        a = [[0.0] * 4 for _ in range(4)]
        b = [0.0] * 4

        for x, y, z in samples:
            dx, dy, dz = x - cx, y - cy, z - cz
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance < 1e-9:
                continue
            row = [-dx / distance, -dy / distance, -dz / distance, -1.0]
            residual = distance - r
            for i in range(4):
                for j in range(4):
                    a[i][j] += row[i] * row[j]
                b[i] -= row[i] * residual

        step = solve(a, b)
        if step is None:
            break
        cx += step[0]
        cy += step[1]
        cz += step[2]
        r += step[3]
        if max(abs(s) for s in step) < 1e-6:
            break

    return (cx, cy, cz), r


def score(samples, centre):
    """How spherical the samples look about a given centre.

    Returns the mean distance, the RMS deviation from it, and that deviation as
    a percentage. The percentage is the number worth comparing between
    candidate offsets: it asks how nearly constant the corrected field strength
    is, which is the one thing that must be true of a real magnetic field
    however the board is turned.
    """
    cx, cy, cz = centre
    distances = [
        math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2) for x, y, z in samples
    ]
    mean = sum(distances) / len(distances)
    rms = math.sqrt(sum((d - mean) ** 2 for d in distances) / len(distances))
    return mean, rms, 100.0 * rms / mean if mean else float("inf")


def spread(samples):
    """27·det(S)/trace(S)^3 over the covariance — 0 if coplanar, 1 if a ball.

    The same metric the firmware gates on, and it has the same blind spot here:
    it measures whether the cloud is equally wide in every direction, not
    whether it covers a sphere. A board left sitting still scores near 1,
    because sensor noise is isotropic. Read it next to the scatter percentage,
    never alone.
    """
    n = len(samples)
    mx = sum(s[0] for s in samples) / n
    my = sum(s[1] for s in samples) / n
    mz = sum(s[2] for s in samples) / n

    s = [[0.0] * 3 for _ in range(3)]
    for x, y, z in samples:
        d = (x - mx, y - my, z - mz)
        for i in range(3):
            for j in range(3):
                s[i][j] += d[i] * d[j]
    for i in range(3):
        for j in range(3):
            s[i][j] /= n

    trace = s[0][0] + s[1][1] + s[2][2]
    det = (
        s[0][0] * (s[1][1] * s[2][2] - s[1][2] * s[2][1])
        - s[0][1] * (s[1][0] * s[2][2] - s[1][2] * s[2][0])
        + s[0][2] * (s[1][0] * s[2][1] - s[1][1] * s[2][0])
    )
    return 27.0 * det / trace**3 if trace > 0 else 0.0


def sectors(samples):
    """Which 45 degree bearing bins were visited, mirroring the firmware.

    Bearings are taken about the bounding box midpoint rather than the origin,
    because the origin is precisely the point not yet trusted — on a board whose
    offset rivals the field, bearings about it all crowd into one bin. The
    half-step centres bin 0 on +X, so the eight bins sit *at* the eight ring
    LEDs rather than between them.
    """
    lo = [min(s[i] for s in samples) for i in range(2)]
    hi = [max(s[i] for s in samples) for i in range(2)]
    mid = [(lo[i] + hi[i]) / 2 for i in range(2)]

    mask = 0
    for x, y, _ in samples:
        dx, dy = x - mid[0], y - mid[1]
        if dx or dy:
            turns = math.atan2(dy, dx) * (4.0 / math.pi)
            mask |= 1 << (math.floor(turns + 0.5) % 8)
    return mask


# Mirrors `magcal::MIN_SPREAD`. Below this the samples are flat enough that the
# solve, while not singular in arithmetic, is meaningless in geometry — see the
# refusal message in `main` for what that looks like in practice.
MIN_SPREAD = 0.05


def vector(text):
    parts = text.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated numbers")
    return tuple(float(p) for p in parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("path", help="CSV of raw mx,my,mz samples in nT")
    parser.add_argument(
        "--compare",
        type=vector,
        action="append",
        default=[],
        metavar="X,Y,Z",
        help="score this candidate offset against the samples; repeatable",
    )
    args = parser.parse_args()

    samples = read(args.path)
    if len(samples) < 4:
        sys.exit(f"{args.path}: {len(samples)} samples, need at least 4 for a sphere")

    mask = sectors(samples)
    how_spread = spread(samples)
    print(args.path)
    print(f"  samples    {len(samples)}")
    print(f"  sectors    {mask:#010b}  ({bin(mask).count('1')} of 8)")
    print(f"  spread     {how_spread:.6f}")

    fitted = algebraic(samples)
    if fitted is None or how_spread < MIN_SPREAD:
        sys.exit(
            f"  REFUSED — spread {how_spread:.6f} is below {MIN_SPREAD}.\n"
            "  These samples are flat. A circle cannot say where the centre of a\n"
            "  sphere is, and turning the board flat on the desk traces one however\n"
            "  long you turn it for — every sample shares a Z, so the fit has no way\n"
            "  to separate the centre's Z from the radius.\n"
            "\n"
            "  Note that the arithmetic does not fail. Remove this check and it\n"
            "  returns a confident centre with a flattering residual, because the\n"
            "  samples really do lie on the sphere it found — along with every other\n"
            "  sphere through that circle. Tilt the board through the sweep."
        )

    centre, radius = fitted
    refined, refined_radius = geometric(samples, centre, radius)

    for name, c, r in (
        ("algebraic", centre, radius),
        ("geometric", refined, refined_radius),
    ):
        mean, rms, percent = score(samples, c)
        print(f"  {name}")
        print(f"    centre   ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}) nT")
        print(f"    offset   {math.sqrt(sum(v * v for v in c)):.1f} nT")
        print(f"    radius   {r:.1f} nT")
        print(f"    residual {rms:.1f} nT")
        print(f"    scatter  {percent:.2f}%  of a {mean:.0f} nT field")

    moved = math.dist(centre, refined)
    print(f"  the two estimators differ by {moved:.1f} nT")
    if moved > 0.05 * radius:
        print("    which is a lot. Two estimators only wander apart like this when")
        print("    the data does not pin the answer down — suspect thin coverage,")
        print("    even though the spread check passed.")

    candidates = [("none (raw)", (0.0, 0.0, 0.0))] + [
        (f"{c[0]:.1f},{c[1]:.1f},{c[2]:.1f}", c) for c in args.compare
    ]
    print("  scored against these samples:")
    for name, c in candidates:
        _, _, percent = score(samples, c)
        print(f"    {percent:6.2f}%   {name}")


if __name__ == "__main__":
    main()
