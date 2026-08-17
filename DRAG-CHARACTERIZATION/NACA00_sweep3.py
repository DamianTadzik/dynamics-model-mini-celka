import subprocess
import os

XFOIL_PATH = r"XFOIL6.99/xfoil.exe"

TIMEOUT = 30
NU = 1.0e-6

# Jeden profil = jedna sesja XFOIL
cases = [
    # thickness [%], chord [m]
    (24, 0.0350),
    (28, 0.0372),
    (31, 0.0390),
    (34, 0.0416),
    (38, 0.0449),
    (43, 0.0500),
]

# Celowo od największego Re do najmniejszego
velocities = [4.0, 3.5, 3.0, 2.5, 2.0]

POLAR_DIR = "xfoil_polars"
os.makedirs(POLAR_DIR, exist_ok=True)


def read_polar_file(filepath):

    rows = []

    with open(filepath, "r") as f:
        for line in f:

            parts = line.split()

            # Standardowy wiersz polar:
            # alpha CL CD CDp CM Top_Xtr Bot_Xtr
            if len(parts) < 7:
                continue

            try:
                alpha = float(parts[0])
                cl = float(parts[1])
                cd = float(parts[2])
                cdp = float(parts[3])
                cm = float(parts[4])
                top_xtr = float(parts[5])
                bot_xtr = float(parts[6])

            except ValueError:
                continue

            rows.append({
                "alpha": alpha,
                "CL": cl,
                "CD": cd,
                "CDp": cdp,
                "CM": cm,
                "Top_Xtr": top_xtr,
                "Bot_Xtr": bot_xtr,
            })

    return rows


def run_xfoil(thickness, chord):

    naca = f"00{thickness:02d}"

    reynolds = [
        velocity * chord / NU
        for velocity in velocities
    ]

    print("\n" + "=" * 70)
    print(f"NACA {naca}")
    print("=" * 70)

    polar_file = os.path.join(
        POLAR_DIR,
        f"polar_{naca}.txt"
    )

    # XFOIL nie nadpisuje poprawnie istniejącego polar file,
    # więc usuwamy stary przed każdym runem.
    if os.path.exists(polar_file):
        os.remove(polar_file)

    commands = f"""NACA {naca}
PANE
OPER
VISC {reynolds[0]:.0f}
ITER 300
PACC
{polar_file}

ALFA 0
"""

    # Kolejne Reynolds numbers w tej samej sesji XFOIL.
    for Re in reynolds[1:]:
        commands += f"""RE {Re:.0f}
ALFA 0
"""

    # Wyłącz PACC i zakończ XFOIL.
    commands += """PACC

QUIT
"""

    process = subprocess.Popen(
        [XFOIL_PATH],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    try:
        stdout, _ = process.communicate(
            input=commands,
            timeout=TIMEOUT
        )

    except subprocess.TimeoutExpired:

        process.kill()
        stdout, _ = process.communicate()

        print(
            f"NACA {naca}: TIMEOUT after {TIMEOUT} s"
        )

        print("\nXFOIL OUTPUT:")
        print(stdout)

        return None

    if not os.path.exists(polar_file):

        print(
            f"NACA {naca}: polar file was not created"
        )

        print("\nXFOIL OUTPUT:")
        print(stdout)

        return None

    polar_rows = read_polar_file(polar_file)

    expected = len(velocities)

    if len(polar_rows) != expected:

        print(
            f"NACA {naca}: expected {expected} polar points, "
            f"got {len(polar_rows)}"
        )

        print("\nPolar file:")
        with open(polar_file, "r") as f:
            print(f.read())

        print("\nXFOIL OUTPUT:")
        print(stdout)

        return None

    profile_results = []

    for velocity, Re, row in zip(
        velocities,
        reynolds,
        polar_rows
    ):

        cl = row["CL"]
        cd = row["CD"]

        valid = True
        reasons = []

        # Symetryczny NACA00xx przy alpha = 0 powinien mieć CL ~ 0.
        if abs(cl) > 0.01:
            valid = False
            reasons.append(
                f"|CL|={abs(cl):.5f} > 0.01"
            )

        # Dodatkowy sanity check.
        if cd <= 0.0 or cd > 0.2:
            valid = False
            reasons.append(
                f"suspicious CD={cd:.5f}"
            )

        profile_results.append({
            "thickness": thickness,
            "chord": chord,
            "velocity": velocity,
            "Re": Re,
            "alpha": row["alpha"],
            "CD": cd,
            "CL": cl,
            "CDp": row["CDp"],
            "CM": row["CM"],
            "Top_Xtr": row["Top_Xtr"],
            "Bot_Xtr": row["Bot_Xtr"],
            "valid": valid,
            "reason": ", ".join(reasons)
        })

    return profile_results


# =============================================================
# MAIN
# =============================================================

all_results = []

for thickness, chord in cases:

    results = run_xfoil(
        thickness,
        chord
    )

    if results is None:
        continue

    all_results.extend(results)

    for result in results:

        status = "OK"

        if not result["valid"]:
            status = f"INVALID ({result['reason']})"

        print(
            f"NACA 00{result['thickness']:02d} | "
            f"V={result['velocity']:.1f} m/s | "
            f"Re={result['Re']:.0f} | "
            f"CD={result['CD']:.5f} | "
            f"CL={result['CL']:.5f} | "
            f"{status}"
        )


# =============================================================
# FINAL SUMMARY
# =============================================================

print("\n")
print("=" * 100)
print("FINAL RESULTS")
print("=" * 100)

print(
    f"{'NACA':<10}"
    f"{'V [m/s]':>10}"
    f"{'Re':>12}"
    f"{'CD':>12}"
    f"{'CL':>12}"
    f"{'STATUS':>15}"
)

for result in all_results:

    status = "OK" if result["valid"] else "INVALID"
    
    print(
        f"{f'00{result['thickness']:02d}':<10}"
        f"{result['velocity']:>10.1f}"
        f"{result['Re']:>12.0f}"
        f"{result['CD']:>12.5f}"
        f"{result['CL']:>12.5f}"
        f"{status:>15}"
    )