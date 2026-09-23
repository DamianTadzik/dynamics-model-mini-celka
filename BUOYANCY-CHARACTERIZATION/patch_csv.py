"""Patch buoyancy_results_heave_pitch.csv with corrected rows from wtf.csv.

Matches rows by (pitch_rad, heave_com_m) rounded to avoid float noise:
- if a wtf.csv row matches an existing (pitch, heave) pair, it replaces it.
- if it doesn't match anything, it's appended as a new row.
"""
import csv

MAIN_CSV = "buoyancy_results_heave_pitch.csv"
PATCH_CSV = "wtf.csv"
ROUND_DECIMALS = 6


def key(row):
    return (round(float(row["pitch_rad"]), ROUND_DECIMALS), round(float(row["heave_com_m"]), ROUND_DECIMALS))


with open(MAIN_CSV, newline="") as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    main_rows = list(reader)

with open(PATCH_CSV, newline="") as f:
    patch_rows = list(csv.DictReader(f))

index = {key(row): i for i, row in enumerate(main_rows)}

n_replaced = 0
n_appended = 0
for row in patch_rows:
    k = key(row)
    if k in index:
        main_rows[index[k]] = row
        n_replaced += 1
    else:
        main_rows.append(row)
        n_appended += 1

with open(MAIN_CSV, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(main_rows)

print(f"[OK] Replaced {n_replaced} rows, appended {n_appended} new rows -> {MAIN_CSV}")
