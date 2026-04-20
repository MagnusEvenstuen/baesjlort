import subprocess
import re
import os

#This entire file is written by AI

# ------------------------------------------------------------
# Funksjon for å trekke ut nanosekund-tidsstempel fra filnavn
# ------------------------------------------------------------
def extract_timestamp_ns(filepath):
    """Returnerer tidsstempel i nanosekunder fra filnavn."""
    basename = os.path.basename(filepath)
    match = re.search(r'_(\d+)\.mp4$', basename)
    if match:
        return int(match.group(1))
    else:
        raise ValueError(f"Kunne ikke finne tidsstempel i {filepath}")

# ------------------------------------------------------------
# Konfigurasjon
# ------------------------------------------------------------
left_input = "/home/gud/Downloads/wetransfer_noe_2026-03-25_0803/gbr_cam_left_image_raw_1774374140039571928.mp4"
right_input = "/home/gud/Downloads/wetransfer_noe_2026-03-25_0803/gbr_cam_right_image_raw_1774374140047241766.mp4"

# Ønsket segment i venstre video (i sekunder fra starten av venstre video)
start_seconds = (20*60 + 50)*2
end_seconds   = (21*60 + 15)*2
duration = end_seconds - start_seconds

# Midlertidige filer
left_slow = "left_slow3.mp4"
right_slow = "right_slow3.mp4"
final_output = "synced_slow3.mp4"

# ------------------------------------------------------------
# Hent absolutte starttider (i sekunder)
# ------------------------------------------------------------
left_start_ns = extract_timestamp_ns(left_input)
right_start_ns = extract_timestamp_ns(right_input)

start_diff = ((left_start_ns - right_start_ns)/1_000_000_000.0)*2

print("Start diff", start_diff)

left_start_sec = left_start_ns / 1_000_000_000.0
right_start_sec = right_start_ns / 1_000_000_000.0

# Beregn absolutt tid for ønsket start i venstre video
abs_start_sec = left_start_sec + start_seconds

# Finn tilsvarende offset i høyre video
right_offset = (abs_start_sec - right_start_sec) - 4.5

if right_offset < 0:
    raise ValueError(f"Ønsket start er før høyre video begynner (offset={right_offset} s).")

print(f"Venstre starttid (absolutt): {left_start_sec:.6f} s")
print(f"Høyre starttid (absolutt):   {right_start_sec:.6f} s")
print(f"Ønsket absolutt start:       {abs_start_sec:.6f} s")
print(f"Høyre offset:                {right_offset:.6f} s")
print(f"Varighet (original):         {duration} s")

# ------------------------------------------------------------
# Hjelpefunksjon for å kjøre ffmpeg
# ------------------------------------------------------------
def run_ffmpeg(cmd):
    print("Kjører:", " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("FEIL fra ffmpeg:")
        print(result.stderr)
        raise RuntimeError(f"ffmpeg feilet med kode {result.returncode}")
    return result

# ------------------------------------------------------------
# 1. Klipp venstre video og halver hastigheten
# ------------------------------------------------------------
run_ffmpeg([
    "ffmpeg", "-y",
    "-i", left_input,
    "-ss", str(start_seconds),
    "-t", str(duration),
    "-filter:v", "setpts=2.0*PTS",
    "-an",
    "-c:v", "libx264",
    "-preset", "fast",
    "-crf", "23",
    left_slow
])

# ------------------------------------------------------------
# 2. Klipp høyre video med beregnet offset og halver hastigheten
# ------------------------------------------------------------
run_ffmpeg([
    "ffmpeg", "-y",
    "-i", right_input,
    "-ss", str(right_offset),
    "-t", str(duration),
    "-filter:v", "setpts=2.0*PTS",
    "-an",
    "-c:v", "libx264",
    "-preset", "fast",
    "-crf", "23",
    right_slow
])

# ------------------------------------------------------------
# 3. Kombiner side om side
# ------------------------------------------------------------
run_ffmpeg([
    "ffmpeg", "-y",
    "-i", left_slow,
    "-i", right_slow,
    "-filter_complex", "[0:v][1:v]hstack=inputs=2[v]",
    "-map", "[v]",
    "-c:v", "libx264",
    "-preset", "fast",
    "-crf", "23",
    final_output
])

print(f"Ferdig! Synkronisert video med halv hastighet lagret som {final_output}")