# Project site

Static site for the Robocon 25 basketball robot stack. No build step, no dependencies —
plain HTML, one CSS file and four small JS files.

## Publishing

GitHub Pages, served from this folder:

**Settings → Pages → Build and deployment → Deploy from a branch**, then pick the branch and
set the folder to **`/docs`**. The `.nojekyll` file stops Jekyll from stripping paths.

## Layout

```
docs/
  index.html          overview + system diagram
  architecture.html   dataflow, serial protocol, drive ladder, state machines, pin tables
  simulations.html    three interactive simulators
  code.html           module walkthrough + twelve findings
  media.html          detector HUD reconstruction + annotated clips
  assets/css/site.css design tokens, light and dark
  assets/js/          sim-drive · sim-pid · sim-shoot · media
  assets/media/       video files (see below)
  assets/img/         poster frames
```

## Media

Three clips ship with the site, transcoded to 720p in two formats — WebM/VP9 first, H.264 MP4 as
the fallback — because some Chromium builds have no H.264 decoder at all:

| File | Length | Source |
| --- | --- | --- |
| `assets/media/robots-in-action.*` | 22 s | “DD Robocon25 Robots in Action” |
| `assets/media/shooting-mechanism.*` | 23 s | “Team Pheonix shooting mechanism” |
| `assets/media/mechanism-detail.*` | 31 s | “Team Pheonix Mechanism-2” |

Poster frames sit in `assets/img/`. To replace a clip, keep the filenames and re-run:

```bash
ffmpeg -i in.mp4 -vf scale=-2:720 -c:v libx264 -crf 24 -movflags +faststart out.mp4
ffmpeg -i out.mp4 -c:v libvpx-vp9 -crf 36 -b:v 0 -c:a libopus out.webm
```

Note that video seeking needs HTTP Range support. GitHub Pages has it; Python's
`http.server` does not, so seeking will look broken under a naive local preview.

## Editing annotations

Each clip carries a timed track as inline JSON inside its `<figure>`. Coordinates are
percentages of the frame, so they survive any resize:

```json
{ "start": 4, "end": 9, "label": "Approach",
  "note": "Closing on the hoop…",
  "boxes": [ { "x": 44, "y": 22, "w": 18, "h": 20, "label": "hoop", "colour": "cyan" } ] }
```

`colour` accepts `cyan`, `green`, `violet`, or omit it for the default orange.
Add `"chapter": false` to keep a short cue out of the chapter button row — useful for a box that
tracks something moving fast, where the window has to be a fraction of a second to stay accurate.

Overlapping entries all render their boxes; the last matching one supplies the caption.

## A note on the simulators

`sim-drive.js` and `sim-pid.js` are ports of the committed firmware, including Arduino's
integer `map()` and the `pid()` axis mapping — if you change the firmware, change these to
match or the site will describe a robot you no longer have.

Two constants are **not** from the repository and are exposed as sliders: PWM → m/s in the
alignment sim, and PWM → exit velocity in the shooter sim. Calibrate them from a real run.

## Local preview

```bash
cd docs && python3 -m http.server 8000
```
