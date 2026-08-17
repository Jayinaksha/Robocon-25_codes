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

## Adding the match clips

The footage page ships with the players hidden and placeholders showing. Drop the files in and
they swap themselves in automatically — no code change needed:

| File | Source in Drive |
| --- | --- |
| `assets/media/robots-in-action.mp4` | “DD Robocon25 Robots in Action” |
| `assets/media/shooting-mechanism.mp4` | “Team Pheonix shooting mechanism” |

Optional poster frames: `assets/img/poster-action.jpg`, `assets/img/poster-shoot.jpg`.

Keep them web-sized — H.264, 720p, a few MB each. Anything committed here is public, because
the repository is public.

## Editing annotations

Each clip carries a timed track as inline JSON inside its `<figure>`. Coordinates are
percentages of the frame, so they survive any resize:

```json
{ "start": 4, "end": 9, "label": "Approach",
  "note": "Closing on the hoop…",
  "boxes": [ { "x": 44, "y": 22, "w": 18, "h": 20, "label": "hoop", "colour": "cyan" } ] }
```

`colour` accepts `cyan`, `green`, `violet`, or omit it for the default orange.

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
