# Smart CCTV Coverage Optimization (Java Swing)

This is a Java Swing implementation of Smart CCTV coverage optimization using N-Queens-style constraints.

## Core Model

- 1 grid cell = 1 square meter (1 m^2)
- `X`/gray cells = obstacles (walls)
- Cameras cannot interfere in direct line-of-sight (obstacle-aware)
- Coverage rays stop at walls

## Camera Types

- `360deg PTZ` (8 directions)
- `Dome 180deg` (5 directions)
- `Bullet 90deg` (3 directions)

Range is configurable in meters (`1 meter = 1 cell`).

## Features

- Java Swing GUI
- Click to add/remove obstacles
- `max_coverage` and `max_cameras` modes
- Coverage %, covered area, blind spots, runtime, explored states
- Larger maps supported:
  - Grid size up to `40 x 40`
  - Tile pixel size adjustable (`8px` to `40px`)
  - Scrollable grid panel

## Build and Run

From project root:

```bash
javac -d out src/Main.java
java -cp out Main
```

## Notes on Bigger Grids

- Yes, the app now supports more tiles and smaller tile display for house-scale layouts.
- Backtracking is exponential in worst case; very large/open grids may be slower.
- For class/demo use, recommended range is `12x12` to `24x24` for fast responses.
