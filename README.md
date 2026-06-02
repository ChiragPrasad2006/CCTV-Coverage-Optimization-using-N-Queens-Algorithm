# Smart CCTV Coverage Optimization (Java Swing)

This is a Java Swing implementation of Smart CCTV coverage optimization using N-Queens-style constraints.

## Core Model

- 1 grid cell = 1 square meter (1 m^2)
- `X`/gray cells = obstacles (walls)
- Cameras are mounted on wall/obstacle cells only
- Camera arrows show the exact facing/tilt direction into the free space
- Cameras cannot interfere in direct line-of-sight (obstacle-aware)
- Neighboring wall-mounted cameras are treated as conflicts, so cameras do not bunch up side by side
- Coverage rays stop at walls

## Camera Types

- `360deg PTZ` (8 directions)
- `Dome 180deg` (5 directions)
- `Bullet 90deg` (3 directions)

Range is configurable in meters (`1 meter = 1 cell`).

## Features

- Java Swing GUI
- Click to add/remove obstacles
- `MAX_COVERAGE`, `LEAST_CAMERAS`, and `MAX_CAMERAS` modes
- In `LEAST_CAMERAS`, the solver maximizes coverage, then uses fewer cameras, then prefers lower overlap
- Coverage %, covered area, overlap cells, blind spots, runtime, explored states
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
