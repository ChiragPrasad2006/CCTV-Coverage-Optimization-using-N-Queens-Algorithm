import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;

public class Main {
    private static final int FREE = 0;
    private static final int WALL = 1;
    private static final int CELL_AREA_M2 = 1;

    enum Mode {
        MAX_COVERAGE, LEAST_CAMERAS, MAX_CAMERAS
    }

    enum CameraType {
        BULLET_90("Bullet 90deg", 90, new Color(0, 119, 182)),
        DOME_180("Dome 180deg", 180, new Color(247, 127, 0)),
        PTZ_360("PTZ 360deg", 360, new Color(217, 4, 41));

        final String name;
        final int fov;
        final Color color;

        CameraType(String name, int fov, Color color) {
            this.name = name;
            this.fov = fov;
            this.color = color;
        }

        @Override
        public String toString() { return name; }
    }

    static class Cell {
        final int r, c;
        Cell(int r, int c) { this.r = r; this.c = c; }
        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            Cell cell = (Cell) o;
            return r == cell.r && c == cell.c;
        }
        @Override
        public int hashCode() { return r * 31 + c; }
    }

    enum Direction {
        N(-1, 0), S(1, 0), E(0, 1), W(0, -1);
        final int dr, dc;
        Direction(int dr, int dc) { this.dr = dr; this.dc = dc; }
    }

    static class MountPoint {
        final Cell cell;
        final Direction dir;
        MountPoint(Cell cell, Direction dir) {
            this.cell = cell;
            this.dir = dir;
        }
        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof MountPoint)) return false;
            MountPoint m = (MountPoint) o;
            return cell.equals(m.cell) && dir == m.dir;
        }
        @Override
        public int hashCode() {
            return Objects.hash(cell, dir);
        }
    }

    static class SolveResult {
        Set<MountPoint> cameras;
        Set<Cell> covered;
        int totalFree;
        double coveragePct;
        int coveredArea;
        int freeArea;
        int overlapCells;
        long states;
        double seconds;
    }

    static class CCTVOptimizer {
        private final int[][] grid;
        private final int n;
        private final Mode mode;
        private final CameraType type;
        private final int range;

        private List<MountPoint> mountCells;
        private List<Cell> targetCells;
        private int totalTargets;
        private int totalMounts;
        private int[][] coverages;
        private int[][] conflicts;
        
        static class SharedState {
            volatile int bestCameras = -1;
            volatile int bestCovered = -1;
            volatile int bestOverlap = Integer.MAX_VALUE;
            boolean[] bestSelected;
            AtomicLong totalStates = new AtomicLong();
            final Mode mode;
            final int totalTargets;
            
            SharedState(Mode mode, int totalTargets, int M) {
                this.mode = mode;
                this.totalTargets = totalTargets;
                this.bestSelected = new boolean[M];
            }
            
            synchronized void update(int selectedCount, int coveredCount, int overlapCount, boolean[] selected) {
                boolean better = false;
                if (mode == Mode.MAX_CAMERAS) {
                    if (bestCameras == -1 || selectedCount > bestCameras) better = true;
                    else if (selectedCount == bestCameras && coveredCount > bestCovered) better = true;
                } else if (mode == Mode.LEAST_CAMERAS) {
                    if (bestCovered == -1 || coveredCount > bestCovered) better = true;
                    else if (coveredCount == bestCovered && (bestCameras == -1 || selectedCount < bestCameras)) better = true;
                    else if (coveredCount == bestCovered && selectedCount == bestCameras && overlapCount < bestOverlap) better = true;
                } else {
                    if (bestCovered == -1 || coveredCount > bestCovered) better = true;
                }
                
                if (better) {
                    bestCameras = selectedCount;
                    bestCovered = coveredCount;
                    bestOverlap = overlapCount;
                    System.arraycopy(selected, 0, bestSelected, 0, selected.length);
                }
            }
            
            boolean prune(int idx, int selectedCount, int coveredCount, int M, int[] maxRem) {
                int remaining = M - idx;
                int bestCam = this.bestCameras;
                int bestCov = this.bestCovered;
                
                if (mode == Mode.MAX_CAMERAS) {
                    if (bestCam != -1 && selectedCount + remaining < bestCam) return true;
                    if (bestCam != -1 && selectedCount + remaining == bestCam) {
                        if (coveredCount + maxRem[idx] <= bestCov) return true;
                    }
                    return false;
                } else if (mode == Mode.LEAST_CAMERAS) {
                    if (bestCov != -1) {
                        if (coveredCount + maxRem[idx] < bestCov) return true;
                        if (coveredCount + maxRem[idx] == bestCov && bestCam != -1 && selectedCount > bestCam) return true;
                    }
                    return false;
                } else {
                    if (bestCov == totalTargets) return true;
                    if (bestCov != -1 && coveredCount + maxRem[idx] <= bestCov) return true;
                    return false;
                }
            }
        }

        CCTVOptimizer(int[][] grid, Mode mode, CameraType type, int range) {
            this.grid = grid;
            this.n = grid.length;
            this.mode = mode;
            this.type = type;
            this.range = Math.max(1, range);
            prepare();
        }

        private boolean isWithinFOV(int r0, int c0, int r1, int c1, Direction dir, int fov) {
            if (r0 == r1 && c0 == c1) return false;
            double vr = r1 - r0;
            double vc = c1 - c0;
            double dot = dir.dr * vr + dir.dc * vc;
            double mag = Math.sqrt(vr * vr + vc * vc);
            if (mag == 0) return false;
            double cosTheta = dot / mag;
            if (cosTheta > 1.0) cosTheta = 1.0;
            if (cosTheta < -1.0) cosTheta = -1.0;
            double angleDeg = Math.toDegrees(Math.acos(cosTheta));
            return angleDeg <= fov / 2.0 + 0.1;
        }

        private boolean hasLineOfSight(int r0, int c0, int r1, int c1, Set<Cell> walls) {
            int dr = Math.abs(r1 - r0);
            int dc = Math.abs(c1 - c0);
            int stepR = r0 < r1 ? 1 : -1;
            int stepC = c0 < c1 ? 1 : -1;
            int err = dr - dc;

            int currR = r0, currC = c0;
            while (true) {
                if (currR == r1 && currC == c1) break;
                if ((currR != r0 || currC != c0) && walls.contains(new Cell(currR, currC))) return false;
                
                int e2 = 2 * err;
                if (e2 > -dc) {
                    err -= dc;
                    currR += stepR;
                }
                if (e2 < dr) {
                    err += dr;
                    currC += stepC;
                }
            }
            return true;
        }

        private void prepare() {
            mountCells = new ArrayList<>();
            targetCells = new ArrayList<>();
            Set<Cell> wallSet = new HashSet<>();
            Set<Cell> targetSet = new HashSet<>();
            
            for (int r = 0; r < n; r++) {
                for (int c = 0; c < n; c++) {
                    if (grid[r][c] == WALL) wallSet.add(new Cell(r, c));
                    else if (grid[r][c] == FREE) {
                        Cell f = new Cell(r, c);
                        targetCells.add(f);
                        targetSet.add(f);
                    }
                }
            }
            
            for (Cell wall : wallSet) {
                for (Direction dir : Direction.values()) {
                    Cell adj = new Cell(wall.r + dir.dr, wall.c + dir.dc);
                    if (targetSet.contains(adj)) {
                        mountCells.add(new MountPoint(wall, dir));
                    }
                }
            }
            
            totalMounts = mountCells.size();
            totalTargets = targetCells.size();
            
            Set<MountPoint> mountSet = new HashSet<>(mountCells);
            Map<Cell, Integer> targetToId = new HashMap<>();
            for (int i = 0; i < totalTargets; i++) targetToId.put(targetCells.get(i), i);
            
            class MountData {
                MountPoint mp;
                Set<Cell> covered = new HashSet<>();
                Set<MountPoint> conflicts = new HashSet<>();
            }
            List<MountData> mounts = new ArrayList<>();
            for (MountPoint mp : mountCells) {
                MountData md = new MountData();
                md.mp = mp;
                
                for (Direction dir : Direction.values()) {
                    if (dir != mp.dir) {
                        MountPoint confMp = new MountPoint(mp.cell, dir);
                        if (mountSet.contains(confMp)) md.conflicts.add(confMp);
                    }
                }

                for (MountPoint otherMp : mountCells) {
                    if (otherMp.equals(mp)) continue;
                    int rowGap = Math.abs(otherMp.cell.r - mp.cell.r);
                    int colGap = Math.abs(otherMp.cell.c - mp.cell.c);
                    if (Math.max(rowGap, colGap) <= 1) {
                        md.conflicts.add(otherMp);
                    }
                }
                
                for (Cell target : targetSet) {
                    double dist = Math.sqrt(Math.pow(target.r - mp.cell.r, 2) + Math.pow(target.c - mp.cell.c, 2));
                    if (dist <= range && dist > 0) {
                        if (isWithinFOV(mp.cell.r, mp.cell.c, target.r, target.c, mp.dir, type.fov)) {
                            if (hasLineOfSight(mp.cell.r, mp.cell.c, target.r, target.c, wallSet)) {
                                md.covered.add(target);
                            }
                        }
                    }
                }
                
                for (MountPoint otherMp : mountCells) {
                    if (otherMp.cell.equals(mp.cell)) continue;
                    if (isWithinFOV(mp.cell.r, mp.cell.c, otherMp.cell.r, otherMp.cell.c, mp.dir, type.fov)) {
                        if (hasLineOfSight(mp.cell.r, mp.cell.c, otherMp.cell.r, otherMp.cell.c, wallSet)) {
                            md.conflicts.add(otherMp);
                        }
                    }
                }
                mounts.add(md);
            }
            mounts.sort((a, b) -> Integer.compare(b.covered.size(), a.covered.size()));
            mountCells.clear();
            for (MountData md : mounts) mountCells.add(md.mp);
            Map<MountPoint, Integer> mountToId = new HashMap<>();
            for (int i = 0; i < totalMounts; i++) mountToId.put(mountCells.get(i), i);
            coverages = new int[totalMounts][];
            conflicts = new int[totalMounts][];
            for (int i = 0; i < totalMounts; i++) {
                MountData md = mounts.get(i);
                coverages[i] = md.covered.stream().mapToInt(targetToId::get).toArray();
                conflicts[i] = md.conflicts.stream().mapToInt(mountToId::get).toArray();
            }
        }

        SolveResult solve() {
            long start = System.nanoTime();
            int M = totalMounts;
            int T = totalTargets;
            SharedState shared = new SharedState(mode, T, M);
            if (M > 0) {
                int[] maxRem = new int[M + 1];
                Set<Integer> union = new HashSet<>();
                for (int i = M - 1; i >= 0; i--) {
                    for (int cId : coverages[i]) union.add(cId);
                    maxRem[i] = union.size();
                }
                int cores = Runtime.getRuntime().availableProcessors();
                ExecutorService executor = Executors.newFixedThreadPool(cores);
                List<Callable<Void>> tasks = new ArrayList<>();
                generateTasks(0, new boolean[M], new int[T], new int[M], 0, 0, 0, Math.min(M, 10), tasks, shared, maxRem);
                try { executor.invokeAll(tasks); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
                executor.shutdown();
            }
            double seconds = (System.nanoTime() - start) / 1_000_000_000.0;
            SolveResult res = new SolveResult();
            res.cameras = new HashSet<>();
            res.covered = new HashSet<>();
            if (shared.bestSelected != null) {
                for (int i = 0; i < M; i++) {
                    if (shared.bestSelected[i]) {
                        res.cameras.add(mountCells.get(i));
                        for (int cId : coverages[i]) res.covered.add(targetCells.get(cId));
                    }
                }
            }
            res.totalFree = T;
            res.coveragePct = T == 0 ? 0.0 : (res.covered.size() * 100.0 / T);
            res.coveredArea = res.covered.size() * CELL_AREA_M2;
            res.freeArea = T * CELL_AREA_M2;
            res.overlapCells = shared.bestOverlap == Integer.MAX_VALUE ? 0 : shared.bestOverlap;
            res.states = shared.totalStates.get();
            res.seconds = seconds;
            return res;
        }

        private void generateTasks(int idx, boolean[] selected, int[] coverCount, int[] unsafeCount, int selectedCount, int coveredCount, int overlapCount, int depthLimit, List<Callable<Void>> tasks, SharedState shared, int[] maxRem) {
            if (shared.prune(idx, selectedCount, coveredCount, totalMounts, maxRem)) return;
            if (coveredCount == shared.totalTargets) { shared.update(selectedCount, coveredCount, overlapCount, selected); if (shared.mode != Mode.MAX_CAMERAS) return; }
            if (idx == depthLimit) {
                tasks.add(new SolverTask(idx, selected.clone(), coverCount.clone(), unsafeCount.clone(), selectedCount, coveredCount, overlapCount, shared, maxRem));
                return;
            }
            if (unsafeCount[idx] == 0) {
                selected[idx] = true;
                int newCovered = coveredCount;
                int newOverlap = overlapCount;
                for (int cId : coverages[idx]) {
                    if (coverCount[cId]++ == 0) newCovered++;
                    else newOverlap++;
                }
                for (int cId : conflicts[idx]) unsafeCount[cId]++;
                generateTasks(idx + 1, selected, coverCount, unsafeCount, selectedCount + 1, newCovered, newOverlap, depthLimit, tasks, shared, maxRem);
                selected[idx] = false;
                for (int cId : coverages[idx]) coverCount[cId]--;
                for (int cId : conflicts[idx]) unsafeCount[cId]--;
            }
            generateTasks(idx + 1, selected, coverCount, unsafeCount, selectedCount, coveredCount, overlapCount, depthLimit, tasks, shared, maxRem);
        }
        
        class SolverTask implements Callable<Void> {
            int startIdx, selectedCount, coveredCount, overlapCount, M;
            boolean[] selected;
            int[] coverCount, unsafeCount, maxRem;
            SharedState shared;
            long states = 0;
            SolverTask(int idx, boolean[] sel, int[] cc, int[] uc, int sc, int cv, int ov, SharedState sh, int[] mr) {
                startIdx = idx; selected = sel; coverCount = cc; unsafeCount = uc; selectedCount = sc; coveredCount = cv; overlapCount = ov; shared = sh; maxRem = mr; M = sel.length;
            }
            public Void call() { backtrack(startIdx); shared.totalStates.addAndGet(states); return null; }
            void backtrack(int idx) {
                states++;
                if (shared.prune(idx, selectedCount, coveredCount, M, maxRem)) return;
                if (coveredCount == shared.totalTargets) { shared.update(selectedCount, coveredCount, overlapCount, selected); if (shared.mode != Mode.MAX_CAMERAS) return; }
                if (idx == M) { shared.update(selectedCount, coveredCount, overlapCount, selected); return; }
                if (unsafeCount[idx] == 0) {
                    selected[idx] = true;
                    int prevCovered = coveredCount;
                    int prevOverlap = overlapCount;
                    for (int cId : coverages[idx]) {
                        if (coverCount[cId]++ == 0) coveredCount++;
                        else overlapCount++;
                    }
                    for (int cId : conflicts[idx]) unsafeCount[cId]++;
                    backtrack(idx + 1);
                    selected[idx] = false;
                    coveredCount = prevCovered;
                    overlapCount = prevOverlap;
                    for (int cId : coverages[idx]) coverCount[cId]--;
                    for (int cId : conflicts[idx]) unsafeCount[cId]--;
                }
                backtrack(idx + 1);
            }
        }
    }

    static class GridPanel extends JPanel {
        int[][] grid;
        Set<MountPoint> cameras = new HashSet<>();
        Set<Cell> covered = new HashSet<>();
        int tileSize = 20;
        CameraType cameraType = CameraType.values()[0];

        GridPanel() {
            addMouseListener(new MouseAdapter() {
                @Override
                public void mouseClicked(MouseEvent e) {
                    if (grid == null) return;
                    int c = e.getX() / tileSize, r = e.getY() / tileSize;
                    if (r >= 0 && c >= 0 && r < grid.length && c < grid.length) {
                        grid[r][c] = (grid[r][c] == FREE) ? WALL : FREE;
                        cameras.clear(); covered.clear(); repaint();
                    }
                }
            });
        }
        void setGrid(int[][] grid) { this.grid = grid; cameras.clear(); covered.clear(); revalidate(); repaint(); }
        void setTileSize(int tileSize) { this.tileSize = Math.max(8, tileSize); revalidate(); repaint(); }
        void setSolution(Set<MountPoint> cameras, Set<Cell> covered, CameraType type) { this.cameras = cameras; this.covered = covered; this.cameraType = type; repaint(); }
        @Override public Dimension getPreferredSize() { int n = grid == null ? 10 : grid.length; return new Dimension(n * tileSize, n * tileSize); }
        private void drawMountedCamera(Graphics2D g2, int x, int y, Direction dir) {
            int centerX = x + tileSize / 2;
            int centerY = y + tileSize / 2;
            int edgePadding = Math.max(2, tileSize / 8);
            int mountX = centerX + dir.dc * (tileSize / 2 - edgePadding);
            int mountY = centerY + dir.dr * (tileSize / 2 - edgePadding);
            int arrowLen = Math.max(7, tileSize / 2);
            int tipX = mountX + dir.dc * arrowLen;
            int tipY = mountY + dir.dr * arrowLen;

            g2.setStroke(new BasicStroke(Math.max(2, tileSize / 8), BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.setColor(cameraType.color);
            g2.fillOval(mountX - 4, mountY - 4, 8, 8);
            g2.drawLine(mountX, mountY, tipX, tipY);

            int sideX = -dir.dr;
            int sideY = dir.dc;
            int wing = Math.max(4, tileSize / 5);
            int backX = tipX - dir.dc * wing;
            int backY = tipY - dir.dr * wing;
            Polygon head = new Polygon();
            head.addPoint(tipX, tipY);
            head.addPoint(backX + sideX * wing / 2, backY + sideY * wing / 2);
            head.addPoint(backX - sideX * wing / 2, backY - sideY * wing / 2);
            g2.fillPolygon(head);

            g2.setStroke(new BasicStroke(1));
        }

        @Override protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            if (grid == null) return;
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            int n = grid.length;
            for (int r = 0; r < n; r++) {
                for (int c = 0; c < n; c++) {
                    int x = c * tileSize, y = r * tileSize;
                    g2.setColor(grid[r][c] == WALL ? new Color(140, 140, 140) : (covered.contains(new Cell(r, c)) ? new Color(188, 245, 184) : Color.WHITE));
                    g2.fillRect(x, y, tileSize, tileSize);
                    g2.setColor(new Color(70, 70, 70));
                    g2.setStroke(new BasicStroke(1.2f));
                    g2.drawRect(x, y, tileSize, tileSize);
                    if (grid[r][c] == WALL) {
                        if (tileSize >= 14) {
                            g2.setColor(Color.BLACK); g2.drawString("X", x + 5, y + 15);
                        }
                    }
                }
            }

            // Draw cameras after cells so arrows are not covered by neighboring tiles.
            for (MountPoint camera : cameras) {
                int x = camera.cell.c * tileSize;
                int y = camera.cell.r * tileSize;
                drawMountedCamera(g2, x, y, camera.dir);
            }
        }
    }

    static class AppFrame extends JFrame {
        private int[][] grid;

        private final JSpinner nSpinner = new JSpinner(new SpinnerNumberModel(12, 4, 40, 1));
        private final JSpinner tileSpinner = new JSpinner(new SpinnerNumberModel(20, 8, 40, 1));
        private final JComboBox<Mode> modeBox = new JComboBox<>(Mode.values());
        private final JComboBox<CameraType> typeBox = new JComboBox<>(CameraType.values());
        private final JSpinner rangeSpinner = new JSpinner(new SpinnerNumberModel(6, 1, 20, 1));

        private final JButton createBtn = new JButton("Create Grid");
        private final JButton clearBtn = new JButton("Clear Obstacles");
        private final JButton solveBtn = new JButton("Solve");
        private final JProgressBar progressBar = new JProgressBar();

        private final JLabel status = new JLabel("Set grid (1 cell = 1 m^2), add obstacles, choose camera type and solve.");
        private final GridPanel gridPanel = new GridPanel();

        AppFrame() {
            super("Smart CCTV Coverage Optimization (Java Swing)");
            setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            setLayout(new BorderLayout());

            JPanel controls = new JPanel(new FlowLayout(FlowLayout.LEFT));
            controls.add(new JLabel("Grid N:"));
            controls.add(nSpinner);
            controls.add(new JLabel("Tile px:"));
            controls.add(tileSpinner);
            controls.add(new JLabel("Mode:"));
            controls.add(modeBox);
            controls.add(new JLabel("Camera:"));
            controls.add(typeBox);
            controls.add(new JLabel("Range(m):"));
            controls.add(rangeSpinner);

            controls.add(createBtn);
            controls.add(clearBtn);
            controls.add(solveBtn);

            add(controls, BorderLayout.NORTH);

            JScrollPane scrollPane = new JScrollPane(gridPanel);
            add(scrollPane, BorderLayout.CENTER);
            
            progressBar.setIndeterminate(true);
            progressBar.setVisible(false);

            JPanel bottom = new JPanel(new BorderLayout());
            bottom.add(new JLabel("Legend: White=Free(1m^2), Gray=Wall/Obstacle, Green=Covered, Arrow=Wall-mounted camera facing direction"), BorderLayout.NORTH);
            
            JPanel statusPanel = new JPanel(new BorderLayout());
            statusPanel.add(status, BorderLayout.CENTER);
            statusPanel.add(progressBar, BorderLayout.EAST);
            bottom.add(statusPanel, BorderLayout.SOUTH);
            add(bottom, BorderLayout.SOUTH);

            createBtn.addActionListener(e -> createGrid());
            clearBtn.addActionListener(e -> clearObstacles());
            solveBtn.addActionListener(e -> solve());

            tileSpinner.addChangeListener(e -> {
                gridPanel.setTileSize((Integer) tileSpinner.getValue());
            });

            loadDefaultLayout();
            
            setSize(1200, 800);
            setLocationRelativeTo(null);
        }

        private void loadDefaultLayout() {
            int n = (Integer) nSpinner.getValue();
            grid = new int[n][n];
            for (int r = 0; r < n; r++) {
                for (int c = 0; c < n; c++) {
                    if (r == 0 || r == n - 1 || c == 0 || c == n - 1) {
                        grid[r][c] = WALL;
                    }
                }
            }
            int r = n / 2;
            for (int c = 3; c < n - 3; c++) {
                grid[r][c] = WALL;
            }
            for (int r2 = 3; r2 < n - 3; r2++) {
                grid[r2][n / 2] = WALL;
            }
            gridPanel.setGrid(grid);
            gridPanel.setTileSize((Integer) tileSpinner.getValue());
            status.setText("Loaded default layout.");
        }

        private void createGrid() {
            int n = (Integer) nSpinner.getValue();
            grid = new int[n][n];
            for (int r = 0; r < n; r++) {
                for (int c = 0; c < n; c++) {
                    if (r == 0 || r == n - 1 || c == 0 || c == n - 1) {
                        grid[r][c] = WALL;
                    }
                }
            }
            gridPanel.setGrid(grid);
            gridPanel.setTileSize((Integer) tileSpinner.getValue());
            status.setText("Grid created: " + n + "x" + n + " (" + (n * n) + " m^2 total). Borders set as walls.");
        }

        private void clearObstacles() {
            if (grid == null) return;
            int n = grid.length;
            for (int r = 0; r < n; r++) {
                for (int c = 0; c < n; c++) {
                    if (r == 0 || r == n - 1 || c == 0 || c == n - 1) {
                        grid[r][c] = WALL;
                    } else {
                        grid[r][c] = FREE;
                    }
                }
            }
            gridPanel.setSolution(new HashSet<>(), new HashSet<>(), (CameraType) typeBox.getSelectedItem());
            gridPanel.repaint();
            status.setText("Obstacles cleared (borders kept as walls).");
        }

        private void solve() {
            if (grid == null) return;
            int freeCount = 0;
            int wallCount = 0;
            for (int[] row : grid) {
                for (int cell : row) {
                    if (cell == FREE) freeCount++;
                    else if (cell == WALL) wallCount++;
                }
            }
            if (freeCount == 0) {
                JOptionPane.showMessageDialog(this, "No free cells available to cover.", "Warning", JOptionPane.WARNING_MESSAGE);
                return;
            }
            if (wallCount == 0) {
                JOptionPane.showMessageDialog(this, "No walls available to mount cameras.", "Warning", JOptionPane.WARNING_MESSAGE);
                return;
            }

            Mode mode = (Mode) modeBox.getSelectedItem();
            CameraType type = (CameraType) typeBox.getSelectedItem();
            int range = (Integer) rangeSpinner.getValue();
            
            createBtn.setEnabled(false);
            clearBtn.setEnabled(false);
            solveBtn.setEnabled(false);
            progressBar.setVisible(true);
            status.setText("Solving... please wait.");

            SwingWorker<SolveResult, Void> worker = new SwingWorker<SolveResult, Void>() {
                @Override
                protected SolveResult doInBackground() {
                    CCTVOptimizer optimizer = new CCTVOptimizer(grid, mode, type, range);
                    return optimizer.solve();
                }

                @Override
                protected void done() {
                    try {
                        SolveResult res = get();
                        int blindSpots = res.totalFree - res.covered.size();
                        int blindArea = blindSpots * CELL_AREA_M2;

                        gridPanel.setSolution(res.cameras, res.covered, type);
                        status.setText(String.format(
                            "Type=%s (%dm) | Mode=%s | Cameras=%d | Coverage=%.2f%% (%dm^2/%dm^2) | Overlap=%d cells | Blind=%d (%dm^2) | States=%d | Time=%.4fs",
                            type.name, range, mode, res.cameras.size(), res.coveragePct, res.coveredArea, res.freeArea, res.overlapCells, blindSpots, blindArea, res.states, res.seconds
                        ));
                    } catch (Exception ex) {
                        status.setText("Error during solve: " + ex.getMessage());
                        ex.printStackTrace();
                    } finally {
                        createBtn.setEnabled(true);
                        clearBtn.setEnabled(true);
                        solveBtn.setEnabled(true);
                        progressBar.setVisible(false);
                    }
                }
            };
            worker.execute();
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new AppFrame().setVisible(true));
    }
}
