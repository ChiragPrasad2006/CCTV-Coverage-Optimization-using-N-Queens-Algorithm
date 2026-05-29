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

    static class CameraType {
        final String name;
        final int[][] directions;
        final int defaultRange;
        final Color color;

        CameraType(String name, int[][] directions, int defaultRange, Color color) {
            this.name = name;
            this.directions = directions;
            this.defaultRange = defaultRange;
            this.color = color;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    static final CameraType[] CAMERA_TYPES = new CameraType[] {
        new CameraType("360deg PTZ", new int[][] {
            {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
        }, 6, new Color(217, 4, 41)),
        new CameraType("Dome 180deg", new int[][] {
            {0,1},{1,1},{-1,1},{1,0},{-1,0}
        }, 5, new Color(247, 127, 0)),
        new CameraType("Bullet 90deg", new int[][] {
            {0,1},{1,1},{-1,1}
        }, 4, new Color(0, 119, 182))
    };

    static class Cell {
        final int r;
        final int c;

        Cell(int r, int c) {
            this.r = r;
            this.c = c;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Cell)) return false;
            Cell cell = (Cell) o;
            return r == cell.r && c == cell.c;
        }

        @Override
        public int hashCode() {
            return Objects.hash(r, c);
        }
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
            boolean[] bestSelected;
            AtomicLong totalStates = new AtomicLong();
            final Mode mode;
            final int totalTargets;
            
            SharedState(Mode mode, int totalTargets, int M) {
                this.mode = mode;
                this.totalTargets = totalTargets;
                this.bestSelected = new boolean[M];
            }
            
            synchronized void update(int selectedCount, int coveredCount, boolean[] selected) {
                boolean better = false;
                if (mode == Mode.MAX_CAMERAS) {
                    if (bestCameras == -1 || selectedCount > bestCameras) better = true;
                    else if (selectedCount == bestCameras && coveredCount > bestCovered) better = true;
                } else if (mode == Mode.LEAST_CAMERAS) {
                    if (bestCovered == -1 || coveredCount > bestCovered) better = true;
                    else if (coveredCount == bestCovered && (bestCameras == -1 || selectedCount < bestCameras)) better = true;
                } else {
                    if (bestCovered == -1 || coveredCount > bestCovered) better = true;
                }
                
                if (better) {
                    bestCameras = selectedCount;
                    bestCovered = coveredCount;
                    System.arraycopy(selected, 0, bestSelected, 0, selected.length);
                }
            }
            
            boolean prune(int idx, int selectedCount, int coveredCount, int M) {
                int remaining = M - idx;
                int bestCam = this.bestCameras;
                int bestCov = this.bestCovered;
                
                if (mode == Mode.MAX_CAMERAS) {
                    return bestCam != -1 && selectedCount + remaining <= bestCam;
                } else if (mode == Mode.LEAST_CAMERAS) {
                    if (bestCov == totalTargets) {
                        return coveredCount == totalTargets && bestCam != -1 && selectedCount >= bestCam;
                    }
                    return false;
                } else {
                    return bestCov == totalTargets;
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

        private int[] rotate(int r, int c, Direction dir) {
            switch(dir) {
                case N: return new int[]{-c, r};
                case S: return new int[]{c, -r};
                case W: return new int[]{-r, -c};
                case E: default: return new int[]{r, c};
            }
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
                        if (mountSet.contains(confMp)) {
                            md.conflicts.add(confMp);
                        }
                    }
                }
                
                for (int[] d : type.directions) {
                    int[] rot = rotate(d[0], d[1], mp.dir);
                    
                    int rr = mp.cell.r + rot[0];
                    int cc = mp.cell.c + rot[1];
                    int steps = 0;
                    while (rr >= 0 && rr < n && cc >= 0 && cc < n && steps < range) {
                        Cell hit = new Cell(rr, cc);
                        if (wallSet.contains(hit)) break;
                        if (targetSet.contains(hit)) md.covered.add(hit);
                        rr += rot[0];
                        cc += rot[1];
                        steps++;
                    }
                    
                    rr = mp.cell.r + rot[0];
                    cc = mp.cell.c + rot[1];
                    while (rr >= 0 && rr < n && cc >= 0 && cc < n) {
                        Cell hit = new Cell(rr, cc);
                        if (wallSet.contains(hit)) {
                            for (Direction dir : Direction.values()) {
                                MountPoint confMp = new MountPoint(hit, dir);
                                if (mountSet.contains(confMp)) {
                                    md.conflicts.add(confMp);
                                }
                            }
                            break;
                        }
                        rr += rot[0];
                        cc += rot[1];
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
                int cores = Runtime.getRuntime().availableProcessors();
                ExecutorService executor = Executors.newFixedThreadPool(cores);
                List<Callable<Void>> tasks = new ArrayList<>();
                
                int depthLimit = Math.min(M, 10);
                generateTasks(0, new boolean[M], new int[T], new int[M], 0, 0, depthLimit, tasks, shared);
                
                try {
                    executor.invokeAll(tasks);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
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
                    }
                }
                for (int i = 0; i < M; i++) {
                    if (shared.bestSelected[i]) {
                        for (int cId : coverages[i]) {
                            res.covered.add(targetCells.get(cId));
                        }
                    }
                }
            }
            
            res.totalFree = T;
            res.coveragePct = T == 0 ? 0.0 : (res.covered.size() * 100.0 / T);
            res.coveredArea = res.covered.size() * CELL_AREA_M2;
            res.freeArea = T * CELL_AREA_M2;
            res.states = shared.totalStates.get();
            res.seconds = seconds;
            return res;
        }

        private void generateTasks(int idx, boolean[] selected, int[] coverCount, int[] unsafeCount, int selectedCount, int coveredCount, int depthLimit, List<Callable<Void>> tasks, SharedState shared) {
            if (idx == depthLimit) {
                boolean[] taskSelected = selected.clone();
                int[] taskCoverCount = coverCount.clone();
                int[] taskUnsafeCount = unsafeCount.clone();
                tasks.add(new SolverTask(idx, taskSelected, taskCoverCount, taskUnsafeCount, selectedCount, coveredCount, shared));
                return;
            }
            
            if (unsafeCount[idx] == 0) {
                selected[idx] = true;
                int newCovered = coveredCount;
                for (int cId : coverages[idx]) {
                    if (coverCount[cId]++ == 0) newCovered++;
                }
                for (int cId : conflicts[idx]) {
                    unsafeCount[cId]++;
                }
                
                generateTasks(idx + 1, selected, coverCount, unsafeCount, selectedCount + 1, newCovered, depthLimit, tasks, shared);
                
                selected[idx] = false;
                for (int cId : coverages[idx]) {
                    coverCount[cId]--;
                }
                for (int cId : conflicts[idx]) {
                    unsafeCount[cId]--;
                }
            }
            
            generateTasks(idx + 1, selected, coverCount, unsafeCount, selectedCount, coveredCount, depthLimit, tasks, shared);
        }
        
        class SolverTask implements Callable<Void> {
            int startIdx;
            boolean[] selected;
            int[] coverCount;
            int[] unsafeCount;
            int selectedCount;
            int coveredCount;
            SharedState shared;
            long states = 0;
            int M;
            
            SolverTask(int startIdx, boolean[] selected, int[] coverCount, int[] unsafeCount, int selectedCount, int coveredCount, SharedState shared) {
                this.startIdx = startIdx;
                this.selected = selected;
                this.coverCount = coverCount;
                this.unsafeCount = unsafeCount;
                this.selectedCount = selectedCount;
                this.coveredCount = coveredCount;
                this.shared = shared;
                this.M = selected.length;
            }
            
            @Override
            public Void call() {
                backtrack(startIdx);
                shared.totalStates.addAndGet(states);
                return null;
            }
            
            void backtrack(int idx) {
                states++;
                if (shared.prune(idx, selectedCount, coveredCount, M)) return;
                
                if (idx == M) {
                    shared.update(selectedCount, coveredCount, selected);
                    return;
                }
                
                if (unsafeCount[idx] == 0) {
                    selected[idx] = true;
                    int prevCovered = coveredCount;
                    for (int cId : coverages[idx]) {
                        if (coverCount[cId]++ == 0) coveredCount++;
                    }
                    for (int cId : conflicts[idx]) {
                        unsafeCount[cId]++;
                    }
                    
                    backtrack(idx + 1);
                    
                    selected[idx] = false;
                    coveredCount = prevCovered;
                    for (int cId : coverages[idx]) {
                        coverCount[cId]--;
                    }
                    for (int cId : conflicts[idx]) {
                        unsafeCount[cId]--;
                    }
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
        CameraType cameraType = CAMERA_TYPES[0];

        GridPanel() {
            addMouseListener(new MouseAdapter() {
                @Override
                public void mouseClicked(MouseEvent e) {
                    if (grid == null) return;
                    int c = e.getX() / tileSize;
                    int r = e.getY() / tileSize;
                    if (r >= 0 && c >= 0 && r < grid.length && c < grid.length) {
                        grid[r][c] = (grid[r][c] == FREE) ? WALL : FREE;
                        cameras.clear();
                        covered.clear();
                        repaint();
                    }
                }
            });
        }

        void setGrid(int[][] grid) {
            this.grid = grid;
            cameras.clear();
            covered.clear();
            revalidate();
            repaint();
        }

        void setTileSize(int tileSize) {
            this.tileSize = Math.max(8, tileSize);
            revalidate();
            repaint();
        }

        void setSolution(Set<MountPoint> cameras, Set<Cell> covered, CameraType type) {
            this.cameras = cameras;
            this.covered = covered;
            this.cameraType = type;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            int n = grid == null ? 10 : grid.length;
            return new Dimension(n * tileSize, n * tileSize);
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            if (grid == null) return;
            Graphics2D g2 = (Graphics2D) g;
            int n = grid.length;

            for (int r = 0; r < n; r++) {
                for (int c = 0; c < n; c++) {
                    int x = c * tileSize;
                    int y = r * tileSize;

                    Color fill = Color.WHITE;
                    if (grid[r][c] == WALL) fill = new Color(158, 158, 158);
                    else if (covered.contains(new Cell(r, c))) fill = new Color(200, 247, 197);

                    g2.setColor(fill);
                    g2.fillRect(x, y, tileSize, tileSize);
                    g2.setColor(new Color(85, 85, 85));
                    g2.drawRect(x, y, tileSize, tileSize);

                    if (grid[r][c] == WALL) {
                        g2.setColor(Color.BLACK);
                        g2.drawString("X", x + tileSize / 2 - 3, y + tileSize / 2 + 4);
                        for (Direction dir : Direction.values()) {
                            MountPoint mp = new MountPoint(new Cell(r, c), dir);
                            if (cameras.contains(mp)) {
                                g2.setColor(cameraType.color);
                                int rSize = Math.max(4, tileSize / 2);
                                int cx = x + tileSize / 2;
                                int cy = y + tileSize / 2;
                                int shift = tileSize / 3;
                                
                                if (dir == Direction.N) cy -= shift;
                                else if (dir == Direction.S) cy += shift;
                                else if (dir == Direction.E) cx += shift;
                                else if (dir == Direction.W) cx -= shift;
                                
                                g2.fillOval(cx - rSize/2, cy - rSize/2, rSize, rSize);
                                g2.setColor(Color.WHITE);
                                if (dir == Direction.N) g2.drawLine(cx, cy, cx, cy - rSize/2);
                                else if (dir == Direction.S) g2.drawLine(cx, cy, cx, cy + rSize/2);
                                else if (dir == Direction.E) g2.drawLine(cx, cy, cx + rSize/2, cy);
                                else if (dir == Direction.W) g2.drawLine(cx, cy, cx - rSize/2, cy);
                            }
                        }
                    }
                }
            }
        }
    }

    static class AppFrame extends JFrame {
        private int[][] grid;

        private final JSpinner nSpinner = new JSpinner(new SpinnerNumberModel(12, 4, 40, 1));
        private final JSpinner tileSpinner = new JSpinner(new SpinnerNumberModel(20, 8, 40, 1));
        private final JComboBox<Mode> modeBox = new JComboBox<>(Mode.values());
        private final JComboBox<CameraType> typeBox = new JComboBox<>(CAMERA_TYPES);
        private final JSpinner rangeSpinner = new JSpinner(new SpinnerNumberModel(CAMERA_TYPES[0].defaultRange, 1, 20, 1));

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
            bottom.add(new JLabel("Legend: White=Free(1m^2), Gray=Obstacle, Green=Covered, Dot=Camera"), BorderLayout.NORTH);
            
            JPanel statusPanel = new JPanel(new BorderLayout());
            statusPanel.add(status, BorderLayout.CENTER);
            statusPanel.add(progressBar, BorderLayout.EAST);
            bottom.add(statusPanel, BorderLayout.SOUTH);
            add(bottom, BorderLayout.SOUTH);

            createBtn.addActionListener(e -> createGrid());
            clearBtn.addActionListener(e -> clearObstacles());
            solveBtn.addActionListener(e -> solve());

            typeBox.addActionListener(e -> {
                CameraType selected = (CameraType) typeBox.getSelectedItem();
                if (selected != null) rangeSpinner.setValue(selected.defaultRange);
            });

            tileSpinner.addChangeListener(e -> {
                gridPanel.setTileSize((Integer) tileSpinner.getValue());
            });

            createGrid();
            setSize(1200, 800);
            setLocationRelativeTo(null);
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
                            "Type=%s (%dm) | Mode=%s | Cameras=%d | Coverage=%.2f%% (%dm^2/%dm^2) | Blind=%d (%dm^2) | States=%d | Time=%.4fs",
                            type.name, range, mode, res.cameras.size(), res.coveragePct, res.coveredArea, res.freeArea, blindSpots, blindArea, res.states, res.seconds
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
