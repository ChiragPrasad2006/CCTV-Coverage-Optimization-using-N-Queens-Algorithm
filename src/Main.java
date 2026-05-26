import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.*;
import java.util.List;

public class Main {
    private static final int FREE = 0;
    private static final int WALL = 1;
    private static final int CELL_AREA_M2 = 1;

    enum Mode {
        MAX_COVERAGE, MAX_CAMERAS
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
            if (!(o instanceof Cell cell)) return false;
            return r == cell.r && c == cell.c;
        }

        @Override
        public int hashCode() {
            return Objects.hash(r, c);
        }
    }

    static class SolveResult {
        Set<Cell> cameras;
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

        private List<Cell> freeCells;
        private Set<Cell> freeSet;
        private Map<Cell, Set<Cell>> coverageMap;
        private Map<Cell, Set<Cell>> conflictMap;

        private Set<Cell> bestCameras = new HashSet<>();
        private Set<Cell> bestCovered = new HashSet<>();
        private int totalFree;
        private long states;

        CCTVOptimizer(int[][] grid, Mode mode, CameraType type, int range) {
            this.grid = grid;
            this.n = grid.length;
            this.mode = mode;
            this.type = type;
            this.range = Math.max(1, range);
            prepare();
        }

        private void prepare() {
            freeCells = new ArrayList<>();
            for (int r = 0; r < n; r++) {
                for (int c = 0; c < n; c++) {
                    if (grid[r][c] == FREE) freeCells.add(new Cell(r, c));
                }
            }
            freeSet = new HashSet<>(freeCells);
            totalFree = freeCells.size();

            coverageMap = new HashMap<>();
            conflictMap = new HashMap<>();
            for (Cell cell : freeCells) {
                coverageMap.put(cell, coverageFor(cell));
                conflictMap.put(cell, conflictsFor(cell));
            }

            freeCells.sort((a, b) -> Integer.compare(coverageMap.get(b).size(), coverageMap.get(a).size()));
        }

        private boolean inBounds(int r, int c) {
            return r >= 0 && r < n && c >= 0 && c < n;
        }

        private Set<Cell> ray(int r, int c, int dr, int dc) {
            Set<Cell> out = new HashSet<>();
            int rr = r + dr;
            int cc = c + dc;
            int steps = 0;
            while (inBounds(rr, cc) && steps < range) {
                if (grid[rr][cc] == WALL) break;
                out.add(new Cell(rr, cc));
                rr += dr;
                cc += dc;
                steps++;
            }
            return out;
        }

        private Set<Cell> coverageFor(Cell cell) {
            Set<Cell> covered = new HashSet<>();
            covered.add(cell);
            for (int[] d : type.directions) {
                covered.addAll(ray(cell.r, cell.c, d[0], d[1]));
            }
            return covered;
        }

        private Set<Cell> conflictsFor(Cell cell) {
            Set<Cell> conflicts = new HashSet<>();
            for (int[] d : type.directions) {
                for (Cell x : ray(cell.r, cell.c, d[0], d[1])) {
                    if (freeSet.contains(x)) conflicts.add(x);
                }
            }
            return conflicts;
        }

        private boolean isSafe(Cell cell, Set<Cell> selected) {
            Set<Cell> conflicts = conflictMap.get(cell);
            for (Cell c : selected) {
                if (conflicts.contains(c)) return false;
            }
            return true;
        }

        private boolean better(Set<Cell> selected, Set<Cell> covered) {
            if (mode == Mode.MAX_CAMERAS) {
                if (selected.size() != bestCameras.size()) return selected.size() > bestCameras.size();
                return covered.size() > bestCovered.size();
            }
            if (covered.size() != bestCovered.size()) return covered.size() > bestCovered.size();
            return bestCameras.isEmpty() || selected.size() < bestCameras.size();
        }

        private boolean prune(int idx, Set<Cell> selected, Set<Cell> covered) {
            int remaining = freeCells.size() - idx;
            if (mode == Mode.MAX_CAMERAS) {
                return selected.size() + remaining <= bestCameras.size();
            }
            if (bestCovered.size() == totalFree) {
                return covered.size() == totalFree && !bestCameras.isEmpty() && selected.size() >= bestCameras.size();
            }
            return false;
        }

        SolveResult solve() {
            long start = System.nanoTime();
            backtrack(0, new HashSet<>(), new HashSet<>());
            double seconds = (System.nanoTime() - start) / 1_000_000_000.0;

            SolveResult res = new SolveResult();
            res.cameras = bestCameras;
            res.covered = bestCovered;
            res.totalFree = totalFree;
            res.coveragePct = totalFree == 0 ? 0.0 : (bestCovered.size() * 100.0 / totalFree);
            res.coveredArea = bestCovered.size() * CELL_AREA_M2;
            res.freeArea = totalFree * CELL_AREA_M2;
            res.states = states;
            res.seconds = seconds;
            return res;
        }

        private void backtrack(int idx, Set<Cell> selected, Set<Cell> covered) {
            states++;
            if (prune(idx, selected, covered)) return;

            if (idx == freeCells.size()) {
                if (better(selected, covered)) {
                    bestCameras = new HashSet<>(selected);
                    bestCovered = new HashSet<>(covered);
                }
                return;
            }

            Cell cell = freeCells.get(idx);
            if (isSafe(cell, selected)) {
                selected.add(cell);
                Set<Cell> nextCovered = new HashSet<>(covered);
                nextCovered.addAll(coverageMap.get(cell));
                backtrack(idx + 1, selected, nextCovered);
                selected.remove(cell);
            }
            backtrack(idx + 1, selected, covered);
        }
    }

    static class GridPanel extends JPanel {
        int[][] grid;
        Set<Cell> cameras = new HashSet<>();
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

        void setSolution(Set<Cell> cameras, Set<Cell> covered, CameraType type) {
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
                    } else if (cameras.contains(new Cell(r, c))) {
                        g2.setColor(cameraType.color);
                        int pad = Math.max(2, tileSize / 5);
                        g2.fillOval(x + pad, y + pad, tileSize - 2 * pad, tileSize - 2 * pad);
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

            JButton createBtn = new JButton("Create Grid");
            JButton clearBtn = new JButton("Clear Obstacles");
            JButton solveBtn = new JButton("Solve");
            controls.add(createBtn);
            controls.add(clearBtn);
            controls.add(solveBtn);

            add(controls, BorderLayout.NORTH);

            JScrollPane scrollPane = new JScrollPane(gridPanel);
            add(scrollPane, BorderLayout.CENTER);

            JPanel bottom = new JPanel(new BorderLayout());
            bottom.add(new JLabel("Legend: White=Free(1m^2), Gray=Obstacle, Green=Covered, Dot=Camera"), BorderLayout.NORTH);
            bottom.add(status, BorderLayout.SOUTH);
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
            gridPanel.setGrid(grid);
            gridPanel.setTileSize((Integer) tileSpinner.getValue());
            status.setText("Grid created: " + n + "x" + n + " (" + (n * n) + " m^2 total). Click cells to toggle obstacles.");
        }

        private void clearObstacles() {
            if (grid == null) return;
            for (int r = 0; r < grid.length; r++) {
                Arrays.fill(grid[r], FREE);
            }
            gridPanel.setSolution(new HashSet<>(), new HashSet<>(), (CameraType) typeBox.getSelectedItem());
            gridPanel.repaint();
            status.setText("Obstacles cleared.");
        }

        private void solve() {
            if (grid == null) return;
            int freeCount = 0;
            for (int[] row : grid) {
                for (int cell : row) if (cell == FREE) freeCount++;
            }
            if (freeCount == 0) {
                JOptionPane.showMessageDialog(this, "No free cells available.", "Warning", JOptionPane.WARNING_MESSAGE);
                return;
            }

            Mode mode = (Mode) modeBox.getSelectedItem();
            CameraType type = (CameraType) typeBox.getSelectedItem();
            int range = (Integer) rangeSpinner.getValue();

            CCTVOptimizer optimizer = new CCTVOptimizer(grid, mode, type, range);
            SolveResult res = optimizer.solve();

            int blindSpots = res.totalFree - res.covered.size();
            int blindArea = blindSpots * CELL_AREA_M2;

            gridPanel.setSolution(res.cameras, res.covered, type);
            status.setText(String.format(
                "Type=%s (%dm) | Mode=%s | Cameras=%d | Coverage=%.2f%% (%dm^2/%dm^2) | Blind=%d (%dm^2) | States=%d | Time=%.4fs",
                type.name, range, mode, res.cameras.size(), res.coveragePct, res.coveredArea, res.freeArea, blindSpots, blindArea, res.states, res.seconds
            ));
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new AppFrame().setVisible(true));
    }
}
