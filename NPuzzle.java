import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.AbstractSet;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * NPuzzle is an encoding of the N-puzzle.
 *
 * The nodes are string encodings of an N x N matrix of tiles.
 * The tiles are represented by characters starting from the letter A
 * (A...H for N=3, and A...O for N=4).
 * The empty tile is represented by "_", and
 * to make it more readable for humans every row is separated by "/".
 */
public class NPuzzle implements DirectedGraph<NPuzzle.State> {

    private static final String SEPARATOR = "/";

    // The characters '_', 'A', ..., 'Z', '0', ..., '9', 'a', ..., 'z'.
    // A fixed NPuzzle uses only an initial prefix of these characters.
    private static final Character[] ALL_TILE_NAMES = Stream.of(
            IntStream.of('_'),
            IntStream.rangeClosed('A', 'Z'),
            IntStream.rangeClosed('0', '9'),
            IntStream.rangeClosed('a', 'z')
    ).flatMapToInt(Function.identity()).mapToObj(i -> (char) i).toArray(Character[]::new);

    private static final Point[] MOVES = {
        new Point(-1, 0),
        new Point(1, 0),
        new Point(0, -1),
        new Point(0, 1)
    };

    private final int N;
    private final Character[] tileNames;

    /**
     * Creates a new n-puzzle of size {@code N}
     * @param N  the size of the puzzle
     */
    public NPuzzle(int N) {
        if (!(N >= 2 && N <= 6))
            throw new IllegalArgumentException("We only support sizes of 2 <= N <= 6.");

        this.N = N;
        this.tileNames = Arrays.copyOf(ALL_TILE_NAMES, N * N);
    }

    /**
     * A possible state of the N-puzzle.
     *
     * We represent the tiles as numbers from 0 to N * N.
     * The empty tile is represented by 0.
     *
     * The array {@code positions} stores the position of each tile.
     *
     * Optional task: try out different representations of states:
     * - coding the positions as indices (sending a point p to p.y * N + p.x)
     * - using an array {@code tiles} that stores the tile at each point
     * - combinations (more space usage, but better runtime?)
     */
    public class State {
        private final Point[] positions;

        private State(Point[] positions) {
            this.positions = positions;
        }

        /**
         * @return the state given by swapping the tiles {@code i} and {@code j}
         */
        public State swap(int i, int j) {
            Point[] positionsNew = positions.clone();
            positionsNew[i] = positions[j];
            positionsNew[j] = positions[i];
            return new State(positionsNew);
        }

        /**
         * @return a randomly shuffled state
         */
        public State shuffled() {
            Point[] positionsNew = positions.clone();
            Collections.shuffle(Arrays.asList(positionsNew));
            return new State(positionsNew);
        }

        /**
         * @return the NxN-matrix of tiles of this state
         */
        public int[][] tiles() {
            int[][] tiles = new int[N][N];
            for (int i = 0; i != positions.length; i++) {
                Point p = positions[i];
                tiles[p.y][p.x] = i;
            }
            return tiles;
        }

        @Override
        public boolean equals(Object o) {
            if (o == this) // equality of references
                return true;
            if (!(o instanceof State))
                return false;
            return Arrays.deepEquals(positions, ((State) o).positions);
        }

        @Override
        public int hashCode() {
            return Arrays.hashCode(positions);
        }

        /**
         * @return string representation of this state
         */
        public String toString() {
            return Arrays.stream(tiles()).map(rowTiles -> Arrays.stream(rowTiles)
                    .mapToObj(NPuzzle.this::formatTile)
                    .map(String::valueOf)
                    .collect(Collectors.joining())
            ).collect(Collectors.joining(SEPARATOR, SEPARATOR, SEPARATOR));
        }
    }

    // Helper methods for formatting and parsing tiles.

    public char formatTile(int tile) {
        return tileNames[tile];
    }

    public int parseTile(char tileName) {
        int tile = Arrays.asList(tileNames).indexOf(tileName);
        if (tile == -1)
            throw new IllegalArgumentException("invalid tile " + tileName);
        return tile;
    }

    /**
     * @return the state specified by an NxN-matrix of tiles.
     * @throws IllegalArgumentException if there are duplicate tiles (equivalently, missing tiles).
     */
    public State stateFromTiles(int[][] tiles) {
        Point[] positions = new Point[N * N];
        for (int y = 0; y != N; y++)
            for (int x = 0; x != N; x++) {
                int tile = tiles[y][x];
                if (positions[tile] != null)
                    throw new IllegalArgumentException("duplicate tile " + formatTile(tile));
                positions[tile] = new Point(x, y);
            }
        return new State(positions);
    }

    /**
     * @return parses a state from its string representation
     * @throws IllegalArgumentException if the string representation is invalid.
     *
     * For example, a valid string representation for N = 3 is "/FDA/CEH/GB_/".
     */
    public State parseNode(String str) {
        String[] rows = Arrays.stream(str.split(Pattern.quote(SEPARATOR), -1))
                .filter(s -> !s.isEmpty())
                .toArray(String[]::new);
        if (rows.length != N)
            throw new IllegalArgumentException(str + " does not have " + N + " rows");
        for (String row : rows)
            if (row.length() != N)
                throw new IllegalArgumentException("row " + row + " does not have " + N + " columns");

        int[][] tiles = new int[N][N];
        for (int y = 0; y != N; y++)
            for (int x = 0; x != N; x++)
                tiles[y][x] = parseTile(rows[y].charAt(x));
        return stateFromTiles(tiles);
    }

    /**
     * @return the traditional goal state
     * The empty tile is in the bottom right corner.
     */
    public State goalState() {
        return new State(IntStream.range(0, N * N)
                .map(i -> Math.floorMod(i - 1, N * N))
                .mapToObj(i -> new Point(i % N, i / N))
                .toArray(Point[]::new));
    }

    /**
     * @return checks if the point {@code p} is valid (lies inside the matrix).
     */
    public boolean valid(Point p) {
        return p.x >= 0 && p.y >= 0 && p.x < N && p.y < N;
    }

    /**
     * All states are nodes of this graph.
     * However, the set of such nodes is typically too large to enumerate.
     * So we do not implement those operations.
     */
    @Override
    public Set<State> nodes() {
        return new AbstractSet<>() {
            @Override
            public Iterator<State> iterator() {
                throw new UnsupportedOperationException("too expensive!");
            }

            @Override
            public int size() {
                throw new UnsupportedOperationException("too expensive!");
            }

            @Override
            public boolean contains(Object o) {
                return o instanceof State;
            }
        };
    }

    /**
     * @param  s  a puzzle state
     * @return a list of the graph edges that originate from state {@code s}
     */
    @Override
    public List<DirectedEdge<State>> outgoingEdges(State s) {
        // Consider all possible moves...
        return Arrays.stream(MOVES).map(move -> {
            // The current position of the empty tile.
            Point emptyPos = s.positions[0];
            // The position of the tile that moves to the empty tile in the direction `move`.
            return emptyPos.subtract(move);
        }).filter(this::valid).map(tilePos -> {
            // The tile to swap with the empty tile.
            int tile = Arrays.asList(s.positions).indexOf(tilePos);
            // The new state after swapping.
            State t = s.swap(0, tile);
            return new DirectedEdge<>(s, t);
        }).collect(Collectors.toList());
    }

    /**
     * @param  s  one puzzle state
     * @param  t  another puzzle state
     * @return the guessed cost for getting from {@code s} to {@code t}
     */
    @Override
    public double guessCost(State s, State t) {
        // We consider all tiles except the empty one,
        return IntStream.range(1, N * N)
                // take the Manhattan distance between its positions in the two states,
                .map(i -> s.positions[i].subtract(t.positions[i]).manhattanNorm())
                // and return the sum.
                .sum();
    }

    /**
     * @return a string representation of the puzzle graph
     */
    @Override
    public String toString() {
        StringWriter buffer = new StringWriter();
        PrintWriter w = new PrintWriter(buffer);
        w.println("NPuzzle graph of size " + N + " x " + N + ".");
        w.println("States are " + N + " x " + N + " matrices of unique characters in '" + formatTile(1) + "'...'" + formatTile(N*N-1) + "' and '" + formatTile(0) + "' (for the empty tile); rows are interspersed with '" + SEPARATOR + "'.");

        State goal = goalState();
        w.println("The traditional goal state is: " + goal);
        w.println();

        w.println("Random example states with outgoing edges:");
        DirectedEdge.printOutgoingEdges(w, this, () -> goal.shuffled());
        return buffer.toString();
    }

}
