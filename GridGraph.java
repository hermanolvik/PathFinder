import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * GridGraph is a 2D-map encoded as a bitmap, or an N x M matrix of characters.
 *
 * Some characters are passable, others denote obstacles.
 * A node is a point in the bitmap, consisting of an x- and a y-coordinate.
 * This is defined by the helper class `Point`.
 * You can move from each point to the eight point around it.
 * The edge costs are 1.0 (for up/down/left/right) and sqrt(2) (for diagonal movement).
 * The graph can be read from a simple ASCII art text file.
 */

public class GridGraph implements DirectedGraph<Point> {

    private final char[][] grid;
    private final int width;
    private final int height;

    // Characters from Moving AI Lab:
    //   . - passable terrain
    //   G - passable terrain
    //   @ - out of bounds
    //   O - out of bounds
    //   T - trees (unpassable)
    //   S - swamp (passable from regular terrain)
    //   W - water (traversable, but not passable from terrain)
    // Characters from http://www.delorie.com/game-room/mazes/genmaze.cgi
    //   | - +  walls
    //   space  passable
    // Note: "-" must come last in allowedChars, because we use it unescaped in a regular expression.

    private static final String allowedChars = ".G@OTSW +|-";
    private static final String passableChars = ".G ";

    // The eight directions, as points.
    private static final Point[] directions =
        IntStream.rangeClosed(-1, 1).boxed().flatMap(x ->
            IntStream.rangeClosed(-1, 1).boxed().flatMap(y ->
                Stream.of(new Point(x, y)).filter(p -> !p.equals(Point.ORIGIN))
            )
        ).toArray(Point[]::new);

    /**
     * Creates a new graph with edges from a text file.
     * The file describes the graph as ASCII art,
     * in the format of the graph files from the Moving AI Lab.
     * @param file  path to a text file
     */
    public GridGraph(String file) throws IOException {
        grid = Files.lines(Paths.get(file))
            .filter(line -> line.matches("^[" + allowedChars + "]+$"))
            .map(String::toCharArray)
            .toArray(char[][]::new);
        height = grid.length;
        width = grid[0].length;
        for (char[] row : grid)
            if (row.length != width)
                throw new IllegalArgumentException("Malformed grid, row widths don't match.");
    }

    /**
     * @return the width of grid
     */
    public int width() {
        return width;
    }

    /**
     * @return the height of grid
     */
    public int height() {
        return height;
    }

    /**
     * @return true if you're allowed to pass through the point {@code <x,y>}
     */
    private boolean passable(Point p) {
        return p.x >= 0 && p.y >= 0 && p.x < width && p.y < height && passableChars.indexOf(grid[p.y][p.x]) >= 0;
    }

    @Override
    public Set<Point> nodes() {
        HashSet<Point> nodes = new HashSet<>();
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                Point p = new Point(x, y);
                if (passable(p))
                    nodes.add(p);
            }
        return nodes;
    }

    /**
     * @param  p  a graph node (point)
     * @return a list of the graph edges that originate from point {@code p}
     */
    @Override
    public List<DirectedEdge<Point>> outgoingEdges(Point p) {
        // We consider all directions,
        return Arrays.stream(directions)
            // compute the edge in that direction,
            .map(dir -> new DirectedEdge<>(p, p.add(dir), dir.euclideanNorm()))
            // keep the ones with passable target,
            .filter(edge -> passable(edge.to()))
            // and return them as a list.
            .collect(Collectors.toList());
    }

    /**
     * @param  p  one point
     * @param  q  another point
     * @return the guessed best cost for getting from {@code p} to {@code q}
     * (the Euclidean distance between the points)
     */
    @Override
    public double guessCost(Point p, Point q) {
        return Math.sqrt(Math.pow(q.x - p.x, 2) + Math.pow(q.y - p.y, 2));

        //return Math.abs(p.x - q.x) + Math.abs(p.y - q.y); //The Manhattan distance

    }

    /**
     * @return parse a point from the string representation {@code str}
     * @throws IllegalArgumentException if the string representation is malformed
     *
     * For example, a valid string representation is "39:18".
     */
    @Override
    public Point parseNode(String str) {
        try {
            return Point.valueOf(str);
        } catch (NumberFormatException | ArrayIndexOutOfBoundsException ignored) {
            throw new IllegalArgumentException(str + " does not represent a point");
        }
    }

    /**
     * @return a string representation of the grid
     */
    public String showGrid() {
        return showGrid(new LinkedList<>());
    }

    /**
     * @param  path  a list of points that constitutes a path
     * @return a string representation of the grid, with the given path shown
     */
    public String showGrid(List<Point> path) {
        StringWriter buffer = new StringWriter();
        PrintWriter w = new PrintWriter(buffer);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++)
                w.print(path.contains(new Point(x, y)) ? '*' : grid[y][x]);
            w.println();
        }
        return buffer.toString();
    }

    /**
     * @return a string representation of this graph,
     * including some random points and edges
     */
    @Override
    public String toString() {
        StringWriter buffer = new StringWriter();
        PrintWriter w = new PrintWriter(buffer);

        w.println("Bitmap graph of dimensions " + width + " x " + height + " pixels");
        w.println(showGrid());

        w.println("Random example points with outgoing edges:");
        DirectedEdge.printOutgoingEdges(w, this, null);
        return buffer.toString();
    }

}
