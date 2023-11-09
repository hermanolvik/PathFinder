/**
 * A class for two-dimensional points with integral coordinates.
 * Used in GridGraph and NPuzzle.
 */
public class Point {

    public static final Point ORIGIN = new Point(0, 0);

    public final int x, y;

    public Point(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public int manhattanNorm() {
        return Math.abs(x) + Math.abs(y);
    }

    public double euclideanNorm() {
        return Math.sqrt(x * x + y * y);
    }

    public Point add(Point p) {
        return new Point(x + p.x, y + p.y);
    }

    public Point subtract(Point p) {
        return new Point(x - p.x, y - p.y);
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) // equality of references
            return true;
        if (!(o instanceof Point))
            return false;
        Point other = (Point) o;
        return this.x == other.x && this.y == other.y;
    }

    @Override
    public int hashCode() {
        return Integer.rotateLeft(this.x, 16) ^ this.y;
    }

    public String toString() {
        return x + ":" + y;
    }

    public static Point valueOf(String s) {
        String[] cs = s.split(":", 2);
        return new Point(Integer.parseInt(cs[0]), Integer.parseInt(cs[1]));
    }
}
