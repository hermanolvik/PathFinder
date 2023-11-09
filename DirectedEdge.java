import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Random;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * This is a class for directed graph edges.
 */

public class DirectedEdge<Node> {

    private final Node from;
    private final Node to;
    private final double weight;

    /**
     * Initializes a directed edge from node {@code from} to node {@code to}
     * with the given {@code weight}.
     * @param  from    the starting node
     * @param  to      the ending node
     * @param  weight  the weight of the directed edge
     * @throws IllegalArgumentException if the edge weight is negative
     */
    public DirectedEdge(Node from, Node to, double weight) {
        this.from = from;
        this.to = to;
        this.weight = weight;
        if (weight < 0)
            throw new IllegalArgumentException("edge " + this + " has negative weight");
    }

    /**
     * Initializes a directed edge from node {@code from} to node {@code to}
     * with the default weight 1.0.
     * @param from  the starting node
     * @param to    the ending node
     */
    public DirectedEdge(Node from, Node to) {
        this(from, to, 1.0);
    }

    /**
     * @return a new edge with the direction reversed
     */
    public DirectedEdge<Node> reverse() {
        return new DirectedEdge<>(to, from, weight);
    }

    /**
     * @return the starting node of the directed edge
     */
    public Node from() {
        return from;
    }

    /**
     * @return the ending node of the directed edge
     */
    public Node to() {
        return to;
    }

    /**
     * @return the weight of the directed edge
     */
    public double weight() {
        return weight;
    }

    /**
     * @param  other  the object to compare with
     * @return true if the given argument is equal to this edge
     */
    @Override
    public boolean equals(Object other) {
        if (other == this) // equality of references
            return true;
        if (!(other instanceof DirectedEdge))
            return false;
        DirectedEdge<?> o = (DirectedEdge<?>) other;
        return from.equals(o.from()) && to.equals(o.to()) && weight == o.weight();
    }

    /**
     * @return the hash code of this edge
     */
    @Override
    public int hashCode() {
        return Objects.hash(from, to, weight);
    }

    // Formatting helpers.

    public static final DecimalFormat DECIMAL_FORMAT = new DecimalFormat("0.##");

    public String targetString(boolean withWeight) {
        return withWeight ? to.toString() + " [" + DECIMAL_FORMAT.format(weight) + "]" : to.toString();
    }

    public String targetString() {
        return targetString(weight != 1.0);
    }

    public String toString(boolean withWeight, boolean includeFrom, boolean includeTo) {
        String fromStr = includeFrom ? from.toString() : "";
        String toStr = includeTo ? to.toString() : "";
        return fromStr + (withWeight ? " --[" + DECIMAL_FORMAT.format(weight) + "]-> " : " -> ") + toStr;
    }

    public String toString(boolean includeFrom, boolean includeTo) {
        return toString(weight != 1.0, includeFrom, includeTo);
    }

    /**
     * @return a string representation of the directed edge
     * (the weight is omitted if it is 1.0)
     */
    @Override
    public String toString() {
        return toString(true, true);
    }

    /**
     * A helper method for printing some graph information.
     * Should belong to a separate Tools class.
     */
    public static<Node> void printOutgoingEdges(PrintWriter w, DirectedGraph<Node> graph, Supplier<Node> getNode) {
        if (getNode == null) {
            ArrayList<Node> words = new ArrayList<>(graph.nodes());
            Random random = new Random();
            getNode = () -> words.get(random.nextInt(words.size()));
        }

        Stream.generate(getNode).map(from -> {
                List<DirectedEdge<Node>> outgoing = graph.outgoingEdges(from);
                return from + " " + (outgoing.isEmpty() ? "with no outgoing edges" :
                    "---> "
                    + outgoing.stream()
                        .map(e -> e.targetString())
                        .collect(Collectors.joining(", ")));
            })
            .limit(8)
            .forEach(w::println);
    }

}
