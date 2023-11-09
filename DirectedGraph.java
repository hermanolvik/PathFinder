import java.util.List;
import java.util.Set;

/**
 * A simplistic interface for directed graphs.
 *
 * Note that this interface differs from the graph interface in the course API:
 * - it lacks several of the methods in the course API,
 * - it has an additional method {@code guessCost}.
 */

public interface DirectedGraph<Node> {

    /**
     * @return the set of nodes of this graph
     * (does not support updates)
     */
    Set<Node> nodes();

    /**
     * @param  n  a graph node
     * @return a list of the graph edges that originate from node {@code n}
     */
    List<DirectedEdge<Node>> outgoingEdges(Node n);

    /**
     * @return the number of nodes in this graph
     * (warning: may be expensive to compute)
     */
    default int numNodes() {
        return nodes().size();
    }

    /**
     * @return the number of edges in this graph
     * (warning: may be expensive to compute)
     */
    default int numEdges() {
        return nodes().stream().mapToInt(n -> outgoingEdges(n).size()).sum();
    }

    /**
     * @param  n  one node
     * @param  m  another node
     * @return the guessed best cost for getting from {@code n} to {@code m}
     *
     * The default guessed cost is 0, this is always admissible.
     */
    default double guessCost(Node n, Node m) {
        return 0;
    }

    /**
     * @param str  a string
     * @return a node parsed from the given string
     *
     * This is really an operation associated with the type Node, not DirectedGraph,
     * but there's no easy way to do that in Java.
     * So the result is not related to the nodes currently contained in the graph.
     */
    Node parseNode(String str);

}
