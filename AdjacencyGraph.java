import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * This is a class for a generic finite graph, with string nodes.
 *
 * The edges are stored as an adjacency list as described in
 * the course book and the lectures.
 * The graphs can be anything, such as a road map or a web link graph.
 * The graph can be read from a simple text file with one edge per line.
 */

public class AdjacencyGraph implements DirectedGraph<String> {

    private final Map<String, List<DirectedEdge<String>>> adjacencyList;

    /**
     * Creates a new empty graph.
     */
    public AdjacencyGraph() {
        adjacencyList = new HashMap<>();
    }

    /**
     * Adds a node to this graph.
     * @param n  the node
     */
    public void addNode(String n) {
        adjacencyList.putIfAbsent(n, new LinkedList<>());
    }

    /**
     * Adds the directed edge {@code e} (and its source and target nodes) to this edge-weighted graph.
     * Note: This does not test if the edge is already in the graph!
     * @param e  the edge
     */
    public void add(DirectedEdge<String> e) {
        addNode(e.from());
        addNode(e.to());
        adjacencyList.get(e.from()).add(e);
    }

    /**
     * Creates a new graph with edges from a text file.
     * The file should contain one edge per line, each on the form
     * "from TAB to TAB weight" or "from TAB to".
     * @param file  path to a text file
     */
    public AdjacencyGraph(String file) throws IOException {
        this();
        Files.lines(Paths.get(file))
            .filter(line -> !line.startsWith("#"))
            .map(line -> line.split("\t"))
            .map(edge -> (edge.length == 2
                          ? new DirectedEdge<>(edge[0].trim(), edge[1].trim())
                          : new DirectedEdge<>(edge[0].trim(), edge[1].trim(), Double.parseDouble(edge[2].trim()))
                          ))
            .forEach(this::add);
    }

    public Set<String> nodes() {
        return Collections.unmodifiableSet(adjacencyList.keySet());
    }

    /**
     * @param  n  a graph node
     * @return a list of the graph edges that originate from node {@code n}
     */
    @Override
    public List<DirectedEdge<String>> outgoingEdges(String n) {
        return Collections.unmodifiableList(adjacencyList.get(n));
    }

    @Override
    public String parseNode(String n) {
        return n;
    }

    /**
     * @return a string representation of the graph
     */
    @Override
    public String toString() {
        StringWriter buffer = new StringWriter();
        PrintWriter w = new PrintWriter(buffer);
        w.println("Adjacency graph with " + numNodes() + " nodes and " + numEdges() + " edges");
        w.println();

        w.println("Random example nodes with outgoing edges:");
        DirectedEdge.printOutgoingEdges(w, this, null);
        return buffer.toString();
    }

}
