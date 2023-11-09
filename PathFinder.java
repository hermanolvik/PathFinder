import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 * This is a class that can find paths in a given graph.
 *
 * There are several methods for finding paths,
 * and they all return a PathFinder.Result object.
 */
public class PathFinder<Node> {

    private final DirectedGraph<Node> graph;
    private long startTimeMillis;

    /**
     * Creates a new pathfinder for the given graph.
     * @param graph  the graph to search
     */
    public PathFinder(DirectedGraph<Node> graph) {
        this.graph = graph;
    }

    /**
     * The main search method, taking the search algorithm as input.
     * @param  algorithm  "random", "ucs" or "astar"
     * @param  start  the start node
     * @param  goal   the goal node
     */
    public Result search(String algorithm, Node start, Node goal) {
        TreeMap<String, Supplier<Result>> byAlgorithm = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);
        byAlgorithm.put("random", () -> searchRandom(start, goal));
        byAlgorithm.put("ucs"   , () -> searchUCS   (start, goal));
        byAlgorithm.put("astar" , () -> searchAstar (start, goal));

        Supplier<Result> action = byAlgorithm.get(algorithm);
        if (action == null)
            throw new IllegalArgumentException("unknown search algorithm " + algorithm);

        startTimeMillis = System.currentTimeMillis();
        return action.get();
    }

    /**
     * Perform a random walk in the graph, hoping to reach the goal.
     * Warning: this method will give up of the random walk
     * reaches a dead end or after one million iterations.
     * So a negative result does not mean there is no path.
     * @param start  the start node
     * @param goal   the goal node
     */
    public Result searchRandom(Node start, Node goal) {
        int iterations = 0;
        LinkedList<DirectedEdge<Node>> path = new LinkedList<>();
        double cost = 0;
        Random random = new Random();

        Node current = start;
        while (iterations < 1e6) {
            iterations++;
            if (current.equals(goal))
                return new Result(true, start, current, cost, path, iterations);

            List<DirectedEdge<Node>> neighbours = graph.outgoingEdges(current);
            if (neighbours.isEmpty())
                break;

            DirectedEdge<Node> edge = neighbours.get(random.nextInt(neighbours.size()));
            path.add(edge);
            cost += edge.weight();
            current = edge.to();
        }
        return new Result(false, start, goal, -1, null, iterations);
    }

    public static void main(String[] args){
        AdjacencyGraph g = new AdjacencyGraph();
        g.add(new DirectedEdge<String>("A", "Goal", 3));
        g.add(new DirectedEdge<String>("A", "B"));
        g.add(new DirectedEdge<String>("B", "Goal"));
        PathFinder<String> p = new PathFinder(g);
        p.searchAstar("A", "Goal");

    }

    /**
     * Run uniform-cost search for finding the shortest path.
     * @param start  the start node
     * @param goal   the goal node
     */
    public Result searchUCS(Node start, Node goal) {
        int iterations = 0;
        Queue<PQEntry> pqueue = new PriorityQueue<>(Comparator.comparingDouble(e -> e.costToHere));

        PQEntry s = new PQEntry(start, 0, null, null);
        pqueue.add(s);
        Set<Node> visited = new HashSet<>();
        PQEntry v;
        while(pqueue.size() != 0){
            v = pqueue.remove();
            if(!visited.contains(v.node)) {
                visited.add(v.node);

                if (v.node.equals(goal)) {
                    List<DirectedEdge<Node>> p = extractPath(v);
                    return new Result(true, start, v.node, v.costToHere, p, iterations);
                }

                for (DirectedEdge<Node> edge : graph.outgoingEdges(v.node)) {
                    if (!visited.contains(edge.to())) {
                        pqueue.add(new PQEntry(edge.to(), v.costToHere + edge.weight(), edge, v));
                    }
                }
            }
            iterations++;
        }

        // Return this if no path is found.
        return new Result(false, start, goal, -1, null, iterations);
    }

    /**
     * Run the A* algorithm for finding the shortest path.
     * @param start  the start node
     * @param goal   the goal node
     */
    public Result searchAstar(Node start, Node goal) {
        int iterations = 0;
        Queue<PQEntry> pqueue = new PriorityQueue<>(Comparator.comparingDouble(e -> e.FORASTAR));

        PQEntry s = new PQEntry(start, 0, null, null, graph.guessCost(start, goal) );
        pqueue.add(s);
        Set<Node> visited = new HashSet<>();
        PQEntry v;
        while(pqueue.size() != 0){
            v = pqueue.remove();
            if(!visited.contains(v.node)) {
                visited.add(v.node);

                if (v.node.equals(goal)) {
                    List<DirectedEdge<Node>> p = extractPath(v);
                    return new Result(true, start, v.node, v.costToHere, p, iterations);
                }

                for (DirectedEdge<Node> edge : graph.outgoingEdges(v.node)) {
                    if (!visited.contains(edge.to())) {
                        pqueue.add(new PQEntry(edge.to(), v.costToHere + edge.weight(), edge, v, v.costToHere + edge.weight() + graph.guessCost(edge.to(), goal)));
                    }
                }
            }
            iterations++;
        }




        // Return this if no path is found.
        return new Result(false, start, goal, -1, null, iterations);
    }

    /**
     * Extract the path from the start to the current priority queue entry.
     * @param entry  the priority queue entry
     * @return the path from start to goal as a list of edges
     */
    private List<DirectedEdge<Node>> extractPath(PQEntry entry) {
        PQEntry n = new PQEntry(entry.node, entry.costToHere, entry.lastEdge, entry.backPointer);
        List<DirectedEdge<Node>> p = new Stack<>();

        while(n.lastEdge != null) {
            p.add(n.lastEdge);
            n = n.backPointer;
        }

        List<DirectedEdge<Node>> tmp = new ArrayList<>();
        for(int i = p.size() - 1; i>=0; i-- ){
            tmp.add(p.remove(i));
        }
        return tmp;
    }

    /**
     * Entries to put in the priority queues in {@code searchUCS} and {@code searchAstar}.
     */
    private class PQEntry {
        public final Node node;
        public final double costToHere;
        public final DirectedEdge<Node> lastEdge;  // null for starting entry
        public final PQEntry backPointer;          // null for starting entry
        public double FORASTAR;
        /************************************************
         * TODO: Task 3                                 *
         * You can add new fields or constructors here. *
         ************************************************/

        PQEntry(Node node, double costToHere, DirectedEdge<Node> lastEdge, PQEntry backPointer) {
            this.node = node;
            this.costToHere = costToHere;
            this.lastEdge = lastEdge;
            this.backPointer = backPointer;
        }

        PQEntry(Node node, double costToHere, DirectedEdge<Node> lastEdge, PQEntry backPointer, double astar) {
            this.node = node;
            this.costToHere = costToHere;
            this.lastEdge = lastEdge;
            this.backPointer = backPointer;
            this.FORASTAR = astar;
        }
    }



    /**
     * The internal class for search results.
     */
    public class Result {
        public final boolean success;
        public final Node start;
        public final Node goal;
        public final double cost;
        public final List<DirectedEdge<Node>> path;
        public final int iterations;
        public final double elapsedTime;

        public Result(boolean success, Node start, Node goal, double cost, List<DirectedEdge<Node>> path, int iterations) {
            this.success = success;
            this.start = start;
            this.goal = goal;
            this.cost = cost;
            this.path = path;
            this.iterations = iterations;
            this.elapsedTime = (System.currentTimeMillis() - startTimeMillis) / 1000.0;
        }

        private String formatPathPart(boolean suffix, int i, int j) {
            return path.subList(i, j).stream()
                    .map(e -> e.toString(!suffix, suffix))
                    .collect(Collectors.joining());
        }

        public String toString() {
            StringWriter buffer = new StringWriter();
            PrintWriter w = new PrintWriter(buffer);
            if (iterations <= 0)
                w.println("ERROR: You have to iterate over at least the starting node!");
            w.println("Loop iterations: " + iterations);
            w.println("Elapsed time: " + elapsedTime + "s");
            if (success) {
                w.println("Cost of path from " + start + " to " + goal + ": " + DirectedEdge.DECIMAL_FORMAT.format(cost));
                if (path == null)
                    w.println("WARNING: you have not implemented extractPath!");
                else {
                    // Print path.
                    w.println("Number of edges: " + path.size());
                    w.println(path.size() <= 10 ?
                        start + formatPathPart(true, 0, path.size()) :
                        formatPathPart(false, 0, 5) + "....." + formatPathPart(true, path.size() - 5, path.size())
                    );
                    // We sum using left association order to mimic the algorithm.
                    // Then we can use exact comparison of doubles.
                    double actualCost = path.stream().mapToDouble(DirectedEdge::weight).reduce(0, Double::sum);
                    if (cost != actualCost)
                        w.println("WARNING: the actual path cost " + actualCost + " differs from the reported cost " + cost);
                }
            } else
                w.println("No path found from " + start + " to " + goal);
            return buffer.toString();
        }

    }

}
