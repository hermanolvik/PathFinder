import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Scanner;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * This is the main class for finding paths in graphs.
 *
 * Depending on the command line arguments,
 * it creates different graphs and runs different search algorithms.
 */
public class RunPathFinder {

    public static void main(String[] args) throws IOException {
         // If you don't want to specify arguments on the command-line, just uncomment this block.
        if (args.length == 0)
            args = new String[] {
                "astar",         // Algorithm = random | ucs | astar
                "AdjacencyGraph", // Graphtype = AdjacencyGraph | WordLadder | NPuzzle | GridGraph
                "./graphs/AdjacencyGraph/citygraph-SE.txt",  // Graph
                "Malmö",           // Start node
                "Överkalix"           // Goal node
            };
        ///FDG/HE_/CBA/ or /HFG/BED/C_A/

        String algorithm, graphType, filePath;
        ArrayList<String[]> queries = new ArrayList<>();
        Iterator<String> it = Arrays.stream(args).iterator();
        try {
            algorithm = it.next();
            graphType = it.next();
            filePath = it.next();

            while (it.hasNext())
                queries.add(new String[]{it.next(), it.next()});
        } catch (NoSuchElementException ignored) {
            System.err.println("Usage: java RunPathFinder algorithm graphtype graph [optional: start goal]");
            System.err.println("  where algorithm = random | ucs | astar");
            System.err.println("        graphtype = AdjacencyGraph | WordLadder | NPuzzle | GridGraph");
            System.exit(1);
            return;
        }

        TreeMap<String, RunnableIO> byGraphType = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);
        byGraphType.put("AdjacencyGraph", () -> {
            DirectedGraph<String> graph = new AdjacencyGraph(filePath);
            runGraphSearches(graph, algorithm, queries, System.out::println);
        });
        byGraphType.put("WordLadder", () -> {
            DirectedGraph<String> graph = new WordLadder(filePath);
            runGraphSearches(graph, algorithm, queries, System.out::println);
        });
        byGraphType.put("NPuzzle", () -> {
            DirectedGraph<NPuzzle.State> graph = new NPuzzle(Integer.parseInt(filePath));
            runGraphSearches(graph, algorithm, queries, System.out::println);
        });
        byGraphType.put("GridGraph", () -> {
            GridGraph graph = new GridGraph(filePath);
            runGraphSearches(graph, algorithm, queries, result -> {
                System.out.println(result);
                if (result.success && graph.width() < 250 && graph.height() < 250) {
                    List<Point> pathNodes = Stream.concat(Stream.of(result.start), result.path.stream().map(DirectedEdge<Point>::to)).collect(Collectors.toList());
                    System.out.println(graph.showGrid(pathNodes));
                }
            });
        });
        byGraphType.put("GridGraph-NoGrid", () -> {
            GridGraph graph = new GridGraph(filePath);
            runGraphSearches(graph, algorithm, queries, System.out::println);
        });

        RunnableIO action = byGraphType.get(graphType);
        if (action == null)
            throw new IllegalArgumentException("unknown graph type " + graphType);
        action.run();
    }

    @FunctionalInterface
    interface RunnableIO {
        void run() throws IOException;
    }

    /**
     * Takes a graph {@code graph}, an algorithm {@code algorithm}, and optional queries entered on the command-line.
     * Executes the given action on graph search results for the given queries or, if absent, user-entered queries.
     * Throws an IllegalArgumentException for problems parsing nodes or nodes not part of the graph.
     */
    public static <Node> void runGraphSearches(DirectedGraph<Node> graph, String algorithm, List<String[]> queries, Consumer<PathFinder<Node>.Result> action) {
        // Print graph information if running in interactive mode.
        if (queries.isEmpty())
            System.out.println(graph);

        // Parsing of nodes in the graph.
        final Set<Node> nodes = graph.nodes();
        Function<String, Node> parse = str -> {
            Node node = graph.parseNode(str);
            if (!nodes.contains(node))
                throw new IllegalArgumentException("node " + node + " does not belong to the graph");
            return node;
        };

        // Run the query loop, calling back `action` for every search result.
        runQueryLoop(queries, query -> {
            Node start = parse.apply(query[0]);
            Node goal = parse.apply(query[1]);
            action.accept(new PathFinder<>(graph).search(algorithm, start, goal));
        });
    }

    /**
     * Repeatedly queries the user for start and goal edges and calls {@code action} on them.
     * If queries is non-empty, then those form the query inputs and the user is not queried happens.
     */
    public static void runQueryLoop(List<String[]> queries, Consumer<String[]> action) {
        if (!queries.isEmpty()) {
            for (String[] query : queries)
                action.accept(query);
            return;
        }

        final Scanner in = new Scanner(System.in);
        while (true) {
            Function<String, String> query = name -> {
                System.out.print(name + ": ");
                System.out.flush();
                return in.nextLine();
            };

            String start, goal;
            try {
                start = query.apply("start");
                goal = query.apply("goal");
            } catch (NoSuchElementException ignored) {
                return;
            }

            System.out.println();
            action.accept(new String[] {start, goal});
        }
    }

}