/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Consumer;

import static java.util.Objects.isNull;
import static java.util.stream.Collectors.toList;
import static java.util.stream.Collectors.toMap;
import static roadgraph.Vertex.of;

/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 *
 */
public class MapGraph {

    // repository which consists of vertices
    private Map<GeographicPoint, Vertex> vertexMap;

    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        vertexMap = new HashMap<>();
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return vertexMap.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        return vertexMap.keySet();
    }

    /**
     * Get the number of road segments in the graph
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        return vertexMap.values().stream()
                .map(Vertex::getEdges)
                .flatMap(Collection::stream)
                .collect(toList())
                .size();
    }


    /** Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     * @param location  The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        if (isNull(location)) return false;

        if (vertexMap.containsKey(location)) return false;

        vertexMap.put(location, of(location));
        return true;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     * @param from The starting point of the edge
     * @param to The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *   added as nodes to the graph, if any of the arguments is null,
     *   or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {
        vertexMap.get(from)
                .getEdges()
                .add(Edge.of(from, to, roadName, roadType, length));
    }


    /** Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal The goal location
     * @return The list of intersections that form the shortest (unweighted)
     *   path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /** Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     *   path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        if (isNull(start) || isNull(goal)) {
            System.out.println("Start or goal node is null!  No path exists.");
            return new LinkedList<GeographicPoint>();
        }

        final Vertex vertexStart = vertexMap.get(start);
        final Vertex vertexGoal = vertexMap.get(goal);
        if (isNull(vertexStart) || isNull(vertexGoal)) {
            System.out.println("Start or goal geographic point doesn't exist in Graph.");
            return new LinkedList<GeographicPoint>();
        }

        final Map<Vertex, Vertex> parentMap = new HashMap<>();

        final boolean found = bfsSearch(vertexStart, vertexGoal, parentMap);

        if (!found) {
            System.out.println("No path exists");
            return new ArrayList<>();
        }

        return constructPath(vertexStart, vertexGoal, parentMap);
    }

    private boolean bfsSearch(final Vertex vertexStart, final Vertex vertexGoal,
                              final Map<Vertex, Vertex> parentMap) {
        final Set<Vertex> visited = new HashSet<>();
        final Queue<Vertex> toExplore = new LinkedList<>();

        toExplore.add(vertexStart);

        boolean found = false;

        while (!toExplore.isEmpty()) {
            final Vertex curr = toExplore.remove();
            if (curr.equals(vertexGoal)) {
                found = true;
                break;
            }

            final List<Vertex> neighbors = curr.getEdges().stream()
                    .map(Edge::getEndPoint)
                    .map(point -> vertexMap.get(point))
                    .filter(Objects::nonNull)
                    .collect(toList());

            final ListIterator<Vertex> it = neighbors.listIterator(neighbors.size());
            while (it.hasPrevious()) {
                final Vertex next = it.previous();
                if (!visited.contains(next)) {
                    visited.add(next);
                    parentMap.put(next, curr);
                    toExplore.add(next);
                }
            }
        }
        return found;
    }

    /**
     * Reconstruct the parent path
     */
    private static List<GeographicPoint> constructPath(final Vertex vertexStart, final Vertex vertexGoal,
                                                       final Map<Vertex, Vertex> parentMap) {
        final LinkedList<Vertex> path = new LinkedList<>();
        Vertex curr = vertexGoal;
        while (curr != vertexStart) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }
        path.addFirst(vertexStart);

        return path.stream()
                .map(Vertex::getLocation)
                .collect(toList());
    }

    /** Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal The goal location
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /** Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        return searchOnWeightedGraph(start, goal, nodeSearched, (a, b) -> 0.0);
    }

    private List<GeographicPoint> searchOnWeightedGraph(GeographicPoint start, GeographicPoint goal,
                                                        Consumer<GeographicPoint> nodeSearched,  BiFunction<Vertex, Vertex, Double> func) {

        if (!arePreconditionsFulfilled(start, goal))  { return new LinkedList<>(); }

        final Vertex vertexStart = vertexMap.get(start);
        final Vertex vertexGoal = vertexMap.get(goal);

        final Map<Vertex, Vertex> parentMap = new HashMap<>();
        final Set<Vertex> visited = new HashSet<>();
        final Queue<Vertex> toExplore = new PriorityQueue<>();

        vertexStart.setDistance(0d);
        vertexStart.setActualDistance(0d);

        toExplore.add(vertexStart);
        boolean found = false;

        while (!toExplore.isEmpty()) {

            final Vertex curr = toExplore.remove();

            if (visited.contains(curr)) { continue; }

            visited.add(curr);
            nodeSearched.accept(curr.getLocation());

            if (curr.equals(vertexGoal)) {
                found = true;
                break;
            }

            final Map<Vertex, Double> distancesMap  = calculateDistanesMap(curr);

            for(Vertex neighbor : getNeighbors(curr)){
                if (visited.contains(neighbor)) { continue; }

                double distanceOfNode = curr.getActualDistance() + distancesMap.get(neighbor);
                if (distanceOfNode < neighbor.getActualDistance()) {
                    neighbor.setActualDistance(distanceOfNode);
                    distanceOfNode += func.apply(neighbor, curr);
                    neighbor.setDistance(distanceOfNode);
                    parentMap.put(neighbor, curr);
                    toExplore.offer(neighbor);
                }
            }
        }

        if (!found) {
            System.out.println("No path exists");
            return new ArrayList<>();
        }

        return constructPath(vertexStart, vertexGoal, parentMap);
    }

    public List<Vertex> getNeighbors(final Vertex vertex) {
        return vertex.getNeighbors().stream()
                .map(geographicPoint -> vertexMap.get(geographicPoint))
                .collect(toList());
    }

    public Map<Vertex, Double> calculateDistanesMap(final Vertex vertex) {
        HashMap<Vertex, Double> distancesMap = new HashMap<>();

        for (Edge edge : vertex.getEdges()) {
            distancesMap.put(vertexMap.get(edge.getEndPoint()), edge.getLength());
        }
        return distancesMap;
    }

    /** Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal The goal location
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /** Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     *   start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        return searchOnWeightedGraph(start, goal,  nodeSearched, (a, b) -> a.getLocation().distance(b.getLocation()));
    }

    private boolean arePreconditionsFulfilled(GeographicPoint start, GeographicPoint goal) {
        if (isNull(start) || isNull(goal)) {
            System.out.println("Start or goal node is null!  No path exists.");
            return false;
        }

        if (isNull(vertexMap.get(start)) || isNull(vertexMap.get(goal))) {
            System.out.println("Start or goal geographic point doesn't exist in Graph.");
            return false;
        }
        return true;
    }

    public static void main(String[] args) {
        /*
         * Basic testing System.out.print("Making a new map..."); MapGraph
         * theMap = new MapGraph(); System.out.print(
         * "DONE. \nLoading the map...");
         * GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
         * System.out.println("DONE.");
         */

        // more advanced testing
        System.out.print("Making a new map...");
        final MapGraph theMap_A = new MapGraph();
        System.out.print("DONE. \nLoading the map...");

        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap_A);
        System.out.println("DONE.");

        System.out.println("Num nodes: " + theMap_A.getNumVertices());
        System.out.println("Num edges: " + theMap_A.getNumEdges());

        final List<GeographicPoint> route_A = theMap_A.bfs(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));

        System.out.println(route_A);

        // Use this code in Week 3 End of Week Quiz MapGraph
        final MapGraph theMap_B = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/maps/utc.map", theMap_B);
        System.out.println("DONE.");

        final GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);

        final GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

        final List<GeographicPoint> route_B_a = theMap_B.dijkstra(start, end);
        final List<GeographicPoint> route_B_b = theMap_B.aStarSearch(start, end);

        System.out.println(route_B_a);
        System.out.println(route_B_b);
    }

}