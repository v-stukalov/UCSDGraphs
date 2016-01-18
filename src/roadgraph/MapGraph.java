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
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 *         <p>
 *         A class which represents a graph of geographic locations
 *         Nodes in the graph are intersections between
 */
public class MapGraph {
    // keep the road map graph
    Map<GeographicPoint, MapNode> graph;

    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        graph = new HashMap<>();
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return graph.keySet().size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        return graph.keySet();
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        int numEdges = 0;
        for (Map.Entry entry : graph.entrySet()) {
            numEdges += ((MapNode) entry.getValue()).getNumEdges();
        }
        return numEdges;
    }


    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        if (location != null && !graph.containsKey(location)) {
            MapNode node = new MapNode(location);
            graph.put(location, node);
            return true;
        }
        return false;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {
        MapNode fromNode = graph.get(from);
        MapNode toNode = graph.get(to);
        if (from == null || to == null || fromNode == null || toNode == null || roadName == null || roadType == null || length < 0) {
            throw new IllegalArgumentException();
        }
        fromNode.addEdge(toNode, roadName, roadType, length);
    }


    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        if (start == null || goal == null) {
            System.err.println("Start or goal node is null! No path exists.");
            return null;
        }
        Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
        boolean found = bfsSearch(start, goal, parentMap, nodeSearched);
        if (!found) {
            System.err.println("No path exists.");
            return null;
        }
        return constructPath(start, goal, parentMap);
    }

    /**
     * Construct the path based on linked locations map
     *
     * @param start     The starting location
     * @param goal      The goal location
     * @param parentMap Contains the path that we are looking for. The key of this map means the current position on a path, whereas the value is the next place we have to visit
     * @return the path between two locations
     */
    private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentMap) {
        LinkedList<GeographicPoint> path = new LinkedList<>();
        GeographicPoint curr = goal;
        while (!curr.equals(start)) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }
        path.addFirst(start);
        return path;
    }

    /**
     * Find the path between two locations and store it parentMap variable
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param parentMap    Contains the path that we are looking for
     * @param nodeSearched A hook for visualization
     * @return true is the path has been found, and false otherwise
     */
    private boolean bfsSearch(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentMap, Consumer<GeographicPoint> nodeSearched) {
        Set<GeographicPoint> visited = new HashSet<>();
        Queue<GeographicPoint> toExplore = new LinkedList<>();
        toExplore.add(start);
        boolean found = false;
        while (!toExplore.isEmpty()) {
            GeographicPoint curr = toExplore.remove();
            if (curr.equals(goal)) {
                found = true;
                break;
            }
            Set<MapEdge> edges = graph.get(curr).getEdges();
            Iterator<MapEdge> it = edges.iterator();
            while (it.hasNext()) {
                MapEdge nextEdge = it.next();
                MapNode destination = nextEdge.getDestination();
                GeographicPoint location = destination.getLocation();
                if (!visited.contains(location)) {
                    // Hook for visualization.  See writeup.
                    nodeSearched.accept(location);
                    visited.add(location);
                    parentMap.put(location, curr);
                    toExplore.add(location);
                }
            }
        }
        return found;
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }


    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        System.out.println("DONE.");

        // You can use this method for testing.

		/* Use this code in Week 3 End of Week Quiz
        MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/

    }

}
