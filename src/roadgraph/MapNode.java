package roadgraph;

import geography.GeographicPoint;

import java.util.HashSet;

/**
 * Vertex (node) on the road map graph. Keeps its geographic location and the set of roads (edges) intersecting in it.
 * See also MapEdge.java
 * <p>
 * Created by vitaliistukalov on 1/18/16.
 */
public class MapNode {
    private GeographicPoint location;
    private HashSet<MapEdge> edges;

    /**
     * Create and initialize a new MapGraph
     *
     * @param location geographic location of a new node
     */
    public MapNode(GeographicPoint location) {
        this.location = location;
        this.edges = new HashSet<>();
    }

    /**
     * Create a new edge and add it to this node.
     *
     * @param to       where to go from here
     * @param roadName street(road) name
     * @param roadType type of the road
     * @param length   distance between two points
     */
    public void addEdge(MapNode to, String roadName, String roadType, double length) {
        MapEdge edge = new MapEdge(this.location, to, roadName, roadType, length);
        this.edges.add(edge);

    }

    public int getNumEdges() {
        return this.edges != null ? this.edges.size() : 0;
    }

    public HashSet<MapEdge> getEdges() {
        return this.edges;
    }

    public GeographicPoint getLocation() {
        return location;
    }
}
