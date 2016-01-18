package roadgraph;

import geography.GeographicPoint;

/**
 * Road (edge) on the road map graph. Among other, keeps the data of its starting and destination points.
 * See also MapNode.java
 * <p>
 * Created by vitaliistukalov on 1/18/16.
 */
public class MapEdge {
    private GeographicPoint from;
    private GeographicPoint to;
    private String roadName;
    private String roadType;
    private double length;
    private MapNode destination;

    /**
     * Create a new edge
     *
     * @param from     starting location
     * @param to       destination node
     * @param roadName street (road) name
     * @param roadType type of the road
     * @param length   length of the road
     */
    public MapEdge(GeographicPoint from, MapNode to, String roadName, String roadType, double length) {
        this.from = from;
        this.to = to.getLocation();
        this.destination = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public MapNode getDestination() {
        return destination;
    }
}
