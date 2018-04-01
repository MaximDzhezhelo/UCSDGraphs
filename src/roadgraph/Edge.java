package roadgraph;

import geography.GeographicPoint;

/**
 * A class that represents a edge, represented using a graph.
 *
 * @author makson
 *
 */
public class Edge {

    private GeographicPoint startPoint;
    private GeographicPoint endPoint;
    private String streetName;
    private String roadType;
    private Double length;

    /**
     * Create a new Edge with completed fields
     */
    public static Edge of(final GeographicPoint startPoint,final  GeographicPoint endPoint,
                          final String streetName,final String roadType, final Double length){
        final Edge edge = new Edge();
        edge.setStartPoint(startPoint);
        edge.setEndPoint(endPoint);
        edge.setStreetName(streetName);
        edge.setRoadType(roadType);
        edge.setLength(length);
        return edge;
    }

    public GeographicPoint getStartPoint() { return startPoint; }
    public void setStartPoint(GeographicPoint startPoint) { this.startPoint = startPoint; }

    public GeographicPoint getEndPoint() { return endPoint; }
    public void setEndPoint(GeographicPoint endPoint) { this.endPoint = endPoint; }

    public String getStreetName() {return streetName; }
    public void setStreetName(String streetName) { this.streetName = streetName; }

    public String getRoadType() { return roadType; }
    public void setRoadType(String roadType) { this.roadType = roadType; }

    public Double getLength() { return length; }
    public void setLength(Double length) { this.length = length; }

    @Override
    public String toString() {
        return "Edge{" +
                "startPoint=" + startPoint +
                ", endPoint=" + endPoint +
                ", streetName='" + streetName + '\'' +
                ", roadType='" + roadType + '\'' +
                ", length=" + length +
                '}';
    }
}
