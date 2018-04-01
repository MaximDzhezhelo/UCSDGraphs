package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A class that represents a vertex, represented using a graph.
 *
 * @author makson
 *
 */
public class Vertex {
    private GeographicPoint location;
    private String name;
    private List<Edge> edges = new ArrayList<>();

    /**
     * Create a new vertex with completed location
     */
    public static Vertex of(GeographicPoint location){
        final Vertex vertex = new Vertex();
        vertex.setLocation(location);
        return vertex;
    }

    public GeographicPoint getLocation() { return location; }
    public void setLocation(GeographicPoint location) { this.location = location; }

    public String getName() { return name; }
    public void setName(String name) { this.name = name;}

    public List<Edge> getEdges() { return edges; }
    public void setEdges(List<Edge> edges) { this.edges = edges; }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Vertex)) return false;
        Vertex vertex = (Vertex) o;
        return Objects.equals(getLocation(), vertex.getLocation()) &&
                Objects.equals(getName(), vertex.getName()) &&
                Objects.equals(getEdges(), vertex.getEdges());
    }

    @Override
    public int hashCode() {
        return Objects.hash(getLocation(), getName(), getEdges());
    }

    @Override
    public String toString() {
        return "Vertex{" +
                "location=" + location +
                ", name='" + name + '\'' +
                ", edges=" + edges +
                '}';
    }
}
