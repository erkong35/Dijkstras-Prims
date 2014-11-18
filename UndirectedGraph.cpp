#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"

using namespace std;
    /**
     * Constructs an empty UndirectedGraph with no vertices and
     * no edges.
     */
    UndirectedGraph::UndirectedGraph(){}

    /**
     * Destructs an UndirectedGraph.
     */
    UndirectedGraph::~UndirectedGraph(){}
    
    /**
     * Inserts an edge into the graph. If an edge already exists between
     * the vertices, updates the cost and length of the edge to match the
     * passed parameters.
     *
     * If either of the named vertices does not exist, it is created.
     */
    void UndirectedGraph::addEdge(const string &from, const string &to,
            unsigned int cost, unsigned int length){
        Vertex *fromV = new Vertex(from);
        Vertex *toV = new Vertex(to);

        if(vertices.find(to) == vertices.end() &&
           vertices.find(from) == vertices.end()){
            vertices.insert(make_pair(to, toV));
            vertices.insert(make_pair(from, fromV));
            fromV->addEdge(toV, cost, length); 
        }
        else if(vertices.find(to) != vertices.end() &&
                vertices.find(from) == vertices.end()){
            toV = vertices.at(to);
            vertices.insert(make_pair(from, fromV));
            fromV->addEdge(toV, cost, length);
        }
        else if(vertices.find(to) == vertices.end() &&
                vertices.find(from) != vertices.end()){
            fromV = vertices.at(from);
            vertices.insert(make_pair(to, toV));
            fromV->addEdge(toV, cost, length);
        }
        else{
            toV = vertices.at(to);
            fromV = vertices.at(from);
            fromV->addEdge(toV, cost, length);
        }
    }

    /**
     * Returns the total cost of all edges in the graph.
     *
     * Since this graph is undirected, is calcualted as the cost
     * of all Edges terminating at all Vertices, divided by 2.
     */
    unsigned int UndirectedGraph::totalEdgeCost() const{
        return -1;
    }
    
    /**
     * Removes all edges from the graph except those necessary to
     * form a minimum cost spanning tree of all vertices using Prim's
     * algorithm.
     *
     * The graph must be in a state where such a spanning tree
     * is possible. To call this method when a spanning tree is
     * impossible is undefined behavior.
     */
    void UndirectedGraph::minSpanningTree(){

    }
    
    /**
     * Determines the combined distance from the given Vertex to all
     * other Vertices in the graph using Dijkstra's algorithm.
     *
     * Returns max possible distance if the given Vertex does not appear
     * in the graph, or if any of the Vertices in the graph are not
     * reachable from the given Vertex. Otherwise, returns the combined
     * distance.
     */
    unsigned int UndirectedGraph::totalDistance(const std::string &from){
        return -1;
    } 
    
    /**
     * Determines the combined distance from all Vertices to all other
     * Vertices in the graph.
     *
     * Returns max possible distance if the graph is not connected.
     */
    unsigned int UndirectedGraph::totalDistance(){
        return -1;
    }
    
    /**
     * Comparison functor for use with Dijkstra's algorithm. Allows Vertices
     * to be added to a priority queue more than once, with different weights.
     *
     * Each pair represents a Vertex and its weight when it was added to the
     * queue. This guarantees that the weight used to order the Vertices in
     * the queue never changes (a required invariant of a priority queue),
     * even though the weight of the Vertex itself may change.
     *
     * Returns true if left's weight when it was inserted into the queue is
     * higher than right's weight when it was inserted into the queue.
     */
    class DijkstraVertexComparator {
      public:
        bool operator()(const std::pair<Vertex*, unsigned int> &left,
                const std::pair<Vertex*, unsigned int> &right);
    };
    
