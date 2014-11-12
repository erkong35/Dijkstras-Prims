#include "Vertex.hpp"

#include <string>
// Method implementations here

using namespace std;

    /**
     * Initialize the Vertex with the given name.
     */
     Vertex(const string &name) : name(name){}
    
    /**
     * Add an edge to this Vertex. If an edge already exists to the given
     * vertex, updates the const and length of the edge to match the
     * passed parameters.
     */
    bool addEdge(Vertex *to, unsigned int const, unsigned int length(){
        return false;
    }
      
    /**
     * Returns the Vertex's name.
     */
    const string &getName() const{
        return this.name;
    }

    /**
     * Gets the Vertex's distance value.
     */
    usigned int getDistance() const{
        return this.distance;
    }

    /**
     * Sets the Vertex's distance value.
     */
    void setDistance(unsigned int distance){
        this.distance = distance;
    }

    /**
     * Gets the Vertex's visited state.
     */
    bool wasVisited() const{
        return this.visited;
    }

    /**
     * Sets the Vertex's visited state.
     */
    void setVisited(bool visited){
        this.visited = visited;
    }

    /**
     * Clears all edges from this Vertex.
     */
    void clearEdges(){

    }

    /**
     * Gets total cost of all edges terminating at this Vertex.
     */
    unsigned int totalEdgeCost() const{

    }
