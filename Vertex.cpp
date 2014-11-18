#include "Vertex.hpp"

#include <string>
#include <iostream>
// Method implementations here

using namespace std;

    /**
     * Initialize the Vertex with the given name.
     */
     Vertex::Vertex(const string &name) : name(name){}
    
    /**
     * Add an edge to this Vertex. If an edge already exists to the given
     * vertex, updates the cost and length of the edge to match the
     * passed parameters.
     */
    bool Vertex::addEdge(Vertex *to, unsigned int cost, unsigned int length){
        Edge *myEdge = new Edge(this, to, cost, length);
        if(edges.find(to->getName()) == edges.end()){
            edges.insert(make_pair(to->getName(), *myEdge));
            return true;
        }
        else{
            edges.at(to->getName()).setCost(cost);
            edges.at(to->getName()).setLength(length);
            return false;
        }
    }
      
    /**
     * Returns the Vertex's name.
     */
    const string &Vertex::getName() const{
        return this->name;
    }

    /**
     * Gets the Vertex's distance value.
     */
    unsigned int Vertex::getDistance() const{
        return this->distance;
    }

    /**
     * Sets the Vertex's distance value.
     */
    void Vertex::setDistance(unsigned int distance){
        this->distance = distance;
    }

    /**
     * Gets the Vertex's visited state.
     */
    bool Vertex::wasVisited() const{
        return this->visited;
    }

    /**
     * Sets the Vertex's visited state.
     */
    void Vertex::setVisited(bool visited){
        this->visited = visited;
    }

    /**
     * Clears all edges from this Vertex.
     */
    void Vertex::clearEdges(){
        edges.clear();
    }

    /**
     * Gets total cost of all edges terminating at this Vertex.
     */
    unsigned int Vertex::totalEdgeCost() const{
        int cost = 0;
        for(auto key: edges){
            cost += key.second.getCost();
        }
        return cost;
    }

    /**
     * Returns a reference to the internal map of Edges.
     * Used by UndirectedGraph for Dijkstra/Prim algorithms.
     */
    const std::unordered_map<std::string, Edge> &Vertex::getEdges() const{
        return edges;
    }
