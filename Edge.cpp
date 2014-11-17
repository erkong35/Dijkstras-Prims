#include "Edge.hpp"

// Method implementations here

using namespace std;


    /**
     * Constructs an Edge from the given parameters.
     */
    Edge::Edge(Vertex *from, Vertex *to,
            unsigned int cost,
            unsigned int length) : from(from), to(to), cost(cost),
            length(length) {}

    /**
     * Returns a pointer to the Vertex that this Edge originates
     * from.
     */
    Vertex* Edge::getFrom() const{
        return this->from;
    }

    /**
     * Returns a pointer to the Vertext that this Edge terminates
     * at.
     */
    Vertex* Edge::getTo() const{
        return this->to;
    }

    /**
     * Sets the cost of this Edge.
     */
    void Edge::setCost(unsigned int cost){
        this->cost = cost;
    }

    /**
     * Returns the cost of this Edge.
     */
    unsigned int Edge::getCost() const{
        return this->cost;
    }

    /**
     * Sets the length of this Edge.
     */
    void Edge::setLength(unsigned int length){
        this->length = length;
    }

    /**
     * Returns the length of this Edge.
     */
    unsigned int Edge::getLength() const{
        return this->length;
    }

    /**
     * Compares this Edge to another Edge. Suitable for
     * use witha priority queue where Edges with the lowest
     * weight have the highest priority.
     * 
     * Returns true if this Edge's cost is more than 
     * right's cost.
     */
    bool Edge::operator<(const Edge &right) const{
        return cost > right.cost;
    }
