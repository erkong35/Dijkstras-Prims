#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"
#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;
    /**
     * Constructs an empty UndirectedGraph with no vertices and
     * no edges.
     */
    UndirectedGraph::UndirectedGraph(){}

    /**
     * Destructs an UndirectedGraph.
     */
    UndirectedGraph::~UndirectedGraph(){
        for(auto vert : vertices){
            vert.second->clearEdges();
        }
    }
    
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
            toV->addEdge(fromV, cost, length);
        }
        else if(vertices.find(to) != vertices.end() &&
                vertices.find(from) == vertices.end()){
            toV = vertices.at(to);
            vertices.insert(make_pair(from, fromV));
            fromV->addEdge(toV, cost, length);
            toV->addEdge(fromV, cost, length);
        }
        else if(vertices.find(to) == vertices.end() &&
                vertices.find(from) != vertices.end()){
            fromV = vertices.at(from);
            vertices.insert(make_pair(to, toV));
            fromV->addEdge(toV, cost, length);
            toV->addEdge(fromV, cost, length);
        }
        else{
            toV = vertices.at(to);
            fromV = vertices.at(from);
            fromV->addEdge(toV, cost, length);
            toV->addEdge(fromV, cost, length);
        }
    }

    /**
     * Returns the total cost of all edges in the graph.
     *
     * Since this graph is undirected, is calcualted as the cost
     * of all Edges terminating at all Vertices, divided by 2.
     */
    unsigned int UndirectedGraph::totalEdgeCost() const{
        unsigned int totalCost = 0;
        for(auto vert : vertices){
            totalCost += vert.second->totalEdgeCost();
        }
        return totalCost/2;
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
        if(this->vertices.size() == 0){
            return;
        }
        UndirectedGraph* MST = new UndirectedGraph();
        priority_queue<Edge, vector<Edge>> pq;
        vector<Vertex*> allVerts;
        for(auto vert : vertices){
            vert.second->setVisited(false);    
            allVerts.push_back(vert.second);
        }
        Vertex* tmpVert = allVerts.front();
        tmpVert->setVisited(true);

        for(auto edge : tmpVert->edges){
            pq.push(edge.second);
        }
        
        Vertex* tmpTo;
        Vertex* tmpFrom;
        unsigned int costE, lengthE;
        while(!(pq.empty())){
            tmpFrom = pq.top().getFrom();
            costE = pq.top().getCost();
            lengthE = pq.top().getLength();
            tmpTo = pq.top().getTo();
            pq.pop();
            if(tmpTo->wasVisited()){
                continue;                 
            }
            else{
                tmpTo->setVisited(true);
                MST->addEdge(tmpFrom->getName(), tmpTo->getName(),
                             costE, lengthE); 
                for(auto edge : tmpTo->edges){
                    if(!(edge.second.getTo()->wasVisited())){
                        pq.push(edge.second);
                    }
                }
            }
        }
        this->vertices = MST->vertices;
    }
    
    /**
     * Determines the combined distance from the given Vertex to all
     * other Vertices in the graph using Dijkstra's algorithm.
     *
     * Returns max possible distance if the given Vertex does not appear
     * in the graph, or if any of the Vertices in the graph are not
     * reach
     * distance.
     */
    unsigned int UndirectedGraph::totalDistance(const std::string &from){
        priority_queue<pair<Vertex*,unsigned int>, 
                       vector<pair<Vertex*, unsigned int>>,
                       UndirectedGraph::DijkstraVertexComparator> pq;
        Vertex* tmpV;
        pair<Vertex*, unsigned int> vPair;
        for(auto vert : vertices){
            vert.second->setDistance(INT_MAX);
            vert.second->setVisited(false);
            if(vert.second->getName() == from){
                //tmpV = vert.second;
                vert.second->setDistance(0);
                vPair = make_pair(vert.second, vert.second->getDistance());
            }
        }
       // tmpV->setDistance(0);
        //pair<Vertex*, unsigned int> vPair = make_pair(tmpV, 
                                                    //  tmpV->getDistance());
        unordered_map<string, unsigned int> distances;
        distances.insert(make_pair(from, 0));
        pq.push(vPair);
        pair<Vertex*, unsigned int> tempPair;
        pair<Vertex*, unsigned int> tempPair2;
        unsigned int tmpDist1 = 0;
        unsigned int tmpDist2 = 0;
        unsigned int totalDist = 0;
        while(!(pq.empty())){
            tempPair = pq.top();
            pq.pop();
            if(tempPair.first->wasVisited()){
                continue;
            }
            else{
                tempPair.first->setVisited(true);
                for(auto edge : tempPair.first->edges){
                    if(!(edge.second.getTo()->wasVisited())){
                        //tmpDist1 = tempPair.first->getDistance();
                        tmpDist1 = tempPair.second;
                        tmpDist2 = edge.second.getLength();
                        //tempPair.first->setDistance(tmpDist1 + tmpDist2);
                        if(tmpDist1 + tmpDist2 < 
                           edge.second.getTo()->getDistance()){
                            edge.second.getTo()->setDistance(tmpDist2+tmpDist1);
                            if(distances[edge.second.getTo()->getName()] ==
                               NULL){
                                distances[edge.second.getTo()->getName()] =
                                tmpDist1 + tmpDist2;
                            }
                            else if(tmpDist1 + tmpDist2 <
                                    distances[edge.second.getTo()->getName()]){
                                distances[edge.second.getTo()->getName()] = 
                                tmpDist1 + tmpDist2;
                            }
                            //totalDist += tmpDist2 + tmpDist1;
                            tempPair2 = make_pair(edge.second.getTo(),
                                        edge.second.getTo()->getDistance());
                            pq.push(tempPair2); 
                       }
                    }
                }
            }             
        }
        for(auto dist : distances){
            totalDist += dist.second; 
        }
        return totalDist;
    } 
    
    /**
     * Determines the combined distance from all Vertices to all other
     * Vertices in the graph.
     *
     * Returns max possible distance if the graph is not connected.
     */
    unsigned int UndirectedGraph::totalDistance(){
        unsigned int totalDist = 0;
        for(auto vert : vertices){
            totalDist += this->totalDistance(vert.second->getName());
        }
        return totalDist;
    }
