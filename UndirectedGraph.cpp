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
            delete(vert.second);
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
        /* vertices that hold to and from */
        Vertex *fromV = new Vertex(from);
        Vertex *toV = new Vertex(to);

        /**
         * if neither to or from is in graph add them both and create edge
         * between them
         */
        if(vertices.find(to) == vertices.end() &&
           vertices.find(from) == vertices.end()){
            vertices.insert(make_pair(to, toV));
            vertices.insert(make_pair(from, fromV));
            fromV->addEdge(toV, cost, length); 
            toV->addEdge(fromV, cost, length);
        }
        /**
         * if to is in graph and from is not, add from and create edge to 
         * the vertex to(already in the graph)
         */
        else if(vertices.find(to) != vertices.end() &&
                vertices.find(from) == vertices.end()){
            toV = vertices.at(to);
            vertices.insert(make_pair(from, fromV));
            fromV->addEdge(toV, cost, length);
            toV->addEdge(fromV, cost, length);
        }
        /**
         * if from is in the graph and to is not, add to and create edge to
         * the vertex from(already in the graph
         */
        else if(vertices.find(to) == vertices.end() &&
                vertices.find(from) != vertices.end()){
            fromV = vertices.at(from);
            vertices.insert(make_pair(to, toV));
            fromV->addEdge(toV, cost, length);
            toV->addEdge(fromV, cost, length);
        }
        /**
         * if both to and from are in the graph, create an edge between them
         */
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

        /**
         * iterate through all vertices and add the total edge cost at each 
         * vertex together so we get the total edge cost of all the edges in
         * the graph. Divide by two at the end to account for double counting
         * the edge costs
         */
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
        // if no vertices in graph, just return
        if(this->vertices.size() == 0){
            return;
        }
        // UndirectedGraph to hold our minimum spanning tree
        UndirectedGraph* MST = new UndirectedGraph();

        // priority queue to hold our edges
        priority_queue<Edge, vector<Edge>> pq;

        // vector to hold all the vertices
        vector<Vertex*> allVerts;

        // set all vertices to not visited
        for(auto vert : vertices){
            vert.second->setVisited(false);    
            allVerts.push_back(vert.second);
        }
        // start from arbitrary vertex in the graph and mark visited
        Vertex* tmpVert = allVerts.front();
        tmpVert->setVisited(true);

        // iterate through all edges of this vertex and add to priority queue
        for(auto edge : tmpVert->edges){
            pq.push(edge.second);
        }
        
        Vertex* tmpTo;
        Vertex* tmpFrom;
        unsigned int costE, lengthE;

        /**
         * while priority queue is not empty
         */
        while(!(pq.empty())){
            // get to, from, cost and length from the edge at the top of the 
            // priority queue
            tmpFrom = pq.top().getFrom();
            costE = pq.top().getCost();
            lengthE = pq.top().getLength();
            tmpTo = pq.top().getTo();

            // remove edge from priority queue
            pq.pop();

            // if to has been visited, do nothing and move onto next edge
            if(tmpTo->wasVisited()){
                continue;                 
            }
            // if to has not been visited, proceed
            else{
                // mark to as visited and add edge to minimum spanning tree
                tmpTo->setVisited(true);
                MST->addEdge(tmpFrom->getName(), tmpTo->getName(),
                             costE, lengthE); 

                // add all edges connected to to, to the priority queue
                for(auto edge : tmpTo->edges){
                    if(!(edge.second.getTo()->wasVisited())){
                        pq.push(edge.second);
                    }
                }
            }
        }
        // set current tree equal to the minimum spanning tree
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
        /**
         * priority queue that holds a pair of vertices and their distances
         * from the starting vertex
         */
        priority_queue<pair<Vertex*,unsigned int>, 
                       vector<pair<Vertex*, unsigned int>>,
                       UndirectedGraph::DijkstraVertexComparator> pq;
        pair<Vertex*, unsigned int> vPair;

        /**
         * iterate through all vertices in the graph, set their distances to
         * "infinity" and mark them as not visited, then find the vertex 'from'
         * and set that as visited and its distance to 0
         */
        for(auto vert : vertices){
            vert.second->setDistance(INT_MAX);
            vert.second->setVisited(false);
            if(vert.second->getName() == from){
                vert.second->setDistance(0);
                vPair = make_pair(vert.second, vert.second->getDistance());
            }
        }

        // unordered map holding each vertex' distance from the initial vertex
        unordered_map<string, unsigned int> distances;

        // insert the vertex 'from' and its distance(0) into the unordered map
        distances.insert(make_pair(from, 0));

        // put the pair 'from' and 0 into the priority queue
        pq.push(vPair);
        pair<Vertex*, unsigned int> tempPair;
        pair<Vertex*, unsigned int> tempPair2;
        unsigned int tmpDist1 = 0;
        unsigned int tmpDist2 = 0;
        unsigned int totalDist = 0;

        /**
         * while priorty queue is not empty
         */
        while(!(pq.empty())){
            // get pair at top of priority queue
            tempPair = pq.top();

            //remove this pair from the priority queue
            pq.pop();

            // if the vertex in this pair is marked as visited, go to next 
            // iteration of this loop
            if(tempPair.first->wasVisited()){
                continue;
            }
            // if vertex is not marked as visited, proceed
            else{
                // mark vertex as visited
                tempPair.first->setVisited(true);

                /**
                 * iterate through all edges connected to the vertex of this
                 * pair
                 */
                for(auto edge : tempPair.first->edges){
                    // if edge's 'to' has not been visited, proceed
                    if(!(edge.second.getTo()->wasVisited())){
                        // set temp distances to distance of vertex, and length
                        // of edge
                        tmpDist1 = tempPair.second;
                        tmpDist2 = edge.second.getLength();
                        /**
                         * if the sum of the temp distances is less than the 
                         * distance of edge's 'to', proceed
                         */
                        if(tmpDist1 + tmpDist2 < 
                           edge.second.getTo()->getDistance()){
                            // set distance of edge's 'to' to the sum of the
                            // temp distances
                            edge.second.getTo()->setDistance(tmpDist2+tmpDist1);

                            // if distance of this edge's 'to' is not set, set
                            // set it to the sum of the temp distances 
                            if(distances[edge.second.getTo()->getName()] ==
                               0){
                                distances[edge.second.getTo()->getName()] =
                                tmpDist1 + tmpDist2;
                            }
                            // if sum of temp distances is less than the set 
                            // distance of this edge's to, then set the 
                            // distance of this edge's 'to' to this sum  
                            else if(tmpDist1 + tmpDist2 <
                                    distances[edge.second.getTo()->getName()]){
                                distances[edge.second.getTo()->getName()] = 
                                tmpDist1 + tmpDist2;
                            }
                            // put vertex and distance pair in priority queue
                            tempPair2 = make_pair(edge.second.getTo(),
                                        edge.second.getTo()->getDistance());
                            pq.push(tempPair2); 
                       }
                    }
                }
            }             
        }
        /**
         * iterate through the unordered map distances, and add up all the 
         * distances, to get the total distance(shortest path) from the initial
         * vertex
         */
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

        /**
         * iterate through all the vertices in the graph and use Dijkstra's 
         * algorithm to find the combined distance from all vertices to all
         * other vertices in the graph
         */
        for(auto vert : vertices){
            totalDist += this->totalDistance(vert.second->getName());
        }
        return totalDist;
    }
