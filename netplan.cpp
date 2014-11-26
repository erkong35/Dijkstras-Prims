#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "UndirectedGraph.hpp"
using namespace std;

/**
 * Entry point into the netplan program.
 *
 * -Reads a file from the filesystem according to the specification for
 *  PA3, creating an UndirectedGraph.
 * -Finds the total cost & ping time of the graph as presented in the input
 *  file.
 * -Determines the minimum cost graph from the original graph.
 * -Finds the total cost & ping time of the minimum cost graph.
 * -Finds the change of cost & ping time from the original graph to the
 *  minimum cost graph.
 * -Prints the results to stdout.
 *
 * Usage:
 *   ./netplan infile
 *
 */
int main(int argc, char **argv) {
    // Data Structs to hold the variables
    vector<string> to;
    vector<string> from;
    vector<unsigned int> cost;
    vector<unsigned int> length;

    // if number of arguments passed in is not 2, print usage
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " infile" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::ifstream in(argv[1]);
    if (!in) {
        std::cerr << "Unable to open file for reading." << std::endl;
        return EXIT_FAILURE;
    }

    // string and int variables for adding to the vectors
    string str;
    unsigned int i;

    /**
     * while file is not empty, parse input so that we can make a graph from
     * the input
     */
    while(true){
        in >> str;
        if(in.eof()) break;
        to.push_back(str);

        in >> str;
        from.push_back(str);

        in >> i;
        cost.push_back(i);

        in >> i;
        length.push_back(i);
    }

    /**
     * create undirected graph from the input file
     */
    UndirectedGraph *bob = new UndirectedGraph();
    for(unsigned int j = 0; j < to.size(); j++){
        bob->addEdge(to[j], from[j], cost[j], length[j]);
    }

    // get total edge cost of inital graph
    unsigned int totalCost = bob->totalEdgeCost();

    // get total distance of inital graph, by using Dijkstra's algorithm on 
    // all the vertices
    unsigned int totalTime = bob->totalDistance();

    // create minimum spanning tree of the inital graph, using Prim's algorithm
    bob->minSpanningTree();

    // get total edge cost of minimum spanning tree
    unsigned int mstCost = bob->totalEdgeCost();

    // get total distance of minimum spanning tree, using Dijkstra's algorithm
    // on all the vertices
    unsigned int mstTime = bob->totalDistance();

    // print out all the costs and distances
    cout << totalCost << endl;
    cout << mstCost << endl;
    cout << totalCost - mstCost << endl;
    cout << totalTime << endl;
    cout << mstTime << endl;
    cout << mstTime - totalTime << endl;

    // delete graph
    delete(bob);

    return EXIT_SUCCESS;
}
