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
    vector<int> cost;
    vector<int> length;

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
    int i;

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

    UndirectedGraph *bob = new UndirectedGraph();
    for(unsigned int j = 0; j < to.size(); j++){
        bob->addEdge(to[j], from[j], cost[j], length[j]);
    }
    cout << bob->totalEdgeCost() << endl;
    // Implementation here

    return EXIT_SUCCESS;
}
