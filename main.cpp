#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <array>
#include <set>
#include <sstream>
#include <string>
#include <fstream>
#include <streambuf>
#include <list>
#include <map>

/*
struct AdjListNode {
    int from;
    int dest;
    int weight;
};

struct WatchList {
    int distance;
    int previvousNode;
    int marked;
};

AdjListNode berrechneMin(std::vector<AdjListNode> nodes, int startingNode) {
    int minNextNodeName = 0;
    int minNextNodeWeight = 0;
    AdjListNode returnNode;
    std::for_each(nodes.cbegin(), nodes.cend(),
                  [&startingNode, &minNextNodeName, &minNextNodeWeight](AdjListNode node) {
                      if ((startingNode == node.from) && (minNextNodeWeight < node.weight)) {
                          minNextNodeName = node.dest;
                          minNextNodeWeight = node.weight;
                      }
                  });
    returnNode.weight = minNextNodeWeight;
    returnNode.from = minNextNodeName;
    return returnNode;
}

std::vector<AdjListNode> readFile() {
    std::ifstream file("/home/jonagurk/Dokumente/USA-road-d.NY.gr");
    std::vector<AdjListNode> nodes;
    AdjListNode node;
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::string prefix = "a";
            if (line.rfind(prefix, 0) == 0) {
                //printf( line.c_str());
                int strpos = line.find(" ");
                line.erase(0, strpos + 1);

                strpos = line.find(" ");
                node.from = std::stoi(line.substr(0, strpos));
                line.erase(0, strpos + 1);

                strpos = line.find(" ");
                node.dest = std::stoi(line.substr(0, strpos));
                line.erase(0, strpos + 1);

                strpos = line.find(" ");
                node.weight = std::stoi(line.substr(0, strpos));

                nodes.push_back(node);
            }
        }
        file.close();
    }
    return nodes;
}

std::vector<WatchList> initDijkstra(std::vector<AdjListNode> nodes) {
    std::vector<WatchList> returnNodes;
    returnNodes.resize(nodes.size());
    for( int i = 0; i != nodes.size(); i++) {
        returnNodes[i].distance = 10000000;
        returnNodes[i].previvousNode = 0;
        returnNodes[i].marked = 0;
    }
    return returnNodes;
}

std::vector<AdjListNode> distanceUpdate(int fromNode, int destNode, std::vector<AdjListNode> nodes) {
    double alternativeDistance = nodes[destNode].weight + nodes[fromNode].weight;
    if (alternativeDistance < nodes[fromNode].weight) {
        nodes[fromNode].weight = alternativeDistance;
        nodes[fromNode].dest = destNode;
    }
    return nodes;
}

int getMinDist(std::vector<AdjListNode> nodes,std::vector<WatchList> watchList) {
    int min = 1000000000;
    int idx = 0;
    for (size_t i = 0;i < nodes.size();i++) {
        if (watchList[i].marked==0 && watchList[i].distance < min) {
            min = watchList[i].distance;
            idx = i;
        }
    }
    return idx;
}

std::vector<AdjListNode> getPartnerNodes(int nodeFrom,std::vector<AdjListNode> nodes) {
    std::vector<AdjListNode> returnNodes;
    for (size_t i = 0;i < nodes.size();i++) {
        if (nodes[i].dest==nodeFrom) {
            returnNodes.push_back(nodes[i]);
        }
    }
    return returnNodes;
}


void dijkstra(std::vector<AdjListNode> nodes,std::vector<WatchList> watchList) {
    for (unsigned i = 0; i < nodes.size(); i++) {
        int minDistIdx = getMinDist(nodes,watchList);
        std::vector<AdjListNode> goToNode = getPartnerNodes(nodes[minDistIdx].from,nodes);
        //watchList[minDistIdx].marked=1;
        for (size_t j = 0; j < goToNode.size(); j++) {
            unsigned destIdx = goToNode[j].dest;
            unsigned relDist = goToNode[j].weight;
            if (watchList[destIdx].marked==0) {
                unsigned absDist = watchList[minDistIdx].distance + relDist;
                if (absDist < watchList[destIdx].distance) {
                    watchList[destIdx].distance = absDist;
                    watchList[destIdx].previvousNode = minDistIdx;
                    watchList[minDistIdx].marked=1;
                }
            }
        }
    }
}

std::vector<unsigned> getSteps(std::vector<AdjListNode> nodes,std::vector<WatchList> watchList,unsigned dest)
{
    std::vector<unsigned> hops;
    if(dest < nodes.size() && dest > 0) {
        unsigned dist, prev = dest;
        do {
            prev = watchList[prev].previvousNode;
            dist = watchList[prev].distance;
            hops.push_back(prev + 1);
        } while(dist > 0);
        std::reverse(hops.begin(), hops.end());
    }
    return hops;
}

void printSteps(unsigned dest,std::vector<AdjListNode> nodes,std::vector<WatchList> watchList)
{
    std::vector<unsigned> hops = getSteps(nodes,watchList,dest);
    std::cout <<  std::left << ": " << "1-->" << dest << ": " << watchList[dest].distance << " { ";
    std::copy(hops.begin(), hops.end(), std::ostream_iterator<unsigned>(std::cout, " "));
    std::cout << "}\n\n";
}


int main() {
    //Einlesen der Datei
    std::vector<double> minimumDistances; // Vektor für die kleinsten Abstände
    std::vector<int> previousVertices; // Vektor für die Vorgängerknoten
    std::vector<int> vertexNo; // Vektor für die Namen


    std::vector<AdjListNode> nodes;
    nodes = readFile();
    int startingNode = 11;
    //AdjListNode returnNode;
    //returnNode = berrechneMin(nodes, startingNode);
    //std::cout << returnNode.from << " " << returnNode.weight << std::endl;

    //std::vector<AdjListNode> initNodes;
    //initNodes = initDijkstra(nodes);

    //initNodes = distanceUpdate(startingNode, returnNode.from, initNodes);

    //for (const AdjListNode &e: initNodes) {
    //    //std::cout << e.from <<" "<< e.dest <<" "<< e.weight << std::endl;
    //}

    //return 0;
    int fromNode;
    int toNode;
    std::cout << "From Node:" << std::endl;
    std::cin >> fromNode;
    std::cout << "To Node:" << std::endl;
    std::cin >> toNode;


    std::vector<WatchList> watchList;
    watchList=initDijkstra(nodes);
    std::cout << watchList.size() << std::endl;

    //dijkstra(nodes,watchList);

    //printSteps(toNode,nodes,watchList);

vector<vector<neighbor>> readFile() {
    std::ifstream file("/home/jonagurk/Dokumente/USA-road-d.NY.gr");
    int i=0;
    vector<vector<neighbor>> adjacencyList;


    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::string prefix = "a";
            if (line.rfind(prefix, 0) == 0) {
                //printf( line.c_str());
                int strpos = line.find(" ");
                line.erase(0, strpos + 1);

                strpos = line.find(" ");

                int name= std::stoi(line.substr(0, strpos));
                line.erase(0, strpos + 1);

                strpos = line.find(" ");
                int dest = std::stoi(line.substr(0, strpos));
                line.erase(0, strpos + 1);

                strpos = line.find(" ");
                int weight = std::stoi(line.substr(0, strpos));
                adjacencyList[i].push_back(neighbor(dest, to_string(name), weight));
                i=i+1;
            }
        }
        file.close();
    }
    return adjacencyList;
}

}*/


using namespace std;

const double maximumWeight = numeric_limits<double>::infinity(); // Konstante für das maximale Gewicht

// Datentyp, der die Nachbarknoten eines Knotens definiert
struct neighbor {
    int targetIndex; // Index des Zielknotens
    string name; // Name des
    double weight; // Gewicht der Kante
    neighbor(int _target, string _name, double _weight) : targetIndex(_target), name(_name), weight(_weight) {}
};

struct AdjListNode {
    int from;
    int dest;
    int weight;
};

struct WatchList {
    int distance;
    int previvousNode;
    int marked;
};


std::vector<AdjListNode> readFile() {
    std::ifstream file("/home/jonagurk/Dokumente/USA-road-d.NY.gr");
    std::vector<AdjListNode> nodes;
    AdjListNode node;
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::string prefix = "a";
            if (line.rfind(prefix, 0) == 0) {
                //printf( line.c_str());
                int strpos = line.find(" ");
                line.erase(0, strpos + 1);

                strpos = line.find(" ");
                node.from = std::stoi(line.substr(0, strpos));
                line.erase(0, strpos + 1);

                strpos = line.find(" ");
                node.dest = std::stoi(line.substr(0, strpos));
                line.erase(0, strpos + 1);

                strpos = line.find(" ");
                node.weight = std::stoi(line.substr(0, strpos));

                nodes.push_back(node);
            }
        }
        file.close();
    }
    return nodes;
}

typedef struct {
    unsigned dest;
    unsigned dist;
} Node;


std::vector<std::vector<Node>> getGraph(std::vector<std::vector<Node>> graph) {
    std::ifstream f("/home/jonagurk/Dokumente/USA-road-d.NY.gr");
    std::string line;
    unsigned nodeCount = 0;
    while (std::getline(f, line)) {
        std::stringstream ss(line);
        std::string id;
        ss >> id;
        if (id == "a") {
            unsigned srcs;
            unsigned dest;
            unsigned arc;
            ss >> srcs >> dest >> arc;
            nodeCount = std::max(srcs, nodeCount);
            srcs--;
            dest--;
            if (nodeCount > graph.size()) {
                graph.resize(nodeCount);
            }
            graph[srcs].push_back({.dest = dest,.dist = arc});
        }
    }
    return graph;
}


std::vector<AdjListNode> getPartnerNodes(int nodeFrom, std::vector<AdjListNode> nodes) {
    std::vector<AdjListNode> returnNodes;
    for (size_t i = 0; i < nodes.size(); i++) {
        if (nodes[i].dest == nodeFrom) {
            returnNodes.push_back(nodes[i]);
        }
    }
    return returnNodes;
}


// Berechnet die kürzesten Wege für den Knoten mit startIndex. Der gerichtete Graph wird als Adjazenzliste übergeben.
void ComputeShortestPathsByDijkstra(int startIndex, const vector<vector<Node>> &adjacencyList,
                                    vector<double> &minimumDistances, vector<int> &previousVertices,
                                    map<int, string> &vertexNames) {
    int numberOfVertices = adjacencyList.size(); // Anzahl der Knoten
    // Initialisiert den Vektor für die kleinsten Abstände
    minimumDistances.clear();
    minimumDistances.resize(numberOfVertices, maximumWeight);
    minimumDistances[startIndex] = 0; // Initialisiert den Vektor für die Vorgängerknoten
    previousVertices.clear();
    previousVertices.resize(numberOfVertices, -1);
    set<pair<double, int>> vertexQueue;
    vertexQueue.insert(make_pair(minimumDistances[startIndex],startIndex));
    vertexNames.insert(make_pair(startIndex,vertexNames[startIndex]));
     while (!vertexQueue.empty()) {
        double distance = vertexQueue.begin()->first; // Abstand
        int index = vertexQueue.begin()->second;
        vertexQueue.erase(vertexQueue.begin()); // Entfernt den ersten Knoten der Warteschlange
        const vector<Node> &srcs = adjacencyList[index];
        // Diese for-Schleife durchläuft alle Nachbarn des Knoten mit index
        for (vector<Node>::const_iterator neighborIterator = srcs.begin();
             neighborIterator != srcs.end(); neighborIterator++) {
            int targetIndex = neighborIterator->dest; // Index des Nachbarknotens
            string name = to_string(neighborIterator->dest); // Name des Nachbarknotens
            double weight = neighborIterator->dist; // Abstand zum Nachbarknoten
            double currentDistance = distance + weight; // Abstand vom Startknoten zum Knoten mit index
            if (currentDistance <minimumDistances[targetIndex]) // Wenn der Abstand zum Nachbarknoten kleiner als die Länge des bisher kürzesten Wegs ist
            {
                vertexQueue.erase(make_pair(minimumDistances[targetIndex],targetIndex)); // Entfernt den Knoten aus der Warteschlange
                vertexNames.erase(targetIndex); // Entfernt den Namen des Knotens aus der Zuordnungstabelle
                minimumDistances[targetIndex] = currentDistance; // Speichert den Abstand vom Startknoten
                previousVertices[targetIndex] = index; // Speichert den Index des Vorgängerknotens
                vertexQueue.insert(make_pair(minimumDistances[targetIndex],targetIndex)); // Fügt den Knoten der Warteschlange hinzu
                vertexNames.insert(make_pair(targetIndex, name)); // Fügt den Namen des Knotens der Zuordnungstabelle hinzu
            }
        }
    }
}


list<string> GetShortestPathTo(int index, vector<int> &previousVertices, map<int, string> &vertexNames) {
    list<string> path;
    for (; index != -1; index = previousVertices[index])
    {
        path.push_front(vertexNames[index]);
    }
    return path;
}

int main() {

    int fromNode;
    int toNode;
    std::cout << "From Node:" << std::endl;
    std::cin >> fromNode;
    std::cout << "To Node:" << std::endl;
    std::cin >> toNode;

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<Node>> graph;
    graph = getGraph(graph);
    std::vector<AdjListNode> Nodes = readFile();
    vector<vector<neighbor>> adjacencyList(Nodes.size());

    vector<double> minimumDistances;
    vector<int> previousVertices;
    map<int, string> vertexNames;
    ComputeShortestPathsByDijkstra(fromNode, graph, minimumDistances, previousVertices,
                                   vertexNames);
    cout << "Abstand von Knoten " << fromNode << " nach Knoten " << toNode << " " << minimumDistances[toNode] << endl;
    list<string> path = GetShortestPathTo(toNode, previousVertices,vertexNames);

    cout << "Kürzester Weg:"; // Ausgabe auf der Konsole
    copy(path.begin(), path.end(), ostream_iterator<string>(cout, " "));
    cout << endl; // Ausgabe auf der Konsole

    auto end_time = std::chrono::high_resolution_clock::now();
    auto time = end_time - start_time;
    std::cout << "Dijkstra took " <<
              time/std::chrono::milliseconds(1) << "ms to run.\n";
}



