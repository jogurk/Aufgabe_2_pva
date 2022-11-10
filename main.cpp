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

struct AdjListNode
{
    int from;
    int dest;
    int weight;
};

AdjListNode berrechneMin (std::vector<AdjListNode> nodes,int startingNode) {
    int minNextNodeName=0;
    int minNextNodeWeight=0;
    AdjListNode returnNode;
    std::for_each(nodes.cbegin(), nodes.cend(),[&startingNode,&minNextNodeName,&minNextNodeWeight](AdjListNode node){
        if ((startingNode==node.from)&&(minNextNodeWeight<node.weight)) {
            minNextNodeName=node.dest;
            minNextNodeWeight=node.weight;
        }
    });
    returnNode.weight=minNextNodeWeight;
    returnNode.from=minNextNodeName;
    return returnNode;
}
std::vector<AdjListNode> readFile(){
    std::ifstream file("/home/jonagurk/Dokumente/USA-road-d.NY.gr");
    std::vector<AdjListNode> nodes;
    AdjListNode node;
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::string prefix = "a";
            if (line.rfind(prefix, 0) == 0) {
                //printf( line.c_str());
                int strpos =line.find(" ");
                line.erase(0, strpos+1);

                strpos = line.find(" ");
                node.from =  std::stoi(line.substr(0, strpos));
                line.erase(0, strpos+1);

                strpos = line.find(" ");
                node.dest = std::stoi(line.substr(0, strpos));
                line.erase(0, strpos+1);

                strpos = line.find(" ");
                node.weight = std::stoi(line.substr(0, strpos));

                nodes.push_back(node);
            }
        }
        file.close();
    }
    return nodes;
}

int main() {
    //Einlesen der Datei
    std::vector<double> minimumDistances; // Vektor für die kleinsten Abstände
    std::vector<int> previousVertices; // Vektor für die Vorgängerknoten
    std::vector<int>  vertexNo; // Vektor für die Namen

    //std::vector<AdjListNode> nodesBegin;
    //std::transform(nodes.cbegin(), nodes.cend(),nodesBegin.begin(),[](AdjListNode node){
    //        node.weight=10000000;
    //        node.dest=0;
    //        return node;
    //});



    //for ( const AdjListNode &e : nodesBegin )
    //{
    //    std::cout << e.from <<" "<< e.dest <<" "<< e.weight << std::endl;
    //}


    //for(unsigned i = 0; i < nodes.size(); i++) {

    //}
    std::vector<AdjListNode> nodes;
    nodes=readFile();
    int startingNode=11;
    AdjListNode returnNode;
    returnNode=berrechneMin(nodes,startingNode);
    std::cout << returnNode.from <<" "<< returnNode.weight<< std::endl;


    return 0;
}


