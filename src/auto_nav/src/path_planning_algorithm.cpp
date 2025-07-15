#include "path_planning_algorithm.h"
#include <map>
#include <functional>
#include <iostream>

using namespace std;

// 静态成员变量，存储算法创建函数
static map<string, function<shared_ptr<PathPlanningAlgorithm>()>> algorithm_creators;

shared_ptr<PathPlanningAlgorithm> PathPlanningAlgorithmFactory::createAlgorithm(const string& algorithm_type) 
{
    auto it = algorithm_creators.find(algorithm_type);
    if (it != algorithm_creators.end()) {
        return it->second();
    }
    
    cout << "Error: Unknown algorithm type: " << algorithm_type << endl;
    cout << "Available algorithms: ";
    for (const auto& pair : algorithm_creators) {
        cout << pair.first << " ";
    }
    cout << endl;
    
    return nullptr;
}

vector<string> PathPlanningAlgorithmFactory::getAvailableAlgorithms() 
{
    vector<string> algorithms;
    for (const auto& pair : algorithm_creators) {
        algorithms.push_back(pair.first);
    }
    return algorithms;
}

void PathPlanningAlgorithmFactory::registerAlgorithm(const string& algorithm_type, 
                                                     function<shared_ptr<PathPlanningAlgorithm>()> creator) 
{
    algorithm_creators[algorithm_type] = creator;
}
