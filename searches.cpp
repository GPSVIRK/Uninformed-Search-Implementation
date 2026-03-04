#include <chrono>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <queue>
#include <cstdio>
#include "problem_defnition.hpp"


using namespace std;

void printPath(vector<int>& pathInReverse, ProblemDefinition& milkJars, int nodesExpanded, int nodesGenerated, double time){
  int pathState[3];
  int pathLength = 0;
  for(auto it = pathInReverse.rbegin(); it!=pathInReverse.rend(); it++){
    pathLength++;
    milkJars.decodeState((*it), pathState);
    printf("(%d,%d,%d)-->",pathState[0],pathState[1],pathState[2]);
  }
  printf("GOAL\nPathLength: %d\nTotal nodes expanded: %d\nTotal nodes Generated: %d\nTotal time taken: %lfms\n\n",pathLength,nodesExpanded,nodesGenerated,time);

}

void DFS(int& nodesGenerated, int& nodesExpanded, int currState, ProblemDefinition& milkJars, bool& completed, unordered_set<int>& visited, stack<int>& path){
  completed = milkJars.isGoalState();
  if(completed) return;

  vector<int> generatedStates = milkJars.generateValidStates();
  nodesGenerated += generatedStates.size();
  nodesExpanded++;

  for(int i : generatedStates){
    if(visited.find(i) != visited.end()) continue;

    visited.insert(i);
    path.push(i);

    int newState[3];
    milkJars.decodeState(i, newState);
    milkJars.updateState(newState);

    DFS(nodesGenerated, nodesExpanded, i, milkJars, completed, visited, path);
    if(completed) return;

    path.pop();
  }
}

void DFSHelper(ProblemDefinition milkJars){
  unordered_set<int> visited;
  stack<int> path;
  bool completed = false;
  int currState = 8;//encoding is 358 (max) start is 008
  int nodesExpanded = 0;
  int nodesGenerated = 0;

  visited.insert(currState);
  path.push(currState);

  auto start = chrono::high_resolution_clock::now();
  DFS(nodesGenerated, nodesExpanded, currState, milkJars, completed, visited, path);
  auto end = chrono::high_resolution_clock::now();
  double time = chrono::duration<double,milli>(end-start).count();

  vector<int> pathInReverse;
  while(!path.empty()){
    pathInReverse.push_back(path.top());
    path.pop();
  }

  printPath(pathInReverse, milkJars, nodesExpanded, nodesGenerated, time);
}

void BFS(int& nodesGenerated, int& nodesExpanded, int& finalState, int initState,
         ProblemDefinition& milkJars, queue<int>& que, unordered_set<int>& visited,
         unordered_map<int,int>& parent){

  que.push(initState);
  bool completed = false;

  while(!que.empty()){
    int currStateEnc = que.front();
    que.pop();
    int currState[3];
    milkJars.decodeState(currStateEnc, currState);

    milkJars.updateState(currState);

    completed = milkJars.isGoalState();
    if(completed) {
      finalState = currStateEnc;
      return;
    }

    vector<int> generatedStates = milkJars.generateValidStates();
    nodesGenerated += generatedStates.size();
    nodesExpanded++;

    for(int i: generatedStates){
      if(visited.find(i) != visited.end()) continue;

      visited.insert(i);
      parent.insert({i, currStateEnc});//parent[child] = parent
      que.push(i);
    }
  }
}

void BFSHelper(ProblemDefinition milkJars){
  queue<int> que;
  unordered_set<int> visited;
  unordered_map<int,int> parent;
  int initState = 8;
  int finalState = 0;
  int nodesExpanded = 0;
  int nodesGenerated = 0;

  auto start = chrono::high_resolution_clock::now();
  BFS(nodesGenerated, nodesExpanded, finalState,initState, milkJars, que, visited, parent);
  auto end = chrono::high_resolution_clock::now();
  double time = chrono::duration<double,milli>(end-start).count();

  vector<int> path;
  int currState = finalState;
  while(currState != initState){
    path.push_back(currState);
    currState = parent[currState];
  }
  path.push_back(initState);

  printPath(path, milkJars, nodesExpanded, nodesGenerated,time);
}

//DepthLimitedSearch
void DLS(int& nodesGenerated, int& nodesExpanded, int limit, int currState, ProblemDefinition& milkJars, bool& completed, unordered_set<int>& visited, stack<int>& path){
  limit--;
  if(limit == -1) return;

  completed = milkJars.isGoalState();
  if(completed) return;

  vector<int> generatedStates = milkJars.generateValidStates();
  nodesGenerated += generatedStates.size();
  nodesExpanded++;

  for(int i : generatedStates){
    if(visited.find(i) != visited.end()) continue;

    visited.insert(i);
    path.push(i);

    int newState[3];
    milkJars.decodeState(i, newState);
    milkJars.updateState(newState);

    DLS(nodesGenerated, nodesExpanded, limit, i, milkJars, completed, visited, path);
    if(completed) return;

    path.pop();
  }
}

void IDDFS(ProblemDefinition milkJars){
  int currState = 8;
  bool completed = false;
  stack<int> finalPath;
  int nodesExpanded=0;
  int nodesGenerated = 0;

  int limit_max=10;
  auto start = chrono::high_resolution_clock::now();
  for(int limit=1 ; limit<limit_max; ++limit){
    unordered_set<int> visited;
    stack<int> path;
    visited.insert(currState);
    path.push(currState);

    int initState[3];
    milkJars.decodeState(currState, initState);
    milkJars.updateState(initState);

    DLS(nodesGenerated, nodesExpanded, limit, currState, milkJars, completed, visited, path);
    if(completed) {
      finalPath = path;
      break;
    }
  }
  auto end = chrono::high_resolution_clock::now();
  double time = chrono::duration<double,milli>(end-start).count();

  vector<int> pathInReverse;
  while(!finalPath.empty()){
    pathInReverse.push_back(finalPath.top());
    finalPath.pop();
  }

  printPath(pathInReverse, milkJars, nodesExpanded, nodesGenerated, time);
}


int main(){
  ProblemDefinition milkJars;
  printf("DFS path:\n");
  DFSHelper(milkJars);

  printf("BFS path:\n");
  BFSHelper(milkJars);

  printf("IDDFS path:\n");
  IDDFS(milkJars);
}

