#ifndef PROBLEMDEFINITION_H
#define PROBLEMDEFINITION_H
#include <vector>

using namespace std;

//milk jar problem, max state = 8 5 3 final state 4 y z initial state = 8 0 0
class ProblemDefinition{
  private:
    int state[3] = {8,0,0};
    const int capacity[3] = {8,5,3};

  public:
    bool validate(int* state);

    vector<int> generateValidStates();

    void updateState(int state[3]);

    int encodeState(int* state);

    void decodeState(int encodedState, int* state);

    bool isGoalState();
};

inline bool ProblemDefinition::validate(int* state){
    int sum=0;
    for(int i=0; i<3; ++i){
      if(state[i] < 0 || state[i] > this->capacity[i]) return false;

      sum+=state[i];
    }

    return (sum == 8);
}

inline vector<int> ProblemDefinition::generateValidStates(){
  vector<int> ret;

  for(int i=0 ; i<3; ++i){
    for(int j=0; j<3; ++j){
      if(i == j) continue;

      if(this->state[i]>0 && this->state[j] < this->capacity[j]){
        int amtRemoved = min(this->state[i], this->capacity[j] - this->state[j]);

        int newState[3];
        for(int k=0; k<3; ++k) newState[k] = this->state[k];

        newState[i] = this->state[i]-amtRemoved;
        newState[j] = this->state[j]+amtRemoved;

        if(!validate(newState)) continue;

        ret.push_back(this->encodeState(newState));
      }
    }
  }

  return ret;
}

inline void ProblemDefinition::updateState(int* state){
  for(int i=0; i<3; ++i) this->state[i] = state[i];
}

inline int ProblemDefinition::encodeState(int* state){
  return state[2]*100 + state[1]*10 + state[0];
}

inline void ProblemDefinition::decodeState(int encodedState, int state[3]){
  state[0] = encodedState%10;
  state[1] = (encodedState%100)/10;
  state[2] = encodedState/100;
}

inline bool ProblemDefinition::isGoalState(){
  return (this->state[0] == 4 && this->validate(this->state));
}

#endif
