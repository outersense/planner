/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple> 

#include <chrono>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


// defining a structure to hold x y g h f values and a custom comparison operator
struct State 
{
    int x, y;
    int g; 
    double h; 
    double f; 

    State(int _x, int _y, int _g, double _h)
        : x(_x), y(_y), g(_g), h(_h), f(_g + 50*_h) {} 
    bool operator>(const State& other) const {
        return f > other.f;
    }
};

struct goalinfo 
{
    int x, y;
    int TimetoGoal; 
    double PathLength; 
    std::vector<State> PathtoGoal;

    goalinfo(int _x, int _y, int _TimetoGoal, double _PathLength, std::vector<State> _PathtoGoal)
        : x(_x), y(_y), TimetoGoal(_TimetoGoal), PathLength(_PathLength), PathtoGoal(_PathtoGoal) {} 
};


double getHeuristic(int currx, int curry, int goalx, int goaly){

    // simple euclidean distance heuristic
    double h = 2*(double)sqrt(((currx-goalx)*(currx-goalx) + (curry-goaly)*(curry-goaly)));
    return h;
}

std::tuple<std::vector<State>, std::vector<std::vector<int>>, std::vector<std::vector<int>>> BackwardDjstraSearch(int* map, State start, State target, int numRows, int numCols, int dx[NUMOFDIRS], int dy[NUMOFDIRS], int collision_thresh) {
    // Priority queue to store states to explore.

    std::priority_queue<State, std::vector<State>, std::greater<State>> openSet;
    int flag = 0;
    int count = 0;
    std::vector<State> pathcopy;
    std::vector<std::vector<int>> closedlistnew(numRows, std::vector<int>(numCols, 0));
    openSet.push(target);
    std::vector<std::vector<State>> parent(numRows, std::vector<State>(numCols, State(-1, -1, 0, 0))); // check this
    std::vector<std::vector<int>> gvalueslist(numRows, std::vector<int>(numCols, std::numeric_limits<int>::max()));
    gvalueslist[target.x-1][target.y-1] = 0; // setting the start point cost to be 0

    // while ( closedlistnew[start.x-1][start.y-1]==0 && !openSet.empty()) { // use in general
    while (!openSet.empty()) { // use when expanding all states
        count++;
        State current = openSet.top(); // will always be with the min f so dont need to check
        openSet.pop();
// /////////////////////////////////////////////////////////////////////
        // Check if we've reached the target. check this
        if (current.x == start.x && current.y == start.y) {
            std::vector<State> path; 
            flag = 1; // use this and not return here only when want to expand all states
            State currentState = start;
            while (!(currentState.x == target.x && currentState.y == target.y)) {
                path.push_back(currentState);
                currentState = parent[currentState.x-1][currentState.y-1];
            }
            path.push_back(target);
            // std::reverse(path.begin(), path.end()); // backwards does not need reverse
            // return path; // use this when want to just find path to a goal
            pathcopy = path;
        }
///////////////////////////////////////////////////////////////////////

        // put value in closed list
        closedlistnew[current.x-1][current.y-1] = 1;
        for(int dir = 0; dir < NUMOFDIRS; dir++){
            int newx = current.x + dx[dir];
            int newy = current.y + dy[dir];

            if (newx >= 1 && newx <= numRows && newy >= 1 && newy <= numCols){
                bool inclosed = closedlistnew[newx-1][newy-1];
                if (inclosed == 0){
                    if ((map[GETMAPINDEX(newx,newy,numRows,numCols)] >= 0) && (map[GETMAPINDEX(newx,newy,numRows,numCols)] < collision_thresh))  //if free
                    {
                        double costofcell = map[GETMAPINDEX(newx,newy,numRows,numCols)];
                        int newCost = current.g + costofcell+1;
                        if (newCost < gvalueslist[newx-1][newy-1]){
                            gvalueslist[newx-1][newy-1] = newCost;
                            State newState(newx, newy, newCost, 0);
                            openSet.push(newState);
                            parent[newx-1][newy-1] = current;
                        }                    
                    }
                }
            }
        }
    }
    // No path found.
    if (flag ==1){
        // std::cout << "path found " << std::endl;
        return std::make_tuple(pathcopy, closedlistnew, gvalueslist);

    }
    else{
        // std::cout << "blah blah blah blo blo blah " <<count<< std::endl;
        return std::make_tuple(std::vector<State>(),closedlistnew, gvalueslist );
    }
}

std::vector<State> AStarSearch_greedy(int* map, State start, State target, int numRows, int numCols, int dx[NUMOFDIRS], int dy[NUMOFDIRS], int collision_thresh) {
    // Priority queue to store states to explore.
    std::priority_queue<State, std::vector<State>, std::greater<State>> openSet;
    std::vector<std::vector<int>> closedlistnewplswork(numRows, std::vector<int>(numCols, 0));
    std::vector<State> parentlist;
    openSet.push(start);
    int count = 0;

    // Parent 2D array to reconstruct the path. Check and change this
    std::vector<std::vector<State>> parent(numRows, std::vector<State>(numCols, State(-1, -1, 0, 0))); // check this

    // Setting all cell values to infinity
    std::vector<std::vector<int>> gvalueslist(numRows, std::vector<int>(numCols, std::numeric_limits<int>::max()));
    gvalueslist[start.x][start.y] = 0; // setting the start point cost to be 0

    while (!openSet.empty()) {
        // Get the state with the lowest total cost.
        count++;


        State current = openSet.top(); // will always be with the min f so dont need to check
        openSet.pop();
// /////////////////////////////////////////////////////////////////////
        // Check if we've reached the target. check this
        if (current.x == target.x && current.y == target.y) {
            // Reconstruct the path.
            std::vector<State> path;
            State currentState = target;
            while (!(currentState.x == start.x && currentState.y == start.y)) {
                path.push_back(currentState);
                currentState = parent[currentState.x][currentState.y];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            // std::cout << "blah blah blah " <<count<< std::endl;
            return path;
        }
///////////////////////////////////////////////////////////////////////

        // put value in closed list
        closedlistnewplswork[current.x-1][current.y-1] = 1;
        for(int dir = 0; dir < NUMOFDIRS; dir++){
            int newx = current.x + dx[dir];
            int newy = current.y + dy[dir];

            if (newx >= 1 && newx <= numRows && newy >= 1 && newy <= numCols){
                bool inclosed = closedlistnewplswork[newx-1][newy-1];
                if (inclosed == 0){
                    if ((map[GETMAPINDEX(newx,newy,numRows,numCols)] >= 0) && (map[GETMAPINDEX(newx,newy,numRows,numCols)] < collision_thresh))  //if free
                    {
                        // double costtocell = (double)sqrt(((newx - current.x) * (newx - current.x) + (newy - current.y) * (newy - current.y)));
                        double costofcell = map[GETMAPINDEX(newx,newy,numRows,numCols)];
                        int newCost = current.g + costofcell;
                        if (newCost < gvalueslist[newx][newy]){
                            gvalueslist[newx][newy] = newCost;
                            double hvalue = getHeuristic(newx, newy, target.x, target.y);
                            State newState(newx, newy, newCost, hvalue);
                            openSet.push(newState);
                            parent[newx][newy] = current;
                        }                    
                    }
                }
            }
        }
    }
    // No path found.
    // std::cout << "blah blah blah blo blo " <<count<< std::endl;
    return std::vector<State>();
}

std::tuple<std::vector<State>, std::vector<std::vector<int>>, std::vector<std::vector<int>>, int> AStarSearch(int* map, State startA, State targetA, int numRows, int numCols, int dx[NUMOFDIRS], int dy[NUMOFDIRS], int collision_thresh, std::vector<std::vector<int>> heuristics_list) {
    // Priority queue to store states to explore.

    std::priority_queue<State, std::vector<State>, std::greater<State>> openSetA;
    int flagA = 0; // wont be needed
    int countA = 0; // wont be needed
    std::vector<State> pathcopyA; // wont be needed
    int costtothisgoalcopy;
    std::vector<std::vector<int>> closedlistnewA(numRows, std::vector<int>(numCols, 0));
    openSetA.push(startA);
    std::vector<std::vector<State>> parentA(numRows, std::vector<State>(numCols, State(-1, -1, 0, 0))); // check this
    std::vector<std::vector<int>> gvalueslistA(numRows, std::vector<int>(numCols, std::numeric_limits<int>::max()));
    gvalueslistA[startA.x-1][startA.y-1] = 0; // setting the start point cost to be 0

    // while ( closedlistnew[start.x-1][start.y-1]==0 && !openSet.empty()) { // use in general
    while (closedlistnewA[targetA.x-1][targetA.y-1]==0 && !openSetA.empty()) { // use when expanding all states
        countA++;
        State currentA = openSetA.top(); // will always be with the min f so dont need to check
        openSetA.pop();
// /////////////////////////////////////////////////////////////////////
        // Check if we've reached the target. check this
        if (currentA.x == targetA.x && currentA.y == targetA.y) {
            std::vector<State> pathA; 
            int costtothisgoal = 0;
            flagA = 1; // use this and not return here only when want to expand all states
            State currentStateA = targetA;
            while (!(currentStateA.x == startA.x && currentStateA.y == startA.y)) {
                pathA.push_back(currentStateA);
                costtothisgoal = costtothisgoal + map[GETMAPINDEX(currentStateA.x,currentStateA.y,numRows,numCols)];
                currentStateA = parentA[currentStateA.x-1][currentStateA.y-1];
            }
            pathA.push_back(startA);
            std::reverse(pathA.begin(), pathA.end()); // backwards does not need reverse
            // return pathA; // use this when want to just find path to a goal
            pathcopyA = pathA;
            costtothisgoalcopy = costtothisgoal;

        }
///////////////////////////////////////////////////////////////////////

        // put value in closed list
        closedlistnewA[currentA.x-1][currentA.y-1] = 1;
        for(int dir = 0; dir < NUMOFDIRS; dir++){
            int newxA = currentA.x + dx[dir];
            int newyA = currentA.y + dy[dir];

            if (newxA >= 1 && newxA <= numRows && newyA >= 1 && newyA <= numCols){
                bool inclosedA = closedlistnewA[newxA-1][newyA-1];
                if (inclosedA == 0){
                    if ((map[GETMAPINDEX(newxA,newyA,numRows,numCols)] >= 0) && (map[GETMAPINDEX(newxA,newyA,numRows,numCols)] < collision_thresh))  //if free
                    {
                        double costofcellA = map[GETMAPINDEX(newxA,newyA,numRows,numCols)];
                        int newCostA = currentA.g + costofcellA+1;
                        // if (greedy_flag_ ==1){
                        //     int newCostA = currentA.g + costofcellA;
                        // }
                        if (newCostA < gvalueslistA[newxA-1][newyA-1]){
                            gvalueslistA[newxA-1][newyA-1] = newCostA;
                            double hvalue = heuristics_list[newxA-1][newyA-1]+ (double)sqrt(((newxA-targetA.x)*(newxA-targetA.x) + (newyA-targetA.y)*(newyA-targetA.y)));
                            // if (greedy_flag_ == 1){
                            //     double hvalue = 20*(double)sqrt(((newxA-targetA.x)*(newxA-targetA.x) + (newyA-targetA.y)*(newyA-targetA.y)));
                            // }
                            
                            // double hvalue = heuristics_list[newxA-1][newyA-1];
                            State newStateA(newxA, newyA, newCostA, hvalue);
                            openSetA.push(newStateA);
                            parentA[newxA-1][newyA-1] = currentA;
                        }                    
                    }
                }
            }
        }
    }
    // No path found.
    if (flagA ==1){
        // std::cout << "path found " << std::endl;
        return std::make_tuple(pathcopyA, closedlistnewA, gvalueslistA, costtothisgoalcopy);

    }
    else{
        // std::cout << "blah blah blah blo blo blah " <<countA<< std::endl;
        return std::make_tuple(std::vector<State>(),closedlistnewA, gvalueslistA, costtothisgoalcopy);
    }
}


int a = 1;
int b = 1;
int i = 1;
int traceback = 1;
int correction_factor = 0;
unsigned int pathSize = 0;
int ignore_last = 10;
int greedy_flag = 0;
int keep_goal = 0;
double duration1;
double duration2;
double duration;
int goal_id;
int besttimetaken;
int flag_found_small =0;
int flag_evenly_spaced =0;

int costtothisgoal = std::numeric_limits<int>::max();

std::vector<int> pathlengthlist;
std::vector<std::vector<State>> pathlistarray;
std::vector<State> pathtoactualgoal;
std::vector<State> path;
std::vector<std::vector<int>> closed_list;
std::vector<std::vector<int>> gvalues_list;
std::vector<State> pathA;
std::vector<std::vector<int>> closed_listA;
std::vector<std::vector<int>> gvalues_listA;
std::vector<goalinfo> listofgoalinfo;

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    
    time_t start_time, end_time;
    int numofgoalstocheck = 50;
    // if (target_steps>1000){numofgoalstocheck = 100;}
    if (target_steps<500){numofgoalstocheck = 20;}
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    int goalposeX = target_traj[target_steps-ignore_last];
    int goalposeY = target_traj[target_steps-ignore_last+target_steps];
    
    // initialise the state
    State start(robotposeX, robotposeY, std::numeric_limits<int>::max(), 0);
    State target(goalposeX, goalposeY, 0, 0);

    // std:: cout << "current time is " << curr_time ;

    // call the backwards Djstra function only once
    if(a ==1)
    {
        // std::cout << "target: " << goalposeX<< ", "<< goalposeY << std::endl;
        a = 2;
        auto start_time = std::chrono::high_resolution_clock::now();
        std::tuple<std::vector<State>, std::vector<std::vector<int>>, std::vector<std::vector<int>>> searchResult = BackwardDjstraSearch(map, start, target, x_size, y_size, dX, dY, collision_thresh);
        path = std::get<0>(searchResult);
        closed_list = std::get<1>(searchResult);
        gvalues_list = std::get<2>(searchResult); // use as heuristics for forward A* search
        // for printing the costs and checking if expanded or not
        // for (int j =0; j <target_steps; j++){
        //     int x_ = target_traj[j];
        //     int y_ = target_traj[j+target_steps];
        //     int expanded = closed_list[x_-1][y_-1];
        //     int cost = gvalues_list[x_-1][y_-1];
        //     std::cout << "for the goal ( " << x_ << ", " << y_ << " ) the state is expanded: " << expanded << " and cost is : " << cost<< std::endl;
        
        // }
        // for (int k = 0; k <path.size(); k++){
        //     int gvalueofthispath = path[k].g;
        //     std::cout<< gvalueofthispath<<std::endl;
        // }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()/1000.0;
        // std::cout << "Time taken by Backwards Djkstra is : " << duration1 << " seconds" << std::endl;
        // std:: cout << a << std::endl;
    }
    
    // if (path.empty()) {
    //     std::cout << "No valid path found. #######################" << std::endl;
    // } 
    // add forward A* search here , use H obtained from the g values and then search for look ahead goals (first start with a fixed goal)
    
    int H = gvalues_list[robotposeX-1][robotposeY-1]+ (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    State startA(robotposeX, robotposeY, 0, H);
    
    if(b ==1)
    {
        // runA* for multiple goals
        auto start_timeA = std::chrono::high_resolution_clock::now();
        int numToExplore = std::min(numofgoalstocheck, target_steps / 10);
        int startIdx = target_steps - numToExplore * 10;
        int jump = 10;
        if (flag_evenly_spaced==1){
            jump = int(target_steps/numofgoalstocheck);
            if (jump<1){jump = 1;}
            startIdx =1;

            
        }
        
        int smallestvalue = target_steps;
        int costtothisgoal_;

        for (int runno = startIdx; runno < target_steps; runno += jump) {
            State targetA(target_traj[runno], target_traj[runno+target_steps], std::numeric_limits<int>::max(), 0);
            // std::cout << "target: " << target_traj[runno]<< ", "<< target_traj[runno+target_steps] << "FOR RUN NO : "<< runno<< std::endl;
            
            std::tuple<std::vector<State>, std::vector<std::vector<int>>, std::vector<std::vector<int>>, int> searchResultA = AStarSearch(map, startA, targetA, x_size, y_size, dX, dY, collision_thresh, gvalues_list);
            pathA = std::get<0>(searchResultA);
            costtothisgoal_ = std::get<3>(searchResultA);
            // pathlistarray.push_back(pathA);
            // if (target_traj[runno] == goalposeX && target_traj[runno+target_steps] == goalposeY){
            //     pathtoactualgoal = pathA;
            // }
            // closed_listA = std::get<1>(searchResultA);
            // gvalues_listA = std::get<2>(searchResultA);
            int time_taken = pathA.size();
            // pathlengthlist.push_back(time_taken);


            if (runno-time_taken > 0 ){
                if (costtothisgoal > costtothisgoal_){
                    costtothisgoal = costtothisgoal_;
                    besttimetaken = time_taken;
                    pathtoactualgoal= pathA;
                    int chosengoalx = target_traj[runno];
                    int chosengoaly = target_traj[runno+target_steps];
                    goal_id = runno;
                    // std::cout<< "in the least cost goal : "<< goal_id<< ", " << besttimetaken<< "chosen goal " << chosengoalx<< chosengoaly <<std::endl;
                    flag_found_small =1;




                }

            }

            if (flag_found_small ==0){
                // last value
                State targetA(target_traj[target_steps-1], target_traj[target_steps-1+target_steps], std::numeric_limits<int>::max(), 0);
                std::tuple<std::vector<State>, std::vector<std::vector<int>>, std::vector<std::vector<int>>, int> searchResultA = AStarSearch(map, startA, targetA, x_size, y_size, dX, dY, collision_thresh, gvalues_list);
                pathtoactualgoal = std::get<0>(searchResultA);
                // std::cout<<"did not find any goal less than the path going to last goal"<< std::endl;
                goal_id = target_steps-1;
                besttimetaken = pathtoactualgoal.size();

            }


            // goalinfo GOALINFO(target_traj[runno], target_traj[runno+target_steps], runno, time_taken, pathA);
        
            

            // std::cout << "for the goal " << target_traj[runno] << ", " << target_traj[runno+target_steps] << "the path length is : "<< time_taken << std::endl;

        }
        auto end_timeA = std::chrono::high_resolution_clock::now();
        duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_timeA - start_timeA).count()/1000.0;
        // std::cout << "Time taken by multigoal A* is : " << duration2 << " seconds" << std::endl;
        // State targetA(goalposeX, goalposeY, std::numeric_limits<int>::max(), 0);
        // std::cout << "target." << goalposeX<< ","<< goalposeY << std::endl;
        b = 2;
        // std::cout << "chosen goal is: " <<target_traj[goal_id]<< ", " << target_traj[goal_id+target_steps]<< " Target Time to goal is: " << goal_id << " time to run path: "<< besttimetaken << std::endl;
        // auto start_timeA = std::chrono::high_resolution_clock::now();
        // std::tuple<std::vector<State>, std::vector<std::vector<int>>, std::vector<std::vector<int>>> searchResultA = AStarSearch(map, startA, targetA, x_size, y_size, dX, dY, collision_thresh, gvalues_list);
        // pathA = std::get<0>(searchResultA);
        // closed_listA = std::get<1>(searchResultA);
        // gvalues_listA = std::get<2>(searchResultA); // use as heuristics for forward A* search
        // for printing the costs and checking if expanded or not
        // for (int j =0; j <target_steps; j++){
        //     int x_ = target_traj[j];
        //     int y_ = target_traj[j+target_steps];
        //     int expanded = closed_list[x_-1][y_-1];
        //     int cost = gvalues_list[x_-1][y_-1];
        //     std::cout << "for the goal ( " << x_ << ", " << y_ << " ) the state is expanded: " << expanded << " and cost is : " << cost<< std::endl;
        
        // }
        // int time_taken = pathA.size();
        if ((duration1+duration2)>1){
            // std::cout<< "I am here " << (duration1+duration2) << "  " << int(duration1+duration2)<< std::endl;
            duration = int(duration1+duration2)-1;}
        else{duration = 0;}

        // choose goal 

        // for (int Timetogoal = 0 ; Timetogoal<target_steps; )










        // auto end_timeA = std::chrono::high_resolution_clock::now();
        // auto durationA = std::chrono::duration_cast<std::chrono::milliseconds>(end_timeA - start_timeA).count()/1000.0;
        // std::cout << "Time taken by A* is : " << durationA << " seconds" << std::endl;
        
        // duration = int(duration1+duration2)-1;
        // std::cout<< "total time is " << duration<< std::endl;
        // pathSize = pathtoactualgoal.size();
        int steps_remaining = target_steps - besttimetaken - duration;
        // std::cout << "pathsize is :" << besttimetaken << "  remaining time : " << steps_remaining<< std::endl;
        if (steps_remaining % 2==0){
            correction_factor = -1;
        }
        
        // checking if cannot catch
        if (besttimetaken + duration > goal_id){
            // std::cout << "i did not find a path that would catch in time "<< std::endl;
            State targetA_(target_traj[target_steps-1], target_traj[target_steps-1+target_steps], std::numeric_limits<int>::max(), 0);
            std::tuple<std::vector<State>, std::vector<std::vector<int>>, std::vector<std::vector<int>>, int> searchResultA = AStarSearch(map, startA, targetA_, x_size, y_size, dX, dY, collision_thresh, gvalues_list);
            pathtoactualgoal = std::get<0>(searchResultA);
            besttimetaken = pathtoactualgoal.size();
            if (besttimetaken + duration > target_steps+10){
                // std::cout<< "I am going greedy" << std::endl;
                // greedy_flag = 1;
                double H = getHeuristic(robotposeX, robotposeY, goalposeX, goalposeY);
                State startplswork(robotposeX, robotposeY, 0, H);
                State targetplswork(target_traj[target_steps-1], target_traj[target_steps-1 +target_steps], std::numeric_limits<int>::max(), 0);
                pathtoactualgoal = AStarSearch_greedy(map, startplswork, targetplswork, x_size, y_size, dX, dY, collision_thresh);
                besttimetaken = pathtoactualgoal.size();
                goal_id = target_steps;
                // pathA = std::get<0>(searchResultA_);
            }
            
        }
    }
    
    if (pathtoactualgoal.empty()) {
        std::cout << "No valid path found by A* " << std::endl;
    } 

    
    
    
    // std::cout << i << " "<< besttimetaken<< std::endl;
    
    if (i < besttimetaken){
        action_ptr[0] = pathtoactualgoal[i].x;
        action_ptr[1] = pathtoactualgoal[i].y;
        robotposeX = action_ptr[0]; 
        robotposeY = action_ptr[1];
        // std::cout << action_ptr[0] << ", "<< action_ptr[1] << std::endl;
    }
    else{
        // for (int m =0; m < target_steps; m++){
        //     std::cout << "target trajectory is :" << target_traj[m] << ", " << target_traj[m+ target_steps]<<std::endl;
        // }
        // std::cout<< "YAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR" << std::endl;
        if (map[GETMAPINDEX(target_traj[goal_id -traceback-correction_factor],target_traj[goal_id - traceback-correction_factor+target_steps],x_size,y_size)]>
        map[GETMAPINDEX(robotposeX,robotposeY,x_size,y_size)]){
            // action_ptr[0] = pathtoactualgoal[besttimetaken-1].x;
            // action_ptr[1] = pathtoactualgoal[besttimetaken-1].y;
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            robotposeX = action_ptr[0];
            robotposeY = action_ptr[1];

            // std::cout<<"Decided to not move ahead";
            // std::cout<< "cost to stay " <<map[GETMAPINDEX(robotposeX,robotposeY,x_size,y_size)]<< "cost to move " << map[GETMAPINDEX(target_traj[goal_id -traceback-correction_factor],target_traj[goal_id - traceback-correction_factor+target_steps],x_size,y_size)] << std::endl;

        }

        else{
            // std::cout<< "Move "<< std::endl;
            // std::cout<< "cost to stay " <<map[GETMAPINDEX(robotposeX,robotposeY,x_size,y_size)]<< "cost to move " << map[GETMAPINDEX(target_traj[goal_id -traceback-correction_factor],target_traj[goal_id - traceback-correction_factor+target_steps],x_size,y_size)] << std::endl;
            action_ptr[0] = target_traj[goal_id -traceback-correction_factor];
            action_ptr[1] = target_traj[goal_id - traceback-correction_factor+target_steps];
            
            // action_ptr[0] = pathtoactualgoal[besttimetaken-1].x;
            // action_ptr[1] = pathtoactualgoal[besttimetaken-1].y;
            robotposeX = action_ptr[0];
            robotposeY = action_ptr[1];
            traceback ++;

            

        }
        
        // std::cout << action_ptr[0] << ", $$"<< action_ptr[1] << std::endl;
    }
    
    i++;
    // std::cout << "Steps to reach target:"<< pathSize << std::endl;
    // std::cout << "##########################################################################"<< std::endl;

    // std::cout << i<< std::endl; 
    
    return;
}
