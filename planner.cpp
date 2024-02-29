// /*=================================================================
//  *
//  * planner.cpp
//  *
//  *=================================================================*/
#include "planner.h"
#include <math.h>
#include <queue>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <stack>
#include <set>
#include <climits>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using namespace std;

// struct Node{
//     pair<int, int> parent;
//     double g,h,f;
//     Node(pair<int, int> parent, double g, double h): g(g), h(h), f(f){
//         parent = {-1,-1};
//     }

//     // bool operator<(const Node& other) const{
//     //     return g+h > other.g+other.h;
//     // }    
// };

struct Node {

    pair<int,int> parent;
    bool expanded;

    int f, g, h;
    Node(): parent(-1, -1), f(-1), g(-1), h(-1) {}
};


bool isValid(int x, int y, int x_size, int y_size, int* map, int collision_thresh){
    //cout<<"isValid"<<endl;
    if(((int)map[GETMAPINDEX(x,y,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(x,y,x_size,y_size)] < collision_thresh)){
        return true;
    }
    //cout<<"isValid Checked"<<endl;
    return false;
}

vector<int> getPath(vector<vector<Node>> &grid, int goalposeX, int goalposeY){
    int currx = goalposeX;
    int curry = goalposeY;

    stack<pair<int, int>> path;

    while(!(grid[currx][curry].parent.first == currx && grid[currx][curry].parent.second == curry)){
        path.push(make_pair(currx, curry));
        int tempx = grid[currx][curry].parent.first;
        int tempy = grid[currx][curry].parent.second;
        currx = tempx;
        curry = tempy;
    }
    //cout<<"path found"<<endl;

    pair<int, int> step = path.top();
    return {step.first, step.second};
}
// // struct VectorHash {
// //     size_t operator()(const std::vector<int>& v) const {
// //         std::hash<int> hasher;
// //         size_t result = 0;
// //         for (int value : v) {
// //             result ^= hasher(value) + 0x9e3779b9 + (result << 6) + (result >> 2);
// //         }
// //         return result;
// //     }
// // };

double heuristic(int x1, int y1, int x2, int y2){
    return (double)sqrt(((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)));
}



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

    // int goalposeX = target_traj[curr_time];
    // int goalposeY = target_traj[curr_time+target_steps];
    double weight = 100;
    double delta = 0.5;
    double dist_to_object = (int)sqrt(((robotposeX-targetposeX)*(robotposeX-targetposeX) + (robotposeY-targetposeY)*(robotposeY-targetposeY)));

    int goalposeX = (int)target_traj[curr_time+(int)(delta*dist_to_object)];
    int goalposeY = (int)target_traj[curr_time+target_steps+(int)(delta*dist_to_object)];
    //cout<<"here"<<endl;
    
    
    if(dist_to_object < 20 || (int)(*(&target_traj + 1) - target_traj)==0){
        int goalposeX = targetposeX;
        int goalposeY = targetposeY;
        // action_ptr[0] = targetposeX;
        // action_ptr[1] = targetposeY;
        
        // return;
    }


    // // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};


    //cout<< "goal " << goalposeX << ", "<<goalposeY<< endl;

    vector<vector<Node>> grid(x_size, vector<Node>(y_size));
    vector<vector<bool>> closed(x_size, vector<bool>(y_size, false));
    vector<vector<bool>> OPEN(x_size, vector<bool>(y_size, false));

    int curr_pose_x,curr_pose_y;

    for(int i=0; i<x_size; i++){
        for(int j=0; j<y_size;j++){
            grid[i][j].f = INT_MAX;
            grid[i][j].g = INT_MAX;
            grid[i][j].h = INT_MAX;
            grid[i][j].parent = make_pair(-1,-1);
            grid[i][j].expanded = false;
        }
    }
    //cout<<"here2"<<endl;
    curr_pose_x = robotposeX;
    curr_pose_y = robotposeY;
    grid[curr_pose_x][curr_pose_y].f = 0;
    grid[curr_pose_x][curr_pose_y].g = 0;
    grid[curr_pose_x][curr_pose_y].h = 0;
    grid[curr_pose_x][curr_pose_y].parent = make_pair(robotposeX, robotposeY);
    grid[curr_pose_x][curr_pose_y].expanded = false;

    set<pair<int, pair<int,int>>> open;
    open.insert(make_pair(0, make_pair(robotposeX, robotposeY)));
    bool found = false;

    vector<int> newpose = {robotposeX, robotposeY};
    int newx, newy;

    //cout<<"Current "<<robotposeX<<","<<robotposeY<<endl;

    if(robotposeX != goalposeX && robotposeY != goalposeY){
        //cout<<"inside if()"<<endl;
        while(!open.empty()){
            //cout<<"inside while"<<endl;
            pair<int, pair<int,int>> curr = *open.begin();
            open.erase(open.begin());
            curr_pose_x = curr.second.first;
            curr_pose_y = curr.second.second;
            // if(curr_pose_x == goalposeX && curr_pose_y == goalposeY){
            //     grid[newx][newy].parent.first = curr_pose_x;
            //     grid[newx][newy].parent.second = curr_pose_y;
            //     newpose = getPath(grid, goalposeX, goalposeY);
            //     found = true;
            //     break;
            // }
            //cout<<"CURR "<<curr_pose_x<<","<<curr_pose_y<<endl;
            //closed[curr_pose_x][curr_pose_y] = true;
            grid[curr_pose_x][curr_pose_y].expanded = true;

            for(int dir = 0; dir < NUMOFDIRS; dir++){
                newx = curr_pose_x + dX[dir];
                newy = curr_pose_y + dY[dir];
                //cout<< "NEW "<<newx<<","<<newy<<endl;

                if(newx > 0 && newx < x_size && newy > 0 && newy < y_size){ // VAILD GRID DIMENSIONS
                    //cout<<"in map limits"<<endl;
                    if(newx == goalposeX && newy == goalposeY){
                        //cout<< "in found"<<endl;
                        grid[newx][newy].parent.first = curr_pose_x;
                        grid[newx][newy].parent.second = curr_pose_y;
                        newpose = getPath(grid, goalposeX, goalposeY);
                        //cout<<"after path"<<endl;
                        found = true;
                    
                    }
                    //cout<<"here"<<endl;
                    //cout<<grid[newx][newy].expanded<<endl;
                    
                    else if(isValid(newx,newy,x_size,y_size,map,collision_thresh) && grid[newx][newy].expanded==false) // Checking validity and successor not in CLOSED list
                    {
                        //cout<<"here2"<<endl;
                        double new_g = grid[curr_pose_x][curr_pose_y].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                        double h = sqrt(2)*MIN(abs(newx-goalposeX),abs(newy-goalposeY)) + ((MAX(abs(newx-goalposeX),abs(newy-goalposeY)))-(MIN(abs(newx-goalposeX),abs(newy-goalposeY))));
                        double new_h = h; //(int)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                        double new_f = new_g + weight*new_h;
                        if(grid[newx][newy].g > new_g || grid[newx][newy].g == INT_MAX){
                            open.insert(make_pair(new_f, make_pair(newx, newy)));
                            grid[newx][newy].g = new_g;
                            grid[newx][newy].h = new_h;
                            grid[newx][newy].f = new_f;
                            grid[newx][newy].parent = make_pair(curr_pose_x, curr_pose_y);
                            // if(OPEN[newx][newy] == false){
                                
                            //     OPEN[newx][newy] = true;
                            // }
                            //cout<<"here"<<endl;
                        }
                    }
                }       
            }
            if(found == true) break;
        }
    }
    //cout<<"here3"<<endl;

    robotposeX = newpose[0];
    robotposeY = newpose[1];
    //cout<<"New "<<robotposeX<<","<<robotposeY<<endl;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}

//***********************************************************************************//
