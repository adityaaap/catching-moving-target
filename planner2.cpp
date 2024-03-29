// /*=================================================================
//  *
//  * planner.c
//  *
//  *=================================================================*/
// #include <math.h>
// //#include <mex.h>
// #include <iostream>
// #include <vector>
// #include <stack>
// #include <set>

// using namespace std;

// // /* Input Arguments */
// // #define	MAP_IN                  prhs[0]
// // #define	ROBOT_IN                prhs[1]
// // #define	TARGET_TRAJ             prhs[2]
// // #define	TARGET_POS              prhs[3]
// // #define	CURR_TIME               prhs[4]
// // #define	COLLISION_THRESH        prhs[5]


// // /* Output Arguments */
// // #define	ACTION_OUT              plhs[0]

// //access to the map is shifted to account for 0-based indexing in the map, whereas
// //1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
// #define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

// #if !defined(MAX)
// #define	MAX(A, B)	((A) > (B) ? (A) : (B))
// #endif

// #if !defined(MIN)
// #define	MIN(A, B)	((A) < (B) ? (A) : (B))
// #endif

// #define NUMOFDIRS 8

// typedef pair<int, pair<int, int> > listPair;



// struct cell {

//     pair<int,int> parent;

//     int f, g, h;
//     cell()
//         : parent(-1, -1)
//         , f(-1)
//         , g(-1)
//         , h(-1)
//     {
//     }
// };

// bool isValid(int x, int y, int x_size, int y_size, double* map, int collision_thresh){
//     if(((int)map[GETMAPINDEX(x,y,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(x,y,x_size,y_size)] < collision_thresh)){
//         return true;
//     }
//     return false;
// }

// vector<int> getPath(vector<vector<cell>> &grid, int goalposeX, int goalposeY)
// {
//     int row = goalposeX;
//     int col = goalposeY;
 
//     stack<pair<int,int>> Path;
 
//     while (!(grid[row][col].parent.first == row
//              && grid[row][col].parent.second == col)) {
//         Path.push(make_pair(row, col));
//         int temp_row = grid[row][col].parent.first;
//         int temp_col = grid[row][col].parent.second;
//         row = temp_row;
//         col = temp_col;
//     }
 
//     // Path.push(make_pair(row, col));
//     // Path.pop();
//     pair<int, int> p = Path.top();
//     return {p.first, p.second};
// }


// static void planner(
//         double*	map,
//         int collision_thresh,
//         int x_size,
//         int y_size,
//         int robotposeX,
//         int robotposeY,
//         int target_steps,
//         double* target_traj,
//         int targetposeX,
//         int targetposeY,
//         int curr_time,
//         double* action_ptr
//         )
// {
//     // 8-connected grid
//     int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
//     int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};


//     double delta=0.1;

//     double dist_to_object = sqrt(((robotposeX-targetposeX)*(robotposeX-targetposeX) + (robotposeY-targetposeY)*(robotposeY-targetposeY)));

//     mexPrintf("\n target steps: %d", target_steps);
//     mexPrintf("\n curr_time: %d", curr_time);
//     mexPrintf("\n dist2obj: %f", dist_to_object);
//     mexPrintf("\n delta*dist: %f", delta*dist_to_object);

//     int goalposeX = (int) target_traj[curr_time+(int)(delta*dist_to_object)];
//     int goalposeY = (int) target_traj[curr_time+target_steps+(int)(delta*dist_to_object)];

//     // int goalposeX = (int) target_traj[target_steps-1];
//     // int goalposeY = (int) target_traj[target_steps-1+target_steps];

//     // int goalposeX = targetposeX;
//     // int goalposeY = targetposeY;


//     if(dist_to_object<20 || (int)(*(&target_traj + 1) - target_traj)==0){
//         int goalposeX = targetposeX;
//         int goalposeY = targetposeY;
//     }

//     mexPrintf("\n targetpose is %d,%d", targetposeX, targetposeY);
//     mexPrintf("\n goalpose is %d,%d", goalposeX, goalposeY);

//     vector<vector<bool>> closed(x_size, vector<bool> (y_size, false));
    
//     int i, j;

//     vector<vector<cell>> grid(x_size, vector<cell>(y_size));

//     for (i = 0; i < x_size; i++) {
//         for (j = 0; j < y_size; j++) {
//             grid[i][j].f = INT_MAX;
//             grid[i][j].g = INT_MAX;
//             grid[i][j].h = INT_MAX;
//             grid[i][j].parent = make_pair(-1, -1);
//         }
//     }

//     i=robotposeX, j=robotposeY;

//     grid[i][j].f = 0;
//     grid[i][j].g = 0;
//     grid[i][j].h = 0;
//     grid[i][j].parent = make_pair(i, j);

//     set<listPair> open;
//     open.insert(make_pair(0, make_pair(i, j)));
    
//     int newx, newy;
//     vector<int> new_pose={robotposeX, robotposeY};
//     bool found_path = false;
//     if(robotposeX!=goalposeX && robotposeY!=goalposeY){
//         while (!open.empty()) {
//             listPair curr = *open.begin(); // remove s with the smallest f(s) from OPEN;
//             open.erase(open.begin()); // remove s from OPEN
//             i = curr.second.first;
//             j = curr.second.second;
//             closed[i][j] = true; //insert s into CLOSED
//             // mexPrintf("\n child %d, %d", i, j);
//             int gNew, hNew, fNew;
//             for(int dir = 0; dir < NUMOFDIRS; dir++)
//             {
//                 newx = i + dX[dir];
//                 newy = j + dY[dir];
//                 // mexPrintf("\n child %d, %d", newx, newy);

//                 if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)  //if new pose is within the map
//                 {   
//                     // mexPrintf("\n opensize %d", open.size());      
//                     if (newx==goalposeX && newy==goalposeY)  //if new pose is the goal pose
//                     {
//                         // mexPrintf("\n 3");
//                         grid[newx][newy].parent.first = i;
//                         grid[newx][newy].parent.second = j;
//                         new_pose= getPath(grid, goalposeX, goalposeY);
//                         found_path=true;
//                     }
//                     else if(closed[newx][newy]==false && isValid(newx,newy,x_size,y_size,map,collision_thresh)) // if new pose is not in CLOSED and is valid
//                     {
//                         gNew = grid[i][j].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
//                         hNew = (int)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
//                         fNew = gNew + hNew;
//                         // mexPrintf("\n 4");
//                         if (grid[newx][newy].g == INT_MAX || grid[newx][newy].g > gNew) //if g(s')>g(s)+c(s,s')
//                         {
//                             open.insert(make_pair(fNew, make_pair(newx, newy))); // insert s' in OPEN
//                             // mexPrintf("\n 5");
//                             grid[newx][newy].f = fNew;  
//                             grid[newx][newy].g = gNew;  // update g(s')
//                             grid[newx][newy].h = hNew;
//                             grid[newx][newy].parent = make_pair(i, j);
//                         }
//                     }
//                 }
//             }
//             if(found_path) break;   
//         }
//     }
    

//     // // :::::::::::::::::::::: planner :::::::::::::::::::::::::::::::::::::::::::::::::
//     // mexPrintf("\n found path %d", found_path);
//     // mexPrintf("\n robot: %d %d", robotposeX, robotposeY);
//     // mexPrintf("\n next goal is %d,%d \n", new_pose[0], new_pose[1]);

//     robotposeX = new_pose[0];
//     robotposeY = new_pose[1];
//     action_ptr[0] = robotposeX;
//     action_ptr[1] = robotposeY;

//     // mexPrintf("object: %d %d;\n", (int) target_traj[curr_time],(int) target_traj[curr_time+target_steps]);
//     // mexPrintf("dist_to_object: %d ;\n", dist_to_object);
    
//     return;
// }