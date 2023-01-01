#include <stdlib.h> 
#include <fstream> 
#include <vector>
#include <string.h>
#include <iostream>
#include <limits>
#include <ctime>
#include <queue>
#include <functional>
#include <math.h>
#include <cstdlib>
using namespace std;


const int n_parameters = 4;                                         // Number of parameters
int n_visited = 0;                                                  // Number of visited nodes
int n_explored = 0;                                                 // Number of explored nodes
int n_leafs = 0;                                                    // Number of leaf nodes
int updated_n_leafs = 0;                                            // Number of updated leafs
int no_feasible = 0;                                                // Number of no feasible discards 
int no_promising = 0;                                               // Number of no promising discards
int lastly_discarded_promising = 0;                                 // Number of nodes that were promising but lastly stopped being
int n_directions = 8;                                               // Number of posible directions
bool no_exit = false; 
int n_updates_pessimistic = 0;                                      // Number of best updates from pessimistic bounds
// Node data structure , with optimal bound, length
struct Node
{
    int steps; 
    long unsigned int i;
    long unsigned int j;
    int opt_bound = numeric_limits<int>::max();

    bool operator< (const Node& n) const 
    {
        return steps > n.steps;
    }
};


int maze_greedy(vector<vector<int>> &matrix,int n ,int m,int i ,int j){
    auto pasos = 1;
    bool flag = false;

    if(matrix[i][j] == 1){

        while(true)
        {   
            if(i == n - 1 && j == m - 1){
                flag = true;
                break;
            }
            if(i + 1 < n && j + 1 < m && matrix[i+1][j+1] == 1){
                pasos++;
                i++;
                j++;
            }
            else if(i + 1 < n && j < m && matrix[i+1][j] == 1){
                pasos++;
                i++;
            }
            else if(i < n && j + 1 < m && matrix[i][j+1] == 1){
                pasos++;
                j++;
            }
            else{
                break;
            }
        }
    }
    
    if(flag){
        return pasos;
    }
    else{
        return numeric_limits<int>::max();
    }

}

// No feasible
bool not_feasible(Node &n,vector<vector<int>> &maze)
{
    // Out of maze or not accesible
    return (n.i < 0 || n.j < 0 || n.j > maze[0].size()-1 || n.i > maze.size()-1 || maze[n.i][n.j] == 0);
}

// Promising
bool is_promising(vector <vector <long int>> &visit,Node &n,int n_final , int m_final)
{   
    // Matrix of visits bigger than actual cost or actual cost with chebysnev as optimistic level is less than optimal bound 
    if((visit[n.i][n.j] > n.steps) || (n.steps + max((n_final - int(n.i)),(m_final-int(n.j))) < n.opt_bound))
    {
        n.opt_bound =  n.steps + max((n_final - int(n.i)),(m_final-int(n.j)));
        return true;
    }

    return false;
}


// Initialize node to aproppiate values
void initialize(Node &n,int i , int j , int steps)
{
    n.i = i; 
    n.j = j; 
    n.steps = steps;
    n.opt_bound = 0;
}

// Expansion of nodes, width search 
vector<Node> expand(Node n, vector<vector<int>>& maze,vector<vector<long int>> &visited) {
    vector<Node> v_nodes;
    
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            
            // Where you are at the moment
            if (i == 0 && j == 0) continue;
            
            // Generating adyacent node
            Node a;
            a.i = n.i + i;
            a.j = n.j + j;
            a.steps = n.steps + 1;
            a.opt_bound = (maze.size()-1 - n.i) + (maze[0].size() - 1 - n.j);

            // Adyacent node is feasible or valid
            if (not_feasible(a,maze))
            {
                no_feasible++;
                continue;
            }
            
            // Cutting branches and pruning  
            if(!is_promising(visited,a,maze.size()-1,maze[0].size()-1))
            {
                no_promising++;
                continue;
            }
            
            
            // Adding adyacent promising node 
            v_nodes.push_back(a);
        }
    }
    
    return v_nodes;
}


// Maze solver using the methods from branching and pruning 
int maze_bb(vector<vector<int>> maze , int i , int j,vector<vector<long int>> &visited)
{
    if(maze[0][0] == 1)
    {
        // Creating and assigning values
        Node initial; 
        initialize(initial,0,0,1); 
        int best_steps =  maze_greedy(maze,maze.size()-1,maze[0].size()-1,initial.i,initial.j);;
        priority_queue <Node> pq;
        pq.push(initial);

        // General algorithm
        while( !pq.empty() )
        {
            Node n = pq.top();                                          // Save top  node on n 
            pq.pop();                                                   // Eliminate top node from q

            if(!is_promising(visited,n,maze.size()-1,maze[0].size()-1))
            {
                lastly_discarded_promising++;
                continue;
            }
            
            // Arrive at exit
            if(n.i == maze.size() - 1 && n.j == maze[0].size() - 1)
            {
                n_leafs++;
                no_exit = false;
                // Replace if you get a better solution 
                if(best_steps > n.steps)    
                {
                    updated_n_leafs++;
                    // Shortest solution
                    best_steps = n.steps;                               
                    visited[n.i][n.j] = n.steps;
                    break;
                }
            }

            visited[n.i][n.j] = n.steps;

            vector<Node> v_nodes = expand(n,maze,visited); 
            // Expanding the nodes
            for(Node a : v_nodes)                                                       // By default 8 , all posible movements 
            {   
                n_visited++;
                                
                // Feasible 
                if(!not_feasible(a,maze))
                {   
                    // Pessimistic level with greedy algorithm
                    int greedy = maze_greedy(maze,maze.size()-1,maze[0].size()-1,a.i,a.j);
                    if(greedy != numeric_limits<int>::max())
                    {
                        greedy = greedy + a.steps;

                    }

                    if(greedy < best_steps)
                    {
                        n_updates_pessimistic++;
                        best_steps = greedy;
                        continue;
                    } 

                    // Promising
                    if(is_promising(visited,a,maze.size()-1,maze[0].size()-1)) 
                    {   
                        n_explored++;
                        pq.push(a);
                    }
                    else
                    {
                        no_promising++;
                    }
                }

            }
        }
        return best_steps;
    }
    n_visited = 1;  
    no_exit = true;
    return 0;
}

int min_neighbour(vector<vector<long int>> visited,int x , int y)
{
    long int min = visited[visited.size()-1][visited[0].size()-1];
    int x_aux = 0; 
    int y_aux = 0;
    

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            
            if (i == 0 && j == 0) continue;

            if((x + i < 0 || x + i >= int(visited.size()) || y + j < 0 || y + j >= int(visited[0].size()) || visited[x+i][x+j] == 0)) continue;

            if(min > visited[x+i][y+j])
            {
                x_aux = x + i;
                y_aux = y + j;
                min = visited[x+i][y+j];
            }
        }
    }
        
    if(x_aux == x - 1 && y_aux == y )                       // North
    {
        return 1;
    } 
    else if(x_aux ==  x - 1 && y_aux == y + 1)              // North East
    {
        return 2;
    }
    else if(x_aux == x && y_aux == y + 1)                   // East 
    {
        return 3;
    }
    else if(x_aux == x + 1 && y_aux == y + 1)               // South East
    {
        return 4;
    }
    else if(x_aux == x + 1 && y_aux == y)                   // South
    {
        return 5;
    }
    else if(x_aux == x + 1 && y_aux == y- 1)                // South West
    {
        return 6;
    }
    else if(x_aux == x && y_aux == y - 1)                   // West 
    {
        return 7;
    }
    else if(x_aux == x - 1 && y_aux == y - 1)               // North West
    {
        return 8;
    }

    return -1;
}

void maze_p2D_p(vector<vector<long int>> &visited,vector<int> &best_directions,vector<vector<int>> &maze,vector<bool> &parameters)
{   
    if(parameters[1] && !no_exit)
    {   
        int i = visited.size()-1, j = visited[0].size()-1;
        vector<vector<long int>> aux = visited;
        visited[i][j] = -1;
        while(true)
        {
            int min_direction = min_neighbour(aux,i,j);
            if(i == 0 && j == 0) break; 
            // Where i come from
            switch(min_direction)
            {
                case 1:                                                 
                    i--; 

                    best_directions.push_back(5);                       // From North 
                    break;
                case 2: 
                    i--; 
                    j++;

                    best_directions.push_back(6);                       // From North East 
                    break;
                case 3: 
                    j++;

                    best_directions.push_back(7);                       // From East
                    break;
                case 4: 
                    i++;
                    j++;
                    
                    best_directions.push_back(8);                       // From South East
                    break;
                case 5: 
                    i++;

                    best_directions.push_back(1);                        // From South
                    break;
                case 6:
                    i++;
                    j--;

                    best_directions.push_back(2);                        // From South West
                    break;
                case 7:
                    j--;

                    best_directions.push_back(3);                        // From West
                    break;
                case 8:
                    j--;
                    i--;
                    best_directions.push_back(4);                       // From North West
                    break;
        
            }
        visited[i][j] =  -1;
        }

        // Printing shortest Path
        for(int i = 0 ; i < int(visited.size());i++)
        {
            for(int j = 0 ; j < int(visited[0].size()) ; j++)
            {
                if(visited[i][j] == -1)
                {
                    cout << "*";
                }
                else
                {
                    cout << maze[i][j] ;
                }
            }
            cout << endl;
        }

    }
    else if(parameters[1])
    {
        cout << "NO EXIT" << endl;
    }


    // Shows Vector with best directions of the shortest path 
    if(parameters[0] && !no_exit)
    {
        cout << "<";
        for(int i = 0 ; i < int(best_directions.size()); i++)
        {
            cout << best_directions[i];
        }
        cout << ">" << endl;
    }
    else if(!parameters[0])
    {
        cout << "<>" << endl;    
    }
    else
    {
        cout << "<NO EXIT>" << endl;
    }

}

int main(int argc,char **argv)
{

    vector<bool> parameters(n_parameters,false);                    // Parameters
    vector<vector<int>> maze;                                       // Maze  
    vector<vector<long int>> visited;                               // Visited positions of the maze
    
    vector<int> best_directions;                                    // Best directions
    int n , m;                                                      // Dimensions of the maze
    long int step = 0;                                              // Number of steps 
    fstream f;                                                      // File that will contain the maze

    // ******************************************************* Parameters **********************************************************
    
 /*    0 : -p
    1 : --p2D
    2 : -f
    3 : File name  */

    // **************************************************** Error Handling *********************************************************

    if(argc == 1){
        cerr << "Usage: " <<endl << "maze [-p] -f file" << endl;
        exit(EXIT_FAILURE);
    }


    for(int i = 1 ; i < argc ; i++)
    {
        if(parameters[2] && !parameters[3]){
            parameters[3] = true;
            f.open(argv[i],fstream::in);
            if(!f.is_open()){
                cerr << "ERROR : can’t open file: " << argv[i] << endl << "Usage: " <<endl << "maze [-p] [--p2D] -f file" << endl;
                exit(EXIT_FAILURE);
            }
        }
        else{
            if(!strcmp(argv[i],"-p")){
                parameters[0] = true;
            }
            else if(!strcmp(argv[i],"--p2D")){
                parameters[1] = true; 
            }
            else if(!strcmp(argv[i],"-f")){
                parameters[2] = true;
            }
            else{
                cerr << "ERROR : unknown option  " << argv[i] << endl << "Usage: " <<endl << "maze [-p] [--p2D] -f file" << endl;
                exit(EXIT_FAILURE);
            }
        }
    }

    if(parameters[1] && !f.is_open())
    {
        cerr << "ERROR : missing filename " << endl << "Usage: " <<endl << "maze [-p] [--p2D] -f file" << endl;
        exit(EXIT_FAILURE);
    }
    

    // ******************************************* Assigning values from file **********************************************
    f >> n >> m; 
    
    maze.resize( n , vector<int> (m, 0)); 
    visited.resize( n , vector<long int>(m,0));
   

    for(int i = 0 ; i < n ; i++){
        for(int j = 0 ; j < m ; j++){
            f >> maze[i][j];
            visited[i][j] = numeric_limits<long int>::max();
        }
    }    
    
    // Time of the algorithm    
    clock_t start = clock();
    step = maze_bb(maze,0,0,visited);
    clock_t end = clock();
    double time = 1000 * double((end-start)) / CLOCKS_PER_SEC;
    
    // Printing the number of all types of node.q
    cout << step << endl;
    cout << n_visited << " " << n_explored << " " << n_leafs << " " << no_feasible << " ";
    cout << no_promising << " " << lastly_discarded_promising << " ";
    cout << updated_n_leafs << " " << n_updates_pessimistic <<endl;
    cout << time << endl;

    // --P2D or -p has been selected
    if(parameters[0] || parameters[1])
    {
       maze_p2D_p(visited,best_directions,maze,parameters);
    }

    f.close();

    return 0; 
} 
