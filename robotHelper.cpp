#pragma GCC optimize("Ofast")

#include <iostream>
#include <fstream>
#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>

// For Hash
#include <functional>

// For all useful constants
#include <climits>

// Data-Structures and related stuff.
#include <map>
#include <set>
#include <stack>
#include <random>
#include <deque>
#include <queue>   // Includes Priority Queue
#include <vector>
#include <unordered_map>
#include <unordered_set>

using namespace std;

#define fastio() ios_base::sync_with_stdio(false);cin.tie(NULL);cout.tie(NULL)
#define MOD 1000000007
#define MOD1 998244353
#define INF 1e18
#define nline "\n"
#define pb push_back
#define ppb pop_back
#define mp make_pair
#define ff first
#define ss second
#define PI 3.141592653589793238462
#define set_bits __builtin_popcountll
#define sz(x) ((int)(x).size())
#define all(x) (x).begin(), (x).end()

typedef long long ll;
typedef unsigned long long ull;
typedef long double lld;
// typedef tree<pair<int, int>, null_type, less<pair<int, int>>, rb_tree_tag, tree_order_statistics_node_update > pbds; // find_by_order, order_of_key

#ifndef ONLINE_JUDGE

#define debug(x) cerr << #x <<" "; _print(x); cerr << endl;
ifstream  i_data("data.in");
ofstream  o_data("data.out");
#define cin  i_data
#define cout o_data
#else
#define debug(x)
#endif

#define printOutput(x)  cout<<x<<" ";
#define printOutputN(x)  cout<<x<<endl;

void _print(ll t) {cerr << t;}
void _print(int t) {cerr << t;}
void _print(string t) {cerr << t;}
void _print(char t) {cerr << t;}
void _print(lld t) {cerr << t;}
void _print(double t) {cerr << t;}
void _print(ull t) {cerr << t;}

template <class T, class V> void _print(pair <T, V> p);
template <class T> void _print(vector <T> v);
template <class T> void _print(set <T> v);
template <class T, class V> void _print(map <T, V> v);
template <class T> void _print(multiset <T> v);
template <class T, class V> void _print(pair <T, V> p) {cerr << "{"; _print(p.ff); cerr << ","; _print(p.ss); cerr << "}";}
template <class T> void _print(vector <T> v) {cerr << "[ "; for (T i : v) {_print(i); cerr << " ";} cerr << "]";}
template <class T> void _print(set <T> v) {cerr << "[ "; for (T i : v) {_print(i); cerr << " ";} cerr << "]";}
template <class T> void _print(multiset <T> v) {cerr << "[ "; for (T i : v) {_print(i); cerr << " ";} cerr << "]";}
template <class T, class V> void _print(map <T, V> v) {cerr << "[ "; for (auto i : v) {_print(i); cerr << " ";} cerr << "]";}


enum RETURN_CODES
{
        FAILURE,
        SUCCESS
};
class HelpTheRobot
{
public:
    void solve();
    bool verify(string expectedOutput);
    HelpTheRobot(string inputFilePath, int inputInitialEnergy);

private:
    RETURN_CODES createGraph();
    void dijkstra();
    void tracePath();
    bool isDigits(const std::string &str);

    vector<vector<int>> graph;
    vector<vector<int>> energy;
    vector<vector<int>> steps;
    pair<int,int> start;
    pair<int,int> destination;
    map<pair<int,int>,pair<int,int>> parent;
    int m;
    int n;
    string filePath;
    int initialEnergy;
    const int xIndex;
    const int yIndex;
    const int stepsIndex;
    const int energyIndex;
    string outputMinimumStepsPath;
};

// public method : that acts as an interface
void HelpTheRobot::solve() {
    if(createGraph() == RETURN_CODES::SUCCESS)
        dijkstra();

}

bool HelpTheRobot::verify(string expectedOutput) {

    return outputMinimumStepsPath == expectedOutput;
}


HelpTheRobot::HelpTheRobot(string inputFilePath, int inputInitialEnergy)
    : filePath(inputFilePath),
    initialEnergy(inputInitialEnergy),
      xIndex(0),
      yIndex(1),
      stepsIndex(2),
      energyIndex(3),
      outputMinimumStepsPath("")
{

}

// helper function to check if string contains only digits, spaces and returns
bool HelpTheRobot::isDigits(const std::string &str)
{
    return  str.find_first_not_of("0123456789 \r") == std::string::npos;
}

// traces back the path from the destination to the beginning node
// outputs the path from the start to the destination node
void HelpTheRobot::tracePath()
{
    vector<pair<int,int>> output;
    while(destination != start)
    {
        output.push_back(destination);
        destination = parent[destination];
    }

    output.push_back(start);

    for(int i = (int)(output.size()-1); i>-1;i--)
    {
        // x and y coordinates of the system are opposite
        // so print y,x
        outputMinimumStepsPath = outputMinimumStepsPath + to_string(output[i].second) + "," + to_string(output[i].first) + " -> ";
    }
    outputMinimumStepsPath = outputMinimumStepsPath.substr(0, outputMinimumStepsPath.size()-4);
    printOutputN(outputMinimumStepsPath);
}

// runs dijkstra algorithm to find the shortest path
void HelpTheRobot::dijkstra()
{
    auto comp = [](vector<int> a, vector<int> b)
    {
        const int stepIndex = 2;
        const int costIndex = 3;

        // if stepIndex are same, then compare the energyIndex
        if(a[stepIndex] == b[stepIndex])
        {
            return a[costIndex] > b[costIndex];
        }

        // sort items with respect tot the smaller stepsIndex
        return a[stepIndex] > b[stepIndex];
    };

    // we will story elements like this
    priority_queue<vector<int>,vector<vector<int>>, decltype(comp)> pq(comp);

    // initial coordinates are the start coordinates
    // initial steps and energy consumed are zero
    pq.push({start.first, start.second, 0, 0});

    while(pq.size() > 0)
    {
        auto current = pq.top();
        pq.pop();

        debug(current);

        // if current node is our destination, we have found the shortest path, hence print the path
        if(current[xIndex] == destination.first && current[yIndex] == destination.second)
        {
            debug(current[stepsIndex]);
            debug(current[energyIndex]);
            tracePath();
            return;
        }

        // find the new neighbours
        vector<int> dir = {0,1,0,-1,0};
        for(int i=0;i<(int)(dir.size()-1);i++)
        {
            int row = dir[i] + current[xIndex];
            int col = dir[i+1] + current[yIndex];
            // if the row and col are within the boundaries and if our current energy is within the initial energy range and current steps is lesser than the steps required to reach the node
            if(row<m && col< n && row > -1 && col > -1 &&  (current[energyIndex] + graph[row][col]) <= initialEnergy && (current[energyIndex] + graph[row][col])  <= energy[row][col] && current[stepsIndex] <= steps[row][col] )
            {
                // push the new node, update the energy, parent and steps
                debug(row);
                debug(col);
                pq.push({row,col,current[stepsIndex]+1, (current[energyIndex] + graph[row][col])});
                parent[{row,col}] = {current[xIndex],current[yIndex]};
                energy[row][col] = (current[energyIndex] + graph[row][col]);
                steps[row][col] = current[stepsIndex]+1;
            }
        }
    }
}

// parses the input file and creates the graph , energy and steps 2d matrix
RETURN_CODES HelpTheRobot::createGraph()
{
    try {
        std::ifstream file(filePath);

        std::string str;
        while (std::getline(file, str)) {
            vector<int> row;
            std::istringstream iss(str);

            std::string s;
            while (std::getline(iss, s, ',')) {
                if (isDigits(s)) {
                    row.push_back(stoi(s));
                } else {
                    if (s.find('R') != std::string::npos) {
                        start = make_pair((int) (graph.size()), (int) (row.size()));
                    } else if (s.find('D') != std::string::npos) {
                        destination = make_pair((int) (graph.size()), (int) (row.size()));
                    } else {
                        debug("Invalid input ");
                        return RETURN_CODES::FAILURE;
                    }
                    row.push_back(0);
                }
            }
            graph.push_back(row);
        }
        debug(graph);
        debug(start);
        debug(destination);

        m = static_cast<int>(graph.size());

        if(m>0) {
            n = static_cast<int>(graph[0].size());
        }
        else
        {
            return RETURN_CODES::FAILURE;
        }

        debug(m);
        debug(n);
        energy.resize(m, vector<int>(n, INT_MAX));
        steps.resize(m, vector<int>(n, INT_MAX));

    }
    catch (const ifstream::failure& e)
    {
        debug(" Exception error reading or opening file ");
        return RETURN_CODES::FAILURE;
    }

    return  RETURN_CODES::SUCCESS;
}

int main(int argc, char** argv)
{
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    cout.tie(nullptr);

    if(argc == 3)
    {
        string str = argv[2];

        // check if initial energy consists of only digits
        if(str.find_first_not_of("0123456789 \r") != std::string::npos)
        {
            debug("Initial energy must be a number ");
            return 0;
        }

        HelpTheRobot robotHelper(argv[1],stoi(argv[2]));
        robotHelper.solve();

        string expectedOutput =  "";
        debug(robotHelper.verify(expectedOutput));
    }
    else
    {
        debug("Invalid Number of Arguments , correct format is ./main fileName initialEnergy");
    }


    return 0;
}