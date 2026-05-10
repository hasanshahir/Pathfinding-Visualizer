#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <ctime>
#include <set>

using namespace std;

struct Pos
{
    int r, c;
    bool operator<(const Pos &other) const
    {
        if (r != other.r)
            return r < other.r;
        return c < other.c;
    }
    bool operator==(const Pos &other) const
    {
        return r == other.r && c == other.c;
    }
};

struct Node
{
    Pos pos;
    double fCost;
    bool operator>(const Node &other) const { return fCost > other.fCost; }
};

const int ROWS = 15;
const int COLS = 30;
const int RENDER_INTERVAL = 50;

enum Algorithm { ASTAR = 1, DIJKSTRA = 2, BFS = 3 };

double getH(Pos a, Pos b)
{
    return sqrt(pow(a.r - b.r, 2) + pow(a.c - b.c, 2));
}

void clearScreen()
{
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

void render(const vector<vector<char>> &grid, int checks = 0, double duration = 0, double pathLen = 0)
{
    clearScreen();
    for (int i = 0; i < ROWS; ++i)
    {
        for (int j = 0; j < COLS; ++j)
        {
            char cell = grid[i][j];
            if (cell == 'S')
                cout << "\033[32mO \033[0m";
            else if (cell == 'E')
                cout << "\033[31mX \033[0m";
            else if (cell == '#')
                cout << "[]";
            else if (cell == 'v')
                cout << "\033[34m. \033[0m";
            else if (cell == '*')
                cout << "\033[33m* \033[0m";
            else
                cout << ". ";
        }
        cout << "\n";
    }
    cout << "\n--- Statistics ---" << endl;
    cout << "Total Checks: " << checks << " | Path Length: " << pathLen << " units" << endl;
    if (duration > 0)
        cout << "Time Taken: " << duration << " ms" << endl;
}

void clearBuffer()
{
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
}

int countAvailableSpaces(const vector<vector<char>> &grid)
{
    int count = 0;
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLS; j++)
        {
            if (grid[i][j] != '#')
                count++;
        }
    }
    return count;
}

void createWallDesign(vector<vector<char>> &grid, int design)
{
    for (int i = 0; i < ROWS; i++)
        for (int j = 0; j < COLS; j++)
            grid[i][j] = ' ';

    if (design == 1)
    {
        for (int i = 3; i < 12; i++)
            grid[i][12] = '#';
        for (int j = 5; j < 20; j++)
            grid[8][j] = '#';
    }
    else if (design == 2)
    {
        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLS; j++)
            {
                if ((i + j) % 5 == 0 && (i + j) % 7 != 0)
                    grid[i][j] = '#';
            }
        }
    }
    else if (design == 3)
    {
        for (int i = 0; i < ROWS; i++)
        {
            grid[i][7] = '#';
            grid[i][15] = '#';
            grid[i][23] = '#';
        }
        for (int j = 0; j < COLS; j++)
        {
            grid[5][j] = '#';
            grid[10][j] = '#';
        }
        grid[5][7] = ' ';
        grid[5][15] = ' ';
        grid[10][7] = ' ';
        grid[10][15] = ' ';
    }
    else if (design == 4)
    {
        int centerR = ROWS / 2;
        int centerC = COLS / 2;
        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLS; j++)
            {
                int dist = max(abs(i - centerR), abs(j - centerC));
                if (dist % 3 == 0 && dist > 0)
                    grid[i][j] = '#';
            }
        }
    }
    else if (design == 5)
    {
        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLS; j++)
            {
                if ((double)rand() / RAND_MAX < 0.15)
                    grid[i][j] = '#';
            }
        }
    }
}

void solveAStar(vector<vector<char>> &grid, Pos start, Pos end)
{
    auto startTime = chrono::steady_clock::now();
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push({start, getH(start, end)});

    map<Pos, Pos> parentOf;
    map<Pos, double> gCost;
    set<Pos> closed;
    gCost[start] = 0;

    int totalChecks = 0;
    int dr[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dc[] = {0, 0, -1, 1, -1, 1, -1, 1};

    bool found = false;

    while (!pq.empty())
    {
        Pos current = pq.top().pos;
        pq.pop();

        if (closed.find(current) != closed.end())
            continue;

        closed.insert(current);
        totalChecks++;

        if (totalChecks % RENDER_INTERVAL == 0)
        {
            render(grid, totalChecks);
            this_thread::sleep_for(chrono::milliseconds(5));
        }

        if (current == end)
        {
            found = true;
            break;
        }

        for (int i = 0; i < 8; i++)
        {
            int nr = current.r + dr[i];
            int nc = current.c + dc[i];
            Pos neighbor = {nr, nc};
            double weight = (i < 4) ? 1.0 : 1.414;

            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS &&
                grid[nr][nc] != '#' && closed.find(neighbor) == closed.end())
            {
                double newGCost = gCost[current] + weight;
                if (gCost.find(neighbor) == gCost.end() || newGCost < gCost[neighbor])
                {
                    gCost[neighbor] = newGCost;
                    pq.push({neighbor, newGCost + getH(neighbor, end)});
                    parentOf[neighbor] = current;
                    if (grid[nr][nc] != 'E' && grid[nr][nc] != 'S')
                        grid[nr][nc] = 'v';
                }
            }
        }
    }

    auto endTime = chrono::steady_clock::now();
    double timeTaken = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();

    if (found)
    {
        double finalLength = gCost[end];
        Pos temp = parentOf[end];
        while (!(temp == start))
        {
            grid[temp.r][temp.c] = '*';
            temp = parentOf[temp];
            render(grid, totalChecks, timeTaken, finalLength);
            this_thread::sleep_for(chrono::milliseconds(25));
        }
        render(grid, totalChecks, timeTaken, finalLength);
    }
    else
    {
        cout << "\nNo path found!" << endl;
    }
}

void solveDijkstra(vector<vector<char>> &grid, Pos start, Pos end)
{
    auto startTime = chrono::steady_clock::now();
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push({start, 0});

    map<Pos, Pos> parentOf;
    map<Pos, double> gCost;
    set<Pos> closed;
    gCost[start] = 0;

    int totalChecks = 0;
    int dr[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dc[] = {0, 0, -1, 1, -1, 1, -1, 1};

    bool found = false;

    while (!pq.empty())
    {
        Pos current = pq.top().pos;
        pq.pop();

        if (closed.find(current) != closed.end())
            continue;

        closed.insert(current);
        totalChecks++;

        if (totalChecks % RENDER_INTERVAL == 0)
        {
            render(grid, totalChecks);
            this_thread::sleep_for(chrono::milliseconds(5));
        }

        if (current == end)
        {
            found = true;
            break;
        }

        for (int i = 0; i < 8; i++)
        {
            int nr = current.r + dr[i];
            int nc = current.c + dc[i];
            Pos neighbor = {nr, nc};
            double weight = (i < 4) ? 1.0 : 1.414;

            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS &&
                grid[nr][nc] != '#' && closed.find(neighbor) == closed.end())
            {
                double newGCost = gCost[current] + weight;
                if (gCost.find(neighbor) == gCost.end() || newGCost < gCost[neighbor])
                {
                    gCost[neighbor] = newGCost;
                    pq.push({neighbor, newGCost});
                    parentOf[neighbor] = current;
                    if (grid[nr][nc] != 'E' && grid[nr][nc] != 'S')
                        grid[nr][nc] = 'v';
                }
            }
        }
    }

    auto endTime = chrono::steady_clock::now();
    double timeTaken = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();

    if (found)
    {
        double finalLength = gCost[end];
        Pos temp = parentOf[end];
        while (!(temp == start))
        {
            grid[temp.r][temp.c] = '*';
            temp = parentOf[temp];
            render(grid, totalChecks, timeTaken, finalLength);
            this_thread::sleep_for(chrono::milliseconds(25));
        }
        render(grid, totalChecks, timeTaken, finalLength);
    }
    else
    {
        cout << "\nNo path found!" << endl;
    }
}

void solveBFS(vector<vector<char>> &grid, Pos start, Pos end)
{
    auto startTime = chrono::steady_clock::now();
    queue<Pos> q;
    q.push(start);

    map<Pos, Pos> parentOf;
    set<Pos> visited;
    visited.insert(start);

    int totalChecks = 0;
    int dr[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dc[] = {0, 0, -1, 1, -1, 1, -1, 1};

    bool found = false;

    while (!q.empty())
    {
        Pos current = q.front();
        q.pop();
        totalChecks++;

        if (totalChecks % RENDER_INTERVAL == 0)
        {
            render(grid, totalChecks);
            this_thread::sleep_for(chrono::milliseconds(5));
        }

        if (current == end)
        {
            found = true;
            break;
        }

        for (int i = 0; i < 8; i++)
        {
            int nr = current.r + dr[i];
            int nc = current.c + dc[i];
            Pos neighbor = {nr, nc};

            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS &&
                grid[nr][nc] != '#' && visited.find(neighbor) == visited.end())
            {
                visited.insert(neighbor);
                parentOf[neighbor] = current;
                q.push(neighbor);
                if (grid[nr][nc] != 'E' && grid[nr][nc] != 'S')
                    grid[nr][nc] = 'v';
            }
        }
    }

    auto endTime = chrono::steady_clock::now();
    double timeTaken = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();

    if (found)
    {
        double finalLength = 0;
        Pos temp = parentOf[end];
        while (!(temp == start))
        {
            grid[temp.r][temp.c] = '*';
            temp = parentOf[temp];
            finalLength++;
            render(grid, totalChecks, timeTaken, finalLength);
            this_thread::sleep_for(chrono::milliseconds(25));
        }
        render(grid, totalChecks, timeTaken, finalLength);
    }
    else
    {
        cout << "\nNo path found!" << endl;
    }
}

int main()
{
    srand((unsigned)time(0));
    bool keepRunning = true;

    while (keepRunning)
    {
        vector<vector<char>> grid(ROWS, vector<char>(COLS, ' '));

        clearScreen();
        cout << "=== Advanced Pathfinding Visualizer ===" << endl << endl;
        cout << "Select Algorithm:" << endl;
        cout << "1. A* (Heuristic-based, fastest)" << endl;
        cout << "2. Dijkstra (Optimal, slower)" << endl;
        cout << "3. BFS (Simple, explores uniformly)" << endl;
        cout << "Choice: ";
        int algoChoice;
        cin >> algoChoice;
        Algorithm algo = (Algorithm)algoChoice;

        clearScreen();
        cout << "=== Wall Design Selection ===" << endl << endl;
        cout << "1. Cross Pattern" << endl;
        cout << "2. Diagonal Maze" << endl;
        cout << "3. Multiple Rooms" << endl;
        cout << "4. Spiral Pattern" << endl;
        cout << "5. Random Sparse" << endl;
        cout << "Choice: ";
        int designChoice;
        cin >> designChoice;
        if (designChoice < 1 || designChoice > 5)
            designChoice = 1;

        createWallDesign(grid, designChoice);

        int availableSpaces = countAvailableSpaces(grid);

        clearScreen();
        render(grid);
        cout << "\nGrid Info: Total cells = " << (ROWS * COLS) << " | Available spaces = " << availableSpaces << " | Walls = " << (ROWS * COLS - availableSpaces) << endl;

        int r, c;
        cout << "\nSet Start Position (Row 0-" << ROWS - 1 << " Col 0-" << COLS - 1 << "): ";
        while (!(cin >> r >> c) || r < 0 || r >= ROWS || c < 0 || c >= COLS || grid[r][c] == '#')
        {
            cout << "Invalid coordinates. Try again: ";
            clearBuffer();
        }
        Pos start = {r, c};
        grid[r][c] = 'S';

        clearScreen();
        render(grid);
        cout << "\nGrid Info: Total cells = " << (ROWS * COLS) << " | Available spaces = " << availableSpaces << " | Walls = " << (ROWS * COLS - availableSpaces) << endl;

        cout << "\nSet End Position (Row 0-" << ROWS - 1 << " Col 0-" << COLS - 1 << "): ";
        while (!(cin >> r >> c) || r < 0 || r >= ROWS || c < 0 || c >= COLS ||
               grid[r][c] == '#' || (r == start.r && c == start.c))
        {
            cout << "Invalid coordinates. Try again: ";
            clearBuffer();
        }
        Pos end = {r, c};
        grid[r][c] = 'E';

        clearScreen();
        render(grid);
        cout << "\nGrid Info: Total cells = " << (ROWS * COLS) << " | Available spaces = " << availableSpaces << " | Walls = " << (ROWS * COLS - availableSpaces) << endl;

        string algoName;
        if (algo == ASTAR)
            algoName = "A*";
        else if (algo == DIJKSTRA)
            algoName = "Dijkstra";
        else
            algoName = "BFS";

        cout << "\nPress Enter to start " << algoName << " algorithm...";
        clearBuffer();
        cin.get();

        if (algo == ASTAR)
            solveAStar(grid, start, end);
        else if (algo == DIJKSTRA)
            solveDijkstra(grid, start, end);
        else
            solveBFS(grid, start, end);

        cout << "\nRun again? (y/n): ";
        char choice;
        cin >> choice;
        if (choice != 'y' && choice != 'Y')
            keepRunning = false;
    }

    return 0;
}
