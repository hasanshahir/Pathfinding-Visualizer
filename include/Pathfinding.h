#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <vector>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>

using namespace std;

// ================= BASE CLASS =================
class PathfindingAlgorithm
{
protected:
    int rows, cols;

    int getIdx(int r, int c)
    {
        return r * cols + c;
    }

public:
    PathfindingAlgorithm(int r, int c) : rows(r), cols(c) {}

    virtual bool solve(int start, int end, const vector<int> &gridMap) = 0;

    virtual ~PathfindingAlgorithm() {}
};

// ================= DIJKSTRA =================
class DijkstraFinder : public PathfindingAlgorithm
{
public:
    DijkstraFinder(int r, int c) : PathfindingAlgorithm(r, c) {}

    bool solve(int start, int end, const vector<int> &gridMap) override
    {
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        vector<int> dist(rows * cols, 1e9);

        dist[start] = 0;
        pq.push({0, start});

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};

        while (!pq.empty())
        {
            auto top = pq.top();
            pq.pop();
            int d = top.first;
            int curr = top.second;

            if (curr == end)
                return true;
            if (d > dist[curr])
                continue;

            int r = curr / cols;
            int c = curr % cols;

            for (int i = 0; i < 4; i++)
            {
                int nr = r + dr[i];
                int nc = c + dc[i];

                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols)
                {
                    int nIdx = getIdx(nr, nc);

                    if (gridMap[nIdx] != 1)
                    {
                        if (dist[curr] + 1 < dist[nIdx])
                        {
                            dist[nIdx] = dist[curr] + 1;
                            pq.push({dist[nIdx], nIdx});
                        }
                    }
                }
            }
        }
        return false;
    }
};

// ================= BFS =================
class BFSFinder : public PathfindingAlgorithm
{
public:
    BFSFinder(int r, int c) : PathfindingAlgorithm(r, c) {}

    bool solve(int start, int end, const vector<int> &gridMap) override
    {
        queue<int> q;
        vector<bool> visited(rows * cols, false);

        q.push(start);
        visited[start] = true;

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};

        while (!q.empty())
        {
            int curr = q.front();
            q.pop();

            if (curr == end)
                return true;

            int r = curr / cols;
            int c = curr % cols;

            for (int i = 0; i < 4; i++)
            {
                int nr = r + dr[i];
                int nc = c + dc[i];

                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols)
                {
                    int nIdx = getIdx(nr, nc);

                    if (!visited[nIdx] && gridMap[nIdx] != 1)
                    {
                        visited[nIdx] = true;
                        q.push(nIdx);
                    }
                }
            }
        }
        return false;
    }
};

// ================= DFS =================
class DFSFinder : public PathfindingAlgorithm
{
public:
    DFSFinder(int r, int c) : PathfindingAlgorithm(r, c) {}

    bool solve(int start, int end, const vector<int> &gridMap) override
    {
        stack<int> st;
        vector<bool> visited(rows * cols, false);

        st.push(start);

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};

        while (!st.empty())
        {
            int curr = st.top();
            st.pop();

            if (visited[curr])
                continue;
            visited[curr] = true;

            if (curr == end)
                return true;

            int r = curr / cols;
            int c = curr % cols;

            for (int i = 0; i < 4; i++)
            {
                int nr = r + dr[i];
                int nc = c + dc[i];

                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols)
                {
                    int nIdx = getIdx(nr, nc);

                    if (!visited[nIdx] && gridMap[nIdx] != 1)
                    {
                        st.push(nIdx);
                    }
                }
            }
        }
        return false;
    }
};

// ================= HEURISTIC =================
int heuristic(int a, int b, int cols)
{
    int x1 = a / cols, y1 = a % cols;
    int x2 = b / cols, y2 = b % cols;
    return abs(x1 - x2) + abs(y1 - y2); // Manhattan distance
}

// ================= GREEDY BEST-FIRST =================
class GreedyFinder : public PathfindingAlgorithm
{
public:
    GreedyFinder(int r, int c) : PathfindingAlgorithm(r, c) {}

    bool solve(int start, int end, const vector<int> &gridMap) override
    {
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        vector<bool> visited(rows * cols, false);

        pq.push({heuristic(start, end, cols), start});

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};

        while (!pq.empty())
        {
            int curr = pq.top().second;
            pq.pop();

            if (visited[curr])
                continue;
            visited[curr] = true;

            if (curr == end)
                return true;

            int r = curr / cols;
            int c = curr % cols;

            for (int i = 0; i < 4; i++)
            {
                int nr = r + dr[i];
                int nc = c + dc[i];

                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols)
                {
                    int nIdx = getIdx(nr, nc);

                    if (!visited[nIdx] && gridMap[nIdx] != 1)
                    {
                        pq.push({heuristic(nIdx, end, cols), nIdx});
                    }
                }
            }
        }
        return false;
    }
};

// ================= A* =================
class AStarFinder : public PathfindingAlgorithm
{
public:
    AStarFinder(int r, int c) : PathfindingAlgorithm(r, c) {}

    bool solve(int start, int end, const vector<int> &gridMap) override
    {
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        vector<int> g(rows * cols, 1e9);

        g[start] = 0;
        pq.push({heuristic(start, end, cols), start});

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};

        while (!pq.empty())
        {
            int curr = pq.top().second;
            pq.pop();

            if (curr == end)
                return true;

            int r = curr / cols;
            int c = curr % cols;

            for (int i = 0; i < 4; i++)
            {
                int nr = r + dr[i];
                int nc = c + dc[i];

                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols)
                {
                    int nIdx = getIdx(nr, nc);

                    if (gridMap[nIdx] != 1)
                    {
                        int newG = g[curr] + 1;

                        if (newG < g[nIdx])
                        {
                            g[nIdx] = newG;
                            int f = newG + heuristic(nIdx, end, cols);
                            pq.push({f, nIdx});
                        }
                    }
                }
            }
        }
        return false;
    }
};

#endif