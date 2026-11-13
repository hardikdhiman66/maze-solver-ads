// Maze Solver - Advanced Data Structures Project
// All core data structures are written by hand (no std::queue / std::stack / priority_queue / etc.)
//
// Data structures used:
//   1. Disjoint Set Union (Union-Find)  -> Kruskal's maze generation
//   2. Queue (array based)              -> BFS
//   3. Stack (array based)              -> DFS
//   4. Min-Heap (priority queue)        -> A*
//
// Build : g++ -O2 -std=c++17 -o maze maze.cpp
// Run   : ./maze [size] [seed]     e.g.  ./maze 21 7
// Output: maze_data.js  (open viewer.html in a browser to see the animation)

#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <random>
#include <algorithm>
#include <cstdlib>
using namespace std;

// ---------------------------------------------------------------
// 1. DISJOINT SET UNION (Union-Find)
//    find()  : path compression
//    unite() : union by rank
// ---------------------------------------------------------------
struct DSU {
    vector<int> parent, rnk;
    DSU(int n) : parent(n), rnk(n, 0) {
        for (int i = 0; i < n; i++) parent[i] = i;
    }
    int find(int x) {
        if (parent[x] != x) parent[x] = find(parent[x]); // path compression
        return parent[x];
    }
    bool unite(int a, int b) {
        a = find(a); b = find(b);
        if (a == b) return false;           // already connected -> would make a cycle
        if (rnk[a] < rnk[b]) swap(a, b);    // attach smaller tree under bigger
        parent[b] = a;
        if (rnk[a] == rnk[b]) rnk[a]++;
        return true;
    }
};

// ---------------------------------------------------------------
// 2. QUEUE (FIFO) - array with a head index. Used by BFS.
// ---------------------------------------------------------------
struct MyQueue {
    vector<int> data;
    int head = 0;
    void enqueue(int x) { data.push_back(x); }
    int dequeue() { return data[head++]; }
    bool empty() const { return head == (int)data.size(); }
};

// ---------------------------------------------------------------
// 3. STACK (LIFO) - array with a top index. Used by DFS.
//    Stores (node, parent) pairs.
// ---------------------------------------------------------------
struct MyStack {
    vector<pair<int,int>> data;
    void push(int node, int from) { data.push_back({node, from}); }
    pair<int,int> pop() { auto t = data.back(); data.pop_back(); return t; }
    bool empty() const { return data.empty(); }
};

// ---------------------------------------------------------------
// 4. MIN-HEAP (priority queue). Array representation of a binary tree:
//    parent of i = (i-1)/2, children = 2i+1 and 2i+2.
//    Used by A*. Each entry = (f score, node).
// ---------------------------------------------------------------
struct MinHeap {
    vector<pair<int,int>> a;
    bool empty() const { return a.empty(); }

    void push(int f, int node) {
        a.push_back({f, node});
        int i = a.size() - 1;
        while (i > 0) {                       // sift up
            int p = (i - 1) / 2;
            if (a[p].first <= a[i].first) break;
            swap(a[p], a[i]);
            i = p;
        }
    }

    pair<int,int> pop() {                      // remove smallest
        pair<int,int> top = a[0];
        a[0] = a.back();
        a.pop_back();
        int i = 0, n = a.size();
        while (true) {                         // sift down
            int l = 2 * i + 1, r = 2 * i + 2, small = i;
            if (l < n && a[l].first < a[small].first) small = l;
            if (r < n && a[r].first < a[small].first) small = r;
            if (small == i) break;
            swap(a[i], a[small]);
            i = small;
        }
        return top;
    }
};

// ---------------------------------------------------------------
// MAZE
// ---------------------------------------------------------------
struct Maze {
    int H, W;
    vector<int> grid;   // 1 = wall, 0 = open
    int start, end;

    int id(int r, int c) const { return r * W + c; }
    bool open(int r, int c) const {
        return r >= 0 && r < H && c >= 0 && c < W && grid[id(r, c)] == 0;
    }
};

// Kruskal's algorithm to generate a perfect maze (exactly one path between any 2 cells)
Maze generateMaze(int size, unsigned seed) {
    Maze m;
    if (size % 2 == 0) size++;           // must be odd
    m.H = m.W = size;
    m.grid.assign(size * size, 1);       // start with all walls

    // cells sit on odd coordinates
    for (int r = 1; r < size; r += 2)
        for (int c = 1; c < size; c += 2)
            m.grid[m.id(r, c)] = 0;

    // list of possible walls to remove: (cellA, cellB, wallBetween)
    struct Edge { int a, b, wall; };
    vector<Edge> edges;
    for (int r = 1; r < size; r += 2) {
        for (int c = 1; c < size; c += 2) {
            if (c + 2 < size) edges.push_back({m.id(r, c), m.id(r, c + 2), m.id(r, c + 1)});
            if (r + 2 < size) edges.push_back({m.id(r, c), m.id(r + 2, c), m.id(r + 1, c)});
        }
    }

    mt19937 rng(seed);
    shuffle(edges.begin(), edges.end(), rng);   // random order = random maze

    DSU dsu(size * size);
    for (auto &e : edges) {
        if (dsu.unite(e.a, e.b))                // different sets? then knock the wall down
            m.grid[e.wall] = 0;
    }

    m.start = m.id(1, 1);
    m.end = m.id(size - 2, size - 2);
    return m;
}

// ---------------------------------------------------------------
// SOLVERS
// ---------------------------------------------------------------
struct Result {
    vector<int> order;   // cells in the order they were explored
    vector<int> path;    // final path start -> end
    long long micros = 0;
};

vector<int> buildPath(const vector<int> &parent, int start, int end) {
    vector<int> path;
    if (parent[end] == -1 && end != start) return path;   // no path
    for (int v = end; v != -1; v = parent[v]) path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

const int DR[4] = {-1, 1, 0, 0};
const int DC[4] = {0, 0, -1, 1};

// BFS: uses a QUEUE -> explores level by level -> always shortest path
Result solveBFS(const Maze &m) {
    auto t0 = chrono::steady_clock::now();
    Result res;
    vector<int> parent(m.H * m.W, -1);
    vector<bool> visited(m.H * m.W, false);
    MyQueue q;
    q.enqueue(m.start);
    visited[m.start] = true;

    while (!q.empty()) {
        int u = q.dequeue();
        res.order.push_back(u);
        if (u == m.end) break;
        int r = u / m.W, c = u % m.W;
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (!m.open(nr, nc)) continue;
            int v = m.id(nr, nc);
            if (!visited[v]) {
                visited[v] = true;
                parent[v] = u;
                q.enqueue(v);
            }
        }
    }
    res.path = buildPath(parent, m.start, m.end);
    res.micros = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t0).count();
    return res;
}

// DFS: uses a STACK -> goes deep first -> finds a path, not necessarily the shortest
Result solveDFS(const Maze &m) {
    auto t0 = chrono::steady_clock::now();
    Result res;
    vector<int> parent(m.H * m.W, -1);
    vector<bool> visited(m.H * m.W, false);
    MyStack st;
    st.push(m.start, -1);

    while (!st.empty()) {
        auto [u, from] = st.pop();
        if (visited[u]) continue;
        visited[u] = true;
        parent[u] = from;
        res.order.push_back(u);
        if (u == m.end) break;
        int r = u / m.W, c = u % m.W;
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (!m.open(nr, nc)) continue;
            int v = m.id(nr, nc);
            if (!visited[v]) st.push(v, u);
        }
    }
    res.path = buildPath(parent, m.start, m.end);
    res.micros = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t0).count();
    return res;
}

// A*: uses a MIN-HEAP ordered by f = g + h
//     g = steps taken so far, h = Manhattan distance to the end (heuristic)
Result solveAStar(const Maze &m) {
    auto t0 = chrono::steady_clock::now();
    Result res;
    const int INF = 1000000000;
    vector<int> parent(m.H * m.W, -1);
    vector<int> g(m.H * m.W, INF);
    vector<bool> closed(m.H * m.W, false);

    int er = m.end / m.W, ec = m.end % m.W;
    auto h = [&](int v) { return abs(v / m.W - er) + abs(v % m.W - ec); };

    MinHeap heap;
    g[m.start] = 0;
    heap.push(h(m.start), m.start);

    while (!heap.empty()) {
        auto [f, u] = heap.pop();
        if (closed[u]) continue;            // stale heap entry, skip
        closed[u] = true;
        res.order.push_back(u);
        if (u == m.end) break;
        int r = u / m.W, c = u % m.W;
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (!m.open(nr, nc)) continue;
            int v = m.id(nr, nc);
            if (closed[v]) continue;
            int ng = g[u] + 1;
            if (ng < g[v]) {
                g[v] = ng;
                parent[v] = u;
                heap.push(ng + h(v), v);
            }
        }
    }
    res.path = buildPath(parent, m.start, m.end);
    res.micros = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t0).count();
    return res;
}

// ---------------------------------------------------------------
// OUTPUT  (JSON-like JS file read by viewer.html)
// ---------------------------------------------------------------
void writeArray(ofstream &out, const vector<int> &v) {
    out << "[";
    for (size_t i = 0; i < v.size(); i++) {
        if (i) out << ",";
        out << v[i];
    }
    out << "]";
}

void writeAlgo(ofstream &out, const string &name, const Result &r, bool last) {
    out << "    \"" << name << "\": {\n      \"order\": ";
    writeArray(out, r.order);
    out << ",\n      \"path\": ";
    writeArray(out, r.path);
    out << ",\n      \"explored\": " << r.order.size()
        << ",\n      \"pathLength\": " << r.path.size()
        << ",\n      \"micros\": " << r.micros << "\n    }" << (last ? "\n" : ",\n");
}

int main(int argc, char *argv[]) {
    int size = (argc > 1) ? atoi(argv[1]) : 21;
    unsigned seed = (argc > 2) ? (unsigned)atoi(argv[2]) : (unsigned)time(nullptr);
    if (size < 5) size = 5;
    if (size > 101) size = 101;

    Maze m = generateMaze(size, seed);

    Result bfs = solveBFS(m);
    Result dfs = solveDFS(m);
    Result astar = solveAStar(m);

    int openCells = 0;
    for (int x : m.grid) if (x == 0) openCells++;

    ofstream out("maze_data.js");
    out << "const DATA = {\n";
    out << "  \"rows\": " << m.H << ", \"cols\": " << m.W << ",\n";
    out << "  \"start\": " << m.start << ", \"end\": " << m.end << ",\n";
    out << "  \"openCells\": " << openCells << ",\n";
    out << "  \"grid\": ";
    writeArray(out, m.grid);
    out << ",\n  \"algos\": {\n";
    writeAlgo(out, "BFS", bfs, false);
    writeAlgo(out, "DFS", dfs, false);
    writeAlgo(out, "A*", astar, true);
    out << "  }\n};\n";
    out.close();

    cout << "Maze " << m.H << "x" << m.W << " (seed " << seed << "), open cells: " << openCells << "\n";
    cout << "BFS : explored " << bfs.order.size() << ", path " << bfs.path.size() << ", " << bfs.micros << " us\n";
    cout << "DFS : explored " << dfs.order.size() << ", path " << dfs.path.size() << ", " << dfs.micros << " us\n";
    cout << "A*  : explored " << astar.order.size() << ", path " << astar.path.size() << ", " << astar.micros << " us\n";
    cout << "Written maze_data.js -> now open viewer.html\n";
    return 0;
}
