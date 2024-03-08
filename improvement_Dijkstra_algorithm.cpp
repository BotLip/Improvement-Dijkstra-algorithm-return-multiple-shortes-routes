#include <map>
#include <set>
#include <functional>
#include <iostream>
#include <vector>
#include <tuple>

#define tiii std::tuple<int, int, int>	// c, next, father
#define INF 1e9
using namespace std;

void buildGraph(vector<vector<int>> edges, map<int, vector<pair<int, int>>> &A) {
	for (auto& e : edges) {
		int u = e[0], v = e[1], w = e[2];
		A[u].push_back({v, w});
		A[v].push_back({ u, w });
	}
	return;
}

/* original Dijkstra */
void Dijkstra(vector<vector<int>>& edges, int n) {
	map<int, vector<pair<int, int>>> A;
	buildGraph(edges, A);
	int st = 0, e = n;
	set<int> closeS;
	set<tuple<int, int, int>> S;	// S{cost, idx};
	vector<int> D(n + 1, INF);
	S.insert({0, st, -1});
	D[0] = 0;

	map<int, vector<int>> P;
	while (!S.empty()) {
		tiii t = *S.cbegin();
		int now = get<1>(t);
		int nowcost = get<0>(t);
		S.erase(S.cbegin());
		closeS.insert(now);

		if (nowcost < D[now]) {
			D[now] = nowcost;
			P[now].clear();
			P[now].push_back(get<2>(t));
		}
		else if (nowcost == D[now]) {
			P[now].push_back(get<2>(t));
		}

		for (auto& node : A[now]) {
			int next = node.first, nextcost = node.second;
			if (!closeS.count(next)) S.insert({nextcost + nowcost, next, now});
		}
	}

	cout << D[n] << endl;
	int fa = P[n][0];
	cout << n << ' ';
	while (fa != -1) {
		cout << fa << ' ';
		fa = P[fa][0];
	}
}

void buildGraph(vector<vector<int>> edges, vector<vector<int>>& A) {
	for (auto& e : edges) {
		int u = e[0], v = e[1], w = e[2];
		A[u][v] = w;
		A[v][u] = w;
	}
}

/* Dijkstra with multi-best path*/
void improvDijkstra(vector<vector<int>>& edges, int n) {
	vector<vector<int>> A(n + 1, vector<int>(n + 1, INF));
	buildGraph(edges, A);
	/* temp arg : st */
	int st = 1;
	vector<int> S;		//S{idx};
	vector<int> D(n + 1, INF);

	S.push_back(st);
	D[st] = 0;
	map<int, set<int>> P;	// P[now]{best fathers};
	P[st].insert(-1);

	set<tiii> expandCost_Node_Father;	// eCF{cost, next, father};
	//int nownodeidx = 0;	// S.back() 
	while (S.size() < n) {
		int nownode = S.back();
		for (int i = 1; i < n + 1; ++i) {
			if (count(S.begin(), S.end(), i)); //expandCost_Node_Father.insert({ INF, i, S[nownodeidx]});
			//else expandCost_Node_Father.insert({ D[S[nownodeidx]] + A[S[nownodeidx]][i], i, S[nownodeidx]});
			else expandCost_Node_Father.insert({ D[nownode] + A[nownode][i], i, nownode });
		}
		for (auto &it : expandCost_Node_Father) {
			int cost = get<0>(it), next = get<1>(it), father = get<2>(it);
			if (cost < D[next]) {
				D[next] = cost;
				P[next].clear();
				P[next].insert(father);
			}
			else if (cost == D[next]) {
				P[next].insert(father);
			}
		}

		int nj = get<1>(*expandCost_Node_Father.begin());
		expandCost_Node_Father.erase(expandCost_Node_Father.cbegin());
		if (!count(S.begin(), S.end(), nj)) S.push_back(nj);
		//nownodeidx++;
	}

	/* temp arg target */
	int tar = 6;
	cout << D[tar] << endl;
	/* DFS for the best routes*/
	vector<int> path;
	function<void(int)> dfs = [&](int now) {
		if (now == st) {
			for (int x : path) cout << x << "<-";
			cout << now <<  endl;
			return;
		}
		path.push_back(now);
		for (int fa : P[now]) {
			dfs(fa);
		}
		path.pop_back();
	};
	dfs(tar);
}

int main() {
	// start : 0, end : 4
	int n = 8;
	// {{0, 1, 3}, {0, 2, 2}, {1, 3, 1}, {2, 4, 2}, {2, 3, 1}, {1, 4, 2}, {3, 4, 1} };
	 //vector<vector<int>> e{ {0, 1, 3}, {0, 2, 1}, {1, 3, 1}, {2, 4, 200}, {2, 3, 1}, {1, 4, 2}, {3, 4, 3} };
	//vector<vector<int>> e{ {0, 1, 2}, {0, 2, 2}, {1, 3, 1}, {2, 4, 2}, {2, 3, 1}, {1, 4, 2}, {3, 4, 1} };
	//Dijkstra(e, n);
	vector<vector<int>> e{ {1, 2, 20}, {1, 3, 30}, {1, 4, 20}, {2, 3, 20}, {2, 5, 30}, {3, 4, 10}, {3, 6, 40},
		{4, 7, 30}, {5, 6, 10}, {5, 8, 20}, {6, 7, 10}, {6, 8, 20}, {7, 8, 50}};
	improvDijkstra(e, n);
	return 0;
}
