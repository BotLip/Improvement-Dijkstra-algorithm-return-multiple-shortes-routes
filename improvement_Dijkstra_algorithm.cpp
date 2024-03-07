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
	int st = 0;
	vector<int> S;		//S{idx};
	vector<int> D(n + 1, INF);

	S.push_back(st);
	D[0] = 0;
	map<int, set<int>> P;	// P[now]{best fathers};
	P[0].insert(-1);

	set<tiii> expandCost_Node_Father;	// eCF{cost, next, father};
	int nownodeidx = 0;
	while (S.size() < n + 1) {
		for (int i = 0; i < n + 1; ++i) {
			if (count(S.begin(), S.end(), i)); //expandCost_Node_Father.insert({ INF, i, S[nownodeidx]});
			else expandCost_Node_Father.insert({ D[nownodeidx] + A[S[nownodeidx]][i], i, S[nownodeidx]});
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
		S.push_back(nj);
		nownodeidx++;
	}


	cout << D[n] << endl;
	/* DFS for the best routes*/
	vector<int> path;
	function<void(int)> dfs = [&](int now) {
		if (now == 0) {
			for (int x : path) cout << x << "<-";
			cout << now <<  endl;
			return;
		}
		for (int fa : P[now]) {
			path.push_back(now);
			dfs(fa);
			path.pop_back();
		}
		
	};
	dfs(n);
}



int main() {
	// start : 0, end : 4
	int n = 4;
	vector<vector<int>> e{ {0, 1, 2}, {0, 2, 2}, {1, 3, 1}, {2, 4, 2}, {2, 3, 1}, {1, 4, 2}, {3, 4, 1} };
	//Dijkstra(e, n);
	improvDijkstra(e, n);
	return 0;
}