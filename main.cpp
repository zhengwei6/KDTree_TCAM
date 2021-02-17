#include <bits/stdc++.h>
#include "intkdtree.h"

using namespace std;
vector<Point> *randomPoints(int number, int min, int max)
{
	vector<Point> *Ps = new vector<Point>;
	struct Point *Ptmp;
	random_device rd;
	default_random_engine eng(rd());
	uniform_real_distribution<> distr(min, max);
	for(int i = 0 ; i < number ; i++) {
		// random three numbers
		Ptmp = new Point();
		(*Ptmp).x = distr(eng);
		(*Ptmp).y = distr(eng);
		(*Ptmp).z = distr(eng);
		(*Ps).push_back((*Ptmp));
	}
	return Ps;
}

vector<Point> *testPoints(int max)
{
	vector<Point> *Ps = new vector<Point>;
	struct Point *Ptmp;
	for(int i = 0 ; i < max ; i += 10) {
		Ptmp = new Point();
		(*Ptmp).x = 1;
		(*Ptmp).y = i;
		(*Ptmp).z = 1;
		(*Ps).push_back((*Ptmp));
	}
	return Ps;
}

Point randomPoint(int min, int max)
{
	struct Point Ptmp;
	random_device rd;
	default_random_engine eng(rd());
	uniform_real_distribution<> distr(min, max);
	Ptmp.x = distr(eng);
	Ptmp.y = distr(eng);
	Ptmp.z = distr(eng);
	return Ptmp;
}

/*
vector<pair<int, float>> queueCopy(KdTreeH::KnnQueue k_queue)
{
	vector<pair<int, float>> copy;
	while(!k_queue.empty()) {
		copy.push_back(k_queue.front());
		k_queue.pop();
	}
	return copy;
}
*/
int main() {

	vector<Point> *Ps = testPoints(1000);
	vector<KdTreeH::NearestInfo> k_kd_elements;
	vector<int> *Index = new vector<int>;
	for(int i = 0 ; i < 100 ; i++) {
	 	(*Index).push_back(i);
 	}
	KdTreeH kd_tree(Ps, Index, 10, 10);
	kd_tree.buildKDTree();
	
	//kd_tree.nearestKSearchV2(query, 10, k_kd_elements);

	// for(int k = 0 ; k < 10; k++) {	
	// 	vector<Point> *Ps = randomPoints(100000, 0, 65535);


	// 	KdTreeH::KnnQueue k_brute_queue;
	// 	vector<KdTreeH::NearestInfo> k_kd_elements;
	// 	vector<int> *Index = new vector<int>;
	// 	for(int i = 0 ; i < 100000 ; i++) {
	// 		(*Index).push_back(i);
	// 	}
	// 	KdTreeH kd_tree(Ps, Index, 10, 10);
	// 	kd_tree.buildKDTree();
	// 	struct Point query = randomPoint(0, 65535);
	// 	clock_t c_start = clock();
	// 	kd_tree.bruteForceKSearchV2(Index, query, k_brute_queue, 1);
	// 	clock_t c_end = clock();
	// 	cout << "brute force search" << endl;
	// 	cout << "random query point: " << "query point.x: " << query.x << " query point.y: " << query.y << " query point.z: " << query.z << endl;
	// 	cout << "index: " << k_brute_queue.top().first << " dist: " << k_brute_queue.top().second << " ";
	// 	cout << "execution time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	// 	kd_tree.print_points(k_brute_queue.top().first);

	// 	c_start = clock();
	// 	kd_tree.nearestKSearchV2(query, 10, k_kd_elements);
	// 	c_end = clock();
	// 	cout << "kd tree search" << endl;
	// 	cout << "index: " << k_kd_elements[0].first << " dist: " << k_kd_elements[0].second << " ";
	// 	cout << "execution time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	// 	kd_tree.print_points(k_kd_elements[0].first);
	// 	cout << endl;
    // }
	return 1;
}
