#include <iostream>
#include <limits>
#include <fstream>
#include <string>

#include <unordered_map>
#include <vector>
#include <queue>
#include <deque>
#include <set>
#include <algorithm>

#include <iomanip>
#include <ctime>
#include <cmath>
#include "intkd_tree.h"

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

int main() {
	
	clock_t c_start;
	clock_t c_end;
	cout << "Generate " << 1000000 << " data points" << endl;
	vector<Point> *Ps = randomPoints(1000000, 0, 65535);
	cout << "The range of the data point values: " << 0 << " to " << 65535 << endl;

	//vector<Point> *Ps = testPoints(1000);
	// genrerate point 1 10 1, 1 20 1, 1 30 1
	vector<KdTreeH::NearestInfo> k_kd_elements;
	vector<int> *Index = new vector<int>;
	for(int i = 0 ; i < 1000000 ; i++) {
	 	(*Index).push_back(i);
 	}
	
	// kd tree constructor
	// KdTreeH kd_tree(Ps, Index, 10, 10);
	// create TCAM entries
	Point query = randomPoint(0, 65535);
	cout << "Generate query points: query.x: " << query.x << " query.y: " << query.y << " query.z: " << query.z << endl; 
	cout << "-----------------------------------------------------------------------" << endl;
	KdTreeH general_kd_tree1(Ps, Index, 10, 10);
	cout << "Brute Force Search..." << flush;
	c_start = clock();
	KdTreeH::KnnQueue k_brute_queue;
	general_kd_tree1.BruteForceKSearchV2(Index, query, k_brute_queue, 1);



	// brute force search
	//cout << "Brute Force Search..." << flush;
	cout << "-----------------------------------------------------------------------" << endl;
	// build general kd tree
	// KD Tree constructor
	KdTreeH general_kd_tree(Ps, Index, 10, 10);
	c_start = clock();
	general_kd_tree.BuildKDTreeV1();
	c_end   = clock();
	cout << "Build kd tree time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	// general kd tree search
	vector<int> k_indices;
	vector<int> k_Dists;
	cout << "Classic KD Tree search:" << endl << endl;

	c_start = clock();
	general_kd_tree.NearestKSearch(query, 1, k_indices, k_Dists);
	c_end   = clock();

	cout << "total search time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	cout << "-----------------------------------------------------------------------" << endl;
	


	// // brute force search
	// cout << "Brute Force Search..." << flush;
	// c_start = clock();
	// KdTreeH::KnnQueue k_brute_queue;
	// kd_tree.BruteForceKSearchV2(Index, query, k_brute_queue, 1);
	// cout << "complete" << endl;
	// cout << "index: " << k_brute_queue.top().first << " dist: " << k_brute_queue.top().second << " ";
	// kd_tree.print_points(k_brute_queue.top().first);
	// c_end = clock();
	// cout << "time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	
	// cout << "Build KD Tree..." << flush;
	// c_start = clock();
	// kd_tree.BuildKDTree();
	// c_end = clock();
	// cout << "complete" << endl;
	// cout << "time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";

	// //build time
	
	// #ifdef DEBUG
	// kd_tree.print_pcount();
	// kd_tree.print_bcount();
	// #endif
	// // brute f
	// // print query point
	// // general kd tree search

	// #ifdef DEBUG
	// cout << "--------------KD Tree Nearest Neighbor Search -------------" << endl;
	// #endif

	// kd_tree.NearestKSearchTCAM(query, 1, k_kd_elements);
	
	// #ifdef DEBUG
	// cout << "--------Complete KD Tree Nearest Neighbor Search----------" << endl << endl;
	// #endif
	
	// #ifdef DEBUG
	// cout << "------Brute Force KD Tree Nearest Neighbor Search---------" << endl;
	// #endif
	// // KdTreeH::KnnQueue k_brute_queue;
	// // kd_tree.bruteForceKSearchV2(Index, query, k_brute_queue, 1);
	// // cout << "index: " << k_brute_queue.top().first << " dist: " << k_brute_queue.top().second << " ";
	// // kd_tree.print_points(k_brute_queue.top().first);
	
	// #ifdef DEBUG
	// cout << "-----End of Brute Force Search----------------------------" << endl << endl;
	// #endif

	// #ifdef DEBUG
	// cout << "--------------Nearest Data Point Info----------------------" << endl; 
	// cout << "query: " << "query.x: " << query.x << " query.y: " << query.y << " query.z: " << query.z << endl;
	// cout << "index: " << k_kd_elements[0].first << " dist: " << k_kd_elements[0].second << " ";
	// kd_tree.print_points(k_kd_elements[0].first);
	// cout << "----------End of Nearest Data Point Info-------------------" << endl << endl;
	// #endif

	//brute force search


	//kd_tree.NearestKSearchTCAM(query, 10, k_kd_elements);

	// for(int k = 0 ; k < 10; k++) {	
	// 	vector<Point> *Ps = randomPoints(100000, 0, 65535);


	// 	KdTreeH::KnnQueue k_brute_queue;
	// 	vector<KdTreeH::NearestInfo> k_kd_elements;
	// 	vector<int> *Index = new vector<int>;
	// 	for(int i = 0 ; i < 100000 ; i++) {
	// 		(*Index).push_back(i);
	// 	}
	// 	KdTreeH kd_tree(Ps, Index, 10, 10);
	// 	kd_tree.BuildKDTree();
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
	// 	kd_tree.NearestKSearchTCAM(query, 10, k_kd_elements);
	// 	c_end = clock();
	// 	cout << "kd tree search" << endl;
	// 	cout << "index: " << k_kd_elements[0].first << " dist: " << k_kd_elements[0].second << " ";
	// 	cout << "execution time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	// 	kd_tree.print_points(k_kd_elements[0].first);
	// 	cout << endl;
    // }
	return 1;
}
