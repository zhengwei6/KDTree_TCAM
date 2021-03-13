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
#include "classic_kd_tree.h"
#include "tcam_m3_kd_tree.h"

using namespace std;
vector<Point> *randomPoints(int number, int min, int max)
{
	vector<Point> *points_reference_ptr = new vector<Point>;
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
		(*points_reference_ptr).push_back((*Ptmp));
	}
	return points_reference_ptr;
}

vector<Point> *testPoints(int max)
{
	vector<Point> *points_reference_ptr = new vector<Point>;
	struct Point *Ptmp;
	for(int i = 0 ; i < max ; i += 10) {
		Ptmp = new Point();
		(*Ptmp).x = 1;
		(*Ptmp).y = i;
		(*Ptmp).z = 1;
		(*points_reference_ptr).push_back((*Ptmp));
	}
	return points_reference_ptr;
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
	string dataset_dir = "./sample/";
	string data_reference = dataset_dir + "000000.bin";
	string data_query = dataset_dir + "000001.bin";

	vector<Point> points_reference;
	vector<Point> * points_reference_ptr = &points_reference;
	vector<Point> points_query;
	vector<Point> * points_query_ptr = &points_query;

	load_bin(data_reference, points_reference);
	cout << "Number of Points Loaded: " << points_reference.size() << std::endl;
	
	load_bin(data_query, points_query);
	cout << "Number of Points Loaded: " << points_query.size() << std::endl;

	int query_num = points_query.size();
	vector<vector<KdTreeH::NearestInfo> > k_kd_elements;
	vector<int> *Index = new vector<int>;
	for(int i = 0 ; i < points_reference.size() ; i++) {
	 	(*Index).push_back(i);
 	}
	 
	cout << "-----------------------------------------------------------------------" << endl;

	cout << "Brute Force Search" << endl;
	KdTreeH::KnnQueue kd_queue;
	KDTreeHClassic classic_kd_tree_b(points_reference_ptr, Index, 10, 10);
	c_start = clock();
	for(int i = 0 ; i < query_num ; i++) {
		classic_kd_tree_b.BruteForceKSearch(Index, points_query[i], kd_queue, 1);
		if(i == 0) {
			cout << "Ans: ";
			classic_kd_tree_b.print_points(kd_queue.top().first);
		}
	}

	c_end = clock();
	cout << "Complete" << endl;
	cout << "Brute force search time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	cout << "Ans: ";

	cout << "-----------------------------------------------------------------------" << endl;

	k_kd_elements.resize(query_num);
	int back_track = 0;
	cout << "Classic KD Tree Nearest Neighbor Search" << endl;
	KDTreeHClassic classic_kd_tree(points_reference_ptr, Index, 10, 10);
	c_start = clock();
	classic_kd_tree.BuildKDTree();
	c_end   = clock();
	cout << "Building Tree: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	classic_kd_tree.print_leaf_node_num();
	cout << "Classic KD Tree search...";
	c_start = clock();
	for(int i = 0 ; i < query_num ; i++) {
		classic_kd_tree.NearestKSearch(points_query[i], 1, k_kd_elements[i]);
	}
	c_end   = clock();
	cout << "Backtrack: " << classic_kd_tree.back_track << endl;
	cout << "Complete" << endl;
	cout << "Classic KD Tree search time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	cout << "Ans: ";
	classic_kd_tree.print_points(k_kd_elements[0][0].first);

	cout << "-----------------------------------------------------------------------" << endl;

	k_kd_elements.resize(query_num);
	cout << "KD Tree Nearest Neighbor Search with TCAM" << endl;
	KdTreeHTCAM_M3 tcam_kd_tree(points_reference_ptr, Index, 10, 10);
	c_start = clock();
	tcam_kd_tree.BuildKDTree();
	c_end   = clock();
	tcam_kd_tree.print_store_prefix_count();
	cout << "Building Tree: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";

	c_start = clock();
	for(int i = 0 ; i < query_num ; i++) {
		tcam_kd_tree.NearestKSearch(points_query[i], 1, k_kd_elements[i]);
	}
	c_end   = clock();
	tcam_kd_tree.print_search_prefix_conversion_time();
	tcam_kd_tree.print_search_prefix_count();
	cout << "total search time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	cout << "Ans: ";
	tcam_kd_tree.print_points(k_kd_elements[0][0].first);

	cout << "-----------------------------------------------------------------------" << endl;
	return 1;
}
