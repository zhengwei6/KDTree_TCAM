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
	std::vector< std::vector<int> > k_indices;
	std::vector< std::vector<int> > k_dists; 
	cout << "-----------------------------------------------------------------------" << endl;

	cout << "Brute Force Search" << endl;
	KdTreeH general_kd_tree1(points_reference_ptr, Index, 10, 10);
	k_indices.resize(query_num);
	k_dists.resize(query_num);
	c_start = clock();
	for(int i = 0 ; i < query_num ; i++) {
		general_kd_tree1.BruteForceKSearch(*Index, points_query[i], k_indices[i], k_indices[i]);
	}
	c_end = clock();
	cout << "Complete" << endl;
	cout << "Brute force search time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	cout << "Ans: ";
	general_kd_tree1.print_points(k_indices[0][0]);
	k_indices.clear();
	k_dists.clear();
	k_indices.resize(query_num);
	k_dists.resize(query_num);

	cout << "-----------------------------------------------------------------------" << endl;
	int back_track = 0;
	cout << "Classic KD Tree Nearest Neighbor Search" << endl;
	// build general kd tree
	// KD Tree constructor
	k_indices.resize(query_num);
	k_dists.resize(query_num);
	KdTreeH general_kd_tree(points_reference_ptr, Index, 10, 10);
	c_start = clock();
	general_kd_tree.BuildKDTreeV1();
	c_end   = clock();
	cout << "Building Tree: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	// // general kd tree search
	general_kd_tree.print_leaf_node_num(); 
	cout << "Classic KD Tree search...";

	c_start = clock();
	for(int i = 0 ; i < query_num ; i++) {
		back_track += general_kd_tree.NearestKSearch(points_query[i], 1, k_indices[i], k_dists[i]);
	}
	c_end   = clock();
	cout << "Backtrack: " << back_track << endl;
	cout << "Complete" << endl;
	cout << "Classic KD Tree search time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	cout << "Ans: ";
	general_kd_tree.print_points(k_indices[0][0]);
	k_indices.clear();
	k_dists.clear();
	back_track = 0;
	cout << "-----------------------------------------------------------------------" << endl;
	k_kd_elements.resize(query_num);
	k_indices.resize(query_num);
	k_dists.resize(query_num);
	cout << "KD Tree Nearest Neighbor Search with TCAM" << endl;
	KdTreeH kd_tree_tcam(points_reference_ptr, Index, 10, 10);
	// // build time
	c_start = clock();
	kd_tree_tcam.BuildKDTree();
	c_end   = clock();
	kd_tree_tcam.print_store_prefix_count();
	cout << "Building Tree: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	
	kd_tree_tcam.print_prefix_conversion_time();
	kd_tree_tcam.print_insert_prefix_time();
	
	c_start = clock();
	for(int i = 0 ; i < query_num ; i++) {
		kd_tree_tcam.NearestKSearchTCAM(points_query[i], 1, k_kd_elements[i]);
	}
	c_end   = clock();

	kd_tree_tcam.print_search_prefix_conversion_time();
	kd_tree_tcam.print_search_prefix_count();

	cout << "total search time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << " ms\n";
	cout << "Ans: ";
	kd_tree_tcam.print_points(k_kd_elements[0][0].first);
	cout << "-----------------------------------------------------------------------" << endl;
	return 1;
}
