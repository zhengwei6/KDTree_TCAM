#ifndef INITKD_TREE_H
#define INITKD_TREE_H
#include <limits>
#include <fstream>
#include <iostream>
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
#include "prefix_tree.h"
#include "tools.h"

//#define DEBUG

using namespace std;
struct Interval {
  int low, high;  
};

struct Point {
public:
    int x;
    int y;
    int z;
    int dim;
    Point() :\
    x(0), y(0), z(0), dim(3) {}
    int operator[] (int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        if (i == 2) return z;
        return z;
    }
};

struct Point_f {
public:
	float x;
	float y;
	float z;

	int dim; // dimension

	// to-do: add normal information
	Point_f () : \
	x(.0), y(.0), z(.0), dim(3) {}
public:
	float operator[] (int i) {
		if (i == 0) return x;
		if (i == 1) return y;
		if (i == 2) return z;

		// Default: return z value 
		return z;
	}
};

struct Node
{
    int s_dim; // split dim
    int s_val; // split value
    int leaf_idx; 
    Node *left_child;
    Node *right_child;
    vector<Interval> bbox;
    vector<int> index_list;
    
    Node () :\
    s_dim(-1), s_val(0), leaf_idx(-1), \
    left_child(NULL), right_child(NULL)
    {
        bbox.resize(0);
        index_list.resize(0);
    }
    ~Node()
    {
        bbox.clear();
    }
	void deleteNode(Node * node_p);
};

class KdTreeH
{
    
public:
    typedef vector<Point> Points;
    typedef vector<Point> *PointsPtr;
    typedef vector<int> *IndexPtr;
	typedef pair<int, int> NearestInfo;
	struct kComp {
		constexpr bool operator()( NearestInfo const& a, NearestInfo const& b) const noexcept 
    	{
        	return a.second > b.second; 
    	} 
	};
	typedef priority_queue<NearestInfo, vector<NearestInfo>, kComp> KnnQueue;
    typedef Node *NodePtr;
	
	KdTreeH(vector <Point> *points_ptr,vector <int> *index_ptr,int max_leaf_size, int para_NN);
	~KdTreeH();	
	void BuildKDTree();
	void BuildKDTreeV1();
	void print_param();
	void print_pcount();
	void print_bcount();
	void print_points(int i);
	void print_leaf_node_num();
	void print_prefix_conversion_time();
	void print_insert_prefix_time();
	void print_search_prefix_conversion_time();
	void print_search_prefix_count();
	void print_store_prefix_count();
	void NearestKSearchTCAM(Point query, int knn, vector<NearestInfo> &k_elements);
	int  NearestKSearch(Point query, int knn,vector<int> &k_indices, vector<int> &k_Dists);
	void BruteForceKSearchV2(vector<int> *ind, Point query, KnnQueue &k_priority_queue, int knn);	
	void BruteForceKSearch(vector<int> ind, Point query, vector<int> &k_indices, vector<int> &k_Dists);

private:
	NodePtr root_node_;
	IndexPtr index_;
    PointsPtr points_;
    int dim_;
    int max_leaf_size_;
	vector<PrefixTrie *> prefix_tries_;
	PrefixTrie prefix_trie_dim1;
	PrefixTrie prefix_trie_dim2;
	PrefixTrie prefix_trie_dim3;
    vector<NodePtr> leaf_nodes_;
    int para_NN_;
	int pcount = 0; // counting for points
	int bcount = 0; // counting for bounding box
	long prefix_conversion_time = 0;
	long insert_prefix_time     = 0;
	long search_prefix_conversion_time = 0;
	int search_prefix_count = 0;
	int store_prefix_count = 0;

	vector<NearestInfo> QueueCopy(KnnQueue &k_queue);
	int GetDistance(Point point, Point centroid);
	int Dist(Point p1, Point p2);
	NodePtr DivideTreeV1(int left, int right, vector<Interval> *bbox_ptr);
	NodePtr DivideTree(int left, int right, vector<Interval> *bbox_ptr);
	void FindSplitDim(int &best_dim, int &span, vector<Interval> *bbox_ptr);
    void GetValueList(int left, int right, int split_dim, vector<int> &value_list);
	void QSelectMedian(vector<int>* value_list, int &median_value);
	void FindMedian(int left, int right, int split_dim, int &median_value);
	void PlaneSplit(int left, int right, int split_dim, int split_val, int &lim1, int& lim2);
	inline void swap(vector<int>* value_list, int a, int b);
	void ComputeBoundingBox(int left, int right, vector<Interval> &bbox); 
	void ComputeMinMax(int left, int right, int dim, int& min_val, int& max_val);
	NodePtr TraverseTree(NodePtr cur_node, Point query, vector<Node *> & backtrack_stack);
	bool CheckIntersection(Point query, int radius, NodePtr cur_node);
	void InsertPrefixTrieDim(int dim, int num, int prefix_num, int leaf_index);
};
void load_bin(std::string infile, std::vector<Point> & points);
#endif