#ifndef INTKD_TREE_H
#define INTKD_TREE_H
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
	KdTreeH(){};
	KdTreeH(vector <Point> *points_ptr, vector <int> *index_ptr, int max_leaf_size, int para_NN);
	~KdTreeH();	
	virtual void BuildKDTree() = 0;
	virtual void NearestKSearch(Point query, int knn, vector<NearestInfo> &k_elements) = 0; // pure virtual function for nearest neighbor
	virtual void BruteForceKSearch(vector<int> *ind, Point query, KnnQueue &k_priority_queue, int knn) = 0; // virtual function for brute force search
	void print_param();
	void print_pcount(); // print number of points in KD tree
	void print_bcount(); // print number of bounding boxes 
	void print_points(int i);
	void print_leaf_node_num();

protected:
	NodePtr root_node_;
	IndexPtr index_;
	PointsPtr points_;
    int dim_;
    int max_leaf_size_;
    vector<NodePtr> leaf_nodes_;
    int para_NN_;
	int pcount = 0; // counting for points
	int bcount = 0; // counting for bounding box
	
	virtual NodePtr DivideTree(int left, int right, vector<Interval> *bbox_ptr) = 0;
	vector<NearestInfo> QueueCopy(KnnQueue &k_queue);
	int GetDistance(Point point, Point centroid);
	int Dist(Point p1, Point p2);
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
};

void load_bin(std::string infile, std::vector<Point> & points);

#endif