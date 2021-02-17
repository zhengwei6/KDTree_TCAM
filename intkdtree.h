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

// #include <stdio.h>
// #include <sys/stat.h>
#include <ctime>
#include <cmath>
#include "prefix_tree.h"
#include "tools.h"

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

struct Node
{
    int s_dim; // split dim
    int s_val; // split value
    int leaf_idx; 
    Node *left_child;
    Node *right_child;
    std::vector<Interval> bbox;
    std::vector<int> index_list;
    
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
};

class KdTreeH
{
    
public:
    typedef std::vector<Point> Points;
    typedef std::vector<Point> *PointsPtr;
    typedef std::vector<int> *IndexPtr;
	typedef std::pair<int, int> NearestInfo;
	struct kComp {
		constexpr bool operator()( NearestInfo const& a, NearestInfo const& b) const noexcept 
    	{
        	return a.second > b.second; 
    	} 
	};
	typedef std::priority_queue<NearestInfo, std::vector<NearestInfo>, kComp> KnnQueue;
    typedef Node *NodePtr;

    KdTreeH(
        std::vector <Point> *points_ptr, \
        std::vector <int> *index_ptr, \
        int max_leaf_size, int para_NN):dim_(3), leaf_node_num_(0)
    {
        index_ = index_ptr;
        points_ = points_ptr;
        max_leaf_size_ = max_leaf_size;
        dim_ = (*points_ptr)[0].dim;
        para_NN_ = para_NN;
    }

    ~KdTreeH() {}
    void deleteNode(Node * node_p) {
        if (node_p->left_child) deleteNode(node_p->left_child);
        if (node_p->right_child) deleteNode(node_p->right_child);
        delete node_p;
    }
	
	void buildKDTree() 
	{
		std::vector<Interval> bbox;
		std::vector<Interval> *bbox_ptr = NULL;

		int left = 0;
		int right = (index_->size()) - 1;

		computeBoundingBox(left, right, bbox);
		bbox_ptr = &bbox;
		root_node_ = divideTree(left, right, bbox_ptr);
	}

    void print_param()
	{
		std::cout << "index size: " << (*index_).size() << std::endl;
		std::cout << "points number: " << (*points_).size() << std::endl;

	}

	void print_points(int i)
	{
		std::cout << "point x: " << (*points_)[i][0] << " point.y: " << (*points_)[i][1] << " point.z: " << (*points_)[i][2] << std::endl;
		return;
	}

	int nearestKSearchV2(Point query, int knn,\
		std::vector<NearestInfo> &k_elements
	)
	{
		int back_track = 0;
		std::vector<NodePtr> backtrack_stack;
		KnnQueue kd_queue;
		NodePtr cur_node = root_node_;
		cur_node = traverse_tree(cur_node, query, backtrack_stack);
		bruteForceKSearchV2(&cur_node->index_list, query, kd_queue, knn);
		for(;backtrack_stack.size();)
		{
			cur_node = backtrack_stack.back();
			backtrack_stack.pop_back();
			int dist_min_ = kd_queue.top().second;
			bool visit_ = check_intersection(query, dist_min_, cur_node);
			if(visit_)
			{
				back_track += 1;
				if (cur_node->s_dim != 1)
					cur_node = traverse_tree(cur_node, query, backtrack_stack);
				bruteForceKSearchV2(&cur_node->index_list, query, kd_queue, knn);
			}
		}
		k_elements = queueCopy(kd_queue);
		return back_track;
	}

	int nearestKSearch(Point query, int knn,\
		std::vector<int> &k_indices, \
		std::vector<int> &k_dists)
	{
		int back_track = 0;
		std::vector<NodePtr> backtrack_stack;
		
		NodePtr cur_node = root_node_;
		cur_node = traverse_tree(cur_node, query, backtrack_stack);

		bruteForceSearch(cur_node->index_list, query, k_indices, k_dists);
		for(;backtrack_stack.size();)
		{
			cur_node = backtrack_stack.back();
			backtrack_stack.pop_back();
			int dist_min_ = k_dists[0];
			bool visit_ = check_intersection(query, dist_min_, cur_node);
			if(visit_)
			{
				back_track += 1;
				if (cur_node->s_dim != 1)
					cur_node = traverse_tree(cur_node, query, backtrack_stack);
				bruteForceSearch(cur_node->index_list, query, k_indices, k_dists);
			}
		}
		return back_track;
	}

	void bruteForceKSearchV2(std::vector<int> *ind, Point query, \
		KnnQueue &k_priority_queue, int knn)
	{
		int dist_ = 0;
		for(int i = 0 ; i < (*ind).size() ; i++)
		{
			dist_ = dist(query, (*points_)[ (*ind)[i] ]);
			if(k_priority_queue.size() >= knn && dist_ < k_priority_queue.top().second) {
				k_priority_queue.pop();
				k_priority_queue.push(std::make_pair((*ind)[i], dist_));
			}
			else if(k_priority_queue.size() < knn) {
				k_priority_queue.push(std::make_pair((*ind)[i], dist_));
			}
		}
		return;
	}

	void bruteForceSearch(std::vector<int> ind, Point query, \
		std::vector<int> &k_indices, \
		std::vector<int> &k_dists)
	{
		int dist_ = 0;
		for (int i = 0; i < ind.size() ; i++) 
		{
			dist_ = dist(query, (*points_)[ ind[i] ]);
			if(k_dists.size())
			{
				if(dist_ < k_dists[0])
				{
					k_indices.pop_back();
					k_dists.pop_back();
					k_indices.push_back(ind[i]);
					k_dists.push_back(dist_);
				}
			}
			else
			{
				k_indices.push_back(ind[i]);
				k_dists.push_back(dist_);
			}
		}
	}

private:
    NodePtr root_node_;
    IndexPtr index_;
    PointsPtr points_;
    int dim_;
    int max_leaf_size_;
    int leaf_node_num_;
	PrefixTrie prefix_trie[3];

    std::vector<NodePtr> leaf_nodes_;
    int para_NN_;

	std::vector<NearestInfo> queueCopy(KnnQueue &k_queue)
	{
		std::vector<NearestInfo> copy;
		while(!k_queue.empty()) {
			copy.push_back(k_queue.top());
			k_queue.pop();
		}
		return copy;
	}

    int getDistance(Point point, Point centroid) {
        int distance = pow((centroid.x - point.x), 2) + pow((centroid.y - centroid.y), 2) + pow((centroid.z - centroid.z), 2);
        return distance;
    }

    int dist(Point p1, Point p2) {
        return sqrt(pow((p1.x - p2.x), 2)\
             + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
    }

    NodePtr divideTree(int left, int right, std::vector<Interval> *bbox_ptr) {
        NodePtr node = new Node();
		// save parent's indexs in index_list
        for(int i = left; i<= right; i++) {
            (node->index_list).push_back((*index_)[i]);
        }
		// parent decide bounding box for child
        node->bbox = *bbox_ptr;
        int count = right - left;
        if(count <= max_leaf_size_) {
            node->s_dim = -1;
			node->s_val = -1;
			node->leaf_idx = leaf_node_num_;
			leaf_node_num_ += 1;
			leaf_nodes_.push_back(node);
			//create tcam entries
			for(int i = 0 ; i < (*bbox_ptr).size() ; i++) {
				std::cout << "dimension: " << i << ""<< std::endl;
				std::cout << "interval: high " << (*bbox_ptr)[i].high << " low " << (*bbox_ptr)[i].low << std::endl;
			}
			std::cout << std::endl;
			return node;
        }
        else {
            int split_dim = 0;
            int span = 0;
            std::vector<int> value_list;
            findSplitDim(split_dim, span, bbox_ptr);
            node->s_dim = split_dim;
            int split_val = 0;
			// test
			// copy the value list range to value_list: left to right
            getValueList(left, right, split_dim, value_list);
            std::vector<int>* value_list_ptr = &value_list;
			qSelectMedian(value_list_ptr, split_val);
			node->s_val = split_val;
			//test
			//std::cout << "split_dim: " << split_dim << " split_val: "  << split_val << std::endl; 
			int lim1 = 0, lim2 = 0, split_delta = 0;
			// planeSplit: swap index
			planeSplit(left, right, split_dim, split_val, lim1, lim2);
			split_delta = (lim1 + lim2) / 2;
			std::vector<Interval> bbox_l;
			std::vector<Interval> bbox_r;
			computeBoundingBox(left, left + split_delta, bbox_l);
			computeBoundingBox(left + split_delta + 1, right, bbox_r);
			node->left_child = divideTree(left, left + split_delta, &bbox_l);
			node->right_child = divideTree(left + split_delta + 1, right, &bbox_r);
			return node;
        }
    }

    void findSplitDim(int &best_dim, int &span, \
            std::vector<Interval> *bbox_ptr) {
        // find most diverse dimension
        int cur_dim = 0;
        int min_ = 0, max_ = 0;
        for(; cur_dim < dim_; cur_dim++) {
            min_ = (*bbox_ptr)[cur_dim].low;
            max_ = (*bbox_ptr)[cur_dim].high;
            if((max_ - min_) > span) {
                best_dim = cur_dim;
                span = (max_ - min_);
            }
        }
    }

    void getValueList(int left, int right, int split_dim, std::vector<int> &value_list) {
        for(int i = left; i <= right ; i++)
            value_list.push_back((*points_)[(*index_)[i]][split_dim]);
    }
	
    void qSelectMedian(std::vector<int>* value_list, int &median_value) {
		int left   = 0;
		int right  = value_list->size() - 1;
		int middle = value_list->size() / 2;
		while(1) {
			int pivot = (*value_list)[middle];
			swap(value_list, middle, right);
			int store = left;
			for (int i = left; i < right ; i++) {
				if((*value_list)[i] < pivot) {
					if(i != store)
						swap(value_list, i, store);
					store++;
				}
			}
			swap(value_list, store, right);
			if ((*value_list)[store] == (*value_list)[middle]) {
				median_value = (*value_list)[middle];
				break;
			}
			if (store > middle) right = store - 1;
			else left = store;
		}
	}
	
	void findMedian(int left, int right, int split_dim, int &median_value) {
		std::vector<int> dim_sorted;
		for (int i = left; i <= right; ++i) {
			dim_sorted.push_back((*points_)[(*index_)[i]][split_dim]);
		}
		std::sort(dim_sorted.begin(), dim_sorted.end());
		int median_pos = (right - left) / 2;
		median_value = dim_sorted[median_pos];
	}
	
	void planeSplit(int left, int right, int split_dim,
	int split_val, int &lim1, int& lim2) 
	{
		int start = 0;
		int end = right - left;
		for(;;) {
			while(start <= end && (*points_)[(*index_)[left + start]][split_dim] < split_val)
				++start;
			while(start <= end && (*points_)[(*index_)[left + end]][split_dim] >= split_val)
				--end;
			if (start > end) break;
			std::swap((*index_)[left + start], (*index_)[left + end]);++start; --end;
		}
		lim1 = start;
		end = right - left;
		for(;;) {
			while(start <= end && (*points_)[(*index_)[left + start]][split_dim] <= split_val)
				++start;
			while(start <= end && (*points_)[(*index_)[left + end]][split_dim] > split_val)
				--end;
			if (start > end) break;
			std::swap((*index_)[left + start], (*index_)[left + end]); ++start; --end;
		}
		lim2 = end;
	}

	inline void swap(std::vector<int>* value_list, int a, int b) {
		int tmpa = (*value_list)[a];
		int tmpb = (*value_list)[b];
		(*value_list)[a] = tmpb;
		(*value_list)[b] = tmpa;
	}
	
	void computeBoundingBox(int left, int right, \
	std::vector<Interval> &bbox) 
	{
		int cur_dim = 0;
		for(; cur_dim < dim_ ; cur_dim++) {
			Interval bounds;
			computeMinMax(left, right, cur_dim, bounds.low, bounds.high);
			// test
			//std::cout << "bounds.low: " << bounds.low << " bounds.high :" << bounds.high << std::endl; 			
			bbox.push_back(bounds);
		}
		//test
		//std::cout << std::endl;
	}
	
	void computeMinMax(int left, int right, int dim, int& min_val, int& max_val)
	{
		min_val = (*points_)[(*index_)[left]][dim];
		max_val = (*points_)[(*index_)[left]][dim];
		for (int i = left + 1; i <= right; ++i) {
			int val = (*points_)[(*index_)[i]][dim];
			if (val < min_val) min_val = val;
			if (val > max_val) max_val = val;
		}
	}

	/*traverse tree */
	NodePtr traverse_tree(NodePtr cur_node, Point query, \
		std::vector<Node *> & backtrack_stack)
	{
		while(cur_node->s_dim != -1) {
			int s_dim = cur_node->s_dim;
			int s_val = cur_node->s_val;
			
			if(query[s_dim] > s_val) {
				backtrack_stack.push_back(cur_node->left_child);
				cur_node = cur_node->right_child;
			}
			else {
				backtrack_stack.push_back(cur_node->right_child);
				cur_node = cur_node->left_child;
			}
		}
		return cur_node;
	}
	
	bool check_intersection(Point query, int radius, NodePtr cur_node)
	{
		int dist_squared = radius * radius;
		
		int x_min = (cur_node->bbox)[0].low;
		int x_max = (cur_node->bbox)[0].high;
		
		int y_min = (cur_node->bbox)[1].low;
		int y_max = (cur_node->bbox)[1].high;
		
		int z_min = (cur_node->bbox)[2].low;
		int z_max = (cur_node->bbox)[2].high;
		
		if (query.x < x_min)
			dist_squared -= (query.x - x_min) * (query.x - x_min);
		else if (query.x > x_max)
			dist_squared -= (query.x - x_max) * (query.x - x_max);

		if (query.y < y_min)
			dist_squared -= (query.y - y_min) * (query.y - y_min);
		else if (query.y > y_max)
			dist_squared -= (query.y - y_max) * (query.y - y_max);

		if (query.z < z_min)
			dist_squared -= (query.z - z_min) * (query.z - z_min);
		else if (query.z > z_max)
			dist_squared -= (query.z - z_max) * (query.z - z_max);
		
		return dist_squared > 0;
	}

	void InsertPrefixTrieDim(int dim, int num, int prefix_num, int leaf_index) {
		prefix_trie[dim].InsertPrefixTrieV2(int num, int prefix_num, int leaf_index);
		return;
	}
	
};
