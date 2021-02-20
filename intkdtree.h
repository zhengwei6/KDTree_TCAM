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
        int max_leaf_size, int para_NN):dim_(3), prefix_trie_dim1(), prefix_trie_dim2(), prefix_trie_dim3()
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

	void print_pcount()
	{
		cout << "number of points in kd tree: " << pcount << endl; 
	}

	void print_bcount()
	{
		cout << "number of bounding boxes: " << bcount << endl;	
	}

	void print_points(int i)
	{
		std::cout << "point x: " << (*points_)[i][0] << " point.y: " << (*points_)[i][1] << " point.z: " << (*points_)[i][2] << std::endl;
		return;
	}

	// int nearestKSearchV2(Point query, int knn,\
	// 	std::vector<NearestInfo> &k_elements
	// )
	// {
	// 	int back_track = 0;
	// 	std::vector<NodePtr> backtrack_stack;
	// 	KnnQueue kd_queue;
	// 	NodePtr cur_node = root_node_;
	// 	cur_node = traverseTree(cur_node, query, backtrack_stack);
		// bruteForceKSearchV2(&cur_node->index_list, query, kd_queue, knn);
		// for(;backtrack_stack.size();)
		// {
		// 	cur_node = backtrack_stack.back();
		// 	backtrack_stack.pop_back();
		// 	int dist_min_ = kd_queue.top().second;
		// 	bool visit_ = check_intersection(query, dist_min_, cur_node);
		// 	if(visit_)
		// 	{
		// 		back_track += 1;
		// 		if (cur_node->s_dim != 1)
		// 			cur_node = traverseTree(cur_node, query, backtrack_stack);
		// 		bruteForceKSearchV2(&cur_node->index_list, query, kd_queue, knn);
		// 	}
		// }
		// k_elements = queueCopy(kd_queue);
		// return back_track;
	// }

	void nearestKSearchV2(Point query, int knn, vector<NearestInfo> &k_elements)
	{
		KnnQueue kd_queue;
		NodePtr cur_node = root_node_;
		vector<NodePtr> backtrack_stack;
		cur_node = traverseTree(cur_node, query, backtrack_stack);
		#ifdef DEBUG
		cout << "interval of first bounding box: " << endl;
		if(cur_node == NULL) {
			cout << "123" << endl;
		}
		for(int i = 0 ; i < cur_node->bbox.size() ; i++) {
			cout << "dimension: " << i << " low " << cur_node->bbox[i].low << " high " << cur_node->bbox[i].high << endl; 
		}
		
		cout << "print all the points in bounding box: " << endl;
		for(const auto &k : cur_node->index_list) {
			print_points(k);
		}
		#endif

		bruteForceKSearchV2(&cur_node->index_list, query, kd_queue, knn);
		
		#ifdef DEBUG
		//print nearest data point info
		cout << "the nearest distance: ";
		cout << kd_queue.top().second << endl;
		cout << "the nearest data point: ";
		print_points(kd_queue.top().first);
		#endif
		
		int dist_min_ = kd_queue.top().second;
		// create range
		pair<int, int> x_range;
		pair<int, int> y_range;
		pair<int, int> z_range;
		vector<int> res_index_x;
		vector<int> res_index_y;
		vector<int> res_index_z;
		vector<pair<int, int>> prefix_range_x;
		vector<pair<int, int>> prefix_range_y;
		vector<pair<int, int>> prefix_range_z;
		x_range.first  = (query.x + dist_min_) > 0 ? (query.x + dist_min_) : 0;
		x_range.second = (query.x - dist_min_) > 0 ? (query.x - dist_min_) : 0;
		y_range.first  = (query.y + dist_min_) > 0 ? (query.y + dist_min_) : 0;
		y_range.second = (query.y - dist_min_) > 0 ? (query.y - dist_min_) : 0;
		z_range.first  = (query.z + dist_min_) > 0 ? (query.z + dist_min_) : 0;
		z_range.second  = (query.z - dist_min_) > 0 ? (query.z - dist_min_) : 0;

		// direction conversion
		DirectConversion(x_range.second, x_range.first, prefix_range_x);
		DirectConversion(y_range.second, y_range.first, prefix_range_y);
		DirectConversion(z_range.second, z_range.first, prefix_range_z);
		// dim1 search dim2 search dim3 search
		prefix_trie_dim1.SearchRangeV2(prefix_range_x, res_index_x);
		prefix_trie_dim2.SearchRangeV2(prefix_range_y, res_index_y);
		prefix_trie_dim3.SearchRangeV2(prefix_range_z, res_index_z);
		//print search dimension info

		//intersection
		vector<int> tmp  = {};
		vector<int> tmp2 = {};
		set_intersection(res_index_x.begin(), res_index_x.end(), res_index_y.begin(), res_index_y.end(), inserter(tmp, tmp.begin()));
		set_intersection(tmp.begin(), tmp.end(), res_index_z.begin(), res_index_z.end(), inserter(tmp2, tmp2.begin()));

		for(int i = 0 ; i < tmp2.size() ; i++) {
			NodePtr search_node = leaf_nodes_[tmp2[i]];
			bruteForceKSearchV2(&search_node->index_list, query, kd_queue, knn);
		}
		k_elements = queueCopy(kd_queue);
		return;
	}

	int nearestKSearch(Point query, int knn,\
		std::vector<int> &k_indices, \
		std::vector<int> &k_dists)
	{
		int back_track = 0;
		std::vector<NodePtr> backtrack_stack;
		
		NodePtr cur_node = root_node_;
		cur_node = traverseTree(cur_node, query, backtrack_stack);

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
					cur_node = traverseTree(cur_node, query, backtrack_stack);
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
	PrefixTrie prefix_trie_dim1;
	PrefixTrie prefix_trie_dim2;
	PrefixTrie prefix_trie_dim3;
    std::vector<NodePtr> leaf_nodes_;
    int para_NN_;
	int pcount = 0; // counting for points
	int bcount = 0; // counting for bounding box

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
			leaf_nodes_.push_back(node);
			node->leaf_idx = leaf_nodes_.size() - 1;
			bcount += 1; // increase number of count for bounding box
			//create tcam entries
			for(int i = 0 ; i < (*bbox_ptr).size() ; i++) {
				// insert the entries into prefix trie
				// create index number and push the leaf node into vector
				vector<pair<int, int>> prefix_range;
				
				#ifdef DEBUG
				std::cout << "dimension: " << i << ""<< std::endl;
				cout << "interval: low " <<  (*bbox_ptr)[i].low << " interval: high " << (*bbox_ptr)[i].high << endl;
				#endif
				
				DirectConversion((*bbox_ptr)[i].low, (*bbox_ptr)[i].high, prefix_range);
				// PrintStack(prefix_range);
				for (int j = 0 ; j < prefix_range.size() ; j++) {
					InsertPrefixTrieDim(i, prefix_range[j].first << prefix_range[j].second , prefix_range[j].second, node->leaf_idx);		
				}
			}
			for(const auto &k : node->index_list) {
				
				#ifdef DEBUG
				print_points(k);
				#endif

				pcount += 1;
			}
			
			#ifdef DEBUG
			cout << endl;
			#endif

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

			#ifdef DEBUG
			//test
			//std::cout << "split_dim: " << split_dim << " split_val: "  << split_val << std::endl; 
			#endif

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
			bbox.push_back(bounds);
		}
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
	NodePtr traverseTree(NodePtr cur_node, Point query, \
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

	// std::vector<int> SearchBoundingBox(int dim, vector<pair<int, int>> &search_ranges)
	// {
	// 	// conversion
	// 	std::vector<int> res_index = {};
	// 	if(dim == 0) {
	// 		cout << "pppp" << endl;
	// 		cout << search_ranges.size() << endl;
	// 		prefix_trie_dim1.SearchRangeV2(search_ranges, res_index);
	// 	}
	// 	else if(dim == 1) {
	// 		cout << "gggg" << endl;
	// 		cout << search_ranges.size() << endl;
	// 		prefix_trie_dim2.SearchRangeV2(search_ranges, res_index);
	// 	}
	// 	else if(dim == 2) {
	// 		cout << "tttt" << endl;
	// 		cout << search_ranges.size() << endl;
	// 		prefix_trie_dim3.SearchRangeV2(search_ranges, res_index);
	// 	}
	// 	return res_index;
	// }


	// NodePtr traverseTreeV2(Point query)
	// {
	// 	vector<pair<int, int>> search_range_dim1_v;
	// 	vector<pair<int, int>> search_range_dim2_v;
	// 	vector<pair<int, int>> search_range_dim3_v;
	// 	DirectConversion(query.x, query.x, search_range_dim1_v);
	// 	DirectConversion(query.y, query.y, search_range_dim2_v);
	// 	DirectConversion(query.z, query.z, search_range_dim3_v);
	// 	cout << query.x << " " << query.y << " " << query.z << endl;
	// 	cout << search_range_dim1_v.size() << endl;
	// 	cout << search_range_dim2_v.size() << endl;
	// 	cout << search_range_dim3_v.size() << endl;
	// 	vector<int> res1 = SearchBoundingBox(0, search_range_dim1_v); //intersection
	// 	vector<int> res2 = SearchBoundingBox(1, search_range_dim2_v);
	// 	vector<int> res3 = SearchBoundingBox(2, search_range_dim3_v);
	// 	for(const auto &k : res1) {
	// 		cout << k << " ";
	// 	}
	// 	cout << endl;
	// 	for(const auto &k : res2) {
	// 		cout << k << " ";
	// 	}
	// 	cout << endl;
	// 	for(const auto &k : res3) {
	// 		cout << k << " ";
	// 	}
	// 	cout << endl;

	// 	vector<int> tmp = {};
	// 	vector<int> tmp2 = {};
		
	// 	set_intersection(res1.begin(), res1.end(), res2.begin(), res2.end(), inserter(tmp, tmp.begin()));
	// 	set_intersection(res3.begin(), res3.end(), tmp.begin(), tmp.end(), inserter(tmp2, tmp2.begin()));
		
	// 	for(const auto &k : tmp2) {
	// 		cout << k << " ";
	// 	}
	// 	cout << endl;

	// 	if (!tmp2.empty()) return leaf_nodes_[tmp2[0]];
	// 	return NULL;
	// }

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
		if(dim == 0) prefix_trie_dim1.InsertPrefixTrieV2(num, prefix_num, leaf_index);
		if(dim == 1) prefix_trie_dim2.InsertPrefixTrieV2(num, prefix_num, leaf_index);
		if(dim == 2) prefix_trie_dim3.InsertPrefixTrieV2(num, prefix_num, leaf_index);
		return;
	}
	
};
