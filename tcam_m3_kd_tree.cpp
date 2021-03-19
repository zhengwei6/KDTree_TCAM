#include "tcam_m3_kd_tree.h"

KdTreeHTCAM_M3::KdTreeHTCAM_M3(vector <Point> *points_ptr, vector <int> *index_ptr, int max_leaf_size, int para_NN): KdTreeH::KdTreeH(points_ptr, index_ptr, max_leaf_size, para_NN), prefix_trie_dim1(), prefix_trie_dim2(), prefix_trie_dim3(), prefix_conversion_time(0) {
	for(int i = 0 ; i < dim_ ; i++) {
		prefix_tries_.push_back(new PrefixTrie());
	}
}

void KdTreeHTCAM_M3::BuildKDTree() {
    vector<Interval> bbox;
    vector<Interval> *bbox_ptr = NULL;
    int left = 0;
    int right = (index_->size()) - 1;
    ComputeBoundingBox(left, right, bbox);
    bbox_ptr = &bbox;
    root_node_ = DivideTree(left, right, bbox_ptr);
}

KdTreeH::NodePtr KdTreeHTCAM_M3::DivideTree(int left, int right, ::vector<Interval> *bbox_ptr) {
    NodePtr node = new Node();
    for(int i = left; i<= right; i++) {
		(node->index_list).push_back((*index_)[i]);
	}
    node->bbox = *bbox_ptr;
    int count = right - left;
    if(count <= max_leaf_size_) {
        node->s_dim = -1;
        node->s_val = -1;
        leaf_nodes_.push_back(node);
        node->leaf_idx = leaf_nodes_.size() - 1;
        bcount += 1; // increase number of count for bounding box
        for(int i = 0 ; i < (*bbox_ptr).size() ; i++) {
            vector<pair<int, int>> prefix_range;
            DirectConversion((*bbox_ptr)[i].low, (*bbox_ptr)[i].high, prefix_range);
            store_prefix_count += prefix_range.size();
            for (int j = 0 ; j < prefix_range.size() ; j++) {
                InsertPrefixTrieDim(i, prefix_range[j].first << prefix_range[j].second , prefix_range[j].second, node->leaf_idx);
            }
        }
        pcount += node->index_list.size();
        return node;
    }
    else {
        int split_dim = 0;
        int span = 0;
        vector<int> value_list;
        FindSplitDim(split_dim, span, bbox_ptr);
        node->s_dim = split_dim;
        int split_val = 0;
        GetValueList(left, right, split_dim, value_list);
        vector<int>* value_list_ptr = &value_list;
        QSelectMedian(value_list_ptr, split_val);
        node->s_val = split_val;
        int lim1 = 0, lim2 = 0, split_delta = 0;
        PlaneSplit(left, right, split_dim, split_val, lim1, lim2);
        split_delta = (lim1 + lim2) / 2;
        vector<Interval> bbox_l;
		vector<Interval> bbox_r;
		ComputeBoundingBox(left, left + split_delta, bbox_l);
		ComputeBoundingBox(left + split_delta + 1, right, bbox_r);
		node->left_child  = DivideTree(left, left + split_delta, &bbox_l);
		node->right_child = DivideTree(left + split_delta + 1, right, &bbox_r);
		return node;
    }
}

void KdTreeHTCAM_M3::InsertPrefixTrieDim(int dim, int num, int prefix_num, int leaf_index) {
	if(dim == 0) prefix_trie_dim1.InsertPrefixTrieV2(num, prefix_num, leaf_index);
	if(dim == 1) prefix_trie_dim2.InsertPrefixTrieV2(num, prefix_num, leaf_index);
	if(dim == 2) prefix_trie_dim3.InsertPrefixTrieV2(num, prefix_num, leaf_index);
	return;
}

void KdTreeHTCAM_M3::NearestKSearch(Point query, int knn, vector<NearestInfo> &k_elements) {
    KnnQueue kd_queue;
	NodePtr cur_node = root_node_;
	vector<NodePtr> backtrack_stack;
	cur_node = TraverseTree(cur_node, query, backtrack_stack);

	BruteForceKSearch(&cur_node->index_list, query, kd_queue, knn);
	
	
	int Dist_min_ = kd_queue.top().second;
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
	x_range.first  = (query.x + Dist_min_) > 0 ? (query.x + Dist_min_) : 0;
	x_range.second = (query.x - Dist_min_) > 0 ? (query.x - Dist_min_) : 0;
	y_range.first  = (query.y + Dist_min_) > 0 ? (query.y + Dist_min_) : 0;
	y_range.second = (query.y - Dist_min_) > 0 ? (query.y - Dist_min_) : 0;
	z_range.first  = (query.z + Dist_min_) > 0 ? (query.z + Dist_min_) : 0;
	z_range.second  = (query.z - Dist_min_) > 0 ? (query.z - Dist_min_) : 0;

	// direction conversion
	clock_t c_start, c_end;
	
	c_start = clock();
	DirectConversion(x_range.second, x_range.first, prefix_range_x);
	DirectConversion(y_range.second, y_range.first, prefix_range_y);
	DirectConversion(z_range.second, z_range.first, prefix_range_z);
	c_end   = clock();
	search_prefix_conversion_time += (c_end - c_start);
	search_prefix_count += prefix_range_x.size() + prefix_range_y.size() + prefix_range_z.size();

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
		BruteForceKSearch(&search_node->index_list, query, kd_queue, knn);
	}
	k_elements = QueueCopy(kd_queue);
	return;
}

void KdTreeHTCAM_M3::BruteForceKSearch(::vector<int> *ind, Point query, \
	KnnQueue &k_priority_queue, int knn)
{
	int Dist_ = 0;
	for(int i = 0 ; i < (*ind).size() ; i++)
	{
		Dist_ = Dist(query, (*points_)[ (*ind)[i] ]);
		if(k_priority_queue.size() >= knn && Dist_ < k_priority_queue.top().second) {
			k_priority_queue.pop();
			k_priority_queue.push(make_pair((*ind)[i], Dist_));
		}
		else if(k_priority_queue.size() < knn) {
			k_priority_queue.push(make_pair((*ind)[i], Dist_));
		}
	}
	return;
}

void KdTreeHTCAM_M3::print_store_prefix_count()
{
	cout << "store_prefix_count: " << store_prefix_count << endl;
}

void KdTreeHTCAM_M3::print_search_prefix_count()
{
	cout << "search_prefix_count: " << search_prefix_count << endl; 
}

void KdTreeHTCAM_M3::print_search_prefix_conversion_time()
{
	cout << "search_prefix_conversion_time: " << (search_prefix_conversion_time * 1000.0) / CLOCKS_PER_SEC << " ms"<< endl;
}

void KdTreeHTCAM_M3::print_insert_prefix_time()
{
	cout << "insert_prefix_time: " << (insert_prefix_time * 1000.0) / CLOCKS_PER_SEC << " ms"<< endl;
}

void KdTreeHTCAM_M3::print_prefix_conversion_time()
{
	cout << "prefix_conversion_time: " << (prefix_conversion_time * 1000.0) / CLOCKS_PER_SEC << " ms"  << endl;
}
