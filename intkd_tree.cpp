#include "intkd_tree.h"

KdTreeH::KdTreeH(
        ::vector <Point> *points_ptr, \
        ::vector <int> *index_ptr, \
        int max_leaf_size, int para_NN):dim_(3), prefix_trie_dim1(), prefix_trie_dim2(), prefix_trie_dim3()
{
        index_ = index_ptr;
        points_ = points_ptr;
        max_leaf_size_ = max_leaf_size;
        dim_ = (*points_ptr)[0].dim;
        para_NN_ = para_NN;
		for(int i = 0 ; i < dim_ ; i++) {
			prefix_tries_.push_back(new PrefixTrie());
		}
}

KdTreeH::~KdTreeH() {}

void Node::deleteNode(Node * node_p) {
    if (node_p->left_child) deleteNode(node_p->left_child);
    if (node_p->right_child) deleteNode(node_p->right_child);
    delete node_p;
}
	

void KdTreeH::BuildKDTreeV1()
{
	vector<Interval> bbox;
	vector<Interval> *bbox_ptr = NULL;

	int left  = 0;
	int right = (index_->size()) - 1;
	ComputeBoundingBox(left, right, bbox);
	
	bbox_ptr = &bbox;
	root_node_ = DivideTreeV1(left, right, bbox_ptr); 
}

void KdTreeH::BuildKDTree() 
{
	vector<Interval> bbox;
	vector<Interval> *bbox_ptr = NULL;

	int left = 0;
	int right = (index_->size()) - 1;

	ComputeBoundingBox(left, right, bbox);
	bbox_ptr = &bbox;
	root_node_ = DivideTree(left, right, bbox_ptr);
}

void KdTreeH::print_param()
{
	cout << "index size: " << (*index_).size() << ::endl;
	cout << "points number: " << (*points_).size() << ::endl;
}

void KdTreeH::print_pcount()
{
	cout << "number of points in kd tree: " << pcount << endl; 
}

void KdTreeH::print_bcount()
{
	cout << "number of bounding boxes: " << bcount << endl;	
}

void KdTreeH::print_points(int i)
{
	cout << "point x: " << (*points_)[i][0] << " point.y: " << (*points_)[i][1] << " point.z: " << (*points_)[i][2] << ::endl;
	return;
}

void KdTreeH::print_leaf_node_num()
{
	cout << "leaf_nodes_: " << leaf_nodes_.size() << endl;	
}


void KdTreeH::NearestKSearchTCAM(Point query, int knn, vector<NearestInfo> &k_elements)
{
	KnnQueue kd_queue;
	NodePtr cur_node = root_node_;
	vector<NodePtr> backtrack_stack;
	cur_node = TraverseTree(cur_node, query, backtrack_stack);
	#ifdef DEBUG
	cout << "interval of first bounding box: " << endl;
	for(int i = 0 ; i < cur_node->bbox.size() ; i++) {
		cout << "dimension: " << i << " low " << cur_node->bbox[i].low << " high " << cur_node->bbox[i].high << endl; 
	}
	
	cout << "print all the points in bounding box: " << endl;
	for(const auto &k : cur_node->index_list) {
		print_points(k);
	}
	#endif

	BruteForceKSearchV2(&cur_node->index_list, query, kd_queue, knn);
	
	#ifdef DEBUG
	//print nearest data point info
	cout << "the nearest Distance: ";
	cout << kd_queue.top().second << endl;
	cout << "the nearest data point: ";
	print_points(kd_queue.top().first);
	#endif
	
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
		BruteForceKSearchV2(&search_node->index_list, query, kd_queue, knn);
	}
	k_elements = QueueCopy(kd_queue);
	return;
}

int KdTreeH::NearestKSearch(Point query, int knn,\
	::vector<int> &k_indices, \
	::vector<int> &k_Dists)
{
	clock_t c_start, c_end;
	long brute_force_search_time = 0;
	long back_track_time = 0;
	
	int back_track = 0;
	vector<NodePtr> backtrack_stack;
	
	NodePtr cur_node = root_node_;
	
	c_start = clock();
	cur_node = TraverseTree(cur_node, query, backtrack_stack);
	c_end   = clock();

	back_track_time +=  (c_end - c_start) ;

	c_start = clock();
	BruteForceKSearch(cur_node->index_list, query, k_indices, k_Dists);
	c_end   = clock();
	cout << "Traverse to first point " << endl;
	print_points(k_indices[0]);
	cout << "Dist to first data point: " << k_Dists[0] << endl ;
	cout << "Backtrack stack size: " << backtrack_stack.size() << endl << endl;
	brute_force_search_time += (c_end - c_start);
	for(;backtrack_stack.size();)
	{
		cout << "Backtrack" << endl;
		cur_node = backtrack_stack.back();
		backtrack_stack.pop_back();
		int Dist_min_ = k_Dists[0];
		cout << "check intersection" << endl;
		bool visit_ = CheckIntersection(query, Dist_min_, cur_node);
		// test
		cout << "Dim1: " << (cur_node->bbox)[0].low << " " << (cur_node->bbox)[0].high << endl;
		cout << "Dim2: " << (cur_node->bbox)[1].low << " " << (cur_node->bbox)[1].high << endl;
		cout << "Dim3: " << (cur_node->bbox)[2].low << " " << (cur_node->bbox)[2].high << endl;
		if(visit_)
		{
			cout << "Traverse the tree from the node" << endl;
			back_track += 1;
			if (cur_node->s_dim != 1) {
				c_start = clock();
				cur_node = TraverseTree(cur_node, query, backtrack_stack);
				c_end   = clock();
				back_track_time += (c_end - c_start);
			}
			c_start = clock();
			BruteForceKSearch(cur_node->index_list, query, k_indices, k_Dists);
			c_end   = clock();
			brute_force_search_time += (c_end - c_start);
			//cout << brute_force_search_time << endl;
		}
		else {
			cout << "Prune the node" << endl;
		}
		cout << endl;
	}

	back_track_time 		= (back_track_time * (1000.0)) / CLOCKS_PER_SEC;
	brute_force_search_time = (brute_force_search_time * (1000.0)) / CLOCKS_PER_SEC;

	// #ifdef DEBUG
	cout << "traverse tree time: " << back_track_time << endl;
	cout << "brute force time: " << brute_force_search_time << endl;
	// #endif
	cout << "back_track: " << back_track << endl;
	return back_track;
}

void KdTreeH::BruteForceKSearchV2(::vector<int> *ind, Point query, \
	KnnQueue &k_priority_queue, int knn)
{
	int Dist_ = 0;
	for(int i = 0 ; i < (*ind).size() ; i++)
	{
		Dist_ = Dist(query, (*points_)[ (*ind)[i] ]);
		if(k_priority_queue.size() >= knn && Dist_ < k_priority_queue.top().second) {
			k_priority_queue.pop();
			k_priority_queue.push(::make_pair((*ind)[i], Dist_));
		}
		else if(k_priority_queue.size() < knn) {
			k_priority_queue.push(::make_pair((*ind)[i], Dist_));
		}
	}
	return;
}

void KdTreeH::BruteForceKSearch(::vector<int> ind, Point query, \
	::vector<int> &k_indices, \
	::vector<int> &k_Dists)
{
	int Dist_ = 0;
	for (int i = 0; i < ind.size() ; i++) 
	{
		Dist_ = Dist(query, (*points_)[ ind[i] ]);
		if(k_Dists.size())
		{
			if(Dist_ < k_Dists[0])
			{
				k_indices.pop_back();
				k_Dists.pop_back();
				k_indices.push_back(ind[i]);
				k_Dists.push_back(Dist_);
			}
		}
		else
		{
			k_indices.push_back(ind[i]);
			k_Dists.push_back(Dist_);
		}
	}
}

::vector<KdTreeH::NearestInfo> KdTreeH::QueueCopy(KnnQueue &k_queue)
{
	::vector<NearestInfo> copy;
	while(!k_queue.empty()) {
		copy.push_back(k_queue.top());
		k_queue.pop();
	}
	return copy;
}

int KdTreeH::GetDistance(Point point, Point centroid) {
	int Distance = pow((centroid.x - point.x), 2) + pow((centroid.y - centroid.y), 2) + pow((centroid.z - centroid.z), 2);
	return Distance;
}

int KdTreeH::Dist(Point p1, Point p2) {
	return sqrt(pow((p1.x - p2.x), 2)\
			+ pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
}

KdTreeH::NodePtr KdTreeH::DivideTreeV1(int left, int right, ::vector<Interval> *bbox_ptr) {
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
		node->leaf_idx = leaf_nodes_.size() - 1;
		return node;
	}
	else {
		int split_dim = 0;
		int span = 0;
		vector<int> value_list;
		FindSplitDim(split_dim, span, bbox_ptr);
		node->s_dim = split_dim;
		int split_val = 0;
		// test
		// copy the value list range to value_list: left to right
		GetValueList(left, right, split_dim, value_list);
		vector<int>* value_list_ptr = &value_list;
		QSelectMedian(value_list_ptr, split_val);
		node->s_val = split_val;

		int lim1 = 0, lim2 = 0, split_delta = 0;
		// PlaneSplit: swap index
		PlaneSplit(left, right, split_dim, split_val, lim1, lim2);
		split_delta = (lim1 + lim2) / 2;
		vector<Interval> bbox_l;
		vector<Interval> bbox_r;
		ComputeBoundingBox(left, left + split_delta, bbox_l);
		ComputeBoundingBox(left + split_delta + 1, right, bbox_r);
		node->left_child  = DivideTreeV1(left, left + split_delta, &bbox_l);
		node->right_child = DivideTreeV1(left + split_delta + 1, right, &bbox_r);
		return node;
	}
}

KdTreeH::NodePtr KdTreeH::DivideTree(int left, int right, ::vector<Interval> *bbox_ptr) {
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
			::cout << "dimension: " << i << ""<< ::endl;
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
		vector<int> value_list;
		FindSplitDim(split_dim, span, bbox_ptr);
		node->s_dim = split_dim;
		int split_val = 0;
		// test
		// copy the value list range to value_list: left to right
		GetValueList(left, right, split_dim, value_list);
		vector<int>* value_list_ptr = &value_list;
		QSelectMedian(value_list_ptr, split_val);
		node->s_val = split_val;

		#ifdef DEBUG
		//test
		//::cout << "split_dim: " << split_dim << " split_val: "  << split_val << ::endl; 
		#endif

		int lim1 = 0, lim2 = 0, split_delta = 0;
		// PlaneSplit: swap index
		PlaneSplit(left, right, split_dim, split_val, lim1, lim2);
		split_delta = (lim1 + lim2) / 2;
		vector<Interval> bbox_l;
		vector<Interval> bbox_r;
		ComputeBoundingBox(left, left + split_delta, bbox_l);
		ComputeBoundingBox(left + split_delta + 1, right, bbox_r);
		node->left_child = DivideTree(left, left + split_delta, &bbox_l);
		node->right_child = DivideTree(left + split_delta + 1, right, &bbox_r);
		return node;
	}
}

void KdTreeH::FindSplitDim(int &best_dim, int &span, \
		::vector<Interval> *bbox_ptr) {
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

void KdTreeH::GetValueList(int left, int right, int split_dim, ::vector<int> &value_list) {
	for(int i = left; i <= right ; i++)
		value_list.push_back((*points_)[(*index_)[i]][split_dim]);
}
	
void KdTreeH::QSelectMedian(::vector<int>* value_list, int &median_value) {
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
	
void KdTreeH::FindMedian(int left, int right, int split_dim, int &median_value) {
	::vector<int> dim_sorted;
	for (int i = left; i <= right; ++i) {
		dim_sorted.push_back((*points_)[(*index_)[i]][split_dim]);
	}
	::sort(dim_sorted.begin(), dim_sorted.end());
	int median_pos = (right - left) / 2;
	median_value = dim_sorted[median_pos];
}
	
void KdTreeH::PlaneSplit(int left, int right, int split_dim,
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

inline void KdTreeH::swap(::vector<int>* value_list, int a, int b) {
	int tmpa = (*value_list)[a];
	int tmpb = (*value_list)[b];
	(*value_list)[a] = tmpb;
	(*value_list)[b] = tmpa;
}
	
void KdTreeH::ComputeBoundingBox(int left, int right, \
::vector<Interval> &bbox) 
{
	int cur_dim = 0;
	for(; cur_dim < dim_ ; cur_dim++) {
		Interval bounds;
		KdTreeH::ComputeMinMax(left, right, cur_dim, bounds.low, bounds.high);			
		bbox.push_back(bounds);
	}
}
	
void KdTreeH::ComputeMinMax(int left, int right, int dim, int& min_val, int& max_val)
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
KdTreeH::NodePtr KdTreeH::TraverseTree(NodePtr cur_node, Point query, \
	vector<Node *> & backtrack_stack)
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

bool KdTreeH::CheckIntersection(Point query, int radius, NodePtr cur_node)
{
	cout << "radius: "<< radius << endl;
	int Dist_squared = radius * radius;
	
	int x_min = (cur_node->bbox)[0].low;
	int x_max = (cur_node->bbox)[0].high;
	
	int y_min = (cur_node->bbox)[1].low;
	int y_max = (cur_node->bbox)[1].high;
	
	int z_min = (cur_node->bbox)[2].low;
	int z_max = (cur_node->bbox)[2].high;
	
	if (query.x < x_min)
		Dist_squared -= (query.x - x_min) * (query.x - x_min);
	else if (query.x > x_max)
		Dist_squared -= (query.x - x_max) * (query.x - x_max);

	if (query.y < y_min)
		Dist_squared -= (query.y - y_min) * (query.y - y_min);
	else if (query.y > y_max)
		Dist_squared -= (query.y - y_max) * (query.y - y_max);

	if (query.z < z_min)
		Dist_squared -= (query.z - z_min) * (query.z - z_min);
	else if (query.z > z_max)
		Dist_squared -= (query.z - z_max) * (query.z - z_max);
	cout << "Dist_squared: "<< Dist_squared << endl;
	return Dist_squared > 0;
}

void KdTreeH::InsertPrefixTrieDim(int dim, int num, int prefix_num, int leaf_index) {
	if(dim == 0) prefix_trie_dim1.InsertPrefixTrieV2(num, prefix_num, leaf_index);
	if(dim == 1) prefix_trie_dim2.InsertPrefixTrieV2(num, prefix_num, leaf_index);
	if(dim == 2) prefix_trie_dim3.InsertPrefixTrieV2(num, prefix_num, leaf_index);
	return;
}

