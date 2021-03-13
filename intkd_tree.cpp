#include "intkd_tree.h"

KdTreeH::KdTreeH(
        vector <Point> *points_ptr, \
        vector <int> *index_ptr, \
        int max_leaf_size, int para_NN)
{
        index_ = index_ptr;
        points_ = points_ptr;
        max_leaf_size_ = max_leaf_size;
        dim_ = (*points_ptr)[0].dim;
        para_NN_ = para_NN;
}

KdTreeH::~KdTreeH() {}

void Node::deleteNode(Node * node_p) {
    if (node_p->left_child) deleteNode(node_p->left_child);
    if (node_p->right_child) deleteNode(node_p->right_child);
    delete node_p;
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
	cout << "leaf node number: " << leaf_nodes_.size() << endl;	
}

vector<KdTreeH::NearestInfo> KdTreeH::QueueCopy(KnnQueue &k_queue)
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
	return Dist_squared > 0;
}

void load_bin(std::string infile, std::vector<Point> & points)
{
	std::cout << "Loading " << infile << std::endl;

	std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);

	if(!input.good()){
		std::cerr << "Could not read file: " << infile << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	int p = 0;
	for (int j=0; input.good() && !input.eof(); j++) {
		
		Point_f point; 
		Point point_gt;

		input.read((char *) &point.x, 3 * sizeof(float));
		input.seekg(sizeof(float), std::ios::cur);    
        float x, y ,z;
        x = -point.y;
        y = -point.z;
        z = point.x;
		point_gt.x = (int)((x + 100) * 10);
        point_gt.y = (int)((y + 100) * 10);
        point_gt.z = (int)((z + 100) * 10);
		points.push_back(point_gt);
	}
	input.close();
}