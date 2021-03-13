#include "classic_kd_tree.h"

KDTreeHClassic::KDTreeHClassic(vector <Point> *points_ptr, vector <int> *index_ptr, int max_leaf_size, int para_NN):KdTreeH::KdTreeH(points_ptr, index_ptr, max_leaf_size, para_NN) {
}

void KDTreeHClassic::BuildKDTree() {
    vector<Interval> bbox;
    vector<Interval> *bbox_ptr = NULL;
    int left  = 0;
	int right = (index_->size()) - 1;
	ComputeBoundingBox(left, right, bbox);
	
	bbox_ptr = &bbox;
	root_node_ = DivideTree(left, right, bbox_ptr);     
}

void KDTreeHClassic::NearestKSearch(Point query, int knn, vector<NearestInfo> &k_elements) {
    KnnQueue kd_queue;
    long brute_force_search_time = 0;
    long back_track_time = 0;
    vector<NodePtr> backtrack_stack;
    NodePtr cur_node = root_node_;
    cur_node = TraverseTree(cur_node, query, backtrack_stack); 
    BruteForceKSearch(&cur_node->index_list, query, kd_queue, knn);
    for(;backtrack_stack.size();) {
        cur_node = backtrack_stack.back();
        backtrack_stack.pop_back();
        int Dist_min_ = kd_queue.top().second;
        bool visit_ = CheckIntersection(query, Dist_min_, cur_node);
        if(visit_) {
            back_track += 1;
			if (cur_node->s_dim != 1) {
				cur_node = TraverseTree(cur_node, query, backtrack_stack);
			}
			BruteForceKSearch(&cur_node->index_list, query, kd_queue, knn);
        }
    }
	k_elements = QueueCopy(kd_queue);
    return;
}

void KDTreeHClassic::BruteForceKSearch(vector<int> *ind, Point query, \
	KnnQueue &k_priority_queue, int knn) {
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


KdTreeH::NodePtr KDTreeHClassic::DivideTree(int left, int right, vector<Interval> *bbox_ptr) {
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
		node->left_child  = DivideTree(left, left + split_delta, &bbox_l);
		node->right_child = DivideTree(left + split_delta + 1, right, &bbox_r);
		return node;
	}
}
