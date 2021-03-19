#ifndef CLASSIC_TREE_H
#define CLASSIC_TREE_H
#include "intkd_tree.h"
#include "limits.h"

class KDTreeHClassic : public KdTreeH {
public:
	int back_track = 0;
    KDTreeHClassic(vector <Point> *points_ptr, vector <int> *index_ptr, int max_leaf_size, int para_NN);
	void BuildKDTree();
	void NearestKSearch(Point query, int knn, vector<NearestInfo> &k_elements);
	void BruteForceKSearch(vector<int> *ind, Point query, KnnQueue &k_priority_queue, int knn);
protected:
	NodePtr DivideTree(int left, int right, vector<Interval> *bbox_ptr);
	
};

#endif