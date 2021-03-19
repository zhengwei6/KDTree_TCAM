#ifndef TCAM_M3_KD_TREE_H
#define TCAM_M3_KD_TREE_H
#include "intkd_tree.h"

class KdTreeHTCAM_M3 : public KdTreeH{
public:
    KdTreeHTCAM_M3(vector <Point> *points_ptr, vector <int> *index_ptr, int max_leaf_size, int para_NN);
	void BuildKDTree();
	void NearestKSearch(Point query, int knn, vector<NearestInfo> &k_elements);
	void BruteForceKSearch(vector<int> *ind, Point query, KnnQueue &k_priority_queue, int knn);
	void print_prefix_conversion_time();
	void print_insert_prefix_time();
	void print_search_prefix_conversion_time();
	void print_search_prefix_count();
	void print_store_prefix_count();

protected:
	NodePtr DivideTree(int left, int right, vector<Interval> *bbox_ptr);

private:
	vector<PrefixTrie *> prefix_tries_;
	PrefixTrie prefix_trie_dim1;
	PrefixTrie prefix_trie_dim2;
	PrefixTrie prefix_trie_dim3;
	long prefix_conversion_time = 0;
	long insert_prefix_time     = 0;
	long search_prefix_conversion_time = 0;
	int search_prefix_count = 0;
	int store_prefix_count = 0;
	void InsertPrefixTrieDim(int dim, int num, int prefix_num, int leaf_index);
};

#endif