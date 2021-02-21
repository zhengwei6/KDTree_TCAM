#ifndef PREFIX_TREE_H
#define PREFIX_TREE_H

#include <iostream>
#include <string>
#include <queue>
#include <bitset>
#include <iomanip>
#include <algorithm>

using namespace std;
int get_bit(int num, int pos, int num_bits);
void PrintStack(vector<pair<int, int>> &prefix_entries);
class PrefixTrie;
class TrieNode{
public:
    TrieNode *left_child;
    TrieNode *right_child;
    vector<int> leaf_index;
    bool isPrefix;
    TrieNode():left_child(0), right_child(0), isPrefix(0){};
    TrieNode(bool isPrefix):left_child(0), right_child(0), isPrefix(isPrefix), leaf_index(0){};
    friend class PrefixTrie;
};

class PrefixTrie{
public:
    TrieNode *root;
    PrefixTrie():root(new TrieNode()){};
    PrefixTrie(TrieNode *node):root(node){};
    ~PrefixTrie();
    void DeleteTrie();
    void _deleteTrie(TrieNode *node);
    void InsertPrefixTrie(int num, int prefix_num);
    void InsertPrefixTrieV2(int num, int prefix_num, int leaf_index);
    bool SearchNum(int num);
    void SearchNumV2(int num, vector<int> &res);
    bool SearchDonCare(int num, int prefix_num);
    void SearchDonCareV2(int num, int prefix_num, vector<int> &res);
    void DfSearchNode(TrieNode *node, vector<int> &res);
    bool SearchRange(vector<pair<int, int>> &prefix_array);
    void SearchRangeV2(vector<pair<int, int>> &prefix_array, vector<int> &res);
};

#endif