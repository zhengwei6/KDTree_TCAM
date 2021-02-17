/*
給多個 ranges & leaf_index
insert  進入 prefixtrie 中
給多個 number
每個 number 輸出出對應的 leaf_index
*/
#include "bits/stdc++.h"
#include "prefix_tree.h"
#include "tools.h"
using namespace std;
int main(int argc, char *argv[]) {
    int range_num;
    cout << "input number of ranges ";
    cin >> range_num;
    vector<pair<int, int>> input_ranges;
    vector<vector<pair<int, int>>> prefix_ranges(range_num);
    vector<int> leaf_index(range_num, 0);
    for(int i = 0 ; i < range_num ; i++) {
        pair<int, int> input_range;
        cout << "input range " << i << " :";
        cin >> input_range.first >> input_range.second;
        input_ranges.push_back(input_range);
        cout << "input index " << i << " :";
        cin >> leaf_index[i];
    }
    
    PrefixTrie *prefix_trie = new PrefixTrie();
    for(int i = 0 ; i < range_num ; i++) {
        cout << "input range: " << i << endl;
        direct_conversion(input_ranges[i].first, input_ranges[i].second, prefix_ranges[i]);
        print_stack(prefix_ranges[i]);
    }
    
    for (int i = 0 ; i < range_num ; i++) {
        for (int j = 0 ; j < prefix_ranges[i].size() ; j++) {
            prefix_trie->InsertPrefixTrieV2(prefix_ranges[i][j].first << prefix_ranges[i][j].second, prefix_ranges[i][j].second, leaf_index[i]);
        }
    }

    pair<int, int> search_range;
    vector<int> res_index = {};
    cout << "input search range: ";
    cin >> search_range.first >> search_range.second;
    vector<pair<int, int>> prefix_source;
    direct_conversion(search_range.first, search_range.second, prefix_source);
    prefix_trie->SearchRangeV2(prefix_source, res_index);
    for(const auto &i : res_index) {
        cout << i << " ";
    }
    cout << endl;
    return 0;
}