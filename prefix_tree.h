#include <iostream>
#include <string>
#include <queue>

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

PrefixTrie::~PrefixTrie() {
    cout << "Deleting Trie..." << endl;
    this->_deleteTrie(this->root);
    cout << "Complete" << endl;
    this->root = new TrieNode();
}

void PrefixTrie::DeleteTrie() {
    cout << "Deleting Trie..." << endl;
    this->_deleteTrie(this->root);
    this->root = new TrieNode();
    cout << "Complete" << endl;
    return ;
}

void PrefixTrie::_deleteTrie(TrieNode *node) {
    if(node == NULL) return;
    _deleteTrie(node->left_child);
    _deleteTrie(node->right_child);
    delete node;
}

void PrefixTrie::InsertPrefixTrieV2(int num, int prefix_num, int leaf_index) {
    TrieNode *current = this->root;
    for(int i = 0 ; i < 32 - prefix_num ; i++) {
        int zero_one = get_bit(num, i, 32);
        if(zero_one == 0) {
            if(current->left_child != NULL) {
                current = current->left_child;
            }
            else {
                current->left_child = new TrieNode();
                current = current->left_child;  
            }
            if(i == (32 - prefix_num - 1)) {
                current->isPrefix = 1;
                current->leaf_index.push_back(leaf_index);
            }
        }
        else if(zero_one == 1) {
            if(current->right_child != NULL) {
                current = current->right_child;
            }
            else {
                current->right_child = new TrieNode();
                current = current->right_child;
            }
            if(i == (32 - prefix_num - 1)) {
                current->isPrefix = 1;
                current->leaf_index.push_back(leaf_index);
            }
        }
    }
}

void PrefixTrie::InsertPrefixTrie(int num, int prefix_num) {
    TrieNode *current = root;
    for(int i = 0 ; i < 32 - prefix_num ; i++) {
        int zero_one = get_bit(num, i, 32);
        if(zero_one == 0) {
            if(current->left_child != NULL) {
                current = current->left_child;
            }
            else {
                current->left_child = new TrieNode();
                current = current->left_child;  
            }
            if(i == (32 - prefix_num - 1)) {
                current->isPrefix = 1;
            }
        }
        else if(zero_one == 1) {
            if(current->right_child != NULL) {
                current = current->right_child;
            }
            else {
                current->right_child = new TrieNode();
                current = current->right_child;
            }
            if(i == (32 - prefix_num - 1)) {
                current->isPrefix = 1;
            }
        }
    }
}

void PrefixTrie::SearchNumV2(int num, vector<int> &res) {
    TrieNode *current = root;
    for(int i = 0 ; i < 32 ; i++) {
        int zero_one = get_bit(num, i, 32);
        if(zero_one == 0) {
            if(current->left_child != NULL)
                current = current->left_child;
            else
                return;
        }
        else if(zero_one == 1) {
            if(current->right_child != NULL)
                current = current->right_child;
            else
                return;
        }
        if(current->isPrefix == 1) {
            //res union with internal node indices
            vector<int> tmp = {};
            sort(current->leaf_index.begin(), current->leaf_index.end());
            sort(res.begin(), res.end());
            set_union(current->leaf_index.begin(), current->leaf_index.end(), res.begin(), res.end(), inserter(tmp, tmp.begin()));
            res = tmp;
        }
    }
    return;
}

bool PrefixTrie::SearchNum(int num) {
    TrieNode *current = root;
    for(int i = 0 ; i < 32; i++) {
        int zero_one = get_bit(num, i, 32);
        if(zero_one == 0) {
            if(current->left_child != NULL)
                current = current->left_child;
            else{
                return 0;
            }
        }
        else if(zero_one == 1) {
            if(current->right_child != NULL)
                current = current->right_child;
            else
                return 0;
        }
        if(current->isPrefix == 1) {
            return 1;
        }
    }
    return 1;
}

void PrefixTrie::DfSearchNode(TrieNode *node, vector<int> &res) {
    if(node->left_child != NULL)
        DfSearchNode(node->left_child, res);
    if(node->right_child != NULL)
        DfSearchNode(node->right_child, res);
    if(node->isPrefix == 1) {
        vector<int> tmp = {};
        sort(node->leaf_index.begin(), node->leaf_index.end());
        sort(res.begin(), res.end());
        set_union(node->leaf_index.begin(), node->leaf_index.end(), res.begin(), res.end(), inserter(tmp, tmp.begin()));
        res = tmp;
    }
    return;
}

void PrefixTrie::SearchDonCareV2(int num, int prefix_num, vector<int> &res) {
    TrieNode *current = root;
    for(int i = 0 ; i < 32 - prefix_num; i++) {
        int zero_one = get_bit(num, i, 32);
        if(zero_one == 0) {
            if(current->left_child != NULL)
                current = current->left_child;
            else
                return;
        }
        else if(zero_one == 1) {
            if(current->right_child != NULL)
                current = current->right_child;
            else
                return;
        }
        if(current->isPrefix == 1) {
            //res union with internal node indices
            vector<int> tmp = {};
            sort(current->leaf_index.begin(), current->leaf_index.end());
            set_union(current->leaf_index.begin(), current->leaf_index.end(), res.begin(), res.end(), inserter(tmp, tmp.begin()));
            res = tmp;
        }
    }
    //dfs
    DfSearchNode(current, res);
    return;
}

bool PrefixTrie::SearchDonCare(int num, int prefix_num) {
    TrieNode *current = root;
    for(int i = 0 ; i < 32 - prefix_num; i++) {
        int zero_one = get_bit(num, i, 32);
        if(zero_one == 0) {
            if(current->left_child != NULL)
                current = current->left_child;
            else
                return 0;
        }
        else if(zero_one == 1) {
            if(current->right_child != NULL)
                current = current->right_child;
            else
                return 0;
        }
        if(current->isPrefix == 1) {
            return 1;
        }
    }
    return 1;
}

bool PrefixTrie::SearchRange(vector<pair<int, int>> &prefix_array) {
    for(int i = 0 ; i < prefix_array.size() ; i++) {
        if(this->SearchDonCare(prefix_array[i].first << prefix_array[i].second, prefix_array[i].second)) {
            return 1;
        }
    }
    return 0;
}

void PrefixTrie::SearchRangeV2(vector<pair<int, int>> &prefix_array, vector<int> &res) {
    for(int i = 0 ; i < prefix_array.size() ; i++) {
        this->SearchDonCareV2(prefix_array[i].first << prefix_array[i].second, prefix_array[i].second, res);
    }
    return;
}

int get_bit(int num, int pos, int num_bits) {
    return num >> (num_bits - pos - 1) & 1;
}

void PrintStack(vector<pair<int, int>> &prefix_entries) {
    for(int i = 0; i < prefix_entries.size() ; i++) {
        string binary = bitset<32>(prefix_entries[i].first).to_string();
        binary.erase(0, binary.find_first_not_of('0'));
        for(int j = 0 ; j < prefix_entries[i].second ; j++) {
            binary += "*";
        }
        string tag = "[ " + to_string(i) + " ]";
        cout << left << setw(10) << tag;
        cout << right << setw(32) << binary<<endl;
    }
    return;
}

