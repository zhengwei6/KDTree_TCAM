#include "tools.h"
void DirectConversion(int start, int end, vector<pair<int, int>> &prefix_entries) {
    vector<int> vect_rec(end + 2, 0);
    for(int i = start ; i <= end ; i++) {
        vect_rec[i] = 1;
    }
    int count    = 0;
    int has_push = 1;
    while(has_push) {
        has_push = 0;
        for(int i = 0; i <= end; i += 2) {
            if(vect_rec[i] == 1 && vect_rec[i + 1] == 1) {
                vect_rec[i/2] = 1;
                has_push = 1;
                if(i != 0)
                    vect_rec[i] = 0;
                vect_rec[i+1] = 0;
            }
            else if(vect_rec[i] == 1 && vect_rec[i + 1] == 0) {
                vect_rec[i/2] = 0;
                pair<int, int> push_element = {i, count};
                prefix_entries.push_back(push_element);
                vect_rec[i] = 0;
            }
            else if(vect_rec[i] == 0 && vect_rec[i + 1] == 1) {
                vect_rec[i/2] = 0;
                pair<int, int> push_element = {i + 1, count};
                prefix_entries.push_back(push_element);
                vect_rec[i + 1] = 0;
            }
            else {
            }
        }
        count += 1;
    }
    return;
}

pair<int, int> random_range(int start, int end) {
    pair<int, int> return_range;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> distrib(start, end);
    int first, second;
    first  = distrib(gen);
    second = distrib(gen);
    return_range.first = first > second ? second : first;
    return_range.second = first > second ? first : second;
    return return_range;
}