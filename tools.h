#ifndef TOOLS_H
#define TOOLS_H
#include <random>
#include <iostream>
#include <limits>
#include <fstream>
#include <string>

#include <unordered_map>
#include <vector>
#include <queue>
#include <deque>
#include <set>
#include <algorithm>

using namespace std;
void DirectConversion(int start, int end, vector<pair<int, int>> &prefix_entries);
pair<int, int> random_range(int start, int end);
#endif