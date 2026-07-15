#ifndef BLOCK_SORTER_INCLUDEFILES_H
#define BLOCK_SORTER_INCLUDEFILES_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <map>
#include <unordered_map>

enum COLORS { YELLOW = 0, PURPLE, GREEN, BLUE, ORANGE, BROWN, RED, WHITE };

struct orderType {
    int x, y, id, way;
};

extern int cnt[7];
extern std::vector<orderType> order;
extern std::vector<std::vector<orderType>> orders;

#endif  // BLOCK_SORTER_INCLUDEFILES_H
