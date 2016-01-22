// Name:        qtree_test
// Description: qtree test
// Author:      zhangshiguang
// Email:       andyzhshg@gmail.com
// Create:      2015-12

#include "qtree.h"

#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

using namespace tml;

double distance(double x0, double y0, double x1, double y1) {
    return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

double frand(double a, double b) {
    return a + (double)rand() / RAND_MAX * (b - a);
}

int main() {
    /*
           (0, 0)--------------(10000, 0)
             |   x           x     |
             |       x           x |
             |                     |
             |   x           x     |
             |      x              |
             |  x              x  x|
             |  x       x          |
        (0, 10000)-----------(10000, 10000)
    */
    qtree<double, int> tree(0, 0, 10000, 10000);

    srand(time(NULL));
    int fail_cnt = 0;
    for (int i = 0; i < 100000; i++) {
        double x = frand(0, 10000);
        double y = frand(0, 10000);
        int data = i;
        if (!tree.add_node(x, y, data)) {
            fail_cnt += 1;
        }
    }
    
    printf("%d failed\n", fail_cnt);

    for (int i = 0; i < 10; i++) {
        double x = frand(0, 10000);
        double y = frand(0, 10000);
        double r = frand(1, 50);
        printf("------------------------\n");
        printf("i=%d, x=%f, y=%f, r=%f\n", i, x, y, r);
        std::vector<const qtree<double, int>::node_type *> result;
        tree.search(x, y, r, result);
        printf("result count: %d\n", result.size());
        for (int j = 0; j < result.size(); j++) {
            printf("x=%f, y=%f, v=%d, d=%f\n", result[j]->x, result[j]->y, result[j]->data, 
                distance(x, y, result[j]->x, result[j]->y));
        }
        printf("------\n");
        const qtree<double, int>::node_type *node = tree.find_nearest(x, y, r);
        if (node) {
            printf("r=%f, x=%f, y=%f, v=%d, d=%f\n", r, node->x, node->y, node->data, 
                distance(x, y, node->x, node->y));        
        }
    }
    return 0;
}
