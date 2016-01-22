// Name:        qtree
// Description: quadtree for spatial point search
// Author:      zhangshiguang
// Email:       andyzhshg@gmail.com
// Create:      2015-12

#ifndef __TML_QTREE__
#define __TML_QTREE__

#include <cstddef>
#include <vector>
#include <limits>
#include <cmath>

namespace tml {
    
template<typename COOR_TYPE, typename DATA_TYPE> class qtree;

//qtree node define
template<typename COOR_TYPE, typename DATA_TYPE> class node {
public:
    COOR_TYPE x;        //coordinate x
    COOR_TYPE y;        //coordinate y
    DATA_TYPE data;     //user defined data
private:
    friend class qtree<COOR_TYPE, DATA_TYPE>;
    explicit node(COOR_TYPE left, COOR_TYPE top, COOR_TYPE right, COOR_TYPE bottom) {
        x = left;
        y = top;
        x1 = right;
        y1 = bottom;
        leaf = false;
        child[0] = child[1] = child[2] = child[3] = NULL;
        node_size = 0;
    }
    explicit node(COOR_TYPE x, COOR_TYPE y, DATA_TYPE data) {
        this->x = x;
        this->y = y;
        this->data = data;
        leaf = true;
    }
    COOR_TYPE x1;
    COOR_TYPE y1;
    node *child[4];
    bool leaf;
    size_t node_size;
};

//qtree define
template<typename COOR_TYPE, typename DATA_TYPE> class qtree {
public:
    typedef node<COOR_TYPE, DATA_TYPE> node_type;
   
    //create qtree by specify 4 corners of a rectangle bound
    explicit qtree(COOR_TYPE left, COOR_TYPE top, COOR_TYPE right, COOR_TYPE bottom) {
        root = new node_type(left, top, right, bottom);
    }
    
    ~qtree() {
        release_node(root);
    }
    
    //add a node
    bool add_node(COOR_TYPE x, COOR_TYPE y, DATA_TYPE data) {
        return add_node(root, x, y, data);
    }
    
    //search for the nearest node 
    const node_type *find_nearest(COOR_TYPE x, COOR_TYPE y, COOR_TYPE radius) const {
        std::vector<const node_type *> result;
        if (search(x, y, radius, result)) {
            int index = 0;
            COOR_TYPE d = (result[0]->x - x) * (result[0]->x - x) + (result[0]->y - y) * (result[0]->y - y);
            for (int i = 1; i < result.size(); i++) {
                COOR_TYPE d1 = (result[i]->x - x) * (result[i]->x - x) + (result[i]->y - y) * (result[i]->y - y);
                if (d1 < d) {
                    d = d1;
                    index = i;
                }
            }
            return result[index];
        } else {
            return NULL;            
        }
    }
    
    //search for nodes in a specified circle
    bool search(COOR_TYPE x, COOR_TYPE y, COOR_TYPE radius, std::vector<const node_type *> &result) const {
        result.clear();
        search(root, x, y, radius, result);
        return !result.empty();
    }
    
private:
    node_type *root;

    qtree(const qtree &other) {}        //noncopyable
    qtree& operator=(const qtree&) {}
     
    bool in_reach(COOR_TYPE x0, COOR_TYPE y0, COOR_TYPE x1, COOR_TYPE y1, COOR_TYPE r) const {
        return (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) <= r * r; 
    }
    
    bool intersect(const node_type *n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE r) const {
        COOR_TYPE x0 = (n->x + n->x1) / 2;
        COOR_TYPE y0 = (n->y + n->y1) / 2;
        COOR_TYPE vx = x > x0 ? x - x0 : x0 - x;
        COOR_TYPE vy = y > y0 ? y - y0 : y0 - y;
        COOR_TYPE hx = n->x > x0 ? n->x - x0 : x0 - n->x;
        COOR_TYPE hy = n->y > y0 ? n->y - y0 : y0 - n->y;
        COOR_TYPE ux = vx > hx ? vx - hx : 0;
        COOR_TYPE uy = vy > hy ? vy - hy : 0;
        return ux * ux + uy * uy <= r * r;
    }
    
    void search(const node_type *n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE radius, std::vector<const node_type *> &result) const {
        if (n != NULL) {
            if (n->leaf) {
                if (in_reach(n->x, n->y, x, y, radius)) {                
                    result.push_back(n);
                }
            } else if (intersect(n, x, y, radius) && n->node_size > 0) {
                for (int i = 0; i < 4; i++) {
                    if (n->child[i]) {
                        search(n->child[i], x, y, radius, result);                                        
                    }
                }
            }
        }
    }
    
    bool between(COOR_TYPE a, COOR_TYPE b, COOR_TYPE x) {
        return a <= b ? (a <= x && x < b) : (a >= x && x > b);
    }
    
    bool inbound(COOR_TYPE left, COOR_TYPE top, COOR_TYPE right, COOR_TYPE bottom, COOR_TYPE x, COOR_TYPE y) {
        return between(left, right, x) && between(top, bottom, y);
    }
    
    bool equal(COOR_TYPE a, COOR_TYPE b) {
        return a == b || ((std::fabs(a - b) < 1 - std::numeric_limits<COOR_TYPE>::epsilon())
            && (std::fabs(a - b) < std::numeric_limits<COOR_TYPE>::epsilon() * std::fabs(a + b)
            || std::fabs(a - b) < std::numeric_limits<COOR_TYPE>::min()));
    }
    
    bool add_node(node_type *n, COOR_TYPE x, COOR_TYPE y, DATA_TYPE data) {
        if (!inbound(n->x, n->y, n->x1, n->y1, x, y)) {
            return false;
        }
        COOR_TYPE mid_x = (n->x + n->x1) / 2;
        COOR_TYPE mid_y = (n->y + n->y1) / 2;
        COOR_TYPE left, top, right, bottom;
        int index = -1;
        if (inbound(n->x, n->y, mid_x, mid_y, x, y)) {
            left = n->x;
            top = n->y;
            right = mid_x;
            bottom = mid_y;
            index = 0;
        } else if (inbound(mid_x, n->y, n->x1, mid_y, x, y)) {
            left = mid_x;
            top = n->y;
            right = n->x1;
            bottom = mid_y;
            index = 1;
        } else if (inbound(mid_x, mid_y, n->x1, n->y1, x, y)) {
            left = mid_x;
            top = mid_y;
            right = n->x1;
            bottom = n->y1;
            index = 2;
        } else {
            left = n->x;
            top = mid_y;
            right = mid_x;
            bottom = n->y1;
            index = 3;
        }
        if (equal(left, right) || equal(top, bottom)) {
            return false;
        }
        if (n->child[index]) {
            if (n->child[index]->leaf) {
                if (equal(n->child[index]->x, x) && equal(n->child[index]->y, y)) {
                    return false;
                }
                node_type *new_node = new node_type(left, top, right, bottom);
                if (add_node(new_node, n->child[index]->x, n->child[index]->y, n->child[index]->data)
                    && add_node(new_node, x, y, data)) {
                    release_node(n->child[index]);
                    n->child[index] = new_node;
                    n->node_size += 1;
                    return true;
                } else {
                    release_node(new_node);
                    return false;
                }
            } else {
                if (add_node(n->child[index], x, y, data)) {
                    n->node_size += 1;
                    return true;
                } else {
                    return false;
                }
            }
        } else {
            n->child[index] = new node_type(x, y, data);
            n->node_size += 1;
            return true;
        }
    }
    
    void release_node(node_type *node) {
        if (node) {
            if (!node->leaf) {
                for (int i = 0; i < 4; i++) {
                    release_node(node->child[i]);
                }                
            }
            delete node;            
        }
    }
};

}

#endif //__TML_QTREE__
