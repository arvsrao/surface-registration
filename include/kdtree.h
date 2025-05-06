//
// Created by Arvind Rao on 04.04.21.
//
#ifndef PLAYBACK_KDTREE3D_H
#define PLAYBACK_KDTREE3D_H

#include <vector>
#include <queue>
#include <Eigen/Core>
#include <unordered_set>

template <uint N>
using PointType = Eigen::Matrix<double, N, 1>;

template <uint N>
using PointCollectionType = std::vector<PointType<N>>;

// Structure to represent node of kd tree
    template<uint N>
    struct Node
    {
        PointType<N> point;
        int id; // face reference
        Node<N>* left;
        Node<N>* right;

        Node(PointType<N>& arr, const int _id) : point(arr), id(_id), left(nullptr), right(nullptr) {}
        Node(PointType<N>&& arr, const int _id) : point(arr), id(_id), left(nullptr), right(nullptr) {}

        Node(const Node<N>&& other) {
            delete left, right;
            left = other.left;
            right = other.right;
            id = other.id;
            other.left = nullptr;
            other.right = nullptr;
            other.id = -1;
        }

        Node<N>& operator=(const Node<N>&& other) {
            if (this == &other) {
                delete left, right;
                left = other.left;
                right = other.right;
                id = other.id;
                other.left = nullptr;
                other.right = nullptr;
                other.id = -1;
            }
            return *this;
        }

        ~Node() {
            delete left;
            delete right;
        }
    };

    template<uint N>
    struct NodeWrapper {
        double key;
        Node<N>* node;

        NodeWrapper(double _key, Node<N>* _node) : key(_key), node(_node) {}

        ~NodeWrapper() { delete node; }
    };

    template <uint N> // N is the dimension of the data
    struct KdTree
    {
        Node<N>* root;

        KdTree() : root(nullptr) {}

        KdTree(PointCollectionType<N> &points) : root(nullptr) {

            // create nodes from the given point sequence
            std::vector<Node<N>*> nodes;
            int idx = 0;
            for (auto &point : points) nodes.push_back(new Node(point, idx++));

            // insert nodes into the tree in such a way that the tree remains balanced.
            root = recursiveMedianBatchInsert( nodes, 0);
        }

        KdTree(std::vector<Node<N>*> nodes) : root(recursiveMedianBatchInsert( nodes, 0)) {}

        ~KdTree()
        {
            delete root;
        }

        Node<N>* recursiveMedianBatchInsert(std::vector<Node<N>*>& seq, int depth) {

            if (seq.size() == 0)
                return nullptr;
            else if (seq.size() == 1) {
                return seq.front();
            } else {

                // Using lambda function here, to define comparison function
                // TODO: use instead in-place order statistics sort to only sort the median into its final place.
                std::sort(seq.begin(), seq.end(),
                          [&](Node<N> *a, Node<N> *b) { return a->point[depth % N] < b->point[depth % N]; });

                // fill left and right collections
                std::vector<Node<N>*> left, right;
                int medianIdx = std::ceil(float(seq.size()) / float(2));
                for (int i = 0; i < medianIdx; i++) left.push_back(seq[i]);
                for (int i = medianIdx + 1; i < seq.size(); i++) right.push_back(seq[i]);

                seq[medianIdx]->left = recursiveMedianBatchInsert(left, depth + 1);
                seq[medianIdx]->right = recursiveMedianBatchInsert(right, depth + 1);

                return seq[medianIdx];
            }
        }

        /** helper function inserts node into tree */
        void insertNode(Node<N>** parent, Node<N>* newNode, uint depth) {

            if (*parent == nullptr) {
                *parent = newNode;
                return;
            }

            // based on depth compare x or y coordinate
            uint coordIdx = depth % N;

            // left - right based on y-value
            if (newNode->point(coordIdx) >= (*parent)->point(coordIdx)) {
                insertNode(&((*parent)->right), newNode, depth + 1);
            }
            else {
                insertNode(&((*parent)->left), newNode, depth + 1);
            }
        }

        /**
         * Insert a new point into the tree
         *
         * @param point
         * @param id
         */
        void insert(PointType<N>& point, int id)
        {
            // create a new node
            Node<N>* newNode = new Node<N>(point, id);

            // place the newly created node correctly with in the root
            insertNode(&root, newNode, 0);
        }

        /** helper function for kNearestNeighborsSearch in the neighborhood of a target point. */
        void kNearestNeighborsSearch(Node<N>* parent, std::priority_queue<NodeWrapper<N>>& neighbors, PointType<N> target, uint depth, uint K) {

            if (parent == nullptr) return;

            // based on depth compare x0, x1, x2, ... coordinate
            uint coordIdx = depth % N;

            neighbors.push(NodeWrapper<N>((parent - target).dot(parent - target), parent));
            if (neighbors.size() > K) neighbors.pop();

            // test if parent is in box
            if (parent->point[coordIdx] < target[coordIdx]) {
                kNearestNeighborsSearch(parent->right, neighbors, target, depth + 1, K);

                // the left subtree might have points less than current max distance from target
                if (neighbors.top().key > std::abs(parent[coordIdx] - target[coordIdx]))
                    kNearestNeighborsSearch(parent->left, neighbors, target, depth + 1, K);
            }
            else {
                kNearestNeighborsSearch(parent->left, neighbors, target, depth + 1, K);

                // the right subtree might have points less than current max distance from target
                if (neighbors.top().key > std::abs(parent[coordIdx] - target[coordIdx]))
                    kNearestNeighborsSearch(parent->right, neighbors, target, depth + 1, K);
            }
        }

        /** helper function searches tree for points in the neighborhood of a target point. */
        void searchTree(Node<N>* parent, std::vector<Node<N>*>& ids, PointType<N> target, float distanceTol, uint depth) {

            if (parent == nullptr) return;

            // based on depth compare x or y coordinate
            uint coordIdx = depth % N;

            // compare target's x-range or y-range
            float lowerBound = target[coordIdx] - distanceTol;
            float upperBound = target[coordIdx] + distanceTol;

            // test if parent is in box
            if(parent->point[coordIdx] < lowerBound) {
                searchTree(parent->right, ids, target, distanceTol, depth + 1);
            }
            else if (parent->point[coordIdx] > upperBound) {
                searchTree(parent->left, ids, target, distanceTol, depth + 1);
            }
            else {
                // check the other dimensions (coordIdx + 1 mod 3) to complete the check that parent is in the 2*distanceTol
                // square neighborhood of target.
                bool inBox = true;
                for (int i = 1; i < N; i++) {
                    uint otherId = (coordIdx + i) % N;
                    lowerBound = target[otherId] - distanceTol;
                    upperBound = target[otherId] + distanceTol;
                    inBox = parent->point[otherId] > lowerBound && parent->point[otherId] < upperBound;
                    if (!inBox) break;
                }

                if (inBox) {
                    // parent might be in the distanceTol-Disc around target point, so test it.
                    float dist = 0;
                    for (int i = 0; i < N; i++) dist += (parent->point[i] - target[i]) * (parent->point[i] - target[i]);

                    if (dist < distanceTol * distanceTol) ids.push_back(parent);
                }

                //Otherwise, continue traversing the tree.
                searchTree(parent->left, ids, target, distanceTol, depth + 1);
                searchTree(parent->right, ids, target, distanceTol, depth + 1);
            }
        }

        // return a list of point ids in the tree that are within distance of target
        std::vector<Node<N>*> search(PointType<N>& target, float distanceTol)
        {
            std::vector<Node<N>*> nodePointers;

            searchTree(root, nodePointers, target, distanceTol, 0);

            return nodePointers;
        }

        // return a list of point ids in the tree that are within distance of target
        std::vector<Node<N>*> kNearestNeighbors(int K, PointType<N>& target)
        {
            std::priority_queue<NodeWrapper<N>*> nodes;

            kNearestNeighborsSearch(root, nodes, target, 0, K);

            return nodes;
        }

        /** Euclidean clustering helper method */
        void proximity(const Node<N>* node, std::unordered_set<int>& processed, std::vector<int>& cluster, float distanceTol)
        {
            processed.insert(node->id);
            cluster.push_back(node->id);
            auto nearby = this->search(node->point, distanceTol);

            for (auto nodePtr : nearby) {
                if (!processed.count(nodePtr->id)) {
                    proximity(nodePtr, processed, cluster, distanceTol);
                }
            }
        }

        void clusterTraverse(Node<N>* node, std::unordered_set<int>& processed, std::vector<std::vector<int>>& clusters, float distanceTol)
        {
            if (node != nullptr) {
                clusterTraverse(node->left, processed, clusters, distanceTol);
                if (!processed.count(node->id)) {
                    std::vector<int> cluster;
                    proximity(node, processed, cluster, distanceTol);
                    clusters.push_back(cluster);
                }
                clusterTraverse(node->right, processed, clusters, distanceTol);
            }
        }

        std::vector<std::vector<int>> euclideanCluster(float distanceTol)
        {
            std::vector<std::vector<int>> clusters;
            std::unordered_set<int> processed;
            clusterTraverse(this->root, processed, clusters, distanceTol);

            return clusters;
        }
    };

#endif //PLAYBACK_KDTREE3D_H