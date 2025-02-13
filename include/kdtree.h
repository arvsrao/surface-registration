//
// Created by Arvind Rao on 04.04.21.
//
#ifndef PLAYBACK_KDTREE3D_H
#define PLAYBACK_KDTREE3D_H

#include <vector>
#include <unordered_set>

using PointType = std::vector<float>;
using PointCollectionType = std::vector<PointType>;

// Structure to represent node of kd tree
    struct Node
    {
        std::vector<float> point;
        int id;
        Node* left;
        Node* right;

        Node(std::vector<float> arr, int setId)
                :	point(arr), id(setId), left(nullptr), right(nullptr)
        {}

        ~Node()
        {
            delete left;
            delete right;
        }
    };

    template <uint N> // N is the dimension of the data
    struct KdTree
    {
        Node* root;

        KdTree() : root(nullptr) {}

        KdTree(PointCollectionType &points) : root(nullptr) {

            // create nodes from the given point sequence
            std::vector<Node*> nodes;
            int idx = 0;
            for (auto &point : points) nodes.push_back(new Node(point, idx++));

            // insert nodes into the tree in such a way that the tree remains balanced.
            root = recursiveMedianBatchInsert( nodes, 0);
        }

        ~KdTree()
        {
            delete root;
        }

        Node* recursiveMedianBatchInsert(std::vector<Node*>& seq, int depth) {

            if (seq.size() == 0)
                return nullptr;
            else if (seq.size() == 1) {
                return seq.front();
            } else {

                // Using lambda function here, to define comparison function
                // TODO: use instead in-place order statistics sort to only sort the median into its final place.
                std::sort(seq.begin(), seq.end(),
                          [&](Node *a, Node *b) { return a->point[depth % N] < b->point[depth % N]; });

                // fill left and right collections
                std::vector<Node*> left, right;
                int medianIdx = std::ceil(float(seq.size()) / float(2));
                for (int i = 0; i < medianIdx; i++) left.push_back(seq[i]);
                for (int i = medianIdx + 1; i < seq.size(); i++) right.push_back(seq[i]);

                seq[medianIdx]->left = recursiveMedianBatchInsert(left, depth + 1);
                seq[medianIdx]->right = recursiveMedianBatchInsert(right, depth + 1);

                return seq[medianIdx];
            }
        }

        /** helper function inserts node into tree */
        void insertNode(Node** parent, Node* newNode, uint depth) {

            if (*parent == nullptr) {
                *parent = newNode;
                return;
            }

            // based on depth compare x or y coordinate
            uint coordIdx = depth % N;

            // left - right based on y-value
            if (newNode->point.at(coordIdx) >= (*parent)->point.at(coordIdx)) {
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
        void insert(std::vector<float> point, int id)
        {
            // create a new node
            Node* newNode = new Node(std::move(point), id);

            // place the newly created node correctly with in the root
            insertNode(&root, newNode, 0);
        }

        /** helper function searches tree for points in the neighborhood of a target point. */
        void searchTree(Node* parent, std::vector<Node*>& ids, std::vector<float> target, float distanceTol, uint depth) {

            if (parent == nullptr) {
                return;
            }

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
                    inBox = inBox && parent->point[otherId] > (target[otherId] - distanceTol) &&
                            parent->point[otherId] < (target[otherId] + distanceTol);
                }

                if (inBox) {
                    // parent might be in the distanceTol-Disc around target point, so test it.
                    float dist = 0;
                    for (int i = 0; i < N; i++) dist += (parent->point[i] - target[i]) * (parent->point[i] - target[i]);

                    if (dist < distanceTol * distanceTol)
                        ids.push_back(parent);
                }

                //Otherwise, continue traversing the tree.
                searchTree(parent->left, ids, target, distanceTol, depth + 1);
                searchTree(parent->right, ids, target, distanceTol, depth + 1);
            }
        }

        // return a list of point ids in the tree that are within distance of target
        std::vector<Node*> search(std::vector<float> target, float distanceTol)
        {
            std::vector<Node*> nodePointers;

            searchTree(root, nodePointers, std::move(target), distanceTol, 0);

            return nodePointers;
        }

        /** euclidean clustering helper method */
        void proximity(const Node* node, std::unordered_set<int>& processed, std::vector<int>& cluster, float distanceTol)
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

        void clusterTraverse(Node* node, std::unordered_set<int>& processed, std::vector<std::vector<int>>& clusters, float distanceTol)
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