#include "RRT.h"
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <algorithm>

// コンストラクタ
RRT::RRT(const tPoint& start, const tPoint& goal,double min_x, double min_y, double max_x, double max_y, double max_step, double goal_radius, int max_iterations)
    : start_(start), goal_(goal),min_x_(min_x),min_y_(min_y), max_x_(max_x), max_y_(max_y), max_step_(max_step), goal_radius_(goal_radius), max_iterations_(max_iterations) 
{
    tree_.clear();
    tree_.push_back({ start_, -1 });
    circle_obstacles_.clear();
}

// ランダムな点を生成
tPoint RRT::randomPoint_() {
    //printf("%.3f , %.3f\n ", (max_x_ - min_x_) * ((double)rand() / RAND_MAX) + min_x_, (max_y_ - min_y_) * ((double)rand() / RAND_MAX) + min_y_);
    if (((double)rand() / RAND_MAX) <= GOAL_RATE) {
        return { goal_.x,goal_.y };
    }
    return { (max_x_- min_x_) * ((double)rand() / RAND_MAX) + min_x_ , (max_y_- min_y_) * ((double)rand() / RAND_MAX) + min_y_};

}

// ランダムな点に最も近い既存のノードを見つける
int RRT::nearestNodeIndex_(const tPoint& p) {
    int nearestIdx = 0;
    double nearestDist = std::sqrt(std::pow(tree_[0].point.x - p.x, 2) + std::pow(tree_[0].point.y - p.y, 2));
    for (int i = 1; i < tree_.size(); i++) {
        double dist = std::sqrt(std::pow(tree_[i].point.x - p.x, 2) + std::pow(tree_[i].point.y - p.y, 2));
        if (dist < nearestDist) {
            nearestIdx = i;
            nearestDist = dist;
        }
    }
    return nearestIdx;
}

// 最も近いノードから指定したステップでランダムな点に向かう
tPoint RRT::step_(const tPoint& nearest, const tPoint& randomPoint) {
    double theta = std::atan2(randomPoint.y - nearest.y, randomPoint.x - nearest.x);
    return { nearest.x + max_step_ * std::cos(theta), nearest.y + max_step_ * std::sin(theta) };
}

// ゴールまでの経路を抽出
std::vector<tPoint> RRT::extractPath_() {
    std::vector<tPoint> path;
    int idx = tree_.size() - 1;
    while (idx != -1) {
        path.push_back(tree_[idx].point);
        idx = tree_[idx].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// 経路を計画する
std::vector<tPoint> RRT::plan_() {
    bool is_collision = false;

    for (int i = 0; i < max_iterations_; i++) {
        tPoint randPoint = randomPoint_();
        int nearestIdx = nearestNodeIndex_(randPoint);
        tPoint newPoint = step_(tree_[nearestIdx].point, randPoint);

        is_collision = isCollisionObstacle_(newPoint);
        if (!is_collision) {    //新たな点が障害物に衝突しないなら
            // 新しいノードを木に追加
            tree_.push_back({ newPoint, nearestIdx });

            // ゴールに近づいたか確認
            if (std::sqrt(std::pow(newPoint.x - goal_.x, 2) + std::pow(newPoint.y - goal_.y, 2)) < goal_radius_) {
                tree_.push_back({ goal_, (int)tree_.size() - 1 });
                return extractPath_();
            }
        }

    }
    return std::vector<tPoint>();  // ゴールに到達できなかった場合、空の経路を返す
}

std::vector<Node> RRT::getTree_() {
    return tree_;
}

void RRT::setObstacles_(std::vector<tCircleObstacle>circle_obstacles,double obstacle_margin) {
    /*std::copy(circle_obstacles.begin(), circle_obstacles.end(), circle_obstacles_.begin());*/
    circle_obstacles_ = circle_obstacles;
    obstacle_margin_ = obstacle_margin;
}

bool RRT::isCollisionObstacle_(tPoint point) {
    if (circle_obstacles_.empty()) {    //障害物設定がされてないなら
        return false;
    }

    for (int i = 0; i < circle_obstacles_.size(); i++) {
        double point_center_dist = pow(point.x - circle_obstacles_[i].x, 2) + pow(point.y - circle_obstacles_[i].y, 2);

        if (point_center_dist < pow(circle_obstacles_[i].radius + obstacle_margin_,2)) {    //点と中心の距離が半径以下なら
            return true;
        }
    }
    return false;
}