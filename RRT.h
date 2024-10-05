#pragma once

#include <vector>

// 2Dの座標を表す構造体
struct tPoint {
    double x, y;
};

struct tCircleObstacle {
    double x, y,radius;
};

struct Node {
    tPoint point;
    int parent;
};

// RRTクラス
class RRT {
public:
    // コンストラクタ
    RRT(const tPoint& start, const tPoint& goal,double min_x, double min_y, double max_x, double max_y, double max_step, double goal_radius, int max_iterations);

    // 経路を計画する関数
    std::vector<tPoint> plan_();
    std::vector<Node> getTree_();
    void setStartPoint(tPoint start);
    void setGoalPoint(tPoint goal);
    void setObstacles_(std::vector<tCircleObstacle>circle_obstacles, double obstacle_margin);    //円の障害物を設定

private:
    bool isCollisionObstacle_(tPoint point);
    const double GOAL_RATE = 0.3;   //randomな点でゴールが選ばれる確率
    tPoint start_, goal_;
    double min_x_,min_y_,max_x_, max_y_, max_step_, goal_radius_;
    int max_iterations_;
    std::vector<Node> tree_;

    //障害物関連
    std::vector<tCircleObstacle> circle_obstacles_;
    double obstacle_margin_;

    // ランダムな点を生成
    tPoint randomPoint_();

    // ランダムな点に最も近い既存のノードを見つける
    int nearestNodeIndex_(const tPoint& p);

    // 最も近いノードから指定したステップでランダムな点に向かう
    tPoint step_(const tPoint& nearest, const tPoint& random_point);

    // ゴールまでの経路を抽出
    std::vector<tPoint> extractPath_();
};


