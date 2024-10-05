#pragma once

#include <vector>

// 2D�̍��W��\���\����
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

// RRT�N���X
class RRT {
public:
    // �R���X�g���N�^
    RRT(const tPoint& start, const tPoint& goal,double min_x, double min_y, double max_x, double max_y, double max_step, double goal_radius, int max_iterations);

    // �o�H���v�悷��֐�
    std::vector<tPoint> plan_();
    std::vector<Node> getTree_();
    void setStartPoint(tPoint start);
    void setGoalPoint(tPoint goal);
    void setObstacles_(std::vector<tCircleObstacle>circle_obstacles, double obstacle_margin);    //�~�̏�Q����ݒ�

private:
    bool isCollisionObstacle_(tPoint point);
    const double GOAL_RATE = 0.3;   //random�ȓ_�ŃS�[�����I�΂��m��
    tPoint start_, goal_;
    double min_x_,min_y_,max_x_, max_y_, max_step_, goal_radius_;
    int max_iterations_;
    std::vector<Node> tree_;

    //��Q���֘A
    std::vector<tCircleObstacle> circle_obstacles_;
    double obstacle_margin_;

    // �����_���ȓ_�𐶐�
    tPoint randomPoint_();

    // �����_���ȓ_�ɍł��߂������̃m�[�h��������
    int nearestNodeIndex_(const tPoint& p);

    // �ł��߂��m�[�h����w�肵���X�e�b�v�Ń����_���ȓ_�Ɍ�����
    tPoint step_(const tPoint& nearest, const tPoint& random_point);

    // �S�[���܂ł̌o�H�𒊏o
    std::vector<tPoint> extractPath_();
};


