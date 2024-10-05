#include <GL/glut.h>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <string>
#include "RRT.h"

#define MIN_X (-10)
#define MIN_Y (-10)
#define MAX_X (10)
#define MAX_Y (10)
#define MARGIN (1)  //画面の余白
#define CIRCLE_RESOLUTION (32)  //円の分解能
#define PATH_POINT_SIZE (0.12)    //経路の点の半径
#define NODE_POINT_SIZE (0.08)    //ノードの点の半径

const double PI = 3.14159265359;

int window_width = 640;
int window_height = 640;

double blue[3] = { 0.0,0.0,1.0 };
double obstacle_cololr[3] = { 0.2,0.2,0.2 };

//RRTの設定
tPoint start = { -5, -8 };
tPoint goal = { 9, 9 };
double max_step = 0.3;
double goal_radius = 0.5;
int max_iterations = 1000;

RRT rrt(start, goal, MIN_X, MIN_Y, MAX_X, MAX_Y, max_step, goal_radius, max_iterations);

std::vector<tPoint> path;

//障害物の設定
double obstacle_margin = 0.3;
tCircleObstacle obstacle1 = { 1.0,1.0,2.0 };
tCircleObstacle obstacle2 = { -1.0,-3.0,1.0 };

std::vector<tCircleObstacle>circle_obstacles = { obstacle1,obstacle2 };




void renderBitmapString(float x, float y, const char* string) {
    glRasterPos2f(x, y);  // 描画位置を設定
    while (*string) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *string);  // 文字を描画
        string++;
    }
}


void drawGrid() {
    glLineWidth(1.0);
    glColor3d(0.0, 0.0, 0.0);

    glBegin(GL_LINES); {
        //横線
        for (int i = -MAX_X; i <= MAX_X; i++) {
            glVertex2d(i, MAX_Y);
            glVertex2d(i, -MAX_Y);
        }
        //縦線
        for (int i = -MAX_Y; i <= MAX_Y; i++) {
            glVertex2d(-MAX_X,i);
            glVertex2d(MAX_X, i);
        }
    }
    glEnd();

    //軸は太く
    glLineWidth(3.0);
    glBegin(GL_LINES); {
        //X軸
        glVertex2d(-MAX_X, 0);
        glVertex2d(MAX_X, 0);

        //Y軸
        glVertex2d(0, MAX_Y);
        glVertex2d(0, -MAX_Y);

    }
    glEnd();
}

void drawCircle(double x, double y, double r) {
    glPointSize(3.0);
    glColor3d(0.0, 0.0, 0.0);

    glBegin(GL_POLYGON);
    glVertex2d(x, y);
    for (int i = 0; i <= CIRCLE_RESOLUTION; i++) {
        double  point_x = x + r * cos(2 * PI * i /(double)CIRCLE_RESOLUTION);
        double  point_y = y + r * sin(2 * PI * i /(double)CIRCLE_RESOLUTION);
        glVertex2d(point_x, point_y);
    }

    glEnd();
}

void drawCircle(double x, double y, double r,double* color) {
    glPointSize(3.0);
    glColor3d(color[0], color[1], color[2]);

    glBegin(GL_POLYGON);
    glVertex2d(x, y);
    for (int i = 0; i <= CIRCLE_RESOLUTION; i++) {
        double  point_x = x + r * cos(2 * PI * i / (double)CIRCLE_RESOLUTION);
        double  point_y = y + r * sin(2 * PI * i / (double)CIRCLE_RESOLUTION);
        glVertex2d(point_x, point_y);
    }

    glEnd();
}

void drawPath(std::vector<tPoint>result_path,double* path_color) {
    
    if (!result_path.empty()) {
        for (int i = 0; i < result_path.size(); i++) {
            drawCircle(result_path[i].x, result_path[i].y, PATH_POINT_SIZE, path_color);
        }
        glColor3d(path_color[0], path_color[1], path_color[2]);
        for (int i = 0; i < result_path.size() - 1; i++) {
            glBegin(GL_LINES); {
                glVertex2d(result_path[i].x, result_path[i].y);
                glVertex2d(result_path[i + 1].x, result_path[i + 1].y);
            }
            glEnd();
        }
    }
    else {
        std::cout << "パスが見つかりませんでした" << std::endl;
    }
}

void drawAllNodePoint() {
    auto tree = rrt.getTree_();
    for (int i = 0; i < tree.size(); i++) {
        drawCircle(tree[i].point.x, tree[i].point.y, NODE_POINT_SIZE);
    }
    std::cout << "Num of Points:" << tree.size() << std::endl;
}

void drawCircleObstacle(std::vector<tCircleObstacle>obstacles) {

    for (int i = 0; i < obstacles.size(); i++) {
        drawCircle(obstacles[i].x, obstacles[i].y, obstacles[i].radius, obstacle_cololr);
    }
}

void display(void)
{
    int i;
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3d(1.0, 0.0, 0.0);
    drawGrid();
    //drawCircle(0.0, 0.0,1.0,blue);
    drawPath(path,blue);
    drawCircleObstacle(circle_obstacles);
    drawAllNodePoint();
    glFlush();
}

void resize(int w, int h)
{
    glViewport(0, 0, w, h);

    window_width = w;
    window_height = h;

    glLoadIdentity();
    /* スクリーン上の表示領域をビューポートの大きさに比例させる */
    glOrtho(-MAX_X-MARGIN, MAX_X+MARGIN, -MAX_Y - MARGIN, MAX_Y+MARGIN, -1.0, 1.0);
}

void init(void)
{
    glClearColor(1.0, 1.0, 1.0, 1.0);

    rrt.setObstacles_(circle_obstacles,obstacle_margin);
    path = rrt.plan_();

    if (!path.empty()) {
        std::cout << "Path found: \n";
        for (const auto& p : path) {
            std::cout << "(" << p.x << ", " << p.y << ")\n";
        }
    }
    else {
        std::cout << "No path found within the iteration limit.\n";
    }
}

int main(int argc, char* argv[])
{
    glutInitWindowSize(window_width, window_height);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(display);
    glutReshapeFunc(resize);
    init();
    glutMainLoop();
    return 0;
}