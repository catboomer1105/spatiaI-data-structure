/*******************************************************************************************
* 八叉树 (Octree) 可视化演示
*
* 功能特性：
*   - 可视化展示八叉树的空间分割结构
*   - 支持动态添加/移除点
*   - 显示查询范围和查询结果
*   - 可切换不同视图模式
*
* 键盘控制：
*   W/A/S/D - 移动相机（第一人称模式）
*   鼠标 - 旋转视角
*   鼠标滚轮 - 缩放
*
*   空格键 - 添加随机点
*   X - 移除最近点
*   R - 重置场景
*
*   1 - 显示边界框
*   2 - 显示所有节点
*   3 - 只显示叶子节点
*   4 - 切换查询球体
*   Q - 执行范围查询
*   G - 切换网格显示
*
* 八叉树简介：
*   八叉树是一种常用的3D空间分区数据结构，通过递归地将3D空间划分为8个
*   子区域来组织空间数据。广泛应用于碰撞检测、视锥剔除、光线追踪、LOD等。
********************************************************************************************/

#include "raylib.h"
#include "rcamera.h"
#include "raymath.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <ctime>

//--------------------------------------------------------------------------------------
// 八叉树节点结构
//--------------------------------------------------------------------------------------
struct OctreeNode {
    Vector3 center;
    float halfSize;
    bool isLeaf;
    int children[8];
    std::vector<Vector3> points;

    OctreeNode(Vector3 c, float h) : center(c), halfSize(h), isLeaf(true) {
        for (int i = 0; i < 8; i++) children[i] = -1;
    }
};

//--------------------------------------------------------------------------------------
// 八叉树类
//--------------------------------------------------------------------------------------
// 八叉树类 - 用于3D空间分区管理
//--------------------------------------------------------------------------------------
class Octree {
private:
    std::vector<OctreeNode> nodes;      // 存储所有节点的数组
    int rootIndex;                       // 根节点在数组中的索引
    int maxDepth;                        // 八叉树的最大深度限制
    int maxPointsPerNode;                // 每个叶子节点最多容纳的点数
    Vector3 worldCenter;                 // 八叉树包围盒中心
    float worldHalfSize;                 // 八叉树包围盒半尺寸

    //----------------------------------------------------------------------
    // 根据点相对于节点中心的位置，计算该点所属的子节点索引(0-7)
    // x轴正方向=1, y轴正方向=2, z轴正方向=4, 组合得到0-7的索引
    //----------------------------------------------------------------------
    int getChildIndex(const Vector3& point, const Vector3& nodeCenter) {
        int index = 0;
        if (point.x > nodeCenter.x) index |= 1;
        if (point.y > nodeCenter.y) index |= 2;
        if (point.z > nodeCenter.z) index |= 4;
        return index;
    }

    //----------------------------------------------------------------------
    // 根据父节点信息和子节点索引，计算子节点的中心点位置
    // parentCenter: 父节点中心, parentHalfSize: 父节点半尺寸, childIndex: 子节点索引
    //----------------------------------------------------------------------
    Vector3 getChildCenter(const Vector3& parentCenter, float parentHalfSize, int childIndex) {
        Vector3 offset = {
            (childIndex & 1) ? parentHalfSize * 0.5f : -parentHalfSize * 0.5f,
            (childIndex & 2) ? parentHalfSize * 0.5f : -parentHalfSize * 0.5f,
            (childIndex & 4) ? parentHalfSize * 0.5f : -parentHalfSize * 0.5f
        };
        return Vector3Add(parentCenter, offset);
    }

    //----------------------------------------------------------------------
    // 递归插入点：将点插入到八叉树中
    // nodeIndex: 当前节点索引, point: 要插入的点, depth: 当前递归深度
    // 当叶子节点点数超过阈值或达到最大深度时，会触发细分
    //----------------------------------------------------------------------
    bool insertRecursive(int nodeIndex, const Vector3& point, int depth) {
        OctreeNode& node = nodes[nodeIndex];

        // 检查点是否在当前节点的包围盒内
        Vector3 diff = Vector3Subtract(point, node.center);
        if (fabsf(diff.x) > node.halfSize || fabsf(diff.y) > node.halfSize || fabsf(diff.z) > node.halfSize) {
            return false;
        }

        // 如果是叶子节点
        if (node.isLeaf) {
            // 如果节点未满或已达到最大深度，直接添加点
            if ((int)node.points.size() < maxPointsPerNode || depth >= maxDepth) {
                node.points.push_back(point);
                return true;
            }
            // 否则细分该节点，创建8个子节点
            subdivide(nodeIndex);
        }

        // 找到点所属的子节点，递归插入
        int childIndex = getChildIndex(point, node.center);
        if (node.children[childIndex] != -1) {
            return insertRecursive(node.children[childIndex], point, depth + 1);
        }
        return false;
    }

    //----------------------------------------------------------------------
    // 细分节点：将叶子节点分裂为8个子节点
    // 将当前叶子节点中存储的点重新分配到对应的子节点中
    //----------------------------------------------------------------------
    void subdivide(int nodeIndex) {
        OctreeNode& node = nodes[nodeIndex];
        node.isLeaf = false;

        // 创建8个子节点
        for (int i = 0; i < 8; i++) {
            Vector3 childCenter = getChildCenter(node.center, node.halfSize, i);
            nodes.emplace_back(childCenter, node.halfSize * 0.5f);
            node.children[i] = (int)nodes.size() - 1;
        }

        // 将当前节点存储的点重新分配到子节点
        for (const auto& point : node.points) {
            int childIndex = getChildIndex(point, node.center);
            if (node.children[childIndex] != -1) {
                insertRecursive(node.children[childIndex], point, 0);
            }
        }
        node.points.clear();
    }

    //----------------------------------------------------------------------
    // 递归删除点：从八叉树中移除指定的点
    // nodeIndex: 当前节点索引, point: 要删除的点, tolerance: 删除的容差范围
    //----------------------------------------------------------------------
    bool removeRecursive(int nodeIndex, const Vector3& point, float tolerance) {
        OctreeNode& node = nodes[nodeIndex];

        // 检查点是否可能在当前节点的子树下
        Vector3 diff = Vector3Subtract(point, node.center);
        if (fabsf(diff.x) > node.halfSize + tolerance ||
            fabsf(diff.y) > node.halfSize + tolerance ||
            fabsf(diff.z) > node.halfSize + tolerance) {
            return false;
        }

        // 如果是叶子节点，在其中查找并删除点
        if (node.isLeaf) {
            for (auto it = node.points.begin(); it != node.points.end(); ++it) {
                if (Vector3Distance(*it, point) < tolerance) {
                    node.points.erase(it);
                    return true;
                }
            }
            return false;
        }

        // 否则递归到对应的子节点中查找
        int childIndex = getChildIndex(point, node.center);
        if (node.children[childIndex] != -1) {
            return removeRecursive(node.children[childIndex], point, tolerance);
        }
        return false;
    }

    //----------------------------------------------------------------------
    // 检测球体与轴对齐包围盒(AABB)是否相交
    // 用于范围查询时快速排除不相交的节点
    //----------------------------------------------------------------------
    bool sphereIntersectsAABB(Vector3 sphereCenter, float sphereRadius,
                               Vector3 boxCenter, float boxHalfSize) {
        // 找到球心到包围盒最近点的距离
        float x = std::max(boxCenter.x - boxHalfSize, std::min(sphereCenter.x, boxCenter.x + boxHalfSize));
        float y = std::max(boxCenter.y - boxHalfSize, std::min(sphereCenter.y, boxCenter.y + boxHalfSize));
        float z = std::max(boxCenter.z - boxHalfSize, std::min(sphereCenter.z, boxCenter.z + boxHalfSize));

        float dx = sphereCenter.x - x;
        float dy = sphereCenter.y - y;
        float dz = sphereCenter.z - z;

        // 如果最近点在球体内，则相交
        return (dx * dx + dy * dy + dz * dz) <= (sphereRadius * sphereRadius);
    }

    //----------------------------------------------------------------------
    // 递归范围查询：查找所有在指定球体内的点
    // center: 球心, radius: 半径, results: 存储查询结果的容器
    //----------------------------------------------------------------------
    void queryRangeRecursive(int nodeIndex, Vector3 center, float radius, std::vector<Vector3>& results) {
        OctreeNode& node = nodes[nodeIndex];

        // 如果球体与当前节点不相交，跳过
        if (!sphereIntersectsAABB(center, radius, node.center, node.halfSize)) {
            return;
        }

        // 如果是叶子节点，遍历检查每个点
        if (node.isLeaf) {
            for (const auto& point : node.points) {
                if (Vector3Distance(point, center) <= radius) {
                    results.push_back(point);
                }
            }
        } else {
            // 否则递归检查所有子节点
            for (int i = 0; i < 8; i++) {
                if (node.children[i] != -1) {
                    queryRangeRecursive(node.children[i], center, radius, results);
                }
            }
        }
    }

    //----------------------------------------------------------------------
    // 递归查找最近点：找到距离目标点最近的点
    // 利用包围盒的层次结构进行剪枝，提高查询效率
    //----------------------------------------------------------------------
    void findNearestRecursive(int nodeIndex, const Vector3& target,
                               Vector3& nearest, float* minDist) {
        OctreeNode& node = nodes[nodeIndex];

        // 如果是叶子节点，直接遍历查找最近点
        if (node.isLeaf) {
            for (const auto& p : node.points) {
                float d = Vector3Distance(p, target);
                if (d < *minDist && d > 0.001f) {
                    *minDist = d;
                    nearest = p;
                }
            }
        } else {
            // 否则遍历子节点，但只检查可能包含更近点的子节点
            for (int i = 0; i < 8; i++) {
                if (node.children[i] != -1) {
                    OctreeNode& child = nodes[node.children[i]];
                    // 计算目标点到子节点包围盒的最近点
                    Vector3 closest = {
                        std::max(child.center.x - child.halfSize,
                                 std::min(target.x, child.center.x + child.halfSize)),
                        std::max(child.center.y - child.halfSize,
                                 std::min(target.y, child.center.y + child.halfSize)),
                        std::max(child.center.z - child.halfSize,
                                 std::min(target.z, child.center.z + child.halfSize))
                    };
                    float distToBox = Vector3Distance(closest, target);
                    // 只有当子节点可能包含更近的点时才递归
                    if (distToBox < *minDist) {
                        findNearestRecursive(node.children[i], target, nearest, minDist);
                    }
                }
            }
        }
    }

public:
    //----------------------------------------------------------------------
    // 构造函数：创建八叉树
    // center: 世界包围盒中心, halfSize: 世界包围盒半尺寸
    // maxPoints: 叶子节点最大点数, maxDepthValue: 最大深度
    //----------------------------------------------------------------------
    Octree(Vector3 center, float halfSize, int maxPoints = 4, int maxDepthValue = 6)
        : worldCenter(center), worldHalfSize(halfSize),
          maxPointsPerNode(maxPoints), maxDepth(maxDepthValue) {
        nodes.emplace_back(center, halfSize);  // 创建根节点
        rootIndex = 0;
    }

    //----------------------------------------------------------------------
    // 插入点：公共接口
    //----------------------------------------------------------------------
    bool insert(const Vector3& point) {
        return insertRecursive(rootIndex, point, 0);
    }

    //----------------------------------------------------------------------
    // 删除点：公共接口
    //----------------------------------------------------------------------
    bool remove(const Vector3& point, float tolerance = 0.1f) {
        return removeRecursive(rootIndex, point, tolerance);
    }

    //----------------------------------------------------------------------
    // 范围查询：查找球体内所有点
    //----------------------------------------------------------------------
    void queryRange(Vector3 center, float radius, std::vector<Vector3>& results) {
        queryRangeRecursive(rootIndex, center, radius, results);
    }

    // 获取所有节点
    const std::vector<OctreeNode>& getNodes() const { return nodes; }

    // 获取根节点
    OctreeNode& getRoot() { return nodes[rootIndex]; }

    // 获取节点总数
    int getNodeCount() const { return (int)nodes.size(); }

    // 获取所有节点中存储的点的总数
    int getPointCount() const {
        int count = 0;
        for (const auto& node : nodes) {
            count += (int)node.points.size();
        }
        return count;
    }

    // 获取叶子节点数量
    int getLeafCount() const {
        int count = 0;
        for (const auto& node : nodes) {
            if (node.isLeaf) count++;
        }
        return count;
    }

    //----------------------------------------------------------------------
    // 查找最近点：公共接口
    // 返回最近点，outDistance存储距离
    //----------------------------------------------------------------------
    Vector3 findNearest(const Vector3& point, float* outDistance) {
        Vector3 nearest = {0, 0, 0};
        float minDist = FLT_MAX;
        findNearestRecursive(rootIndex, point, nearest, &minDist);
        if (outDistance) *outDistance = minDist;
        return nearest;
    }

    //----------------------------------------------------------------------
    // 清空八叉树：重置为只有根节点的初始状态
    //----------------------------------------------------------------------
    void clear() {
        nodes.clear();
        nodes.emplace_back(worldCenter, worldHalfSize);
        rootIndex = 0;
    }
};

//--------------------------------------------------------------------------------------
// 全局变量
//--------------------------------------------------------------------------------------
const int screenWidth = 1280;
const int screenHeight = 720;

Octree* octree = nullptr;
std::vector<Vector3> queryResults;
Vector3 queryCenter = {0, 5, 0};
float queryRadius = 3.0f;
bool showQuerySphere = false;

int viewMode = 1;
bool showPoints = true;
bool showGrid = true;
int maxPointsPerNode = 4;

int totalNodes = 0;
int leafNodes = 0;
int totalPoints = 0;

//--------------------------------------------------------------------------------------
// 绘制函数
//--------------------------------------------------------------------------------------
void DrawBoundingBoxLines(Vector3 min, Vector3 max, Color color) {
    Vector3 corners[8] = {
        { min.x, min.y, min.z },
        { max.x, min.y, min.z },
        { max.x, min.y, max.z },
        { min.x, min.y, max.z },
        { min.x, max.y, min.z },
        { max.x, max.y, min.z },
        { max.x, max.y, max.z },
        { min.x, max.y, max.z }
    };

    int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    for (int i = 0; i < 12; i++) {
        DrawLine3D(corners[edges[i][0]], corners[edges[i][1]], color);
    }
}

void DrawOctree(int displayMode) {
    if (!octree) return;

    const auto& nodes = octree->getNodes();

    for (size_t i = 0; i < nodes.size(); i++) {
        const OctreeNode& node = nodes[i];

        bool shouldDraw = false;
        Color boxColor = WHITE;

        if (displayMode == 1) {
            if (i == 0) {
                boxColor = Color{255, 100, 100, 200};
            } else {
                boxColor = Color{100, 200, 255, 100};
            }
            shouldDraw = true;
        } else if (displayMode == 2) {
            shouldDraw = true;
            int depth = 0;
            int temp = (int)i;
            while (temp > 0) { depth++; temp = (temp - 1) / 8; }

            if (node.isLeaf) {
                if (node.points.empty()) {
                    boxColor = Color{100, 100, 100, 150};
                } else {
                    boxColor = Color{50, 200, 100, 180};
                }
            } else {
                boxColor = Color{100, (unsigned char)(150 + depth * 20), 200, 120};
            }
        } else if (displayMode == 3) {
            if (node.isLeaf) {
                shouldDraw = true;
                boxColor = Color{100, 200, 255, 150};
            }
        }

        if (shouldDraw) {
            Vector3 minV = { node.center.x - node.halfSize, node.center.y - node.halfSize, node.center.z - node.halfSize };
            Vector3 maxV = { node.center.x + node.halfSize, node.center.y + node.halfSize, node.center.z + node.halfSize };
            DrawBoundingBoxLines(minV, maxV, boxColor);
        }
    }
}

void DrawPoints() {
    if (!octree) return;

    const auto& nodes = octree->getNodes();

    for (const auto& node : nodes) {
        for (const auto& point : node.points) {
            bool inQuery = false;
            for (const auto& q : queryResults) {
                if (Vector3Distance(q, point) < 0.01f) {
                    inQuery = true;
                    break;
                }
            }

            if (inQuery) {
                DrawSphere(point, 0.3f, GOLD);
                DrawSphereWires(point, 0.32f, 6, 6, YELLOW);
            } else {
                DrawSphere(point, 0.2f, GREEN);
            }
        }
    }
}

void DrawQueryVisualization() {
    if (!showQuerySphere) return;

    DrawSphereWires(queryCenter, queryRadius, 16, 8, ORANGE);
    DrawSphere(queryCenter, 0.2f, RED);
}

void DrawGround() {
    float gridSize = 20.0f;
    float cellSize = 1.0f;
    Color gridColor = Color{200, 200, 200, 100};

    for (float x = -gridSize; x <= gridSize; x += cellSize) {
        DrawLine3D(Vector3{x, 0, -gridSize}, Vector3{x, 0, gridSize}, gridColor);
    }
    for (float z = -gridSize; z <= gridSize; z += cellSize) {
        DrawLine3D(Vector3{-gridSize, 0, z}, Vector3{gridSize, 0, z}, gridColor);
    }

    DrawLine3D(Vector3{-gridSize, 0.01f, 0}, Vector3{gridSize, 0.01f, 0}, Color{255, 100, 100, 200});
    DrawLine3D(Vector3{0, 0.01f, -gridSize}, Vector3{0, 0.01f, gridSize}, Color{100, 100, 255, 200});
}

//--------------------------------------------------------------------------------------
// 主循环分割为三个阶段函数
//--------------------------------------------------------------------------------------

//----------------------------------------------------------------------
// 输入处理阶段：处理键盘和鼠标输入
//----------------------------------------------------------------------
void ProcessInput(Camera* camera, int cameraMode) {
    UpdateCamera(camera, cameraMode);

    // 视图模式切换 (1/2/3)
    if (IsKeyPressed(KEY_ONE)) viewMode = 1;
    if (IsKeyPressed(KEY_TWO)) viewMode = 2;
    if (IsKeyPressed(KEY_THREE)) viewMode = 3;

    // 切换查询球体 (4)
    if (IsKeyPressed(KEY_FOUR)) showQuerySphere = !showQuerySphere;

    // 执行范围查询 (Q)
    if (IsKeyPressed(KEY_Q)) {
        queryResults.clear();
        octree->queryRange(queryCenter, queryRadius, queryResults);
    }

    // 更新统计数据 (T)
    if (IsKeyPressed(KEY_T)) {
        totalNodes = octree->getNodeCount();
        leafNodes = octree->getLeafCount();
        totalPoints = octree->getPointCount();
    }

    // 添加随机点 (空格)
    if (IsKeyPressed(KEY_SPACE)) {
        Vector3 p = {
            (float)GetRandomValue(-80, 80) / 10.0f,
            (float)GetRandomValue(0, 100) / 10.0f,
            (float)GetRandomValue(-80, 80) / 10.0f
        };
        if (octree->insert(p)) {
            totalNodes = octree->getNodeCount();
            leafNodes = octree->getLeafCount();
            totalPoints = octree->getPointCount();
        }
    }

    // 删除最近的点 (X)
    if (IsKeyPressed(KEY_X)) {
        Vector3 rayDir = Vector3Normalize(Vector3Subtract(camera->target, camera->position));
        Vector3 testPoint = Vector3Add(camera->position, Vector3Scale(rayDir, 10.0f));
        testPoint.y = 5.0f;
        testPoint.x = std::max(-9.0f, std::min(9.0f, testPoint.x));
        testPoint.z = std::max(-9.0f, std::min(9.0f, testPoint.z));

        float dist;
        Vector3 nearest = octree->findNearest(testPoint, &dist);
        if (dist < 3.0f) {
            octree->remove(nearest);
            totalNodes = octree->getNodeCount();
            leafNodes = octree->getLeafCount();
            totalPoints = octree->getPointCount();
        }
    }

    // 重置场景 (R)
    if (IsKeyPressed(KEY_R)) {
        octree->clear();
        for (int i = 0; i < 30; i++) {
            Vector3 p = {
                (float)GetRandomValue(-80, 80) / 10.0f,
                (float)GetRandomValue(0, 100) / 10.0f,
                (float)GetRandomValue(-80, 80) / 10.0f
            };
            octree->insert(p);
        }
        totalNodes = octree->getNodeCount();
        leafNodes = octree->getLeafCount();
        totalPoints = octree->getPointCount();
        queryResults.clear();
    }

    // 切换网格显示 (G)
    if (IsKeyPressed(KEY_G)) showGrid = !showGrid;

    // 鼠标滚轮调整查询半径
    if (showQuerySphere) {
        queryRadius += GetMouseWheelMove() * 0.5f;
        queryRadius = std::max(0.5f, std::min(10.0f, queryRadius));
    }
}

//----------------------------------------------------------------------
// 数据更新阶段：更新动画和查询结果
//----------------------------------------------------------------------
void Update() {
    // 动画化查询球体
    if (showQuerySphere) {
        queryCenter.y = 5.0f + sinf((float)GetTime() * 2.0f) * 2.0f;
        queryCenter.x = sinf((float)GetTime() * 0.7f) * 5.0f;
        queryCenter.z = cosf((float)GetTime() * 0.5f) * 5.0f;

        queryResults.clear();
        octree->queryRange(queryCenter, queryRadius, queryResults);
    }

    // 按住TAB更新统计数据
    if (IsKeyDown(KEY_TAB)) {
        totalNodes = octree->getNodeCount();
        leafNodes = octree->getLeafCount();
        totalPoints = octree->getPointCount();
    }
}

//----------------------------------------------------------------------
// 渲染阶段：绘制所有内容
//----------------------------------------------------------------------
void Render(Camera camera) {
    BeginDrawing();
        ClearBackground(Color{25, 25, 40, 255});

        BeginMode3D(camera);

            if (showGrid) DrawGround();
            DrawOctree(viewMode);
            DrawPoints();
            DrawQueryVisualization();

        EndMode3D();

        // 左侧控制说明面板
        DrawRectangle(10, 10, 400, 280, Fade(BLACK, 0.7f));
        DrawRectangleLines(10, 10, 400, 280, BLUE);

        DrawText("Octree Visualization Demo", 20, 20, 20, WHITE);
        DrawText("----------------------------------------", 20, 45, 16, SKYBLUE);
        DrawText("Controls:", 20, 65, 16, YELLOW);
        DrawText("W/A/S/D - Move camera", 30, 85, 14, LIGHTGRAY);
        DrawText("Mouse - Rotate view", 30, 105, 14, LIGHTGRAY);
        DrawText("Mouse wheel - Zoom", 30, 125, 14, LIGHTGRAY);
        DrawText("SPACE - Add random point", 30, 145, 14, LIGHTGRAY);
        DrawText("X - Remove nearest point", 30, 165, 14, LIGHTGRAY);
        DrawText("R - Reset scene", 30, 185, 14, LIGHTGRAY);
        DrawText("G - Toggle grid", 30, 205, 14, LIGHTGRAY);

        DrawText("View Modes:", 20, 225, 16, YELLOW);
        DrawText("1-Boxes  2-All Nodes  3-Leaf Only", 30, 245, 13, LIGHTGRAY);
        DrawText("Q-Query  4-Toggle Query Sphere", 30, 265, 13, LIGHTGRAY);

        // 右侧统计数据面板
        int rightX = screenWidth - 260;
        DrawRectangle(rightX, 10, 250, 140, Fade(BLACK, 0.7f));
        DrawRectangleLines(rightX, 10, 250, 140, GREEN);
        DrawText("Octree Statistics", rightX + 10, 20, 18, GREEN);
        DrawText(TextFormat("Total Nodes: %d", totalNodes), rightX + 15, 50, 14, WHITE);
        DrawText(TextFormat("Leaf Nodes: %d", leafNodes), rightX + 15, 70, 14, WHITE);
        DrawText(TextFormat("Total Points: %d", totalPoints), rightX + 15, 90, 14, WHITE);
        DrawText(TextFormat("Query Results: %d", (int)queryResults.size()), rightX + 15, 110, 14, GOLD);

        // 底部描述面板
        DrawRectangle(10, screenHeight - 100, screenWidth - 20, 90, Fade(BLACK, 0.7f));
        DrawRectangleLines(10, screenHeight - 100, screenWidth - 20, 90, PURPLE);
        DrawText("Octree: Spatial partitioning structure dividing 3D space into 8 sub-regions.", 20, screenHeight - 90, 14, LIGHTGRAY);
        DrawText("Uses: Collision Detection | Frustum Culling | Scene Mgmt | Ray Tracing | LOD", 20, screenHeight - 65, 14, SKYBLUE);
        const char* queryInfo = TextFormat("Query Sphere: Center(%.1f, %.1f, %.1f) Radius %.1f",
                queryCenter.x, queryCenter.y, queryCenter.z, queryRadius);
        DrawText(queryInfo, 20, screenHeight - 40, 13, GOLD);

    EndDrawing();
}

//--------------------------------------------------------------------------------------
// 主程序入口
//--------------------------------------------------------------------------------------
int main(void)
{
    // 初始化随机种子
    SetRandomSeed((unsigned int)time(nullptr));
    GetRandomValue(1, 100); // 预热随机数生成器

    // 初始化八叉树
    Vector3 worldCenter = {0, 5, 0};
    octree = new Octree(worldCenter, 10.0f, maxPointsPerNode);

    // 添加初始点
    for (int i = 0; i < 30; i++) {
        Vector3 p = {
            (float)GetRandomValue(-80, 80) / 10.0f,
            (float)GetRandomValue(0, 100) / 10.0f,
            (float)GetRandomValue(-80, 80) / 10.0f
        };
        octree->insert(p);
    }

    // 初始化统计数据
    totalNodes = octree->getNodeCount();
    leafNodes = octree->getLeafCount();
    totalPoints = octree->getPointCount();

    // 初始化窗口
    InitWindow(screenWidth, screenHeight, "八叉树可视化演示");

    // 初始化相机 (第一人称模式)
    Camera camera;
    camera.position = Vector3{0.0f, 5.0f, 15.0f};
    camera.target = Vector3{0.0f, 5.0f, 0.0f};
    camera.up = Vector3{0.0f, 1.0f, 0.0f};
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    int cameraMode = CAMERA_FIRST_PERSON;

    DisableCursor();
    SetTargetFPS(60);

    // 主循环
    while (!WindowShouldClose())
    {
        // 输入处理
        ProcessInput(&camera, cameraMode);

        // 数据更新
        Update();

        // 渲染绘制
        Render(camera);
    }

    delete octree;
    CloseWindow();
    return 0;
}
