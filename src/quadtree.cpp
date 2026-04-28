/*******************************************************************************************
* 四叉树 (Quadtree) HLOD 演示 - 2D版本
*
* 功能特性：
*   - 以摄像机为中心进行分层查询
*   - 实现类似 HLOD (Hierarchical Level of Detail) 效果
*   - 离相机近的对象显示为圆形（高精度）
*   - 离相机远的对象合并为方形（低精度）
*
* 鼠标控制：
*   鼠标拖拽 - 移动视角
*   鼠标滚轮 - 缩放
*
*   空格键 - 添加随机对象
*   X - 移除最近对象
*   R - 重置场景
*
*   1 - 显示根节点边界
*   2 - 显示所有节点
*   3 - 只显示叶子节点
*   5 - Debug视图（四叉树详细结构）
*   4 - 切换网格显示
*   H - 切换 HLOD 显示模式
*
* HLOD 简介：
*   HLOD (Hierarchical Level of Detail) 是一种高级 LOD 技术，不仅对单个物体进行
*   简化，还会对远离相机的物体簇进行合并显示。离相机越远，物体被合并得越多，
*   细节层次越低，从而在保证视觉效果的同时大幅提升渲染性能。
********************************************************************************************/

#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <cfloat>

//--------------------------------------------------------------------------------------
// 配置常量
//--------------------------------------------------------------------------------------
const int screenWidth = 1280;
const int screenHeight = 720;

// 世界空间设置
const float WORLD_SIZE = 200.0f;

// HLOD 距离阈值（屏幕像素距离）
const float HLOD_LEVEL_0 = 80.0f;   // 近距离：显示圆形（最高精度）
const float HLOD_LEVEL_1 = 150.0f;  // 中距离：中等方形
const float HLOD_LEVEL_2 = 250.0f;  // 远距离：大方形（最低精度）
const float HLOD_LEVEL_3 = 400.0f;  // 超远距离：完全合并

// 颜色定义
const Color COLOR_LEVEL_0 = {0, 255, 127, 255};    // 近距离 - 绿色圆形
const Color COLOR_LEVEL_1 = {255, 200, 0, 255};     // 中距离 - 橙色方形
const Color COLOR_LEVEL_2 = {255, 100, 100, 255};  // 远距离 - 红色大方形
const Color COLOR_LEVEL_3 = {100, 100, 255, 255};  // 超远 - 蓝色大合并
const Color COLOR_BG = {25, 25, 40, 255};

//--------------------------------------------------------------------------------------
// 四叉树节点结构
//--------------------------------------------------------------------------------------
struct QuadtreeNode {
    Vector2 center;
    float halfSize;
    bool isLeaf;
    int children[4];
    std::vector<int> objectIndices;

    QuadtreeNode(Vector2 c, float h) : center(c), halfSize(h), isLeaf(true) {
        for (int i = 0; i < 4; i++) children[i] = -1;
    }
};

//--------------------------------------------------------------------------------------
// 对象结构
//--------------------------------------------------------------------------------------
struct SceneObject {
    Vector2 position;
    float size;
    int quadtreeDepth;
};

//--------------------------------------------------------------------------------------
// HLOD 簇结构
//--------------------------------------------------------------------------------------
struct HLODCluster {
    Vector2 center;
    float radius;
    int objectCount;
    int maxDepth;
    float avgDistance;
    Color color;
};

//--------------------------------------------------------------------------------------
// 四叉树类
//--------------------------------------------------------------------------------------
class Quadtree {
private:
    std::vector<QuadtreeNode> nodes;
    int rootIndex;
    int maxDepth;
    int maxObjectsPerNode;
    Vector2 worldCenter;
    float worldHalfSize;
    std::vector<SceneObject>& objects;

    int getChildIndex(const Vector2& point, const Vector2& nodeCenter) {
        int index = 0;
        if (point.x > nodeCenter.x) index |= 1;
        if (point.y > nodeCenter.y) index |= 2;
        return index;
    }

    Vector2 getChildCenter(const Vector2& parentCenter, float parentHalfSize, int childIndex) {
        Vector2 offset = {
            (childIndex & 1) ? parentHalfSize * 0.5f : -parentHalfSize * 0.5f,
            (childIndex & 2) ? parentHalfSize * 0.5f : -parentHalfSize * 0.5f
        };
        return Vector2Add(parentCenter, offset);
    }

    bool insertRecursive(int nodeIndex, int objectIndex, const Vector2& point, int depth) {
        QuadtreeNode& node = nodes[nodeIndex];

        Vector2 diff = Vector2Subtract(point, node.center);
        if (fabsf(diff.x) > node.halfSize || fabsf(diff.y) > node.halfSize) {
            return false;
        }

        if (node.isLeaf) {
            node.objectIndices.push_back(objectIndex);
            objects[objectIndex].quadtreeDepth = depth;

            if ((int)node.objectIndices.size() < maxObjectsPerNode || depth >= maxDepth) {
                return true;
            }
            subdivide(nodeIndex);
        } else {
            int childIndex = getChildIndex(point, node.center);
            if (node.children[childIndex] != -1) {
                return insertRecursive(node.children[childIndex], objectIndex, point, depth + 1);
            }
        }
        return true;
    }

    void subdivide(int nodeIndex) {
        QuadtreeNode& node = nodes[nodeIndex];
        node.isLeaf = false;

        std::vector<int> oldIndices = node.objectIndices;
        node.objectIndices.clear();

        for (int i = 0; i < 4; i++) {
            Vector2 childCenter = getChildCenter(node.center, node.halfSize, i);
            nodes.emplace_back(childCenter, node.halfSize * 0.5f);
            node.children[i] = (int)nodes.size() - 1;
        }

        for (int objIndex : oldIndices) {
            insertRecursive(nodeIndex, objIndex, objects[objIndex].position, 0);
        }
    }

    void collectLeafObjectsRecursive(int nodeIndex, std::vector<int>& result) {
        QuadtreeNode& node = nodes[nodeIndex];

        if (node.isLeaf) {
            for (int idx : node.objectIndices) {
                result.push_back(idx);
            }
        } else {
            for (int i = 0; i < 4; i++) {
                if (node.children[i] != -1) {
                    collectLeafObjectsRecursive(node.children[i], result);
                }
            }
        }
    }

    int getObjectNodeRecursive(int nodeIndex, const Vector2& point) {
        QuadtreeNode& node = nodes[nodeIndex];

        if (node.isLeaf) {
            return nodeIndex;
        }

        int childIndex = getChildIndex(point, node.center);
        if (node.children[childIndex] != -1) {
            return getObjectNodeRecursive(node.children[childIndex], point);
        }
        return nodeIndex;
    }

public:
    Quadtree(Vector2 center, float halfSize, std::vector<SceneObject>& objs,
             int maxObjs = 8, int maxDepthValue = 6)
        : objects(objs), worldCenter(center), worldHalfSize(halfSize),
          maxObjectsPerNode(maxObjs), maxDepth(maxDepthValue) {
        nodes.emplace_back(center, halfSize);
        rootIndex = 0;
    }

    bool insert(int objectIndex) {
        if (objectIndex < 0 || objectIndex >= (int)objects.size()) return false;
        return insertRecursive(rootIndex, objectIndex, objects[objectIndex].position, 0);
    }

    void rebuild() {
        nodes.clear();
        nodes.emplace_back(worldCenter, worldHalfSize);
        rootIndex = 0;

        for (int i = 0; i < (int)objects.size(); i++) {
            insertRecursive(rootIndex, i, objects[i].position, 0);
        }
    }

    std::vector<int> getAllObjectIndices() {
        std::vector<int> result;
        collectLeafObjectsRecursive(rootIndex, result);
        return result;
    }

    int getObjectNode(int objectIndex) {
        if (objectIndex < 0 || objectIndex >= (int)objects.size()) return -1;
        return getObjectNodeRecursive(rootIndex, objects[objectIndex].position);
    }

    QuadtreeNode* getNode(int index) {
        if (index >= 0 && index < (int)nodes.size()) {
            return &nodes[index];
        }
        return nullptr;
    }

    QuadtreeNode& getRoot() { return nodes[rootIndex]; }

    int getNodeCount() const { return (int)nodes.size(); }

    int getLeafCount() const {
        int count = 0;
        for (const auto& node : nodes) {
            if (node.isLeaf) count++;
        }
        return count;
    }

    int getObjectCount() const { return (int)objects.size(); }

    int getNodeDepth(int nodeIndex) {
        int depth = 0;
        int temp = nodeIndex;
        while (temp > 0) {
            depth++;
            temp = (temp - 1) / 4;
        }
        return depth;
    }

    void clear() {
        nodes.clear();
        nodes.emplace_back(worldCenter, worldHalfSize);
        rootIndex = 0;
    }
};

//--------------------------------------------------------------------------------------
// 全局变量
//--------------------------------------------------------------------------------------
Quadtree* quadtree = nullptr;
std::vector<SceneObject> sceneObjects;

bool showHLOD = true;
int viewMode = 1;
bool showGrid = true;

int totalNodes = 0;
int leafNodes = 0;
int totalObjects = 0;

std::vector<HLODCluster> hlodClusters;

// 玩家点
Vector2 playerPos = {0, 0};
const float PLAYER_SCREEN_SPEED = 200.0f;  // 屏幕空间速度（像素/秒）

// 相机/视角
Vector2 cameraOffset = {0, 0};
float cameraZoom = 1.0f;
Vector2 mouseStart = {0, 0};
bool isDragging = false;
Vector2 dragStartOffset = {0, 0};

//--------------------------------------------------------------------------------------
// 辅助函数
//--------------------------------------------------------------------------------------

int getHLODLevel(float screenDist) {
    if (screenDist < HLOD_LEVEL_0) return 0;
    if (screenDist < HLOD_LEVEL_1) return 1;
    if (screenDist < HLOD_LEVEL_2) return 2;
    return 3;
}

Color getHLODColor(int level) {
    switch (level) {
        case 0: return COLOR_LEVEL_0;
        case 1: return COLOR_LEVEL_1;
        case 2: return COLOR_LEVEL_2;
        default: return COLOR_LEVEL_3;
    }
}

// 世界坐标转屏幕坐标
Vector2 WorldToScreen(Vector2 worldPos) {
    float x = (worldPos.x + WORLD_SIZE * 0.5f) * cameraZoom + cameraOffset.x;
    float y = (worldPos.y + WORLD_SIZE * 0.5f) * cameraZoom + cameraOffset.y;
    return {x, y};
}

// 屏幕坐标转世界坐标
Vector2 ScreenToWorld(Vector2 screenPos) {
    float x = (screenPos.x - cameraOffset.x) / cameraZoom - WORLD_SIZE * 0.5f;
    float y = (screenPos.y - cameraOffset.y) / cameraZoom - WORLD_SIZE * 0.5f;
    return {x, y};
}

// 计算屏幕距离
float GetScreenDistance(Vector2 worldPos1, Vector2 worldPos2) {
    Vector2 screen1 = WorldToScreen(worldPos1);
    Vector2 screen2 = WorldToScreen(worldPos2);
    return Vector2Distance(screen1, screen2);
}

// 计算到相机的屏幕距离
float GetCameraScreenDistance(Vector2 worldPos, Vector2 cameraWorld) {
    return GetScreenDistance(worldPos, cameraWorld);
}

void calculateHLODClusters(Vector2 cameraWorld) {
    hlodClusters.clear();

    if (!quadtree || sceneObjects.empty()) return;

    // 按四叉树节点分组对象
    struct NodeGroup {
        int nodeIndex;
        std::vector<int> objectIndices;
        Vector2 center;
        float maxDist;
    };
    std::vector<NodeGroup> groups;

    std::vector<int> allObjects = quadtree->getAllObjectIndices();

    for (int objIdx : allObjects) {
        if (objIdx < 0 || objIdx >= (int)sceneObjects.size()) continue;

        SceneObject& obj = sceneObjects[objIdx];
        float screenDist = GetCameraScreenDistance(obj.position, cameraWorld);
        int nodeIdx = quadtree->getObjectNode(objIdx);

        bool found = false;
        for (auto& group : groups) {
            if (group.nodeIndex == nodeIdx) {
                group.objectIndices.push_back(objIdx);
                group.maxDist = std::max(group.maxDist, screenDist);
                found = true;
                break;
            }
        }

        if (!found) {
            QuadtreeNode* node = quadtree->getNode(nodeIdx);
            if (node) {
                NodeGroup group;
                group.nodeIndex = nodeIdx;
                group.objectIndices.push_back(objIdx);
                group.center = node->center;
                group.maxDist = screenDist;
                groups.push_back(group);
            }
        }
    }

    // 根据屏幕距离创建 HLOD 簇
    for (const auto& group : groups) {
        if (group.objectIndices.empty()) continue;

        float totalDist = 0;
        Vector2 center = {0, 0};
        int maxDepth = 0;

        for (int objIdx : group.objectIndices) {
            SceneObject& obj = sceneObjects[objIdx];
            float screenDist = GetCameraScreenDistance(obj.position, cameraWorld);
            totalDist += screenDist;
            center = Vector2Add(center, obj.position);
            maxDepth = std::max(maxDepth, obj.quadtreeDepth);
        }

        center = Vector2Scale(center, 1.0f / group.objectIndices.size());
        float avgDist = totalDist / group.objectIndices.size();

        int level = getHLODLevel(avgDist);

        if (level >= 2 && group.objectIndices.size() > 1) {
            // 远距离合并为簇
            HLODCluster cluster;
            cluster.center = center;
            cluster.objectCount = (int)group.objectIndices.size();
            cluster.maxDepth = maxDepth;
            cluster.avgDistance = avgDist;
            cluster.color = getHLODColor(level);

            float baseRadius = 0.3f + sqrtf((float)group.objectIndices.size()) * 0.3f;
            cluster.radius = baseRadius * (1.0f + (level - 1) * 0.5f);

            hlodClusters.push_back(cluster);
        } else {
            // 近距离保留个体对象
            for (int objIdx : group.objectIndices) {
                SceneObject& obj = sceneObjects[objIdx];
                float screenDist = GetCameraScreenDistance(obj.position, cameraWorld);

                HLODCluster cluster;
                cluster.center = obj.position;
                cluster.objectCount = 1;
                cluster.maxDepth = obj.quadtreeDepth;
                cluster.avgDistance = screenDist;
                cluster.color = getHLODColor(getHLODLevel(screenDist));
                cluster.radius = obj.size;

                hlodClusters.push_back(cluster);
            }
        }
    }

    // 按距离排序
    std::sort(hlodClusters.begin(), hlodClusters.end(),
        [](const HLODCluster& a, const HLODCluster& b) {
            return a.avgDistance < b.avgDistance;
        });
}

Vector2 randomPosition() {
    float range = WORLD_SIZE * 0.4f;
    return {
        (float)GetRandomValue(-100, 100) / 100.0f * range,
        (float)GetRandomValue(-100, 100) / 100.0f * range
    };
}

//--------------------------------------------------------------------------------------
// 绘制函数
//--------------------------------------------------------------------------------------

void DrawQuadtree(int displayMode) {
    if (!quadtree) return;

    int nodeCount = quadtree->getNodeCount();
    Color lineColor = Color{100, 100, 100, 180};

    for (int i = 0; i < nodeCount; i++) {
        QuadtreeNode* node = quadtree->getNode(i);
        if (!node) continue;

        bool shouldDraw = false;
        Color boxColor = lineColor;

        if (displayMode == 1) {
            // 只显示根节点
            if (i == 0) {
                boxColor = Color{255, 100, 100, 255};
                shouldDraw = true;
            }
        } else if (displayMode == 2) {
            // 显示所有节点
            shouldDraw = true;
            int depth = quadtree->getNodeDepth(i);
            int alpha = 80 + depth * 30;
            boxColor = Color{100, 180, (unsigned char)(150 + depth * 20), (unsigned char)alpha};
        } else if (displayMode == 3) {
            // 只显示有对象的叶子节点
            if (node->isLeaf && !node->objectIndices.empty()) {
                shouldDraw = true;
                boxColor = Color{100, 200, 255, 180};
            }
        } else if (displayMode == 4) {
            // Debug 模式：显示所有节点及详细信息
            shouldDraw = true;
            int depth = quadtree->getNodeDepth(i);

            // 根据深度选择颜色
            Color depthColors[] = {
                Color{255, 80, 80, 255},    // 深度0 - 红色
                Color{255, 165, 80, 255},   // 深度1 - 橙色
                Color{255, 255, 80, 255},   // 深度2 - 黄色
                Color{80, 255, 80, 255},    // 深度3 - 绿色
                Color{80, 255, 255, 255},   // 深度4 - 青色
                Color{80, 80, 255, 255},    // 深度5 - 蓝色
                Color{255, 80, 255, 255}    // 深度6 - 紫色
            };
            boxColor = depthColors[depth % 7];

            // 计算屏幕尺寸
            Vector2 min = Vector2Subtract(node->center, {node->halfSize, node->halfSize});
            Vector2 max = Vector2Add(node->center, {node->halfSize, node->halfSize});
            Vector2 screenMin = WorldToScreen(min);
            Vector2 screenMax = WorldToScreen(max);
            float screenWidth = (screenMax.x - screenMin.x);
            float screenHeight = (screenMax.y - screenMin.y);

            // 绘制填充
            DrawRectangleRec(Rectangle{screenMin.x, screenMin.y, screenWidth, screenHeight},
                Color{boxColor.r, boxColor.g, boxColor.b, 30});

            // 绘制边框
            DrawRectangleLinesEx(Rectangle{screenMin.x, screenMin.y, screenWidth, screenHeight},
                1.5f, boxColor);

            // 绘制节点中心点
            Vector2 centerScreen = WorldToScreen(node->center);
            DrawCircleV(centerScreen, 3.0f, boxColor);

            // 绘制十字线显示节点范围
            DrawLineV(
                WorldToScreen({node->center.x - node->halfSize, node->center.y}),
                WorldToScreen({node->center.x + node->halfSize, node->center.y}),
                Color{boxColor.r, boxColor.g, boxColor.b, 80}
            );
            DrawLineV(
                WorldToScreen({node->center.x, node->center.y - node->halfSize}),
                WorldToScreen({node->center.x, node->center.y + node->halfSize}),
                Color{boxColor.r, boxColor.g, boxColor.b, 80}
            );

            // 绘制节点信息（仅在节点足够大时）
            if (screenWidth > 30 && screenHeight > 20) {
                const char* info = TextFormat("D%d [%d]", depth, (int)node->objectIndices.size());
                int textWidth = MeasureText(info, 10);
                DrawText(info,
                    centerScreen.x - textWidth * 0.5f,
                    centerScreen.y - 5,
                    10,
                    Color{255, 255, 255, 200});
            }

            // 叶子节点标记
            if (node->isLeaf) {
                Vector2 markerPos = {screenMax.x - 6, screenMin.y + 3};
                DrawRectangleRec(Rectangle{markerPos.x, markerPos.y, 4, 4}, GOLD);
            }

            continue;
        }

        if (shouldDraw) {
            Vector2 min = Vector2Subtract(node->center, {node->halfSize, node->halfSize});
            Vector2 max = Vector2Add(node->center, {node->halfSize, node->halfSize});
            Vector2 screenMin = WorldToScreen(min);
            Vector2 screenMax = WorldToScreen(max);
            DrawRectangleLinesEx(Rectangle{screenMin.x, screenMin.y,
                screenMax.x - screenMin.x, screenMax.y - screenMin.y}, 1.0f, boxColor);
        }
    }
}

void DrawGrid() {
    float cellSize = 2.0f;
    Color gridColor = Color{80, 80, 80, 100};
    Color axisColor = Color{150, 80, 80, 150};

    // 垂直线
    for (float x = -WORLD_SIZE * 0.5f; x <= WORLD_SIZE * 0.5f; x += cellSize) {
        Vector2 start = WorldToScreen({x, -WORLD_SIZE * 0.5f});
        Vector2 end = WorldToScreen({x, WORLD_SIZE * 0.5f});
        DrawLineV(start, end, fabsf(x) < 0.1f ? axisColor : gridColor);
    }

    // 水平线
    for (float y = -WORLD_SIZE * 0.5f; y <= WORLD_SIZE * 0.5f; y += cellSize) {
        Vector2 start = WorldToScreen({-WORLD_SIZE * 0.5f, y});
        Vector2 end = WorldToScreen({WORLD_SIZE * 0.5f, y});
        DrawLineV(start, end, fabsf(y) < 0.1f ? axisColor : gridColor);
    }
}

void DrawWorldBounds() {
    Vector2 min = WorldToScreen({-WORLD_SIZE * 0.5f, -WORLD_SIZE * 0.5f});
    Vector2 max = WorldToScreen({WORLD_SIZE * 0.5f, WORLD_SIZE * 0.5f});
    DrawRectangleLinesEx(Rectangle{min.x, min.y, max.x - min.x, max.y - min.y}, 2.0f,
        Color{200, 200, 100, 200});
}

void DrawObjects(Vector2 cameraWorld) {
    if (!quadtree || sceneObjects.empty()) return;

    if (showHLOD) {
        // HLOD 模式
        for (const auto& cluster : hlodClusters) {
            Vector2 screenPos = WorldToScreen(cluster.center);

            if (cluster.objectCount == 1) {
                // 单个对象：圆形
                float screenRadius = cluster.radius * cameraZoom;
                DrawCircleV(screenPos, screenRadius, cluster.color);
                DrawCircleLinesV(screenPos, screenRadius + 1.0f,
                    Color{255, 255, 255, 100});
            } else {
                // 多个对象：方形
                float screenSize = cluster.radius * 2.0f * cameraZoom;
                Rectangle rect = {
                    screenPos.x - screenSize * 0.5f,
                    screenPos.y - screenSize * 0.5f,
                    screenSize,
                    screenSize
                };
                DrawRectangleRec(rect, Color{cluster.color.r, cluster.color.g,
                    cluster.color.b, 60});
                DrawRectangleLinesEx(rect, 2.0f, cluster.color);

                // 显示合并数量
                if (screenSize > 20.0f) {
                    const char* text = TextFormat("%d", cluster.objectCount);
                    int textWidth = MeasureText(text, 16);
                    DrawText(text, screenPos.x - textWidth * 0.5f, screenPos.y - 8, 16,
                        Color{255, 255, 255, 200});
                }
            }
        }
    } else {
        // 普通模式：所有对象都是圆形
        for (const auto& obj : sceneObjects) {
            float screenDist = GetCameraScreenDistance(obj.position, cameraWorld);
            Color color = getHLODColor(getHLODLevel(screenDist));
            Vector2 screenPos = WorldToScreen(obj.position);
            float screenRadius = obj.size * cameraZoom;

            DrawCircleV(screenPos, screenRadius, color);
        }
    }
}

void DrawDistanceRings(Vector2 cameraWorld) {
    float distances[] = {HLOD_LEVEL_0, HLOD_LEVEL_1, HLOD_LEVEL_2, HLOD_LEVEL_3};
    Color colors[] = {
        Color{COLOR_LEVEL_0.r, COLOR_LEVEL_0.g, COLOR_LEVEL_0.b, 60},
        Color{COLOR_LEVEL_1.r, COLOR_LEVEL_1.g, COLOR_LEVEL_1.b, 50},
        Color{COLOR_LEVEL_2.r, COLOR_LEVEL_2.g, COLOR_LEVEL_2.b, 40},
        Color{COLOR_LEVEL_3.r, COLOR_LEVEL_3.g, COLOR_LEVEL_3.b, 30}
    };

    Vector2 cameraScreen = WorldToScreen(cameraWorld);

    for (int i = 3; i >= 0; i--) {
        DrawCircleLinesV(cameraScreen, distances[i], colors[i]);
    }
}

//--------------------------------------------------------------------------------------
// 输入处理
//--------------------------------------------------------------------------------------
void ProcessInput() {
    float deltaTime = GetFrameTime();

    // WASD 移动玩家点
    Vector2 moveDir = {0, 0};
    if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) moveDir.y -= 1;
    if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) moveDir.y += 1;
    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) moveDir.x -= 1;
    if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) moveDir.x += 1;

    // 标准化方向并移动
    if (Vector2Length(moveDir) > 0) {
        moveDir = Vector2Normalize(moveDir);
        // 屏幕空间统一速度：worldSpeed = screenSpeed / zoom
        float worldSpeed = PLAYER_SCREEN_SPEED / cameraZoom;
        playerPos = Vector2Add(playerPos, Vector2Scale(moveDir, worldSpeed * deltaTime));

        // 限制在世界边界内
        float limit = WORLD_SIZE * 0.5f - 0.5f;
        playerPos.x = Clamp(playerPos.x, -limit, limit);
        playerPos.y = Clamp(playerPos.y, -limit, limit);
    }

    // 鼠标拖拽平移
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && !isDragging) {
        isDragging = true;
        mouseStart = GetMousePosition();
        dragStartOffset = cameraOffset;
    }

    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && isDragging) {
        Vector2 mousePos = GetMousePosition();
        Vector2 delta = Vector2Subtract(mousePos, mouseStart);
        cameraOffset = Vector2Add(dragStartOffset, delta);
    }

    if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
        isDragging = false;
    }

    // 鼠标滚轮缩放 - 以鼠标位置为中心
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        Vector2 mousePos = GetMousePosition();

        // 缩放前的鼠标世界坐标
        Vector2 worldBefore = ScreenToWorld(mousePos);

        // 应用缩放
        float oldZoom = cameraZoom;
        cameraZoom *= (1.0f + wheel * 0.15f);
        cameraZoom = Clamp(cameraZoom, screenHeight / (WORLD_SIZE * 1.5f), screenHeight / 10.0f);

        // 缩放后的鼠标世界坐标
        Vector2 worldAfter = ScreenToWorld(mousePos);

        // 调整 cameraOffset 使鼠标位置保持不变
        float zoomRatio = cameraZoom / oldZoom;
        cameraOffset.x = mousePos.x - (mousePos.x - cameraOffset.x) * zoomRatio;
        cameraOffset.y = mousePos.y - (mousePos.y - cameraOffset.y) * zoomRatio;
    }

    // 视图模式切换
    if (IsKeyPressed(KEY_ONE)) viewMode = 1;
    if (IsKeyPressed(KEY_TWO)) viewMode = 2;
    if (IsKeyPressed(KEY_THREE)) viewMode = 3;
    if (IsKeyPressed(KEY_FIVE)) viewMode = 4;  // Debug视图

    // 切换网格
    if (IsKeyPressed(KEY_FOUR)) showGrid = !showGrid;

    // 切换 HLOD
    if (IsKeyPressed(KEY_H)) showHLOD = !showHLOD;

    // 添加随机对象
    if (IsKeyPressed(KEY_SPACE)) {
        Vector2 pos = randomPosition();
        SceneObject obj;
        obj.position = pos;
        obj.size = 0.3f + (float)GetRandomValue(0, 40) / 100.0f;
        obj.quadtreeDepth = 0;
        sceneObjects.push_back(obj);
        quadtree->rebuild();
    }

    // 删除最近对象
    if (IsKeyPressed(KEY_X)) {
        if (!sceneObjects.empty()) {
            Vector2 mouseWorld = ScreenToWorld(GetMousePosition());
            int nearestIdx = -1;
            float nearestDist = FLT_MAX;

            for (int i = 0; i < (int)sceneObjects.size(); i++) {
                float dist = Vector2Distance(sceneObjects[i].position, mouseWorld);
                if (dist < nearestDist) {
                    nearestDist = dist;
                    nearestIdx = i;
                }
            }

            if (nearestIdx >= 0) {
                sceneObjects.erase(sceneObjects.begin() + nearestIdx);
                quadtree->rebuild();
            }
        }
    }

    // 重置场景
    if (IsKeyPressed(KEY_R)) {
        sceneObjects.clear();
        for (int i = 0; i < 200; i++) {
            Vector2 pos = randomPosition();
            SceneObject obj;
            obj.position = pos;
            obj.size = 0.5f + (float)GetRandomValue(0, 60) / 100.0f;
            obj.quadtreeDepth = 0;
            sceneObjects.push_back(obj);
        }
        quadtree->rebuild();
        playerPos = {0, 0};
        cameraOffset = {(float)screenWidth * 0.5f, (float)screenHeight * 0.5f};
        cameraZoom = screenHeight / (WORLD_SIZE * 0.8f);
    }

    // 更新统计
    totalNodes = quadtree->getNodeCount();
    leafNodes = quadtree->getLeafCount();
    totalObjects = quadtree->getObjectCount();
}

//--------------------------------------------------------------------------------------
// 渲染
//--------------------------------------------------------------------------------------
void Render() {
    Vector2 cameraWorld = ScreenToWorld(cameraOffset);

    BeginDrawing();
        ClearBackground(COLOR_BG);

        // 绘制网格
        if (showGrid) DrawGrid();

        // 绘制世界边界
        DrawWorldBounds();

        // 绘制距离环（以玩家为中心）
        DrawDistanceRings(playerPos);

        // 绘制四叉树
        DrawQuadtree(viewMode);

        // 绘制对象（以玩家为中心计算距离）
        DrawObjects(playerPos);

        // 绘制玩家位置
        Vector2 playerScreen = WorldToScreen(playerPos);
        DrawCircleV(playerScreen, 8.0f, WHITE);
        DrawCircleV(playerScreen, 5.0f, Color{100, 200, 255, 255});
        DrawCircleLinesV(playerScreen, 12.0f, Color{255, 255, 255, 150});

        // ========== 左侧控制面板 ==========
        DrawRectangle(10, 10, 380, 280, Fade(BLACK, 0.8f));
        DrawRectangleLines(10, 10, 380, 280, BLUE);

        DrawText("Quadtree HLOD Demo - 2D", 20, 20, 20, WHITE);
        DrawText("================================", 20, 46, 14, SKYBLUE);
        DrawText("Controls:", 20, 66, 16, YELLOW);

        DrawText("W/A/S/D - Move player (cyan dot)", 30, 88, 14, Color{100, 200, 255, 255});
        DrawText("Mouse drag - Pan view", 30, 108, 14, LIGHTGRAY);
        DrawText("Mouse wheel - Zoom (mouse center)", 30, 128, 14, LIGHTGRAY);
        DrawText("SPACE - Add random object", 30, 148, 14, LIGHTGRAY);
        DrawText("X - Remove nearest object", 30, 168, 14, LIGHTGRAY);
        DrawText("R - Reset scene", 30, 188, 14, LIGHTGRAY);

        DrawText("View Options:", 20, 213, 16, YELLOW);
        DrawText("1 - Root  2 - All  3 - Leaf  5 - Debug", 30, 233, 13, LIGHTGRAY);
        DrawText("G - Grid   H - HLOD Mode", 30, 253, 13, LIGHTGRAY);
        DrawText(TextFormat("Grid: %s", showGrid ? "ON" : "OFF"), 250, 233, 13,
            showGrid ? GREEN : DARKGRAY);

        // ========== HLOD 图例 ==========
        DrawRectangle(10, 300, 380, 120, Fade(BLACK, 0.8f));
        DrawRectangleLines(10, 300, 380, 120, PURPLE);
        DrawText("HLOD Level of Detail:", 20, 310, 16, PURPLE);

        // Level 0
        DrawCircle(40, 355, 10, COLOR_LEVEL_0);
        DrawText(TextFormat("L0: Screen d < %.0f (Circle)", HLOD_LEVEL_0), 60, 347, 14, COLOR_LEVEL_0);

        // Level 1
        DrawRectangle(30, 368, 18, 18, COLOR_LEVEL_1);
        DrawText(TextFormat("L1: %.0f < d < %.0f (Square)", HLOD_LEVEL_0, HLOD_LEVEL_1), 60, 370, 14, COLOR_LEVEL_1);

        // Level 2
        DrawRectangle(27, 394, 24, 24, COLOR_LEVEL_2);
        DrawText(TextFormat("L2: %.0f < d < %.0f (Merged)", HLOD_LEVEL_1, HLOD_LEVEL_2), 60, 398, 14, COLOR_LEVEL_2);

        // Level 3
        DrawRectangle(24, 420, 30, 30, COLOR_LEVEL_3);
        DrawText(TextFormat("L3: d > %.0f (Clustered)", HLOD_LEVEL_2), 60, 428, 14, COLOR_LEVEL_3);

        // ========== 右侧统计面板 ==========
        int rightX = screenWidth - 260;
        DrawRectangle(rightX, 10, 250, 180, Fade(BLACK, 0.8f));
        DrawRectangleLines(rightX, 10, 250, 180, GREEN);

        DrawText("Statistics", rightX + 10, 20, 18, GREEN);
        DrawText("----------------------------------", rightX + 10, 44, 12, DARKGRAY);

        DrawText(TextFormat("Total Nodes: %d", totalNodes), rightX + 15, 62, 14, WHITE);
        DrawText(TextFormat("Leaf Nodes: %d", leafNodes), rightX + 15, 82, 14, WHITE);
        DrawText(TextFormat("Total Objects: %d", totalObjects), rightX + 15, 102, 14, WHITE);
        DrawText(TextFormat("HLOD Clusters: %d", (int)hlodClusters.size()), rightX + 15, 122, 14, GOLD);

        DrawText("----------------------------------", rightX + 10, 145, 12, DARKGRAY);

        Color hlodStatusColor = showHLOD ? GREEN : RED;
        DrawText(TextFormat("HLOD Mode: %s", showHLOD ? "ON" : "OFF"), rightX + 15, 162, 14, hlodStatusColor);

        // ========== 底部说明 ==========
        DrawRectangle(10, screenHeight - 60, screenWidth - 20, 50, Fade(BLACK, 0.8f));
        DrawRectangleLines(10, screenHeight - 60, screenWidth - 20, 50, ORANGE);
        DrawText("Quadtree + HLOD: Camera-centric hierarchical LOD rendering.", 20, screenHeight - 52, 14, LIGHTGRAY);
        DrawText("Near objects = Circles | Far objects merged = Squares", 20, screenHeight - 28, 14, SKYBLUE);

    EndDrawing();
}

//--------------------------------------------------------------------------------------
// 主程序
//--------------------------------------------------------------------------------------
int main(void)
{
    SetRandomSeed((unsigned int)time(nullptr));
    GetRandomValue(1, 100);

    // 初始化场景对象
    for (int i = 0; i < 200; i++) {
        Vector2 pos = randomPosition();
        SceneObject obj;
        obj.position = pos;
        obj.size = 0.5f + (float)GetRandomValue(0, 60) / 100.0f;
        obj.quadtreeDepth = 0;
        sceneObjects.push_back(obj);
    }

    // 初始化四叉树
    Vector2 worldCenter = {0, 0};
    quadtree = new Quadtree(worldCenter, WORLD_SIZE * 0.5f, sceneObjects, 8, 6);
    quadtree->rebuild();

    // 初始化统计数据
    totalNodes = quadtree->getNodeCount();
    leafNodes = quadtree->getLeafCount();
    totalObjects = quadtree->getObjectCount();

    // 初始化窗口
    InitWindow(screenWidth, screenHeight, "Quadtree HLOD Demo - 2D");
    SetTargetFPS(60);

    // 初始化相机和玩家位置
    playerPos = {0, 0};
    cameraOffset = {(float)screenWidth * 0.5f, (float)screenHeight * 0.5f};
    cameraZoom = screenHeight / WORLD_SIZE;

    // 主循环
    while (!WindowShouldClose())
    {
        ProcessInput();
        calculateHLODClusters(playerPos);
        Render();
    }

    delete quadtree;
    CloseWindow();
    return 0;
}
