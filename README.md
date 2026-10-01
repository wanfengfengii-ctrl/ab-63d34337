# 浅地层雷达上下反射界面联合拾取服务

对每一列的多个回波候选点，**联合**追踪上、下两条反射界面（不先追一条再补另一条），
确保上下界面严格分离、不交叉、不突跳、厚度不失真。

## 接口

### `POST /api/horizons/trace`

请求：

```json
{
  "columns": [
    [
      {"id": "u0", "depth": 10, "confidence": 6},
      {"id": "m0", "depth": 4,  "confidence": 1},
      {"id": "l0", "depth": 22, "confidence": 7}
    ]
    // ... 共 8–24 列，每列 3–8 个候选；
    // id 在列内唯一，depth 为整数，confidence 为正整数
  ],
  "limits": {
    "min_thickness": 1,
    "max_thickness": 40,
    "max_slope": 20,
    "max_thickness_change": 20,
    "max_second_difference": 20
  }
}
```

约束（`t_i = lower.depth - upper.depth`）：

| 约束 | 表达式 |
| --- | --- |
| 列内严格分离 | 上下选择不同候选且 `min_thickness ≤ t_i ≤ max_thickness` |
| 相邻坡差（两条界面） | `|d^x_i - d^x_{i-1}| ≤ max_slope` |
| 厚度变化 | `|t_i - t_{i-1}| ≤ max_thickness_change` |
| 二阶差（连续三列，两条界面） | `|d^x_i - 2d^x_{i-1} + d^x_{i-2}| ≤ max_second_difference` |

成功响应（200，`feasible: true`）：

```json
{
  "feasible": true,
  "columns": 8,
  "upper_horizon": [{"id": "u0", "depth": 10, "confidence": 6, "order": 0}],
  "lower_horizon": [{"id": "l0", "depth": 22, "confidence": 7, "order": 2}],
  "thicknesses": [12],
  "slopes": {"upper": [0], "lower": [0]},
  "second_differences": {"upper": [], "lower": []},
  "adjudication": {
    "total_confidence": 104,
    "max_second_difference": 0,
    "total_travel": 0
  }
}
```

无可行组合时返回 **200 + `feasible: false`**，不带任何伪造的局部轨迹：

```json
{"feasible": false, "columns": 8, "message": "no feasible joint upper/lower horizon combination"}
```

请求结构非法返回 400。

## 算法

联合配对动态规划。状态为 `(upper_i, lower_i, upper_{i-1}, lower_{i-1})`，
因此每次转移都同时计算两条界面在连续三列上的二阶差；天然杜绝"逐列选最强回波"
带来的交叉、突跳与厚度失真。

全局裁决依次为：① 置信度总和最大；② 两界面最大二阶差最小；③ 总行程（两界面
绝对坡差之和）最小；④ 按逐列候选原始顺序的字典序稳定决胜。

剪枝保证不改变裁决：同一状态的可行后缀集合完全相同，置信度为第一加性判据，故
每状态只保留最高置信度的路径；行程严格更小且曲率不更差是加性优势可安全剪枝，
曲率为取 `max` 的非加性量，行程相同时必须同时比较曲率与稳定决胜序。

## 本地运行（无 Docker 时）

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python -m pytest -q tests
python scripts/smoke.py        # 需要先启动服务，见下
API_PORT=8000 python -m gunicorn --bind 0.0.0.0:8000 app.server:app
```

## Docker

```bash
# 启动 API（端口可用 API_PORT 配置）
API_PORT=8000 docker compose up --build -d

# 一次性验证：等待 api 健康后依次执行
#   1) 代码测试（pytest）
#   2) 镜像构建检查（docker build --target app）
#   3) 一组联合拾取 API 冒烟
# 全部通过退出 0，任一失败退出非 0
docker compose run --rm verify
```

`verify` 是一次性服务（`restart: "no"`），通过 `depends_on: condition:
service_healthy` 等待 `api` 健康后才启动，并挂载宿主机 Docker socket 执行真实的
镜像构建检查。
