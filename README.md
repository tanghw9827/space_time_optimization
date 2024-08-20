# space_time_optimization
基于iLQR的时空联合规划

交互目标筛选逻辑：选择横向带有 BYPASS\_LEFT/BYPASS\_RIGHT、纵向带有 YIELD/OVERTAKE 标签的动态障碍物作为联合优化的交互目标。

状态变量

$$
X = [x, y, q, k, v, a, j, x', y', q', v', a', j', x'', y'', q'', v'', a''...]^T
$$

其中 \(q\) 是 theta，\(k\) 是角速度。

控制量

$$
U = [dk, dj, k', j', k'', j'']^T
$$

下面模型只写到两车交互，若有其他交互目标则可继续扩展。

状态空间模型：

