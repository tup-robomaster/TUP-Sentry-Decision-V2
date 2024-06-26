# TUP-Sentry-Decision-V2


## 逻辑循环执行策略

    循环频率控制：
        使用 local_rate_ 定义的频率来控制决策循环的执行速率，通过 local_rate_->sleep() 来实现。

    UART消息处理：
        在每次循环开始时，首先检查 UART 消息是否可用。
        如果 UART 消息不可用，记录一次警告，并继续下一次循环。

    游戏阶段检查：
        确保机器人在游戏阶段为开始状态。
        如果不是开始状态，记录一次警告并继续下一次循环。

    获取机器人当前位置：
        使用 tf2 库获取 base_link 到 map 坐标系的变换，得到机器人在地图中的当前位置，并将其更新到黑板中。

    计算当前路标点：
        通过计算机器人当前位置与预定义路标点的距离，确定机器人当前所在的路标点，并将其 ID 更新到黑板中。

    制定决策：
        调用 makeDecision() 方法，根据当前黑板状态和预定义决策，选择最优的决策，并将其存储在 currentDecision 中。

    任务执行：
        如果没有正在执行的任务，机器人将执行 makeDecision() 中选择的决策。
        将该决策中的所有动作添加到任务队列中，以便按顺序执行。

    任务队列管理：
        如果任务队列为空，记录一次信息（表示无任务执行）。
        否则检查任务队列的执行状态，根据任务状态执行不同的处理逻辑，如任务完成、任务取消等。

    决策切换判断：
        在每次循环中，检查是否需要切换决策。
        如果当前决策的权重大于过去决策的权重，或者当前任务执行失败或已取消，或者当前任务已成功且等待任务完成，都可能触发切换。

    决策和任务执行日志记录：

    使用 RCLCPP_INFO 等级的日志记录器，记录关键的执行信息，以便在调试和监控时了解机器人的行为。