"""
AStar search
author: liujiebwu@163.com
"""

class AStar():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name):
        """
        low level search 
        """
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}   # 保存父节点

        g_score = {}
        g_score[initial_state] = 0

        f_score = {}

        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)

        while open_set:
            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current = min(temp_dict, key=temp_dict.get)

            if self.is_at_goal(current, agent_name):
                return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current.location}

            neighbor_list = self.get_neighbors(current,agent_name)   # 从当前节点出发可到达的位置

            for neighbor in neighbor_list:
                if neighbor.location in closed_set:
                    continue
                
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                open_set_location = [open_item.location for open_item in open_set]
                if neighbor.location not in open_set_location:   # 如果邻居节点不在开始列表，将邻居节点加入开始列表
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):  # 如果在开始列表，比较G值，若G值变大，结束本次循环
                    continue

                came_from[neighbor] = current   # 改变邻居节点的父节点

                g_score[neighbor] = tentative_g_score  # G 重新赋值
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)  # 邻居节点的计算F值
        return False

