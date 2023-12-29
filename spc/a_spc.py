"""
author:liujiebwu@163.com
"""
from math import fabs
from itertools import combinations

from a_star import AStar


class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __hash__(self):
        return hash(str(self.x) + str(self.y))
    def __str__(self):
        return str((self.x, self.y))


""" status：0-小车为空, 1-小车搬运着货物 2-小车搬运着空货架,  """
class State(object):
    def __init__(self, time, location, status=0, z=0):
        self.time = time
        self.location = location
        self.status = status
        self.z = z
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))


class Conflict(object):
    VERTEX = 1
    EDGE = 2
    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'


class VertexConstraint(object):
    def __init__(self, time, location, agentName=''):
        self.time = time
        self.location = location
        self.agentName = agentName

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'


class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()   # 顶点约束，集合
        self.edge_constraints = set()     # 边约束，集合

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints]) + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])


class Environment(object):
    def __init__(self, dimension, agents, obstacles, pickers, reserve_node, block_node, rack_static):
        self.dimension = dimension  # 地图尺寸
        self.obstacles = obstacles  # 货架坐标
        self.pickers = pickers  # 拣选台坐标
        self.agents = agents        # 智能体信息
        self.reserve_node = reserve_node #保留节点
        self.block_node = block_node    # 障碍物节点
        self.rack_static = rack_static # 无需拣选货架的节点

        self.agent_plan = {}        # 智能体信息待执行规划信息
        self.agent_dict = {}
        self.make_agent_dict()  # 获取每个智能体的开始位置，结束位置，时间点

        self.constraints = Constraints()

        self.constraint_dict = {}
        self.a_star = AStar(self)

    def get_neighbors(self, state, agentName=None):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location, state.status)
        if self.state_valid(n, agentName) and self.transition_valid(state, n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1), state.status)
        if self.state_valid(n, agentName) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1), state.status)
        if self.state_valid(n, agentName) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y), state.status)
        if self.state_valid(n, agentName) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y), state.status)
        if self.state_valid(n, agentName) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors


    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):   # 智能体两两之间对比
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):   # 同一时刻占据同一个节点
                    result.time = t
                    result.type = Conflict.VERTEX    # 顶点碰撞
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):  # 同一时刻调换位置
                    result.time = t
                    result.type = Conflict.EDGE  # 边碰撞
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False


    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]


    def state_valid(self, state, agentName):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (([state.location.x, state.location.y] not in self.obstacles) or self.is_at_goal(state, agentName) or self.is_at_start(state, agentName) or ([state.location.x, state.location.y] in self.rack_static) and state.status==0) \
            and (([state.location.x, state.location.y] not in self.pickers) or self.is_at_goal(state, agentName) or self.is_at_start(state, agentName)) \
            and (([state.location.x, state.location.y] not in self.reserve_node)) \
            and (([state.location.x, state.location.y] not in self.block_node) or self.is_at_start(state, agentName) or self.is_at_goal(state, agentName))


    def transition_valid(self, state_1, state_2):
        existName = ""
        # 判断 state_2 的位置当前有没有智能体
        for vertex in self.constraints.vertex_constraints:
            if vertex.time == state_1.time and vertex.location == state_2.location:
                existName = vertex.agentName
        if len(existName) == 0:
            return True
        # existName 下一刻要去的位置是否为 state_1 的位置
        for vertex in self.constraints.vertex_constraints:
            if vertex.time == state_2.time and vertex.location == state_1.location and vertex.agentName == existName:
                return False
        return True



    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)


    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)


    def is_at_start(self, state, agent_name):
        start_state = self.agent_dict[agent_name]["start"]
        return state.is_equal_except_time(start_state)


    def make_agent_dict(self):
        for agent in self.agents:
            plans = agent["plan"][:]
            plan_0 = plans[0]
            # 充电桩坐标和第一个货架坐标不一致
            if (agent["avg_id"][0] != plan_0["racks"][0] or agent["avg_id"][1] != plan_0["racks"][1]):
                start_state = State(0, Location(agent["avg_id"][0], agent["avg_id"][1]), 0, 0)
                goal_state = State(0, Location(plan_0["racks"][0], plan_0["racks"][1]), 0, 1)
                self.agent_dict.update({agent['name']: {'start': start_state, 'goal': goal_state, 'plan_type': 0, 'start_time': 0}})  # 更新字典
            else:
                plan_0 = plans.pop(0)
                start_state = State(0, Location(plan_0["racks"][0], plan_0["racks"][1]), 1, 1)
                goal_state = State(0, Location(plan_0["picker"][0], plan_0["picker"][1]), 1, 0)
                self.agent_dict.update({agent['name']: {'start': start_state, 'goal': goal_state, 'plan_type': 1, 'start_time': 0}})  # 更新字典
            self.agent_plan.update({agent['name']: plans})



    def compute_single_solution(self, agentName):
        local_solution = self.a_star.search(agentName)
        if not local_solution:
            return False
        return local_solution



    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])


class SPC(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()

    def custom_search(self):
        # 遍历所有待执行的任务
        global goal_state
        result_solution = {}
        # 为所有的agent 初始化一个空的 solution
        for agentName in self.env.agent_dict.keys():
            result_solution[agentName] = []

        step_time = 0
        while self.env.agent_dict:
            ''' 对任务的优先级进行排序 '''
            exec_sort = []
            agent_dict = []
            for agentName in list(self.env.agent_dict.keys()):
                if self.env.agent_dict[agentName]['plan_type'] == 1:
                    exec_sort.append((agentName, 4))
                elif self.env.agent_dict[agentName]['plan_type'] == 2:
                    exec_sort.append((agentName, 3))
                elif self.env.agent_dict[agentName]['plan_type'] == 0:
                    exec_sort.append((agentName, 2))
                elif self.env.agent_dict[agentName]['plan_type'] == 3:
                    exec_sort.append((agentName, 1))
                else:
                    exec_sort.append((agentName, 0))
            exec_sort.sort(key=lambda x: x[1], reverse=True)
            for i in range(len(exec_sort)):
                agent_dict.append(exec_sort[i][0])

            for agentName in agent_dict:
                agentInfo = self.env.agent_dict[agentName]
                if agentInfo["start_time"] != step_time:
                    continue
                print(agentName, "开始寻找路径=> ",end="")
                solution = self.env.compute_single_solution(agentName)  # 低层采用A*算法为智能体找到一条路径
                print("currentTime=", step_time, "; [start_x=", agentInfo["start"].location.x, "start_y=",
                      agentInfo["start"].location.y,"][ end_x=", agentInfo["goal"].location.x, "; end_y=", agentInfo["goal"].location.y,
                      "] plan_type=", self.get_plan_type(agentInfo["plan_type"]) , end="    ======")
                if not solution:
                    # 记录当前状态，修改 agent_dict time + 1, 即等一步再尝试出发'
                    result_solution[agentName].append(agentInfo["start"])
                    start_state = State(agentInfo["start"].time + 1, agentInfo["start"].location, agentInfo["start"].status)
                    goal_state = State(agentInfo["goal"].time + 1, agentInfo["goal"].location, agentInfo["start"].status)
                    self.env.agent_dict.update({agentName: {'start': start_state, 'goal': goal_state,'plan_type': agentInfo['plan_type'], 'start_time': agentInfo["start_time"] + 1}})  # 更新字典
                    print("\033[91m【任务失败了】\033[0m")
                else:
                    print("\033[92m【任务成功了】\033[0m", "nextTime=", solution[len(solution)-1].time)
                    # self.env.rack_static.append([agentInfo["start"].location.x,agentInfo["start"].location.y])

                    for item in solution:
                        v_constraint = VertexConstraint(item.time, item.location, agentName)
                        self.env.constraints.vertex_constraints |= {v_constraint}
                    # 执行完计划的时间
                    last_time = solution[len(solution)-1].time
                    # 记录已找到的路径
                    result_solution[agentName].extend(solution)
                    # 判断是否还有货架
                    plans = self.env.agent_plan[agentName]
                    """plan_type: 任务/计划类型：0: 停车台-> 货架、 1:货架->拣选台 、 2:拣选台->货架、3:货架->货架  4:货架->充电桩(0, 0)
                       z:拐点改变状态标识位：0-该点无变化, 1-没有货物+货架, 2-有货架
                    """
                    plan_type = agentInfo["plan_type"]
                    if not plans and (plan_type==2 or plan_type==4):
                        if(plan_type!=2):
                            # 货架到充电撞-说明起点货架已搬完,后续这里没有等待的小车，则可以行走了
                            self.env.rack_static.append([agentInfo["start"].location.x, agentInfo["start"].location.y])
                            r_data = self.env.agent_dict.pop(agentName)
                            print(agentName, ":规划结束：", r_data)
                        else:
                            start_state = State(last_time, Location(agentInfo["goal"].location.x, agentInfo["goal"].location.y), 0, 2)
                            goal_state = State(last_time, Location(0, 0), 0, 0)
                            self.env.agent_dict.update({agentName: {'start': start_state, 'goal': goal_state, 'plan_type': 4, 'start_time': last_time}})  # 更新字典
                    else:
                        if(plan_type == 0):
                            # 从货架到拣选台
                            plan = plans.pop(0)
                            start_state = State(last_time,Location(agentInfo["goal"].location.x, agentInfo["goal"].location.y), 1, 1)
                            goal_state = State(last_time, Location(plan["picker"][0], plan["picker"][1]), 1, 0)
                            self.env.agent_dict.update({agentName: {'start': start_state, 'goal': goal_state, 'plan_type': 1, 'start_time': last_time}})  # 更新字典

                        if (plan_type == 1):
                            # 拣选台->货架
                            start_state = State(last_time, Location(agentInfo["goal"].location.x, agentInfo["goal"].location.y), 2, 0)
                            goal_state = State(last_time, Location(agentInfo["start"].location.x, agentInfo["start"].location.y), 2, 2)
                            self.env.agent_dict.update({agentName: {'start': start_state, 'goal': goal_state,'plan_type': 2, 'start_time': last_time}})  # 更新字典

                        if(plan_type == 2):
                            # 货架到货架
                            plan = plans[0]
                            start_state = State(last_time, Location(agentInfo["goal"].location.x, agentInfo["goal"].location.y), 0, 2)
                            goal_state = State(last_time, Location(plan["racks"][0], plan["racks"][1]), 0, 1)
                            self.env.agent_dict.update({agentName: {'start': start_state, 'goal': goal_state,'plan_type': 3, 'start_time': last_time}})  # 更新字典

                        if (plan_type == 3):
                            # 货架到货架-说明第一个货架已搬完,后续这里没有等待的小车，则可以行走了
                            self.env.rack_static.append([agentInfo["start"].location.x, agentInfo["start"].location.y])

                            #货架到拣选台
                            plan = plans.pop(0)
                            start_state = State(last_time, Location(agentInfo["goal"].location.x, agentInfo["goal"].location.y), 1, 1)
                            goal_state = State(last_time, Location(plan["picker"][0], plan["picker"][1]), 1, 0)
                            self.env.agent_dict.update({agentName: {'start': start_state, 'goal': goal_state,'plan_type': 1, 'start_time': last_time}})  # 更新字典

            step_time = step_time + 1
        return self.generate_plan(result_solution)



    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t': state.time, 'x': state.location.x, 'y': state.location.y, 'z': state.z, 'status': state.status} for state in path]
            plan[agent] = path_dict_list
        return plan


    def get_plan_type(self, plan_type):
        if plan_type == 0:
            return "0-起始 -> 货架"
        if plan_type == 1:
            return "1-货架 -> 拣选台 "
        if plan_type == 2:
            return "2-拣选台 -> 货架 "
        if plan_type == 3:
            return "3-货架 -> 货架 "
        if plan_type == 4:
            return "4-货架 -> 充电桩 "