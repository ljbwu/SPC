"""
author: liujiebwu@163.com
"""
import argparse
import time

import yaml
from a_spc import Environment, SPC

import sys
sys.path.insert(0, '../')


# 获取货架信息
def allRacks(agents):
    racks = []
    for agent in agents:
        for plan in agent["plan"]:
            racks.append(plan['racks'])
    return racks

# 获取拣选台信息,并去重
def allPickers(agents):
    pickers = []
    for agent in agents:
        for plan in agent["plan"]:
            pickers.append(plan['picker'])
    set_list = [list(t) for t in set(tuple(_) for _ in pickers)]
    return set_list


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-param", default="input.yaml", help="包含地图和障碍物坐标的输入文件")
    parser.add_argument("-output",default="result/output.yaml",  help="带有时间表的智能体坐标输出文件")
    args = parser.parse_args()

    # Read from input file
    with open(args.param, 'r',encoding='utf-8') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]  # 获取地图尺寸
    agents = param["agents"]  # 获取智能体信息
    racks = allRacks(agents)  # 获取货架的信息
    pickers = allPickers(agents)   # 获取智能体信息

    rack_static = param['rack_static']  # 不需要拣选的货架
    reserve_node = param["reserve_node"]  # 保留节点
    block_node = param["block_node"]      # 障碍物节点
    racks.extend(rack_static)
    pickers.append([0, 0])      # 限制充电桩坐标
    env = Environment(dimension, agents, racks, pickers, reserve_node, block_node, rack_static)  # 创建地图环境
    # Searching
    spc = SPC(env)
    solution = spc.custom_search()
    if not solution:
        print(" Solution not found")
        return

    # Write to output file
    with open(args.output, 'r',encoding='utf-8') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml,sort_keys=False)


if __name__ == "__main__":
    # 1-获取开始时间
    startTime = time.time()
    # 需要执行的函数或程序
    main()
    # 2-获取结束时间
    endtime = time.time()
    # 3-获取时间间隔
    diffrentTime = endtime - startTime
    print('程序运行时间：', diffrentTime)
