import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    time = 0
    while (time < max(len(path1), len(path2))):
        if (get_location(path1, time) == get_location(path2, time)):
            # vertex collision
            return {
                'loc': [get_location(path1, time)],
                'timestep': time
            }
        if (time + 1 < min(len(path1), len(path2))):
            if (get_location(path1, time) == get_location(path2, time + 1) and get_location(path1,
                                                                                            time + 1) == get_location(
                    path2, time)):
                return {
                    'loc': [get_location(path1, time), get_location(path2, time)],
                    'timestep': time + 1
                }
        time += 1
    return None


def detect_collisions(paths):
    collision_list = []
    for agent1 in range(len(paths)):
        for agent2 in range(agent1 + 1, len(paths)):
            first_collision = detect_collision(paths[agent1], paths[agent2])
            if (first_collision != None):
                collision_list.append(
                    {
                        'a1': agent1,
                        'a2': agent2,
                        'loc': first_collision['loc'],
                        'timestep': first_collision['timestep']
                    }
                )
    return collision_list


def standard_splitting(collision):
    constraints = [
        {
            'agent': collision['a1'],
            'loc': collision['loc'],
            'timestep': collision['timestep']
        },
        {
            'agent': collision['a2'],
            'loc': collision['loc'],
            'timestep': collision['timestep']
        }
    ]
    return constraints

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):

    def __init__(self, my_map, starts, goals):

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        print(root['collisions'])

        for collision in root['collisions']:
            print(standard_splitting(collision))

        while len(self.open_list) > 0:
            curr = self.pop_node()
            print('current node: ', curr)
            if (len(curr['collisions']) == 0):
                return curr['paths']

            collision = curr['collisions'][0]
            constraints = standard_splitting(collision)
            for constraint in constraints:
                joined_constraint = curr['constraints'] + [constraint]
                q = {
                    'cost': 0,
                    'constraints': joined_constraint,
                    'paths': curr['paths'] + [],
                    'collisions': [],
                }
                agent = constraint['agent']
                path1 = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                               agent, q['constraints'])
                if path1 is not None:
                    q['paths'][agent] = path1
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    self.push_node(q)


        self.print_results(root)

        return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
