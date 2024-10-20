import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):

    def __init__(self, my_map, starts, goals):
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        start_time = timer.time()
        result = []
        constraints = []
        mymap_size = len(max(self.my_map)) * len(self.my_map)
        print("mymap_size: ", mymap_size)
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            for j in range(self.num_of_agents):
                if (j == i):
                    continue
                timestep = 0
                for cell in path:
                    constraints.append(
                        {
                            'agent': j,
                            'loc': [cell],
                            'timestep': timestep
                        }
                    )
                    timestep += 1
                    if (timestep < len(path)):
                        constraints.append(
                            {
                                'agent': j,
                                'loc': [cell, path[timestep]],
                                'timestep': timestep
                            }
                        )
                # add constraint once it reaches goal to the upper bound timestep

                while (timestep <= mymap_size):
                    constraints.append(
                        {
                            'agent': j,
                            'loc': [path[-1]],
                            'timestep': timestep
                        }
                    )
                    timestep += 1
            result.append(path)
        self.CPU_time = timer.time() - start_time

        print(result)
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        
        return result
