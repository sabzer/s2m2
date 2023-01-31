import numpy as np
from queue import PriorityQueue
from networkx import DiGraph,topological_sort,ancestors,descendants
from copy import deepcopy
from timeit import default_timer

from algs.xref import get_gb_xref
from algs.collision import find_collision,traj2obs

def s2m2W(models, thetas, goals, goal_weights, limits, obstacles,
                       min_segs, max_segs, obs_steps, min_dur, timeout = 300):
    n_agent = len(models)
    params = [models, thetas, goals, limits, obstacles, min_segs, max_segs, obs_steps, min_dur]

    print("\n----- 0 -----")
    root = Node(n_agent,goal_weights)
    for idx in range(n_agent):
        plan = root.update_plan(idx, params)
        if plan is None:
            print('Individual planning {} is infeasible'.format(idx))
            return None
    q = PriorityQueue()
    q.put(root)
    # print('q',q)
    times = 0
    start_time = default_timer()
    while not q.empty():

        if default_timer() - start_time > timeout:
            print("Timeout!")
            return None
        times = times + 1
        node = q.get()
        print("\n-----", times, "-----")
        print("Orders:", node.G.edges)
        print("Weighted flowtime =", node.flowtime)
        # Check Collision
        start_coll = default_timer()
        colliding_agents = node.earliest_colliding_agents(params)
        print("Collision check time:", default_timer()-start_coll,"Colliding Agents:", colliding_agents)
        # Solution Found
        if colliding_agents is None:
            print("[*] Solution Found!")
            return node.plans
        # Resolve Collision
        for (j, i) in [colliding_agents, (colliding_agents[1], colliding_agents[0])]: # This is the branching of the PT node.
            # Update Node
            print("\n[*] Split on (%s < %s)"%(j, i))
            new_node = node.copy()
            new_node = new_node.update_node((j, i), params)
            if new_node is None: print("Infeasible")
            else:
                print("Feasible and Push")
                q.put(new_node) 
                
        # The use of priority queue data structure seem to make the algorithm a Best FS instead of DFS.
        # The authors in S2M2 claim they use DFS when growing the PT in the paper, which is inconsistent with the code.

        # The nodes in the priority queue are always completely sorted, thus for the Best FS.
        # The nodes in the stack are stored in a LIFO way, thus for the DFS.

        # In the PBS paper, the nodes are stored in a modified stack, I call it the 'greedy stack'. 
        # It is a stack formed with some priority information. 
        # Every time only two children nodes are generated. The child with the smaller cost is placed upon the other, thus explored first.
        # Thus the growth of PT in the PBS paper is a greedy DFS.
    return None

class Node:
    def __init__(self, n_agent,weights = None, plans = [], flowtime = 0,
                G = DiGraph(), orders = set(),
                collisions = {}):
        
        self.n_agent = n_agent
        
        if plans == []: plans = [None for i in range(n_agent)]
        self.plans = plans

        if weights is None:
            self.weights = 1
        else:
            self.weights = weights
        
        self.flowtime = flowtime
        
        G.add_nodes_from(list(range(n_agent)))
        if orders != set():  G.add_edges_from(orders)
        self.G = G
        
        self.collisions = collisions # only specified for agents with certain orderigns

    def __lt__(self, other):
        if len(self.G.edges) > len(other.G.edges): return True
        elif len(self.G.edges) < len(other.G.edges): return False
        else:
            return self.flowtime < other.flowtime

    def copy(self, memodict={}):
        return Node(self.n_agent,
                    plans = deepcopy(self.plans), weights = self.weights, flowtime = self.flowtime,
                    G = self.G.copy(), collisions = deepcopy(self.collisions))

    def agents(self):
        return set(list(range(self.n_agent)))

    def higher_agents(self, idx):
        return ancestors(self.G, idx)

    def lower_agents(self, idx):
        return descendants(self.G, idx)

    def sorted_agents(self, nodes):
        H = self.G.subgraph(nodes)
        return list(topological_sort(H))

    def add_order(self, order):
        higher_agents = self.higher_agents(order[1])
        lower_agents = self.lower_agents(order[1])
        if order[0] in higher_agents: status = "existing"
        elif order[0] in lower_agents: status = "inconsistent"
        else:
            self.G.add_edge(*order)
            status = "added"
        return status

    def earliest_colliding_agents(self, params):

        models, _, _, _, _, _, _, _, _ = params

        earliest_time = 1e6
        colliding_agents = None
        for i in range(self.n_agent):
            for j in self.agents() - set([i]):
                time = 1e8
                # print(" -- Check Collison for (%s, %s)" % (i, j))
                if ("%s-%s" % (i, j) in self.collisions
                        and self.collisions["%s-%s" % (i, j)] == "free"):
                    1
                    # print(" -- [Collision Free] Previously Checked.")
                elif ("%s-%s" % (i, j) in self.collisions
                      and self.collisions["%s-%s" % (i, j)] != "free"):
                    # print(" -- [Collision Found] Previously Checked.")
                    time = self.collisions["%s-%s" % (i, j)]
                else:
                    area, _ = find_collision(self.plans[i], models[i],
                                             self.plans[j], models[j])
                    if area is not None:
                        # print(" -- [Collision Found].")
                        self.collisions["%s-%s" % (i, j)] = area[0]
                        self.collisions["%s-%s" % (j, i)] = area[0]
                        time = area[0]
                    else:
                        # print(" -- [Collision Free].")
                        self.collisions["%s-%s" % (i, j)] = "free"
                        self.collisions["%s-%s" % (j, i)] = "free"
                if time < earliest_time:
                    earliest_time = time
                    colliding_agents = (i, j)
                    print(" -- [\] Update Colliding Agents.", i, "-", j)
        return colliding_agents

    def update_node(self, order, params):
        # Check Ordering Status
        status = self.add_order(order)
        print("Adding Order Status: ", status)
        # assert status == "added"
        if status == "inconsistent": return None
        # Plan for All the Affected Agents
        idx = order[1]
        models, _, _, _, _, _, _, _, _ = params
        agents = [idx] # Only Update Affected Agents
        # agents = [idx] + self.sorted_agents(self.lower_agents(idx))
        print("Agents to Update:", agents)
        for k in agents:
            print("- Update Plan for Agent", k)
            higher_agents = self.higher_agents(k)
            # check colliding
            replan = False
            for j in higher_agents:
                # print(" -- Check Collison for (%s, %s)" % (k, j))
                if ("%s-%s" % (k, j) in self.collisions
                        and self.collisions["%s-%s" % (k, j)] == "free"):
                    1
                    # print(" -- [Collision Free] Previously Checked.")
                elif ("%s-%s" % (k, j) in self.collisions
                      and self.collisions["%s-%s" % (k, j)] != "free"):
                    # print(" -- [Collision Found] Previously Checked.")
                    replan = True
                    break
                else:
                    if self.plans[j] is not None:
                        self.collisions["%s-%s" % (k, j)] != area[0]
                        self.collisions["%s-%s" % (j, k)] != area[0]
                        area, _ = find_collision(self.plans[k], models[k],
                                                 self.plans[j], models[j])
                    if area is not None:
                        self.collisions["%s-%s" % (k, j)] != "free"
                        self.collisions["%s-%s" % (j, k)] != "free"
                        # print(" -- [Collision Found].")
                        replan = True
                        break
                    # else: print(" -- [Collision Free].")

            # Update If colliding
            if k == idx or replan:
                plan = self.update_plan(k, params)
                if plan is None:
                    print("- Update Fails for Agent", k)
                    return None
                print("- Update Succeed for Agent", k)
            else: print("- No need to update for Agent", k)
        return self

    def update_plan(self, k, params):

        models, thetas, goals, limits, obstacles, min_segs, max_segs, obs_steps, min_dur = params

        print("- Find Collision and Reroute")
        
        MO = []
        for j in self.higher_agents(k):
            MO = MO + traj2obs(self.plans[j], models[j], steps = obs_steps)
     
        Nmin, Nmax = min_segs, max_segs

        # The next line calls the solver for single-agent path planning.
        new_plan = get_gb_xref([models[k]], [thetas[k]], [goals[k]],
                            limits, obstacles, MO, Nmin, Nmax, min_dur = min_dur)
        
        if new_plan is None:
            print("- No Plan.")
            return None
        else:
            # Update Plan and flowtime
            plan = new_plan[0]
            self.plans[k] = plan
            
            ##########################################################################

            individual_times = []
            for p in self.plans:
                if p is not None:
                    individual_times.append(p[-1][0])
                else:
                    individual_times.append(0)
            
            self.flowtime = np.sum(individual_times * np.array(self.weights))
            
            ###########################################################################

            print("- Single-agent Feasible Plan Found\n -- Weighted flowtime {}\n -- Unweighted flowtime {}".format(self.flowtime,np.sum(individual_times)))

            
            for j in self.higher_agents(k):
                self.collisions["%s-%s" % (k, j)] = "free"
                self.collisions["%s-%s" % (j, k)] = "free"
            # Unfreeze Collision Indicator
            for j in self.agents() - self.higher_agents(k):
                if "%s-%s" % (k, j) in self.collisions: self.collisions.pop("%s-%s" % (k, j))
                if "%s-%s" % (j, k) in self.collisions: self.collisions.pop("%s-%s" % (j, k))
           
            return plan
