from algs.s2m2Weighted import s2m2W
from algs.ref2traj import ref2traj
from problems.util import read_problem,read_configuration
from timeit import default_timer

from viz.plot import plot_results
from viz.animate import animate_results


def test(env, problem_path, config_path):

    name, limits, Obstacles, agents, Thetas, Goals, *GoalWeights = read_problem(problem_path)
    
    min_segs, max_segs, obs_steps = read_configuration(config_path)

    if len(GoalWeights) == 0:
        GoalWeights = 1

    start = default_timer()
    refs = s2m2W(agents, Thetas, Goals, GoalWeights, limits, Obstacles, min_segs, max_segs, obs_steps, 0)
    end = default_timer()
    print("Total Time = ", end - start)
    print("Individual makespans:",["{}:{}".format(i,tp[-1][0]) for i,tp in enumerate(refs)])
    return refs


if __name__ == '__main__':
    env = 'zigzag'
    test(env, "problems/{}/problem.yaml".format(env), "problems/{}/config.yaml".format(env))
