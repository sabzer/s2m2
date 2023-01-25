from algs.decentralized import decentralized_algo
from algs.ref2traj import ref2traj
from problems.util import read_problem,read_configuration
from timeit import default_timer

from viz.plot import plot_results
from viz.animate import animate_results


def test(env, problem_path, config_path):
    name, limits, Obstacles, agents, Thetas, Goals = read_problem(problem_path)
    min_segs, max_segs, obs_steps = read_configuration(config_path)



    start = default_timer()
    refs = decentralized_algo(agents, Thetas, Goals, limits, Obstacles, min_segs, max_segs, obs_steps, 0)
    end = default_timer()
    print("Total Time = ", end - start)
    name = '[%s]'%(env)

    trajs = ref2traj(refs)
    # plot_results(agents, limits, Obstacles, Thetas, Goals, trajs, name, refs=refs)
    # animate_results(agents,  limits, Obstacles, Thetas, Goals, trajs, name)
    return refs
# env = 'parking'

if __name__ == '__main__':
    env = 'zigzag'
    test(env, "problems/{}/problem.yaml".format(env), "problems/{}/config.yaml".format(env))
