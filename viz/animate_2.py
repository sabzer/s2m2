import matplotlib.animation as animation
from viz.util import *
import os
from models.agent import *

def animate_results(models,limits, obstacles, Thetas, goals, paths, save_to_path=None):
    agent_num = len(models)
    dim = len(limits)
    fig = plt.figure()
    
    fig,axes = plot_env(fig,limits, obstacles)
    plot_goals(goals)
    interval = 20
    total_frames = 500
    total_time = max([paths[idx][-1][-1] for idx in range(agent_num)])

    if dim == 2:
        
        tpf = total_time / total_frames
        frame = [0]

        ref_patches = []
        tru_patches = []
        err_patches = []        

        def init():
            # An init function is necessary for the blit option in animation to work, but we don't have to do anything in it.
            if len(ref_patches)==0:
                for idx in range(agent_num):
                    # print(idx)
                    ref_x, ref_y, _, tru_x, tru_y, _, times = paths[idx]
                   
                    ref_patch = plt.Circle((ref_x[0], ref_y[0]), 0, fc='k')
                    tru_patch = plt.Circle((tru_x[0], tru_y[0]), models[idx].size, fc='navy', alpha = 0.7,label='True Traj' if idx==0 else '')
                    err_patch = plt.Circle((ref_x[0], ref_y[0]), models[idx].size, fc='lightsteelblue', alpha = 0.4,label = 'Ref. Traj' if idx==0 else '')
                    ref_patches.append(ref_patch)
                    tru_patches.append(tru_patch)
                    err_patches.append(err_patch)       
                   
                for patch in ref_patches + tru_patches + err_patches: axes.add_patch(patch)
                # print('showing legend')
            plt.legend()
            return ref_patches + tru_patches + err_patches

        def animate(f):
            # The f index does not represent the correct frame number if we are using blit.
            # Everytime init is called, f is reset.

            ref, tru = [], []
            for idx in range(agent_num):
                # Python will automatically parallelize this loop across agents!
                ref_x, ref_y, _, tru_x, tru_y, _, times = paths[idx]

                step = 0
                while (step < len(times) - 1) and (times[step] < tpf * f):
                    step = step + 1
                ref_patches[idx].center = (ref_x[step], ref_y[step])
                tru_patches[idx].center = (tru_x[step], tru_y[step])
                err_patches[idx].center = (ref_x[step], ref_y[step])
                if step == len(ref_x) - 1: error = models[idx].size
                else: error  = (models[idx].size + models[idx].bloating(step))
                err_patches[idx].width = 2 * error
                err_patches[idx].height = 2 * error            

                # print('frame',f,'step',step,'total step',len(times),times[step] , tpf * f,times[-1],'total_time',total_time)

            return ref_patches + tru_patches + err_patches


        ani = animation.FuncAnimation(fig, animate, frames = total_frames, init_func = init,
                                      blit=True, interval = interval)

        # fig.subplots_adjust(left=0.01, bottom=0.01, right=0.99, top=0.99, wspace=None, hspace=None)
        # path = os.path.abspath("results/plots/%s.mp4" % (name))
        if not save_to_path is None:
            ani.save(save_to_path, writer='ffmpeg')
        return ani