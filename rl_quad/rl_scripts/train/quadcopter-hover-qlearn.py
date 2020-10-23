import gym
import time
import numpy as np, pandas as pd
from rl_quad.rl_algorithms.qlearn.qlearn import QLearn
from rl_quad.environment.continous import Environment
import matplotlib.pyplot as plt
from rl_quad.utils.quadPlot import plot_quad_3d

if __name__ == "__main__":
    env = Environment()
    qlearn = QLearn(actions=range(env.action_space.n), alpha=0.1, gamma=0.9, epsilon=0.9)
    done = False
    cumulated_reward = 0
    total_episodes = 100000
    initial_epsilon = qlearn.epsilon
    epsilon_discount = 0.9998
    max_number_of_steps = 2000
    start_time = time.time()
    print(env.action_space)
    highest_reward = 0

    last_time_steps = np.ndarray(0)
    for x in range(total_episodes):

        done = False
        cumulated_reward = 0 #Should going forward give more reward then L/R ?
        highest_reward = 0
        env.reset()    

        state = env.get_state
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        #render() #defined above, not env.render()     

        for i in range(max_number_of_steps):
                # Pick an action based on the current state

            print("Episode: {}, Timestep: {}".format(x, i))
            action = qlearn.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step(action)
            cumulated_reward += reward

            print("Reward: {}, Done: {}, Cumulative Reward: {}".format(reward, done, cumulated_reward))
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = env.get_state
            visited = qlearn.learn(state, action, reward, nextState)

            if not(done):
                state = nextState
            else:
                last_time_steps = np.append(last_time_steps, [int(i + 1)])
                print("Completed the episode: {} in {} steps due to reaching the point".format(x, i))
                break 

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))    

        print("x: {}, cumulated_reward: {}".format(x, cumulated_reward))
        if visited:
            plt.plot(x, cumulated_reward, ".g-")
            plt.plot(x, highest_reward, ".b-")
        else:
            plt.plot(x, cumulated_reward, ".r-")
            plt.plot(x, highest_reward, ".b-")
        plt.pause(0.05)
    
    plt.draw()
    # env.plot("rl")

    print ("\n|"+str(total_episodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |")

    l = last_time_steps.tolist()
    l.sort()
    print(l)
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("q: {}".format(qlearn.q))


