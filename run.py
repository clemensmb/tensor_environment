
from __future__ import absolute_import
from __future__ import print_function

import logging
import os
import sys
import numpy as np
import operator
from collections import defaultdict
import json


#from tensorforce.agents import PPOAgent
from tensorforce.agents import DQNAgent
from tensorforce.execution import Runner
from tensorforce.agents import agents as AgentsDictionary, Agent


try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
import sumolib


from engine import Engine



if __name__ == "__main__":


    env = Engine('ego')

    num_episodes = 1000


    # network = [
    #     #dict(type='embedding', indices=100, size=32),
    #     dict(type='flatten'),
    #     dict(type='dense', size=32),
    #
    #     dict(type='dense', size=32)
    # ]

    #with open ("./agent_specs/dqn.json", "r") as fp:
    with open("./agent_specs/ppo_test.json", "r") as fp:
            agent_config = json.load(fp=fp)

    #with open("./agent_specs/mlp2_network.json", "r") as fp:
    with open("./agent_specs/mlp2_lstm_network.json", "r") as fp:
            network = json.load(fp=fp)



    agent = Agent.from_spec(
        spec=agent_config,
        kwargs=dict(
            states=env.states,
            actions=env.actions,
            network=network,
            # summarizer=dict(directory="./board/",
            #                   steps=50,
            #                   labels=['configuration',
            #                           'gradients_scalar',
            #                           'regularization',
            #                           'inputs',
            #                           'losses',
            #                           'variables']
            #                   ),
    )
    )


    runner = Runner(agent, env)


    def episode_finished(r):
        print(
            "Finished episode {ep} after {ts} timesteps (reward: {reward})".format(ep=r.episode, ts=r.episode_timestep,
                                                                                   reward=r.episode_rewards[-1]))
        if r.episode == num_episodes:
            r.agent.save_model(directory="./models/")

        return True


    print("Starting {agent} for Environment '{env}'".format(agent=agent, env=env))

    #env.generate_routefile_two_intersections()
    #traci.start(['sumo-gui', "-c", "two_intersections/two_intersections.sumocfg", "--start"])
    #traci.start(['sumo', "-c", "two_intersections/two_intersections.sumocfg"])
    traci.start(['sumo', "-c", "one_lane/one_lane.sumocfg", '--no-step-log', 'true'])
    #traci.start(['sumo-gui', "-c", "one_lane/one_lane.sumocfg", "--start"])



    runner.run(episodes=num_episodes, max_episode_timesteps=10000, episode_finished=episode_finished)

    print("Learning finished. Total episodes: {ep}. Average reward of last 1000 episodes: {ar}.".format(
        ep=runner.episode,
        ar=np.mean(runner.episode_rewards[-1000:])))
    runner.close()























