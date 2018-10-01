
from __future__ import absolute_import
from __future__ import print_function

import logging
import os
import sys
import numpy as np
import operator
from collections import defaultdict
import json


from tensorforce.agents import PPOAgent
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


from test_engine import Engine



if __name__ == "__main__":


    t_env = Engine('ego')

    # network = [
    #     dict(type='flatten'),
    #     dict(type='dense', size=32),
    #     dict(type='dense', size=32)
    # ]
    #
    # with open("./agent_specs/dqn_test.json", "r") as fp:
    #     agent_config = json.load(fp=fp)

    with open("./agent_specs/ppo.json", "r") as fp:
            agent_config = json.load(fp=fp)

    with open("./agent_specs/mlp2_network.json", "r") as fp:
            network = json.load(fp=fp)

    agent = Agent.from_spec(
        spec=agent_config,
        kwargs=dict(
            states=t_env.states,
            actions=t_env.actions,
            network=network
        )
    )

    terminal = False
    #agent.restore_model(directory="./models/", file='-7065')
    agent.restore_model(directory="./models/")
    #t_env.generate_routefile()
    #traci.start(['sumo-gui', "-c", "huge_crossing/huge_crossing.sumocfg"])

    #traci.start(['sumo-gui', "-c", "one_lane/one_lane.sumocfg"])

    #traci.start(['sumo-gui', "-c", "three_lanes/three_lanes.sumocfg", "--start"])

    t_env.generate_routefile_two_intersections()
    traci.start(['sumo-gui', "-c", "two_intersections/two_intersections.sumocfg", "--start"])

    #traci.start(['sumo', "-c", "three_lanes/three_lanes.sumocfg"])

    #traci.start(['sumo', "-c", "one_lane/one_lane.sumocfg"])

    state = t_env.reset()
    while not terminal:
        action = agent.act(state)
        print(action)
        state, terminal, reward = t_env.execute(action)
        print(state)
        print(reward)

    # sumo_env = Engine_Test('ego')
    #
    #
    # sumo_agent = DQNAgent(
    #     states=sumo_env.states,
    #     actions=sumo_env.actions,
    #     network=[
    #         dict(type='flatten'),
    #         dict(type='dense', size=32),
    #         dict(type='dense', size=32)
    #     ],
    #     batching_capacity=1000,
    #     actions_exploration={
    #     "type": "epsilon_anneal",
    #     "initial_epsilon": 0.5,
    #     "final_epsilon": 0.0,
    #     "timesteps": 10000
    # },
    # )
    #
    #
    # runner = Runner(sumo_agent, sumo_env)
    #
    #
    #
    #
    # traci.start(['sumo-gui', "-c", "huge_crossing/huge_crossing.sumocfg"])
    #
    # runner.run(episodes=1, max_episode_timesteps=200, episode_finished=episode_finished)
    # runner.close()





















