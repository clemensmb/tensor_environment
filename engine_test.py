from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from assembler import Assembler

from action import Action
from reward import Reward
import sumolib
import traci
import numpy as np


from tensorforce.environments import Environment



class Engine_Test(Environment):

    def __init__(self, carID):
        self.carID = carID
        self.output = None

    def __str__(self):
        return 'Invariant Tensor'


    def reset(self):
        traci.load(["-c", "huge_crossing/huge_crossing.sumocfg"])
        terminate = False
        while terminate == False:
            for i in range(len(num_vec)):
                if num_vec[i] != 'ego':
                    traci.vehicle.setLaneChangeMode(num_vec[i], 512)
                    self.output = run.getTensor()
                    if self.output is not None:
                        terminate = True
            traci.simulationStep()

        return self.getObservation()


    def close(self):
        traci.close()


    def execute(self, action):
        #traci.vehicle.setSpeedMode('ego', 0)
        #print(action)
        run = Assembler(self.carID)

        # self.output = tensor.executeTensor()
        self.output = run.getTensor()
        # print(self.output)
        if self.output is None:
            term = True
            return self.getObservation(), term, 0
        # rew = Reward('ego', self.output)
        rew = Reward('ego')

        coll, term = rew.collision()
        if term is True:
            cost = coll
            return self.getObservation(), term, cost

        traci.vehicle.setSpeedMode('ego', 0)
        num_vec = traci.vehicle.getIDList()
        for i in range(len(num_vec)):
            if num_vec[i] != 'ego':
                traci.vehicle.setLaneChangeMode(num_vec[i], 512)
        # print(action)
        carID = 'ego'
        act = Action(carID)
        if action == 0:
            pass
            #act.decelerate()
            #print('dec')
        elif action == 1:
            pass
            #act.accelerate()
            #print('acc')
        else:
            pass
            #act.remain()
            #print('rem')
        traci.simulationStep()

        #net = sumolib.net.readNet('huge_crossing/huge_crossing.net.xml')
        #tensor = Tensor('ego', net, self.conflictory_list[0], self.connection_list[0])
        #self.output = tensor.executeTensor()
        #rew = Reward('ego', self.output)

        win = rew.target()
        cost = rew.optimum_speed_deviation() + win
        # + brake
        traci.simulationStep()

        return self.getObservation(), term, cost



    @property
    def states(self):
        return dict(shape=(200, 3), type='float')


    @property
    def actions(self):
        return dict(num_actions=3, type='int')


    def getObservation(self):
        return self.output


























