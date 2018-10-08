from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import numpy as np
import operator

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

from framework import Framework
from autonomous import Autonomous
from traffic import Traffic
from safety import Safety
from tensor import Tensor


class Assembler:

    def __init__(self, carID):
        self.carID = carID
        self.rel = None
        self.distances_tfc = None



    def run(self, carID):
        """
        Executes the Traci client and the environment representation
        """
        ego = carID
        step = 0


        while traci.simulation.getTime() < 60:
            traci.simulationStep()

            fw = Framework()


            if carID in traci.vehicle.getIDList():

                ego_trajectory = fw.getTrajectory(carID)


                intersections = fw.getIntersectingTrajectories(carID, ego_trajectory)


                if intersections is not None:
                    #if len(ego_trajectory) > 1:
                    framework = fw.getTensorFramework(carID, intersections, ego_trajectory)

                    auto = Autonomous(carID, framework, ego_trajectory)

                    distances = auto.getDistanceToIntercept()

                    tfc = Traffic(distances)

                    vehicles = tfc.getConflictoryTraffic()

                    distances_tfc = tfc.getDistanceToIntercept(vehicles)

                    safe = Safety(carID, framework, distances, distances_tfc)

                    trc_safety = safe.getTrafficSafetyMeasures()
                    #print(len(distances), len(trc_safety))

                    ego_safety = safe.getEgoSafetyMeasures()
                    #print(ego_safety)

                    prio = safe.getPriotizedTraffic(ego_safety, trc_safety)

                    rel = safe.getRelevantTraffic(prio)
                    #print(rel)

                    # for i in range(len(rel)):
                    #     if rel[i][1]:
                    #         safe.getEgoSecurityDistance(carID, rel[i][1][0][0])

                    env = Tensor(carID, distances, rel, ego_safety, ego_trajectory)
                    env.createEnvironmentTensor()
                    #print(env.createEnvironmentTensor())
                else:
                    corridor = np.full((200, 3), fill_value=-1.0)

                    corridor[0:200, 2] = 100 / np.maximum(traci.vehicle.getSpeed(self.carID), 0.001)

                    #print(corridor)

            step += 1
        traci.close()
        sys.stdout.flush()

    def getTensor(self):
        fw = Framework()

        if self.carID in traci.vehicle.getIDList():

            ego_trajectory = fw.getTrajectory(self.carID)

            intersections = fw.getIntersectingTrajectories(self.carID, ego_trajectory)

            if intersections is not None:
                #if len(ego_trajectory) > 1:
                    framework = fw.getTensorFramework(self.carID, intersections, ego_trajectory)

                    auto = Autonomous(self.carID, framework, ego_trajectory)

                    distances = auto.getDistanceToIntercept()

                    tfc = Traffic(distances)

                    vehicles = tfc.getConflictoryTraffic()

                    distances_tfc = tfc.getDistanceToIntercept(vehicles)
                    self.distances_tfc = distances_tfc

                    safe = Safety(self.carID, framework, distances, distances_tfc)

                    trc_safety = safe.getTrafficSafetyMeasures()
                    # print(len(distances), len(trc_safety))

                    ego_safety = safe.getEgoSafetyMeasures()
                    # print(ego_safety)

                    prio = safe.getPriotizedTraffic(ego_safety, trc_safety)

                    rel = safe.getRelevantTraffic(prio)
                    #print('\n', prio)
                    #print(rel)
                    self.rel = rel

                    # print(rel)

                    # for i in range(len(rel)):
                    #     if rel[i][1]:
                    #         safe.getEgoSecurityDistance(carID, rel[i][1][0][0])

                    env = Tensor(self.carID, distances, rel, ego_safety, ego_trajectory)
                    #print(env.createEnvironmentTensor())

                    return env.createEnvironmentTensor()
            else:
                corridor = np.full((200, 3), fill_value=-1.0)

                #corridor[0:199, 2] = 0
                corridor[0:200, 2] = 100 / np.maximum(traci.vehicle.getSpeed(self.carID), 0.001)

                #tensor = np.reshape(corridor[0:200], 600)

                return corridor

    def getRelevantTraffic(self):
        return self.rel

    def getTraffic(self):
        return self.distances_tfc


if __name__ == "__main__":

    net = sumolib.net.readNet('two_intersections/two_intersections.net.xml')


    # start_gui = input("To start sumo-gui, type 'g' or enter otherwise: ")
    #
    # sumo_command = 'sumo'
    # if start_gui == 'g':
    #     sumo_command = "sumo-gui"

    #traci.start(["sumo-gui", "-c", "two_intersections/two_intersections.sumocfg", "--start"])
    #traci.start(["sumo", "-c", "two_intersections/two_intersections.sumocfg", "--start"])

    #traci.start(["sumo-gui", "-c", "one_intersection_w_priority/one_intersection_w_priority.sumocfg", "--start"])
    traci.start(['sumo', "-c", "one_lane/one_lane.sumocfg","--start"])

    # traci.start(["sumo", "-c", "one_intersection_w_priority/one_intersection_w_priority.sumocfg", "--start"])

    assemble = Assembler('ego')
    assemble.run('ego')