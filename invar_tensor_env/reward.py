


import numpy as np
#from env_tensor import Environment
import traci
import sumolib

class Reward:

    #net = sumolib.net.readNet('huge_crossing/huge_crossing.net.xml')
    #net = sumolib.net.readNet('one_lane/one_lane.net.xml')

    # def __init__(self, carID, tensor):
    #     self.carID = carID
    #     self.tensor = tensor

    def __init__(self, carID, rel):
        self.carID = carID
        self.conflictories = rel


    def optimum_speed_deviation(self):
        allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))
        current_speed = traci.vehicle.getSpeed(self.carID)
        deviation = allowed_speed - current_speed
        if deviation > 0:
            return np.negative(np.absolute(np.power(deviation, 1.5)))
        elif deviation < 0:
            return np.negative(np.absolute(np.power(deviation, 2)))

    # def collision(self):
    #     pain = 10000
    #     num_vec = traci.vehicle.getIDList()
    #     #print(num_vec)
    #     euklid_dist_list = []
    #     for i in range(len(num_vec)):
    #         if num_vec[i] != 'ego':
    #             euklid_dist = np.sqrt(np.sum(np.square(np.subtract(traci.vehicle.getPosition('ego'), traci.vehicle.getPosition(num_vec[i])))))
    #             euklid_dist_list.append(euklid_dist)
    #
    #     for j in range(len(euklid_dist_list)):
    #         if euklid_dist_list[j] < 1:
    #             return np.negative(pain), True
    #     return 0, False

    def collision(self):
        #print('###', traci.simulation.getCollidingVehiclesIDList())
        if 'ego' in traci.simulation.getCollidingVehiclesIDList():
        #if len(traci.simulation.getStartingTeleportIDList()) > 0:
            return -10000, True
        else:
            return 0, False

    def emergency_brake(self):
        #print(self.conflictories)
        if self.conflictories:
            pain = 0
            for i in range(len(self.conflictories)):
                if traci.vehicle.getPosition('ego') is not None and traci.vehicle.getPosition(self.conflictories[i][2][0][0]) is not None:
                    euklid_dist = np.sqrt(np.sum(np.square(np.subtract(traci.vehicle.getPosition('ego'), traci.vehicle.getPosition(self.conflictories[i][2][0][0])))))
                    allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.conflictories[i][2][0][0]))
                    current_speed = traci.vehicle.getSpeed(self.conflictories[i][2][0][0])
                    deviation = 1 / np.maximum((current_speed / allowed_speed), 000.1)
                    signals = traci.vehicle.getSignals(self.conflictories[i][2][0][0])

                    if signals == 8 and euklid_dist < 30:
                        pain = pain + np.negative(deviation * 100)
                #print('###',pain)
                return pain
        else:
            return 0



        # 0
        # 17.571608212423886
        # 13.89

        # 0
        # 31.222660252413277
        # 13.89

        # 0
        # 39.72652898441444
        # 13.89

        # 8
        # 7.0545658130535704
        # 12.89

        # 0
        # 155.27166319877978
        # 8.8

        # 8
        # 20.09021599642559
        # 12.89



    # def collision(self):
    #     pain = 1000
    #     tens = self.tensor
    #
    #     if tens is not None:
    #         num_vec = traci.vehicle.getIDList()
    #         euklid_dist_list = []
    #         for i in range(len(num_vec)):
    #             if num_vec[i] != 'ego':
    #                 euklid_dist = np.sqrt(np.sum(np.square(np.subtract(traci.vehicle.getPosition('ego'), traci.vehicle.getPosition(num_vec[i])))))
    #                 euklid_dist_list.append(euklid_dist)
    #         #print(min(euklid_dist_list))
    #         if min(euklid_dist_list) < 1:
    #             return np.negative(pain), True
    #         # for i in range(200):
    #         #     if (tens[i, 0] < (tens[i, 2])) and (tens[i, 1] > (tens[i, 2])) and (tens[i, 2] < 0.4):
    #         #         return np.negative(pain), True
    #     return 0, False


    # def target(self):
    #     if traci.lane.getEdgeID(traci.vehicle.getLaneID('ego')) == traci.vehicle.getRoute('ego')[-1]:
    #         return 1000
    #     else:
    #         return 0


    # def overtime(self):
    #     if traci.simulation.getCurrentTime() < 200000:
    #         return 0, False
    #     else:
    #         return -1000, True
    #
    #
    #
    #





















