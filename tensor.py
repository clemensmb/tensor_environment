from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import numpy as np
import operator
from math import sin, radians, trunc

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

class Tensor:

    def __init__(self, carID, distances, relevant_tfc, ego_safety, ego_trajectory):
        self.carID = carID
        self.distances = distances
        self.relevant_tfc = relevant_tfc
        self.ego_safety = ego_safety
        self.ego_trajectory = ego_trajectory





    def roundDownTo05(self, number):
        """
        """
        #print(number)
        return trunc(number * 2) / 2



    def getEuclideanDistance(self, shape1, shape2):
        return np.sqrt(np.sum(np.square(np.subtract(shape1, shape2))))


    def getEgoSecurityDistance(self, egoID, ovID):

        def normalize_angle(angle):
            if angle > 360:
                return angle - 360
            else:
                return angle

        if ovID is not None:
            ego_ang_raw = traci.vehicle.getAngle(egoID)
            ov_ang_raw = traci.vehicle.getAngle(ovID)

            # Direction
            correction = 0
            if ego_ang_raw <= 180 and ov_ang_raw > 180:
                correction = 360 - ov_ang_raw
            elif ego_ang_raw > 180 and ov_ang_raw <= 180:
                correction = 360 - ego_ang_raw
            ego_ang = normalize_angle(ego_ang_raw + correction)
            ov_ang = normalize_angle(ov_ang_raw + correction)
            if ov_ang > ego_ang:
                if ov_ang <= ego_ang + 90:
                    same_direction = True
                else:
                    same_direction = False
            elif ov_ang <= ego_ang:
                if ov_ang >= ego_ang - 90:
                    same_direction = True
                else:
                    same_direction = False
            else:
                same_direction = False

            if ego_ang > 90 and ov_ang > 90:
                contact_angle = np.absolute(ego_ang - ov_ang)
            elif ego_ang <= 90 and ov_ang > 90:
                contact_angle = 90 - np.absolute((ego_ang + 90) - ov_ang)
            elif ego_ang > 90 and ov_ang <= 90:
                contact_angle = 90 - np.absolute(ego_ang - (ov_ang + 90))
            else:
                contact_angle = np.absolute(ego_ang - ov_ang)

            ego_breadth = traci.vehicle.getWidth(egoID)

            ov_length = traci.vehicle.getLength(ovID)
            ov_breadth = traci.vehicle.getWidth(ovID)

            if same_direction is False:
                biased_angle = 90 - contact_angle
                sec_dist = (ov_breadth / 2) + ( ((ego_breadth / 2) * sin(radians(biased_angle)) / sin(radians(90))))
            else:
                sec_dist = (ov_breadth / 2) + (((90 - contact_angle) * 0.9) / 100) * ov_length
            return sec_dist
        else:
            return 0.5


    def avoid_div_zero(self, number):
        """
        Avoids division by zero
        """
        if number == 0:
            return 0.001
        else:
            return number



    def createEnvironmentTensor(self):
        """
        Returns a 3-dimensional tensor with following columns:
            [0] Occupation: Time until slot not occupied (Front of other vehicle enters space)
            [1] Vacancy: Time until slot vacant (Rear of other vehicle left space)
            [2] Arrival: Time until ego arrives at slot (Front of ego enters space)
        """

        corridor = np.full((230, 3), fill_value=-1.0)
        sd = 0.50

        # slots: [(4.564157437872743, (-305.22333333333336, 198.47666666666666), 0, ':Crossing_0_0'), (2.7520207065697155, (-301.7833333333333, 198.92666666666668), 1, ':Crossing_0_1'), (2.2110216633068336, (-299.865, 199.60500000000002), 2, ':Crossing_2_0'), (1.1936551009399612, (-298.35, 201.45000000000002), 5, ':Crossing_4_1')]
        # slots_raw [(3.435727579421861, (-7.917999999999999, 98.51), 1, ':gneJ1_1_0'), (2.576795684566395, (-5.343999999999999, 98.63), 2, ':gneJ1_1_1')

        # distances: (dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index)



        slot_range = []
        # (dist_to_next_poc, ego_sec_dist, dist_to_pbi, dist_to_pai, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index)
        #print(self.distances)
        for i in range(len(self.distances)):
            if not slot_range:
                dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index = self.distances[i]
                ego_sec_dist = self.getEgoSecurityDistance(self.carID, self.relevant_tfc[i][1][0][0])
                dist_to_pbi = self.roundDownTo05(dist - ego_sec_dist)
                dist_to_pai = self.roundDownTo05(dist + ego_sec_dist)
                slot_range.append((dist, ego_sec_dist, dist_to_pbi, dist_to_pai, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))
            else:
                dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index = self.distances[i]
                #dist = dist - self.distances[i-1][0]
                ego_sec_dist = self.getEgoSecurityDistance(self.carID, self.relevant_tfc[i][1][0][0])
                #print(dist, ego_sec_dist)
                dist_to_pbi = self.roundDownTo05(dist - ego_sec_dist)
                dist_to_pai = self.roundDownTo05(dist + ego_sec_dist)
                slot_range.append((dist, ego_sec_dist, dist_to_pbi, dist_to_pai, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))

        #print(slot_range)
        #print(self.relevant_tfc)

        for i in range(len(slot_range)):
            if i == 0:
                dist_to_pbi = int(slot_range[i][2] / sd)
                dist_to_pai = int(slot_range[i][3] / sd)

                #corridor[0:dist_to_pbi-1, 0] = -1
                corridor[0:dist_to_pbi-1, 2] = 0

                corridor[dist_to_pbi-1:dist_to_pai-1, 0] = np.round(self.relevant_tfc[i][1][0][1], decimals=2)
                corridor[dist_to_pbi-1:dist_to_pai-1, 1] = np.round(self.relevant_tfc[i][1][0][2], decimals=2)
                corridor[dist_to_pbi-1:dist_to_pai-1, 2] = np.round(self.ego_safety[i][2], decimals=2)


            else:
                if slot_range[i][4] in slot_range[0:i-1]:
                    pass
                else:

                    #dist_to_pai_last_slot = int(slot_range[i-1][3] / sd)
                    dist_to_pbi = int(slot_range[i][2] / sd)
                    dist_to_pai = int(slot_range[i][3] / sd)
                    #corridor[dist_to_pai_last_slot-1:dist_to_pbi-1, 0] = -1
                    #corridor[dist_to_pai_last_slot-1:dist_to_pbi-1, 1] = -1
                    ego_speed = traci.vehicle.getSpeed(self.carID)
                    corridor[int(slot_range[i-1][3] / sd)-1:int(slot_range[i][2] / sd)-1, 2] = np.round(slot_range[i-1][3] / self.avoid_div_zero(ego_speed), decimals=2)


                    corridor[dist_to_pbi-1:dist_to_pai-1, 0] = np.round(self.relevant_tfc[i][1][0][1], decimals=2)
                    corridor[dist_to_pbi-1:dist_to_pai-1, 1] = np.round(self.relevant_tfc[i][1][0][2], decimals=2)
                    corridor[dist_to_pbi-1:dist_to_pai-1, 2] = np.round(self.ego_safety[i][2], decimals=2)

                    corridor[int(slot_range[-1][3] / sd)-1:200, 2] = np.round(slot_range[-1][3] / self.avoid_div_zero(ego_speed), decimals=2)

        #return corridor[0:200]
        #print(corridor[0:200])
        #tensor = np.ravel(np.transpose(corridor[0:200]))
        tensor = np.reshape(corridor[0:200], 600)
        #print(tensor)
        #print(tensor.shape())

        return tensor


















