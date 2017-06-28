# This file contains all the required routines to make an A* search algorithm.
#
__authors__='TO_BE_FILLED'
__group__='DL01'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
import math



class Node:
    # __init__ Constructor of Node Class.
    def __init__(self, station, father):
        """
        __init__: 	Constructor of the Node class
        :param
                - station: STATION information of the Station of this Node
                - father: NODE (see Node definition) of his father
        """

        self.station = station      # STATION information of the Station of this Node
        self.g = 0                  # REAL cost - depending on the type of preference -
                                    # to get from the origin to this Node
        self.h = 0                  # REAL heuristic value to get from the origin to this Node
        self.f = 0                  # REAL evaluate function
        if father ==None:
            self.parentsID=[]
        else:
            self.parentsID = [father.station.id]
            self.parentsID.extend(father.parentsID)         # TUPLE OF NODES (from the origin to its father)
        self.father = father        # NODE pointer to his father
        self.time = 0               # REAL time required to get from the origin to this Node
                                    # [optional] Only useful for GUI
        self.num_stopStation = 0    # INTEGER number of stops stations made from the origin to this Node
                                    # [optional] Only useful for GUI
        self.walk = 0               # REAL distance made from the origin to this Node
                                    # [optional] Only useful for GUI
        self.transfers = 0          # INTEGER number of transfers made from the origin to this Node
                                    # [optional] Only useful for GUI


    def setEvaluation(self):
        """
        setEvaluation: 	Calculates the Evaluation Function. Actualizes .f value

        """
        self.f = self.g + self.h

    def setHeuristic(self, typePreference, node_destination,city):
        """"
        setHeuristic: 	Calculates the heuristic depending on the preference selected
        :params
                - typePreference: INTEGER Value to indicate the preference selected: 
                                0 - Null Heuristic
                                1 - minimum Time
                                2 - minimum Distance 
                                3 - minimum Transfers
                                4 - minimum Stops
                - node_destination: PATH of the destination station
                - city: CITYINFO with the information of the city (see CityInfo class definition)
        """
        if typePreference is 1:#Min time
            distancia = math.sqrt(pow(self.station.x-node_destination.station.x, 2) + pow(self.station.y-node_destination.station.y, 2)) #Distancia en linea recta des de l'estacio fins al desti (Pitagores)
            temps = distancia/city.max_velocity
            self.h = temps
        elif typePreference is 2:#Min dist
            distancia = math.sqrt(pow(self.station.x-node_destination.station.x, 2) + pow(self.station.y-node_destination.station.y, 2))
        elif typePreference is 3:#Min trans
            if self.station.line is node_destination.station.line:
                self.h = 0
            else:               #Si les dues estacions es troben en la mateixa linea no hi haura trasbordos, si no suposem 1
                self.h = 1
        elif typePreference is 4:
            #Min stops  Imaginem que si esta a la mateixa linea es just la seguent. Si fa transbord es la seguent al transbord
            if self.station.name == node_destination.station.name:# Per que ets monger i vas a la mateixa estacio a la que vas
                self.h = 0
            elif self.station.line == node_destination.station.line:
                self.h = 0
            else:
                self.h = 1
        else:
            pass
            #NullHeurisitc



    def setRealCost(self,  costTable):
        """
        setRealCost: 	Calculates the real cost depending on the preference selected
        :params
                 - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.
        """
        if self.father != None:
            self.g = self.father.g + costTable[self.station.id][self.father.station.id]
        else:
            self.g = 0




def Expand(fatherNode, stationList, typePreference, node_destination, costTable, city):
    """
        Expand: It expands a node and returns the list of connected stations (childrenList)
        :params
                - fatherNode: NODE of the current node that should be expanded
                - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)
                - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Null Heuristic
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
                - node_destination: NODE (see Node definition) of the destination
                - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.
                - city: CITYINFO with the information of the city (see CityInfo class definition)
        :returns
                - childrenList:  LIST of the set of child Nodes for this current node (fatherNode)
 
    """

    childrenList = []
    #print costTable[fatherNode.station.id]
    for child in costTable[fatherNode.station.id]:
        childrenList.append(Node(stationList[child-1],fatherNode))
        childrenList[-1].setHeuristic(typePreference,node_destination, city)
        childrenList[-1].setRealCost(costTable)
        childrenList[-1].setEvaluation()

    return childrenList





def RemoveCycles(childrenList):
    """
        RemoveCycles: It removes from childrenList the set of childrens that include some cycles in their path.
        :params
                - childrenList: LIST of the set of child Nodes for a certain Node
        :returns
                - listWithoutCycles:  LIST of the set of child Nodes for a certain Node which not includes cycles
    """




def RemoveRedundantPaths(childrenList, nodeList, partialCostTable):
    """
        RemoveRedundantPaths:   It removes the Redundant Paths. They are not optimal solution!
                                If a node is visited and have a lower g in this moment, TCP is updated.
                                In case of having a higher value, we should remove this child.
                                If a node is not yet visited, we should include to the TCP.
        :params
                - childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
                                or not.
                - nodeList : LIST of NODES to be visited
                - partialCostTable: DICTIONARY of the minimum g to get each key (Node) from the origin Node
        :returns
                - childrenList: LIST of NODES, set of childs without rendundant path.
                - nodeList: LIST of NODES to be visited updated (without redundant paths)
                - partialCostTable: DICTIONARY of the minimum g to get each key (Node) from the origin Node (updated)
    """



def sorted_insertion(nodeList,childrenList):
    """ Sorted_insertion: 	It inserts each of the elements of childrenList into the nodeList.
                                                        The insertion must be sorted depending on the evaluation function value.

                : params:
                        - nodeList : LIST of NODES to be visited
                        - childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
                                or not.
                :returns
                - nodeList: sorted LIST of NODES to be visited updated with the childrenList included 
        """



def setCostTable( typePreference, stationList,city):#En ello
    """
    setCostTable :      Real cost of a travel.
    :param
            - typePreference: INTEGER Value to indicate the preference selected:
                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
            - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)
            - city: CITYINFO with the information of the city (see CityInfo class definition)
    :return:
        - costTable: DICTIONARY. Relates each station with their adjacency an their g, depending on the
                                 type of Preference Selected.
    """
    cost_table = {}
    #Els valors els treiem de destinationDic, que conte els temps reals entre estacions

    if typePreference == 1:#Temps
        for station in stationList:
            temp_dic = station.destinationDic
            cost_table[station.id] = temp_dic

    elif typePreference == 2:#Distancia
        for station in stationList:
            temp_dic = {}
            for j in city.adjacency[station.id].keys():
                station2 = stationList[j-1]
                time = station.destinationDic[station2.id]
                v = time * city.velocity_lines[station.line-1]
                if station.name == station2.name:
                    v = 0
                temp_dic[j] = v
            #for adjacent in stationList:
            #    if adjacent.id in city.adjacency[station.id] and adjacent.name != station.name:
            #        temp_dic[adjacent.id] = station.destinationDic[adjacent.id]*city.velocity_lines[station.line-1]#x=v*t
            #    else:
            #        temp_dic[adjacent.id] = 0
            cost_table[station.id] = temp_dic

    elif typePreference == 3:#Transfers
        for station in stationList:
            temp_dic = {}
            for adjacent in stationList:
                if adjacent.id in city.adjacency[station.id]:
                    if adjacent.name == station.name: #Si coincideixen es que es pot fer trasbordo (mateix nom diff linies)
                        temp_dic[adjacent.id] = 1
                    else:
                        temp_dic[adjacent.id] = 0
                else:
                    temp_dic[adjacent.id] = 0
            cost_table[station.id] = temp_dic

    elif typePreference == 4:#Stops
        for station in stationList:
            temp_dic = {}
            for adjacent in stationList:
                if adjacent.id in city.adjacency[station.id]:
                    if adjacent.name == station.name: #No comptem l'estacio on ens trovem com a parada
                        temp_dic[adjacent.id] = 0
                    else:
                        temp_dic[adjacent.id] = 1
                else: #Nomes es parada nova si esta al canto de l'actual
                    temp_dic[adjacent.id] = 0
            cost_table[station.id] = temp_dic

    return cost_table




def coord2station(coord, stationList):
    """
    coord2station :      From coordinates, it searches the closest station.
    :param
            - coord:  LIST of two REAL values, which refer to the coordinates of a point in the city.
            - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)

    :return:
        - possible_origins: List of the Indexes of the stationList structure, which corresponds to the closest
            station
    """



def AstarAlgorithm(stationList, coord_origin, coord_destination, typePreference,city,flag_redundants):
    """
     AstarAlgorithm: main function. It is the connection between the GUI and the AStar search code.
     INPUTS:
         - stationList: LIST of the stations of a city. (- id, name, destinationDic, line, x, y -)
            - coord_origin: TUPLE of two values referring to the origin coordinates
            - coord_destination: TUPLE of two values referring to the destination coordinates
            - typePreference: INTEGER Value to indicate the preference selected:
                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
            - city: CITYINFO with the information of the city (see CityInfo class definition)
                        - flag_redundants: [0/1]. Flag to indicate if the algorithm has to remove the redundant paths (1) or not (0)

    OUTPUTS:
        - time: REAL total required time to make the route
            - distance: REAL total distance made in the route
            - transfers: INTEGER total transfers made in the route
            - stopStations: INTEGER total stops made in the route
            - num_expanded_nodes: INTEGER total expanded nodes to get the optimal path
            - depth: INTEGER depth of the solution
            - visitedNodes: LIST of INTEGERS, IDs of the stations corresponding to the visited nodes
            - idsOptimalPath: LIST of INTEGERS, IDs of the stations corresponding to the optimal path
            (from origin to destination)
            - min_distance_origin: REAL the distance of the origin_coordinates to the closest station
            - min_distance_destination: REAL the distance of the destination_coordinates to the closest station



            EXAMPLE:
                return optimalPath.time, optimalPath.walk, optimalPath.transfers,optimalPath.num_stopStation,
            len(expandedList), len(idsOptimalPath), visitedNodes, idsOptimalPath, min_distance_origin,
            min_distance_destination
    """
