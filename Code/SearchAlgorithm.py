# This file contains all the required routines to make an A* search algorithm.
#
__authors__='Nuria Centellas(1395084), Hector Garcia(1391463), Miriam Traver(1391805)'
__group__='DJ15_05'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *

import math
import operator #To sort by a class variable



class Node:
    # __init__ Constructor of Node Class.
    def __init__(self, station, father):
        """
        __init__:   Constructor of the Node class
        :param
                - station: STATION information of the Station of this Node
                - father: NODE (see Node definition) of his father
        """

        self.station = station      # STATION information of the Station of this Node
        self.g = 0                  # REAL cost - depending on the type of preference -
                                    # to get from the origin to this Node
        self.h = 0                  # REAL heuristic value to get from the origin to this Node
        self.f = 0                  # REAL evaluate function

        if father == None:
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
        setEvaluation:  Calculates the Evaluation Function. Actualizes .f value

        """
        #obtenemos la g del setRealCost y la h del setHeuristic. FORMULA NORMAL

        self.f = self.g + self.h  

    def setHeuristic(self, typePreference, node_destination,city): 

        """EJERCICIO 2: Calcular el coste segun la preferencia. 
            origin=Node(stationList[5],None)  # Charpennes L2
            destination=Node(stationList[13],None) # Dauphine Lacassagne L4  """

        """"
        setHeuristic:   Calculates the heuristic depending on the preference selected
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
        """Heuristica siempre va en linea recta"""

        # Seleccionamos segun la preferencia.

        if (typePreference == 0):
            self.h = None

        elif (typePreference == 1): #Para el tiempo utilizamos la distancia euclidiana y dividimos entre la velocidad
            distancia = math.sqrt(pow(self.station.x-node_destination.station.x, 2) + pow(self.station.y-node_destination.station.y, 2)) 
            minimoTiempo = distancia/city.max_velocity
            self.h = minimoTiempo

        elif (typePreference == 2): #Para la distancia se utiliza la distancia euclidiana
            minimaDistancia = math.sqrt(pow(self.station.x-node_destination.station.x, 2) + pow(self.station.y-node_destination.station.y, 2))
            self.h = minimaDistancia

        elif (typePreference == 3): #Para el minimo de transbordos, mimsa linea 0, diferente linea 1
            if (self.station.line == node_destination.station.line):
                minimoTransbordos = 0

            else:
                minimoTransbordos = 1

            self.h = minimoTransbordos

        elif (typePreference == 4): #Para el minimo de parada, misma parada 0, diferente parada 1
            if (self.station.name == node_destination.station.name):
                minimoParadas = 0

            else:
                minimoParadas = 1

            self.h = minimoParadas

        else: #Qualquier otro valor para la preferencia no hace nada
            pass

    def setRealCost(self,  costTable):
        """
        setRealCost:    Calculates the real cost depending on the preference selected
        :params
                 - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.
        """

        if (self.father != None): # El coste real es el acumulado anterior mas el de esta iteracion
            self.g = self.father.g + costTable[self.station.id][self.father.station.id]

        else:
            self.g = 0



def Expand(fatherNode, stationList, typePreference, node_destination, costTable,city):
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

    for child in list(fatherNode.station.destinationDic.keys()):
        childrenList.append(Node(stationList[child-1], fatherNode))
        
        childrenList[-1].setHeuristic(typePreference, node_destination, city) #Inicializar todos los hijos con heuristicas y costes
        childrenList[-1].setRealCost(costTable)
        childrenList[-1].setEvaluation()
        
        childrenList[-1].time = fatherNode.time + fatherNode.station.destinationDic[child] # Tiempo acumulado
        
        childrenList[-1].walk = fatherNode.walk # Distancia acumulada 
        childrenList[-1].walk += fatherNode.station.destinationDic[child] * city.velocity_lines[childrenList[-1].station.line-1]
        
        if typePreference == 2: # Si buscamos la preferencia es la distancia g contiene el valor real
            childrenList[-1].walk = childrenList[-1].g
        
        childrenList[-1].transfers = fatherNode.transfers 
        
        if childrenList[-1].station.line != fatherNode.station.line: #Transbordos acumulados
            childrenList[-1].transfers += 1
        
        childrenList[-1].num_stopStation = fatherNode.num_stopStation
        
        if childrenList[-1].station.name != fatherNode.station.name: #Estaciones acumuladas
            childrenList[-1].num_stopStation += 1

    return childrenList



def RemoveCycles(childrenList):
    """
        RemoveCycles: It removes from childrenList the set of childrens that include some cycles in their path.
        :params
                - childrenList: LIST of the set of child Nodes for a certain Node
        :returns
                - listWithoutCycles:  LIST of the set of child Nodes for a certain Node which not includes cycles
    """
    for child in childrenList:#Para eliminar ciclos mira los ancestors de todos los hijos
        papasID = []
        socialService = child
        
        while socialService.father != None:
            papasID.append(socialService.father.station.id)
            socialService = socialService.father
        
        if child.station.id in papasID: #Si algun hijo es ancestro del otro, lo elimina
            childrenList.remove(child)
   
    return childrenList

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

    for child in list(childrenList): #Checks if the assigned value of a child is smaller than the one on the table

        if child.station.id in partialCostTable: #If it is, it gets subtitued
            if child.g < partialCostTable[child.station.id]:
                partialCostTable[child.station.id] = child.g

            else:
                childrenList.remove(child) #Remove child from the list when not used

        else:
            partialCostTable[child.station.id] = child.g #Add the value if not in the table


    nodeList = [a for a in nodeList if a.station.id not in [b.station.id for b in childrenList]] #New node list without the childs
    return childrenList, nodeList, partialCostTable



def sorted_insertion(nodeList,childrenList):
    """ Sorted_insertion:   It inserts each of the elements of childrenList into the nodeList.
                            The insertion must be sorted depending on the evaluation function value.

        : params:
            - nodeList : LIST of NODES to be visited
            - childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
                                or not.
        :returns
                - nodeList: sorted LIST of NODES to be visited updated with the childrenList included 
    """

    for child in childrenList:
        if not nodeList: #Si la lista esta vacia hay que llenarla
            nodeList.append(child)
        
        else:
            for node in nodeList:
                if child.f <= node.f or nodeList.index(node) == (len(nodeList) - 1): #Si el valor es menor o es el ultimo, anyadir
                    nodeList.append(child)
                    break #Hay que hacer break pues se esta modificando la lista que se esta iterando
    
    nodeList.sort(key = operator.attrgetter('f')) #Orden segun la f
    
    return nodeList



def setCostTable( typePreference, stationList,city):
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

    if (typePreference == 1):
        for station in stationList: #La taula de costs es la mateixa que el destinationDic
            temp_dic = station.destinationDic
            cost_table[station.id] = temp_dic

    elif (typePreference == 2):
        for station in stationList:
            temp_dic = {}

            for j in city.adjacency[station.id].keys(): #La distancia es el temps per la velocitat
                station2 = stationList[j-1]
                time = station.destinationDic[station2.id]
                v = time * city.velocity_lines[station.line-1]

                if (station.name == station2.name):
                    v = 0

                temp_dic[j] = v

            cost_table[station.id] = temp_dic

    elif (typePreference == 3): #Nomes es fan trasbordos entre parades coincidents en l'espai
        for station in stationList:
            temp_dic = {}

            for adjacent in stationList:
                if adjacent.id in city.adjacency[station.id]:

                    if (adjacent.name == station.name): #Si coincideixen es que es pot fer trasbordo (mateix nom diff linies)
                        temp_dic[adjacent.id] = 1
                    
                    else:
                        temp_dic[adjacent.id] = 0

                else:
                    temp_dic[adjacent.id] = 0

            cost_table[station.id] = temp_dic

    elif (typePreference == 4):
        for station in stationList:
            temp_dic = {}

            for adjacent in stationList:
                if adjacent.id in city.adjacency[station.id]:

                    if (adjacent.name == station.name): #No comptem l'estacio on ens trovem com a parada
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
    nearStations = []
    
    for i in stationList: #Mira totes les distancies des de les coordenades a cada parada
        dist = math.sqrt(pow(i.x-coord[0], 2) + pow(i.y-coord[1], 2))
        distanceID = (i.id, dist)
        
        if not nearStations or dist == nearStations[-1][1]: #Acumula totes les parades (mateix nom diff id)
            nearStations.append(distanceID)
        
        elif dist < nearStations[-1][1]: #Si les noves parades son mes properes, substitueix les anteriors
            nearStations = [distanceID]
    
    possible_origins = [i[0]-1 for i in nearStations] #id's de l'estacio -1
    
    return possible_origins

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
    
    typePreference=int(typePreference)
    partialCostTable = {}
    IdOrigen = coord2station(coord_origin, stationList)[0] #Nearest station to the proposed coordinates
    IdDesti = coord2station(coord_destination, stationList)[0]
    
    estacio_origen = stationList[IdOrigen] #Stations
    estacio_desti = stationList[IdDesti]
    
    min_dist_origen = math.sqrt((coord_origin[0] - estacio_origen.x)**2 + (coord_origin[1] - estacio_origen.y)**2) #Min_dists
    min_dist_desti = math.sqrt((coord_destination[0] - estacio_desti.x)**2 + (coord_destination[1] - estacio_desti.y)**2)
    
    node_origen = Node(estacio_origen, None) #Nodes
    node_desti = Node(estacio_desti, None)

    costTable = setCostTable(typePreference, stationList, city)
    List = [node_origen]
    visited = []
    total_exps = 0 #This will be the total of expanded nodes
    
    while List:
        C = List.pop(0)
        visited.append(C)
        
        if C.station.id == node_desti.station.id:#Get out when destination is reached
            break
        
        E = Expand(C, stationList, typePreference, node_desti, costTable, city)
        total_exps += len(E)
        
        E = RemoveCycles(E)
        
        if flag_redundants:
            E, List, partialCostTable = RemoveRedundantPaths(E, List, partialCostTable)
        
        List = sorted_insertion(E, List)
    
    cami = []
    
    if List != None:#All the parents visited reversed and the last station
        cami = C.parentsID
        cami.reverse()
        cami.append(C.station.id)

    optimalPath = C

    return (optimalPath.time, optimalPath.walk, optimalPath.transfers, optimalPath.num_stopStation,
            total_exps, len(cami), visited, cami, min_dist_origen,
            min_dist_desti)
