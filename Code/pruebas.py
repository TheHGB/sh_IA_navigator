from SearchAlgorithm import *
from SubwayMap import *

import os
import sys

if __name__ == '__main__':
    filename = os.path.join(os.path.dirname(__file__),"..","CityInformation","Stations_Conv1.txt")
    stationList=readStationInformation(filename) #Cargamos la lista de estaciones
    #read adjacency matrix
    filename = os.path.join(os.path.dirname(__file__),"..","CityInformation","Connections_Conv1.txt")
    adjacency=readCostTable(filename) #Cargamos la matriz de adjacencia

    #Real TIME cost table
    filename = os.path.join(os.path.dirname(__file__),"..","CityInformation","BCN_Conv1_Time.txt")
    timeStations = readCostTable(filename)
    setNextStations(stationList, timeStations) #Carga los tiempos

    # CITY information
    # velocity
    filename = os.path.join(os.path.dirname(__file__),"..","CityInformation","BCN_Conv1_InfoVelocity.txt")
    infoVelocity = readInformation(filename) #Carga las velocidades
    # Transfers times
    filename = os.path.join(os.path.dirname(__file__),"..","CityInformation","BCN_Conv1_InfoTransfers.txt")
    infoTransfers = readInformation(filename) #Carga los tiempos de transbordo
    multipleLines=search_multiple_lines(stationList)
    city=CityInfo(len(infoVelocity),infoVelocity,infoTransfers,adjacency, multipleLines)#Creamos la ciudad a partir de los datos cargados
    print "Velocidad maxima de la ciudad"
    print city.max_velocity
    #Ejemplo de un testeo
    Origen = Node(stationList[19], None) #Definimos como origen la primera estacion
    Destino = Node(stationList[18], Origen) #Definimos como destino la tercera estacion
    Origen.setHeuristic(1, Destino,city) #Calculamos la heuristica para el origen y el destino dados
    print "Heuristica de Origen-Destino: Coste estimado para llegar al destino"
    print Origen.h #Se imprimi por pantalla dicho valor
    costTable = setCostTable(4,stationList,city) #Definimos la tabla de costes
    print "Tabla de costes en funcion de una preferencia seleccionada:"
    print costTable[2]
    Nodes = Expand(Origen,stationList,1,Destino,costTable,city) #Obtenemos la lista de nodos hijos segun una preferencia
    Nodes1 = Expand(Nodes[1], stationList,1, Destino,costTable,city)
    Nodes2 = Expand(Nodes1[0], stationList, 1, Destino, costTable, city)
    Nodes3 = Expand(Nodes1[0], stationList, 1, Destino, costTable, city)
    print "id de las estaciones adjacentes al nodo origen"
    for i in Nodes3:
        print i.station.id #Imprimimos los id de cada una de las estaciones
    print "id de estaciones cercanas a las coordenadas especificas"
    print coord2station([299, 26], stationList) #Imprimimos por pantalla las estaciones mas cercanas
    print "Nodos hijos que quedan al eliminar ciclos"
    for i in  RemoveCycles(Nodes3):
        print i.station.id
    print