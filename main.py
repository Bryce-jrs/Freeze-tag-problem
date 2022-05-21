from copy import copy, deepcopy
from email.policy import default
import sys
import matplotlib.pyplot as plt
import networkx as nx
import math as m
import numpy as np 
import random

N=41

if __name__ == "__main__":
    if len(sys.argv) == 1:
        print( "Please provide a input graph file")
        exit(1)

G = nx.Graph()

G.add_nodes_from([
    (1, {"label": "blue"}),
    (2, {"color": "red"}),
    (3, {"color": "green"}),
])

G.add_edges_from([(1, 2), (1, 3)])

subax1 = plt.subplot(121)
#nx.draw(G, with_labels=True, font_weight='bold')
# plt.show()

# Interprete une une coordonnée (x,y)


def interprateCoord(coords):
    x = coords.split(',')[0]
    y = coords.split(',')[1]
    return (x, y)

# Interprete une ligne contenant des coordonnées conformément à l'exemple donné dans le sujet du projet
# (x,y) ; (x',y')


def interprateLineCoords(line):
    coords = []
    for c in line.split(';'):
        coords.append(c.split('(')[1].split(')')[0])
    return coords


def read_file(file, N):
    G_file = nx.grid_2d_graph(N, N)
    list_robot = {}
    awake_robot = []
    nx.set_edge_attributes(G_file, False, "obstacle")
    nx.set_node_attributes(G_file, False, "obstacle")
    #print(G_file.edges.data())
    file = open(file)

    line = file.readline()
    while(line):
        coords = interprateLineCoords(line)

        # placement du robot éveillé
        if(line[0] == 'R'):
            coords = interprateLineCoords(line)
            # sinon il y a deux robots eveillés au debut, et je ne crois pas que ce ssoit dans le sujet
            assert(len(coords) == 1)
            (x, y) = interprateCoord(coords[0])
            list_robot["R"] = (int(x), int(y))
            awake_robot.append("R")
            print("Robot placé en x=", x, " y=", y)
        # placement des obstacles
        elif (line[0] == 'X'):
            coords = interprateLineCoords(line)
            # il faut une paire de coordonnées pour former un obstacle
            assert(len(coords) % 2 == 0)
            i = 0
            while i < len(coords):
                (x, y) = interprateCoord(coords[i])
                (a, b) = interprateCoord(coords[i+1])
                x = int(x)
                y = int(y)
                a = int(a)
                b = int(b)
                G_file.nodes[(a, b)]["obstacle"] = True
                for i in range(x, a):
                    for j in range(y, b+1):
                        G_file.nodes[(i, j)]["obstacle"] = True
                        G_file.edges[(i, j), (i+1, j)]["obstacle"] = True
                        G_file.edges[(i, j), (i+1, j)]["obstacle"] = True
                for j in range(y, b):
                    for i in range(x, a+1):
                        G_file.nodes[(i, j)]["obstacle"] = True
                        G_file.edges[(i, j), (i, j+1)]["obstacle"] = True
                        G_file.edges[(i, j), (i, j+1)]["obstacle"] = True
                print("Obstacle placé en x=", x, " y=", y, " , x'=", a, " y'=", b)
                i += 2
        # déplacement possibles
        elif (line[0] == 'E'):
            coords = interprateLineCoords(line)
            for coord in coords:
                (r1,r2) = interprateCoord(coord)
                #G_file.add_edge(list_robot[r1], list_robot[r2], obstacle = False)
                print("Déplacement possible entre ",r1, " et ", r2)
        elif(line[0].isdigit()):
            name=line.split(' ')[0]
            # sinon il est à plusieurs endroits en meme temps
            assert(len(coords) == 1)
            (x,y) = interprateCoord(coords[0])
            list_robot[name] = (int(x), int(y))
            print("Robot ",name," placé en x=", x, " y=", y)
            
        else:
            print("Notation inconnue : ",line.split(' ')[0])

        line = file.readline()
    attr_edge = nx.get_edge_attributes(G_file, "obstacle")
    file.close()
    return G_file, list_robot, awake_robot


def test_complexite_etoile_poids_egal(n):
    for k in range(2, n):
        print(k, " = ", (1 + 2 * int(m.log2(k-1))))

# Tester si notre formule est correcte pour les n premiers termes
# test_complexite_etoile_poids_egal(100)


(G, list_robot, awake_r) = read_file(sys.argv[1], 50)


def shorter_path(G, r1, r2):
    Go = deepcopy(G)
    Go.remove_edges_from([ (u, v)  for u, v in Go.edges() if Go[u][v]["obstacle"]])
    Go.remove_nodes_from([u[0] for u in Go.nodes.data() if u[1]["obstacle"]])
    return nx.dijkstra_path(Go, r1, r2)

def display_grid(G, N, list_robot, awake_r, path):
    plt.figure(figsize=(N,N))
    edges = G.edges()
    if len(path) != 0 :
        path.pop(0)
    colors = ['r' if G[u][v]["obstacle"] else 'b' for u,v in edges]
    labels = {x: "|" for y, x in list_robot.items()}
    for k, v in list_robot.items():
        labels[v] += (k + "|")
    node_list = set([y for x, y in list_robot.items()] + [u[0] for u in G.nodes.data() if u[1]["obstacle"]] + path)
    node_list = list(node_list)
    node_c = ['r' if G.nodes.data()[u]["obstacle"] and not path.__contains__(u) else "orange" for u in node_list]
    for i in range(len(awake_r)):
        node_c[node_list.index(list_robot[awake_r[i]])] = 'lightgreen'
    pos = {(x,y):(y,-x) for x,y in G.nodes()}
    nx.draw(G, pos=pos, 
            nodelist=node_list,
            node_color=node_c,
            edge_color=colors,
            labels=labels,
            width=1.5,
            font_size=12,
            node_size=50)
    plt.show()
    plt.close()


#display_grid(G, 100, list_robot, awake_r, shorter_path(G, list_robot["1"], list_robot["R"]))




######################################################################################## PARTIE 3 ##########################################################################################



############################### ALGORITHME DE CHEMIN OPTIMAL ############################

####################### VERSION ITERATIVE ###################

# renvoie le robot le plus proche parmi les robots endormis du point de vu des coordonnées et la distance entre les deux sommets 
# fonctionne que si les arêtes ont même poids 1

def nearTo(robot,E):
    near = E[0]
    length = 0
    for i in range(len(E)):
        n = E[i]
        if np.abs(n[1][0]-robot[1][0]) +  np.abs(n[1][1]-robot[1][1]) < np.abs(near[1][0]-robot[1][0]) +  np.abs(near[1][1]-robot[1][1]):
            near = n
    length = np.abs(near[1][0]-robot[1][0]) +  np.abs(near[1][1]-robot[1][1]) 
    return near,length




# recherche le trajet optimal 
# Lorsqu'un robot reveille un autre, celui-ci l'aide 
# le choix du robot à reveiller se fait grâce à la fonction nearTo

def optimumPath(robots,G):
    Liste_robot_ = list(robots.items())
    Reveillés =[Liste_robot_[0]]
    Endormis = Liste_robot_[1:]
    path = []
    longueur_Endormis = len(Endormis)
    longueur_Reveillés = len(Reveillés)

    while longueur_Endormis > 0:
        it = []
        for i in range(longueur_Reveillés):

            couple= {"Reveillant":"", "Reveillé":"", "length":0}
            near,length= nearTo(Reveillés[i],Endormis)
            couple["Reveillant"]= Reveillés[i][0]
            couple["Reveillé"]=near[0]
            couple["length"]= length
            #display_grid(G, 100, robots, [ x[0] for x in Reveillés ], shorter_path(G, robots[couple["Reveillant"]], robots[couple["Reveillé"]]))
            it.append(couple)
            Reveillés.append(near)
            Endormis.remove(near)
            
            # mise à jour de Réveillé[i] dans Réveillés
            a = near[1]
            Remplacement =(Reveillés[i][0],a)
            del Reveillés[i]
            Reveillés.insert(i,Remplacement)
            
            robots[Reveillés[i][0]] = (robots[near[0]][0] - 1, robots[near[0]][1])
            print(robots)

            #display_grid(G, 100, robots, [x[0] for x in Reveillés], [])
            if len(Endormis)==0:
                break
        
        path.append(it)
        longueur_Endormis = len(Endormis)
        longueur_Reveillés =len(Reveillés)
        
    return path



####################### VERSION RECURSIVE ####################

# calcul une étape du trajet 
# chaque robot réveillé va reveiller le robot le plus proche de lui 

def trajetStep(robots,R,E,it,Reveillés):
    
    if len(R)==0 or len(E)==0:
        return Reveillés,E,it
    else:
        
        couple= {"Reveillant":"", "Reveillé":"", "length":0}
        near,length= nearTo(R[0],E)
        couple["Reveillant"]= R[0][0]
        couple["Reveillé"]=near[0]
        couple["length"]= length
        display_grid(G, 100, robots, [ x[0] for x in Reveillés ], shorter_path(G, robots[couple["Reveillant"]], robots[couple["Reveillé"]]))
        it.append(couple)
        E.remove(near)
        a = near[1]
        Remplacement =(R[0][0],a)
        robots[R[0][0]] = (robots[near[0]][0] - 1, robots[near[0]][1])
        R.remove(R[0])
        Reveillés.append(Remplacement)
        Reveillés.append(near)
        display_grid(G, 100, robots, [x[0] for x in Reveillés], [])

        return trajetStep(robots,R,E,it,Reveillés)
    

# calcul le trajet entier c'est-à-dire jusqu'à ce que le dernier robot endormi soit réveillé 
def optimumPathrec(robots,Reveillés,Endormis,newReveillés,Path,G):
    it =[]
    if len(Endormis) == 0 :
        return Path
    
    else:
        newReveillés,newEndormis,it = trajetStep(robots,Reveillés,Endormis,it,newReveillés)
        Path.append(it)
        it =[]

        return optimumPathrec(robots,newReveillés,newEndormis,newReveillés,Path,G)
        
# Fonction principale 
# retourne le trajet final
def trajetOptimal(robots,G):
    path =[]
    new_reveillé=[]
    Liste_robot_ = list(robots.items())
    Reveillés =[Liste_robot_[0]]
    Endormis = Liste_robot_[1:]
    path =optimumPathrec(list_robot,Reveillés,Endormis,new_reveillé,path,G=G)
    return path    


########################CALCUL DE DISTANCE ET DE TEMPS ########


# calcul la distance parcourue sur tout le trajet 

def distances(bestpath):
    distances = 0
    for i in range(len(bestpath)):
        for j in range(len(bestpath[i])):
            distances+= bestpath[i][j]["length"]
    return distances


# calcul le temps mis sur n itérations du trajet  

def times(bestpath,n):
    compt=0
    times = 0
    for i in range(len(bestpath)):
        max =0
        for j in range(len(bestpath[i])):
            if bestpath[i][j]["length"] > max :
                max = bestpath[i][j]["length"]
        times += max
        compt += 1
        if compt == n:
            break
    return times



########### TESTS ALGO TRAJET OPTIMAL ###########


##### TEST VERSION ITÉRATIVE 

#Pathit =optimumPath(list_robot, G=G)
#Distanceit = distances(Pathit)
#Timeit = times(Pathit,10)

#print("path",Pathit)
#print("Distance",Distanceit)
#print("Times",Timeit)
#print("\n")


##### TEST VERSION RECURSIVE

#Pathrec = trajetOptimal(list_robot,G)
#Distancerec = distances(Pathrec)
#Timerec = times(Pathrec,10)

#print("path",Pathrec)
#print("Distance",Distancerec)
#print("Times",Timerec)
#print("\n")


############################### ALGORITHME AU CHOIX ####################################


# recherche le trajet optimal
# Seul le robot de départ reveille les autres robots
# le choix du robot à reveiller se fait aléatoirement parmi ceux endormis 

def reveilRandom(robots):
    Liste_robot_ = list(robots.items())
    Reveillés =[Liste_robot_[0]]
    Endormis = Liste_robot_[1:]
    path =[]
    distances =0
    longueur_Endormis = len(Endormis)
    while longueur_Endormis>0 :
        near = random.choice(Endormis)
        length = np.abs(near[1][0]-Reveillés[0][1][0]) + np.abs(near[1][1]-Reveillés[0][1][1])
        distances+=length
        Reveillés.append(near)
        Endormis.remove(near)
        a = near[1]

        #mise à jour de Reveillés 
        Remplaçant =(Reveillés[0][0],a)
        del Reveillés[0]
        Reveillés.insert(0,Remplaçant)
        path.append(near)
        longueur_Endormis = len(Endormis)
        times = distances
    return path,distances,times


###### TEST ALGO AU CHOIX 
#path_,Distance_,Times_=reveilRandom(list_robot)
#print("path",path_)
#print("Distance",Distance_)
#print("Times",Times_)


####### MISE EN EVIDENCE DE L'INFLUENCE DE K ET N SUR LE TEMPS ET LA DISTANCE 


# Crée un monde de taille (N,N) avec placement aléatoire de K robots 

def RandomWorld(K,N):
    plt.figure(figsize=(100,100))
    H = nx.grid_2d_graph(N, N)
    Robot_list={}
    Robot_awake=[]
    nx.set_edge_attributes(H, False, "obstacle")
    nx.set_node_attributes(H, False, "obstacle")
    Robot_list["R"] = (random.randint(1,N),random.randint(1,N))
    Robot_awake.append("R")
    for i in range(K-1):
        Robot_list[str(i)] = (random.randint(1,N),random.randint(1,N))

    #pos = {(x,y):(y,-x) for x,y in H.nodes()}
    #nx.draw(H, pos=pos, 
    #        width=1.5,
    #        font_size=12,
    #        node_size=50)
    #plt.show()
    return H,Robot_list,Robot_awake



# Permet de calculer et d'afficher le temps mis et la distance parcourue pour plusieurs valeurs de K et N
def CompareDistanceAndTime(G):
    Liste= [10,20,50,100,200,300,400,500,1000]
    DISTANCE = []
    TEMPS = []
    for i in Liste:
        H,Robot_list,Robot_awake = RandomWorld(i,32)
        path=optimumPath(Robot_list,G)
        Distance = distances(path)
        Times = times(path,10)
        TEMPS.append(Times)
        DISTANCE.append(Distance)
    print(DISTANCE)
    print(TEMPS)
    plt.plot(Liste,DISTANCE)
    plt.title("distance en fonction de K et pour N=1000")
    plt.show()
    plt.plot(Liste,TEMPS)
    plt.title("temps en fonction de K et pour N=1000")
    plt.show()

#CompareDistanceAndTime(G=G)



######################################################################################## PARTIE 4 ##########################################################################################

def tri_insert(list):
    N=len(list)
    for i in range(1,N):
        elem=list[i]
        j=i-1
        while j>=0 and len(list[j][2])>len(elem[2]):
            list[j+1]=list[j]
            j=j-1
        list[j+1]=elem



#retourne liste avec ses doublons supprimés 
def delete_rec_doublon(list,tmp,i):
    if (i<len(list)):
        is_in=False
        for j in range(len(tmp)):
            if list[i][0]==tmp[j][0] or list[i][1]==tmp[j][1]:
                is_in=True
        if is_in==False:
            tmp.append(list[i])
            return delete_rec_doublon(list,tmp,i+1)
        else :
            list.pop(i)
            return delete_rec_doublon(list,tmp,i)
    else : return list


def optimumPathObstacles(G,list_robots):
    AllPath=[]
    Time=[]
    AllTime=0
    awake=[list(list_robots.keys())[0]]
    sleeping=list(list_robots.keys())[1:]
    Go = deepcopy(G)
    Go.remove_edges_from([ (u, v)  for u, v in Go.edges() if Go[u][v]["obstacle"]])
    Go.remove_nodes_from([u[0] for u in Go.nodes.data() if u[1]["obstacle"]])
    while len(sleeping) > 0 :
        #print("----------------itération----------------")
        list_targets=[]
        for robot_start in awake:
            for robot_finish in sleeping:
                target=[robot_start,robot_finish,nx.astar_path(Go,list_robots[robot_start],list_robots[robot_finish])]
                list_targets.append(target)
        tri_insert(list_targets)
        target=list_targets[0]
        #print(target[0]," go to ", target[1])
        awake.append(target[1])
        sleeping.remove(target[1])
        #display_grid(G, 100, list_robot, awake, target[2])
        list_robot[target[0]] = list_robots[target[1]]
        AllPath.append(target[2])
        Time=[]
        Time.append(target[2])
        AllTime+=len(Time[0])
    return AllPath,AllTime


def optimumTimeObstacles(G,list_robots):
    AllPath=[]
    Time=[]
    for i in range(len(list_robots)):
        Time.append(0)
    awake=[list(list_robots.keys())[0]]
    sleeping=list(list_robots.keys())[1:]
    Go = deepcopy(G)
    Go.remove_edges_from([ (u, v)  for u, v in Go.edges() if Go[u][v]["obstacle"]])
    Go.remove_nodes_from([u[0] for u in Go.nodes.data() if u[1]["obstacle"]])
    while len(sleeping) > 0 :
        #print("----------------itération----------------")
        list_targets=[]
        for robot_start in awake:
            for robot_finish in sleeping:
                target=[robot_start,robot_finish,nx.astar_path(Go,list_robots[robot_start],list_robots[robot_finish])]
                list_targets.append(target)
        tri_insert(list_targets)
        list_targets=delete_rec_doublon(list_targets,[],0)
        for target in list_targets:
            #print(target[0]," go to ", target[1])
            awake.append(target[1])
            sleeping.remove(target[1])
            #display_grid(G, 100, list_robot, awake, target[2])
            list_robot[target[0]] = list_robots[target[1]]
            AllPath.append(target[2])
            i=0
            for robot in list_robots:
                if (list_robots[robot]==list_robots[target[0]]) : 
                    Time[i]+=len(target[2])
                elif (list_robots[robot]==list_robots[target[1]]) :
                    j=0
                    for robot2 in list_robots:
                        if (list_robots[robot2]==list_robots[target[0]]) : 
                            Time[i]+=len(target[2])+Time[j]
                        j+=1
                i+=1
    maxtime=0
    for i in range(len(Time)):
        if maxtime<Time[i]:
            maxtime=Time[i]
    return AllPath,maxtime


def gen_graph_obstacles(K,N,O):
    fichier=open("graph_gen.txt","w")
    fichier.write("R : (0,0)\n")
    for i in range(K-1):
        fichier.write(str(i)+" : ("+str(random.randint(1,N))+","+str(random.randint(1,N))+")\n")
    if O==10:
        if N>20:
            fichier.write("X : (5,11) ;(7,12)\nX : (1,2) ; (3,4)\nX : (0,7) ; (3,8)\nX : (11,1) ; (12,10)\nX : (12,1) ; (14,3)\nX : (2,13) ; (2,14)\nX : (2,11) ; (3,12)\nX : (20,4) ; (21,6)\nX : (3,20) ; (4,21)\n")
        elif N>10 : 
            fichier.write("X : (5,11) ;(7,12)\nX : (1,2) ; (3,4)\nX : (0,7) ; (3,8)\nX : (11,1) ; (12,10)\nX : (12,1) ; (14,3)\nX : (2,13) ; (2,14)\nX : (2,11) ; (3,12)\nX : (2,4) ; (2,6)\nX : (3,2) ; (4,2)\n")
        else : 
            fichier.write("X : (5,1) ;(5,2)\nX : (1,2) ; (1,3)\nX : (0,7) ; (0,8)\nX : (2,3) ; (2,4)\nX : (2,2) ; (3,2)\nX : (2,4) ; (2,5)\nX : (3,2) ; (4,2)\n")
    elif O==1:
        fichier.write("X : (1,2) ; (3,4)")
    elif O==0:
        pass
# calcul la distance parcourue sur tout le trajet 

def distances_obstacles(bestpath):
    total=0
    for i in range(len(bestpath)):
        total+=len(bestpath[i])
    return total


def TestN1000(G,list_robot): ## >TROP LONG
    ListeK= [10,20,50,100,200,300,400,500,1000]
    DISTANCE = []
    TEMPS = []
    for i in ListeK:
        gen_graph_obstacles(i,1000,0)
        path,time=optimumTimeObstacles(G,list_robot)
        DISTANCE.append(distances_obstacles(path))
        TEMPS.append(time)
    print("Distance=",DISTANCE)
    print("Temps=",TEMPS)
    plt.plot(ListeK,DISTANCE)
    plt.title("distance en fonction de K et pour N=1000")
    plt.show()
    plt.plot(ListeK,TEMPS)
    plt.title("temps en fonction de K et pour N=1000")
    plt.show()

def TestK(N,O):
    ListeK= [2,3,4,5,6,7,8,9,10]
    DISTANCE = []
    TEMPS = []
    for i in ListeK:
        gen_graph_obstacles(i,N-1,O)
        (G, list_robot, awake_r) = read_file("graph_gen.txt", N)
        path,time=optimumTimeObstacles(G,list_robot)
        DISTANCE.append(distances_obstacles(path))
        TEMPS.append(time)
    print("Distance=",DISTANCE)
    print("Temps=",TEMPS)
    plt.plot(ListeK,DISTANCE)
    plt.title("distance en fonction de K")
    plt.show()
    plt.plot(ListeK,TEMPS)
    plt.title("temps en fonction de K")
    plt.show()


def TestN(K,O):
    ListeN= [10,20,30,40,50,60,70,80,90,100]
    DISTANCE = []
    TEMPS = []
    for i in ListeN:
        gen_graph_obstacles(K,i-1,O)
        (G, list_robot, awake_r) = read_file("graph_gen.txt", i)
        path,time=optimumTimeObstacles(G,list_robot)
        DISTANCE.append(distances_obstacles(path))
        TEMPS.append(time)
    print(DISTANCE)
    print(TEMPS)
    plt.plot(ListeN,DISTANCE)
    plt.title("distance en fonction de N")
    plt.show()
    plt.plot(ListeN,TEMPS)
    plt.title("temps en fonction de N")
    plt.show()

#fichier=open("graph_gen.txt","w")
#fichier.write("")

def comparaison_optimisations():
    gen_graph_obstacles(10,40,0)
    (G, list_robot, awake_r) = read_file("graph2.txt", 41)
    (path1,time1)=optimumPathObstacles(G,list_robot)
    (path2,time2)=optimumTimeObstacles(G,list_robot)
    print("Distance Optimisée")
    print("distance=",distances_obstacles(path1))
    print("temps=",time1)
    print("Temps Optimisé")
    print("distance=",distances_obstacles(path2))
    print("temps=",time2)

#TestN(10)
#TestK(41)

######################################################################################## PARTIE LIBRE ##########################################################################################

def voisin(robot):
    Voisin=[]
    Voisin.append((robot[0] - 1 ,robot[1]))
    Voisin.append((robot[0] - 1 ,robot[1] - 1))
    Voisin.append((robot[0] - 1 ,robot[1] - 1))
    Voisin.append((robot[0] - 1 ,robot[1] + 1))
    Voisin.append((robot[0], robot[1] - 1))
    Voisin.append((robot[0], robot[1] + 1))
    Voisin.append((robot[0] + 1 ,robot[1]))
    Voisin.append((robot[0] + 1 ,robot[1] - 1))
    Voisin.append((robot[0] + 1 ,robot[1] + 1))
    for i in Voisin :
        if i[0] <0 or i[1] <0:
            Voisin.remove(i)
    return Voisin
print(voisin((10,10)))

def filtre(voisin):
    for i in voisin :
        if i[0]<0 or i[1]<0:
            voisin.remove(i)
    return voisin
def parcoursenProfondeur(G,robot,Endormis,Vu):
    Voisin= filtre(voisin(robot))
    Vu.append(robot)
    for v in Voisin :
        print(v)
        if v not in Vu:
            Vu.append(v)
            for i in range(len(Endormis)):
                if v[0]== Endormis[i][1][0] and v[1]== Endormis[i][1][1]:
                    return Endormis[i]
            for u in filtre(voisin(v)):
                    return  parcoursenProfondeur(G, u,Endormis,Vu)


def Naif(G,robot,Endormis,Reveillés):
    GNodes = list(G.nodes())
    for elt in GNodes:
        if elt not in [ x[1] for x in Reveillés]:
            for r in Endormis : 
                if elt[0]== r[1][0] and elt[1]== r[1][1]:
                    return r


def ReveilAveugle(robots,G):
    Liste_robot_ = list(robots.items())
    Reveillés =[Liste_robot_[0]]
    Endormis = Liste_robot_[1:]
    path = []
    longueur_Endormis = len(Endormis)
    longueur_Reveillés = len(Reveillés)
    Vu=[]
    while longueur_Endormis > 0:
        it = []
        for i in range(longueur_Reveillés):

            couple= {"Reveillant":"", "Reveillé":"", "length":0}
            #near= parcoursenProfondeur(G,Reveillés[i][1],Endormis,Vu)
            near = Naif(G,Reveillés[i][1],Endormis,Reveillés)
            print(near)
            length = np.abs(near[1][0]-Reveillés[i][1][0]) +  np.abs(near[1][1]-Reveillés[i][1][1])
            couple["Reveillant"]= Reveillés[i][0]
            couple["Reveillé"]=near[0]
            couple["length"]= length
            #display_grid(G, 100, robots, [ x[0] for x in Reveillés ], shorter_path(G, robots[couple["Reveillant"]], robots[couple["Reveillé"]]))
            it.append(couple)
            Reveillés.append(near)
            Endormis.remove(near)
            
            # mise à jour de Réveillé[i] dans Réveillés
            a = near[1]
            Remplacement =(Reveillés[i][0],a)
            del Reveillés[i]
            Reveillés.insert(i,Remplacement)
            
            robots[Reveillés[i][0]] = (robots[near[0]][0] - 1, robots[near[0]][1])

            #display_grid(G, 100, robots, [x[0] for x in Reveillés], [])
            if len(Endormis)==0:
                break
        
        path.append(it)
        longueur_Endormis = len(Endormis)
        longueur_Reveillés =len(Reveillés)
        
    return path


Pathav = ReveilAveugle(list_robot,G)
Distanceav = distances(Pathav)
Timeav = times(Pathav,10)

print("path",Pathav)
print("Distance",Distanceav)
print("Times",Timeav)
print("\n")
