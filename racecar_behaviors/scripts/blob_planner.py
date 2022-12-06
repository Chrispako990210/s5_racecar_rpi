
import rospkg
import os
import rospy
import cv2
import numpy as np
from nav_msgs.srv import GetMap
from libbehaviors import *
from goal import Goal
from geometry_msgs.msg import Twist, Pose


def m_cost(node_a, node_b):
    # Pour une carte, le coût est toujours de 1.
    # On s'assure tout de même que les cellules sont adjacentes.
    dist_x = abs(node_a[0] - node_b[0])
    dist_y = abs(node_a[1] - node_b[1])
    assert dist_x <= 1 and dist_y <= 1, "m_cost : %s is not a neighbor of %s"%(str(node_a), str(node_b))
    return 1

def m_neighbors_8(node, obs_map):
    # Génère les voisins valides de node selon la carte (obs_map)
    # Connectivité 8.
    # Retourne une liste de tuples dont les cases sont vides (=0).
    ns = []
    x = node[0]
    y = node[1]
    lx = obs_map.shape[0] - 1
    ly = obs_map.shape[1] - 1
    
    min_x = -1 if (x > 0) else 0
    min_y = -1 if (y > 0) else 0
    max_x = 2 if (x < lx) else 1
    max_y = 2 if (y < ly) else 1
    
    for dx in range(min_x, max_x):
        for dy in range(min_y, max_y):
            if ((dx == 0) and (dy == 0)):
                continue
            n = (x+dx, y+dy)
            if (obs_map[n] == 0):
                ns.append(n)
            
    return ns

def m_h(node_a, node_b):
    from math import sqrt
    (ax, ay) = node_a
    (bx, by) = node_b
    return sqrt((ax-bx)**2 + (ay-by)**2)


def astar(start, goal, c_fun, n_fun, h_fun):
    # Cherche le chemin le plus court entre start et goal à l'aide de l'algorithme A*.
    # Retourne un tuple de la séquence et du coût : (seq, cost)
    # Utilise les fonctions :
    #   c_fun(edge): retourne le coût de parcours du lien edge
    #   n_fun(node): retourne les voisins du noeud node
    #   h_fun(node_a, node_b): retourne la valeur de l'heuristique du coût de parcours entre node_a et node_b
    
    from math import inf # Valeur infinie
    
    search_set = [start] # Ensemble de recherche, ne contient que le noeud de départ pour l'instant.
    
    # Dictionnaire contenant le plus bas coût réel du chemin (G) jusqu'au noeud spécifié.
    g = {}
    # On initialise avec g = 0 pour le départ
    g[start] = 0
    
    # Dictionnaire contenant la plus basse valeur de la fonction f(node) pour chaque noeud.
    f = {} 
    # On initialise f(start) avec g (qui est 0) et l'heuristique jusqu'à la fin.
    f[start] = g[start] + h_fun(start, goal)
   
    # Dictionnaire qui conserve pour chaque noeud la provenance permettant la valeur f la plus faible.
    # Utilisé pour reconstruire le chemin lorsque l'objectif est atteint.
    from_node = {}
    # On initialise avec le noeud de départ sans provenance (None).
    from_node[start] = None
    
    while (len(search_set) > 0):
        # On trouve le noeud ayant la plus petite valeur f dans l'ensemble de recherche :
        min_f = inf
        min_n = None
        for n in search_set:
            assert n in f, "Error : %s not in F!"%(str(n))
            if f[n] < min_f:
                min_f = f[n]
                min_n = n
        
        # On retire le noeud ayant le plus petit 'f' et on poursuit avec lui :
        current = min_n
        search_set.remove(min_n)
        
        # Si le noeud en cours est l'objectif, c'est qu'aucun autre noeud dans l'espace de recherche n'a
        # un meilleur potentiel du chemin le plus court. On reconstruit le chemin ensuite. 
        if (current == goal):
            path_r = [goal]
            previous = from_node[goal]
            while (previous is not None):
                path_r.append(previous)
                previous = from_node[previous]             
            path_r.reverse() # On remet la liste dans le bon ordre
            return (path_r, g[goal])
        
        ns = n_fun(current) # Les voisins du noeud en cours
        for n in ns:
            # Pour chaque voisin, on calcule sa fonction f et on l'ajoute à l'ensemble de recherche seulement si
            # la valeur de g est plus basse que celle déjà connue pour ce noeud
            g_n = g[current] + c_fun(current, n)
            f_n = g_n + h_fun(n, goal)
            if ((n not in g) or (g_n < g[n])):
                from_node[n] = current
                g[n] = g_n
                f[n] = f_n
                if (n not in search_set):
                    search_set.append(n)
   
    # Nous avons vidé l'espace de recherche sans trouver de solution. Retourner une liste vide et un coût négatif.
    return ([], -1)

def localFire(row,col,map,a):
    for irow in range(row-1,row+2):
        for icol in range(col-1,col+2):
            try:
                if (map[irow,icol]==0):
                    map[irow,icol]=a+1 
                    
            except:
                pass

#ajoute une couche de securite sur la map du brushfire
# sortie : val 1 = obs , val 0 = safe 
def layerFire(map,nbCouche):
     for layer_value in range(2,nbCouche):
        map[map==layer_value]=1
     map[map!=1]=0
        
     
        
            
# val 1 : obs ou inconnu , val max = +++safe 
def brushfire(occupancyGrid):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)

    mapOfWorld[occupancyGrid==100] = 1# obstacles
    mapOfWorld[occupancyGrid==-1] = 1  # unknowns
 
    nRows, nCols = mapOfWorld.shape
    
    a=0
    while 0 in mapOfWorld:
      a=a+1
      for iRow in range(nRows):
        for iCol in range(nCols):
          if mapOfWorld[iRow][iCol] == a:
    	
            localFire(iRow,iCol,mapOfWorld,a)
        
    return mapOfWorld

#return array map_securitaire(brushfire), map_background (original), coord origine, resolution
def init_map(distSecuritaire):
    # rospy.init_node('brushfire')

    #get map
    prefix = "racecar"
    rospy.wait_for_service('get_map')
    try:
        get_map = rospy.ServiceProxy('get_map', GetMap)
        response = get_map()
    except (rospy.ServiceException) as e:
        print("Service call failed: %s"%e)
        return
    
    rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)    
    grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
    map_origin=(response.map.info.origin.position.x,response.map.info.origin.position.y)
    
    #copie de la carte originale
    background_map = grid.copy()
    maskFree=background_map==0
    background_map[maskFree] = 255 # free cells

    # distance securitaire en m
    map_resolution=response.map.info.resolution
    nbCouche=int(distSecuritaire/response.map.info.resolution)

    #augmentation de la grosseur des obstacle en appliquant brushfire
    map_brushfire = brushfire(grid)
    layerFire(map_brushfire,nbCouche)
    
    return (map_brushfire,background_map,map_origin,map_resolution)


def generate_path(goal: Pose, map:tuple,name):
    origin=(int(-map[2][1]/map[3]),int(-map[2][0]/map[3]))
    m_start = origin
    m_goal =(int((goal.position.y-map[2][1]/map[3])),int((goal.position.x-map[2][0])/map[3]))
    rospy.loginfo("start : (%f,%f)",m_start[0],m_start[1])
    rospy.loginfo("end : (%f,%f)",m_goal[0],m_goal[1])
    
    m_n = lambda node : m_neighbors_8(node, map[0])
    (seq, cost) = astar(m_start, m_goal, m_cost, m_n, m_h)
    
    #Test position sur carte
    # for i in range(89):
    #     map[1][i][36] = 75
    # for i in range(37):
    #     map[1][139][i] = 75


    # cv2.imwrite("map.bmp", cv2.transpose(cv2.flip(map[1], -1)))
    # rospy.loginfo("Exported map.bmp")
    
    #tracer le chemin sur la carte (tracer du resultat a* seq)
    ballon_path=map[1].copy()
    for coord in seq:
        #rospy.loginfo("coord (%d, %d)",coord[0],coord[1])
        ballon_path[coord[0]][coord[1]] = 75

    # #affichage de la carte avec couche de securite sur obstacle
    # map[0][map[0]==0]=255
    # cv2.imwrite("map_securitaire.bmp", cv2.transpose(cv2.flip(map[0], -1)))
    # rospy.loginfo("Exported map_securitaire.bmp")

    #enregistrement de la carte avec le chemin vers ballon
    
    rospack=rospkg.RosPack()
    path_dir=rospack.get_path('racecar_behaviors')
    path = os.path.join(path_dir, f'report/{name}')
    cv2.imwrite(path,cv2.transpose(cv2.flip(ballon_path, -1)))
    #cv2.imwrite("blob_map_planning.bmp", cv2.transpose(cv2.flip(map[1], -1)))
    rospy.loginfo("Exported blob_map_planning.bmp")

    # map[0][map[0]==0]=255
    # path = os.path.join(path_dir, f'report/mapSecure')
    # cv2.imwrite("map_securitaire.bmp", cv2.transpose(cv2.flip(map[0], -1)))

    
# def report_path(distSecuritaire):
# def main():
    

#     rospy.init_node('blob_path_planner')

#     distSecuritaire=0.6
#     goal=Goal("ballon1",PoseStamped(),5)
#     goal.pose.pose.position.x=5.0
#     goal.pose.pose.position.y=0.8
#     map=init_map(0.3)
#     generate_path(goal,map)
    

# if __name__ == '__main__':
# # try:
# #     main()
# # except rospy.ROSInterruptException:
# #     pass
#     main()