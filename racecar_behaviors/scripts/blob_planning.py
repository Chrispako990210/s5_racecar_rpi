
import numpy as np
def rnd_map(size, n_obs):
    # Génère une matrice de taille size * size contenant n_obs obstacles.
    from random import random
    obs = np.zeros((size,size))
    
    for oi in range(n_obs):
        xl = min(size - 1, int(random()*size))
        xh = min(size - 1, xl + max(1, int(random()*size/3)))
        yl = min(size - 1, int(random()*size))
        yh = min(size - 1, yl + max(1, int(random()*size/3)))
        obs[xl:xh, yl:yh] = 1
            
    return obs


def rnd_point(shape):
    # Génère un point au hasard dans les limites définies par shape.
    # Retourne un tuple (x,y)
    from random import random
    w = shape[0]
    h = shape[1]
    x = int(random()*w)
    y = int(random()*h)
    return (x,y)

def rnd_task(obs_map):
    # Génère deux points au hasard (start, goal) dans des cases vides de la carte obs_map.
    # Retourne le tuple (start, goal)
    start = rnd_point(obs_map.shape)
    # On répète si on tombe sur autre chose qu'une case vide :
    while (obs_map[start] != 0):
        start = rnd_point(obs_map.shape)
    # Même chose pour l'objectif :
    goal = rnd_point(obs_map.shape)
    while (obs_map[goal] != 0):
        goal = rnd_point(obs_map.shape)
    return (start, goal)


def draw_map(obs_map, start, goal):
    import matplotlib.pyplot as plt
    # NOTE : Le système de coordonnées est inversé pour le "scatter plot" :
    (s_y, s_x) = start
    (g_y, g_x) = goal
    plt.scatter(x=s_x+0.4, y=s_y+0.4, color="blue")
    plt.scatter(x=g_x+0.4, y=g_y+0.4, color="green")


def main():
    # rospy.init_node('blob_Planning')
    # blob_planning = Blob_planning()
    # rospy.spin()
    obs_map = rnd_map(32, 8)
    (m_start, m_goal) = rnd_task(obs_map)
    draw_map(obs_map, m_start, m_goal)
    print("On cherche le trajet de %s à %s."%(str(m_start), str(m_goal)))

if __name__ == '__main__':
    # try:
    #     main()
    # except rospy.ROSInterruptException:
    #     pass
    main()