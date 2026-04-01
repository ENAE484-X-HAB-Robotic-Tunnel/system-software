import numpy as np



def gen_trajectory(X_init, X_goal, N = 5):
    X_init = np.array(X_init)
    X_goal = np.array(X_goal)
    path = []

    trag_frac = np.linspace(0, 1, N)
    # number of steps
    X_curr = X_init
    path.append(X_init)

    for frac in trag_frac:
        X_curr = X_init + (X_goal - X_curr) * frac
        #   
        path.append(X_curr)

    return path

def main():
    X_init = [1, 0, 0, 0, 90, 0]
    X_goal = [5, 2, 2, 45, 90, 45]

if __name__ == '__main__':
    main()