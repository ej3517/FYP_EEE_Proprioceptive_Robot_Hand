import json
import matplotlib.pyplot as plt
import numpy

# INITIALIZATION
with open('data_pos.json') as f:
    data = json.load(f)

filename = "trial rolling square 2_5x2_5 and corners.png"
def plot_pos_evol(thetaL, thetaR, corners, file=filename):
    """ plotting """
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(thetaL[1], thetaL[0], label=r'$\theta_L$')
    ax.plot(thetaR[1], thetaR[0], label=r'$\theta_R$')
    nb_corners_pts = len(corners)
    if nb_corners_pts != 0:
        for corner in corners:
            ax.plot(corner[0], corner[1], '*', color='green')
    ax.legend(loc='lower right')
    plt.title("Fingers' Position - Rolling - Corners ")
    plt.ylabel('deg')
    plt.xlabel('time')
    plt.grid()
    #plt.show()
    plt.savefig(file)

data_LF = data.get('trial rolling square 2_5x2_5.png').get('LF')
data_RF = data.get('trial rolling square 2_5x2_5.png').get('RF')
data_corners = data.get('trial rolling square 2_5x2_5.png').get('corners')
plot_pos_evol(data_LF, data_RF, data_corners)

