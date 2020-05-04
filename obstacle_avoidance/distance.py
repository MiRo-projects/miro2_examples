import numpy as np


def distance_cal(threeD, pointx, pointy):
    distance = []
    # sum1 = 0
    for i in range(0, 294):
        for j in range(0, 106):
            a = np.array(threeD[i + pointx][j + pointy][2])
            if np.isinf(a):
                a = 400
                # sum0 = a
                distance.append(a)
            else:
                sum0 = abs(a)
                distance.append(sum0)
            # sum1 = sum1 + sum0
    # average1 = sum1 / (98 * 106)
    temp = np.array(distance)
    average2 = round(temp.mean())
    # average3 = round(average2)
    if average2 < 140:
        average2 = 140
    elif average2 > 440:
        average2 = 440
    else:
        average2 = (int(average2 / 5)) * 5
    return average2
