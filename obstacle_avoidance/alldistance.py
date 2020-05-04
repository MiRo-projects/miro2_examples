import distance


def calalldis(threeD):
    average = []
    average1 = distance.distance_cal(threeD, 40, 160)
    average.append(average1)
    average2 = distance.distance_cal(threeD, 40, 266)
    average.append(average2)
    average3 = distance.distance_cal(threeD, 40, 372)
    average.append(average3)
    # average4 = distance.distance_cal(threeD, 138, 160)
    # average.append(average4)
    # average5 = distance.distance_cal(threeD, 138, 266)
    # average.append(average5)
    # average6 = distance.distance_cal(threeD, 138, 372)
    # average.append(average6)
    # average7 = distance.distance_cal(threeD, 236, 160)
    # average.append(average7)
    # average8 = distance.distance_cal(threeD, 236, 266)
    # average.append(average8)
    # average9 = distance.distance_cal(threeD, 236, 372)
    # average.append(average9)
    return average
