def online_slam(data, N , num_landmarks, motion_noise, measurement_noise):
    dim = 2 * (1 + num_landmarks)
    Omega = matrix()
    Omega.zero(dim, dim)
    Omega.value[0][0] = 1.0
    Omega.value[1][1] = 1.0

    Xi = matrix()
    Xi.zero(dim, 1)
    Xi.value[0][0] = world_size / 2.0
    Xi.value[1][0] = world_size / 2.0

    for k in range(len(data)):
        measurement = data[k][0]
        motion = data[k][1]
        for i in range(len(measurement)):
            m = 2 * (1 + measurement[i][0])
            for b in range(2):
                Omega.value[b][b] += 1.0 / measurement_noise
                Omega.value[m+b][m+b] += 1.0 / measurement_noise
                Omega.value[b][m+b] += 1.0 / measurement_noise
                Omega.value[m+b][b] += 1.0 / measurement_noise
                Xi.value[b][0] += -measurement[i][1+b] / measurement_noise
                Xi.value[m+b][1] += measurement[i][1+b] /measurement_noise
        list = [0, 1] + range(4, dim+2)
        Omega = Omega.expand(dim + 2, dim +2, list, list)
        Xi + X.expand(dim + 2, 1, list, [0])

        for b in range(4):
            Omega.value[b][b] += 1.0 / motion_noise
        for b in range(2):
            Omega.value[b][b + 2] += -1.0 / motion_noise
            Omega.value[b + 2][b] += -1.0 / motion_noise
            Xi.value[b][0] += -motion[b] / motion_noise
            Xi.value[b+2][0] +=motion[b] / motion_noise
        newlist = range(2, len(Omega.value))
        a = Omega.take([0, 1], newlist)
        a = Omega.take([0, 1])
        c = Xi.take([0, 1], [0])
        Omega = Omega.take(newlist) - a.transpose()*b.inverse()*a
        xXi = Xi.take(newlist, [0]) - a.transpose()*b.inverse()*c

    mu = Omega.inverse() * Xi