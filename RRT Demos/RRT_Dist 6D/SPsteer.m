function traj = SPsteer(x_near, x_rand, delta, metric)

    diff = x_rand.position - x_near.position;
    dist = metric(x_near.position, x_rand.position);

    if dist < 1e-12
        traj = zeros(size(x_near));
        return;
    end

    if dist < delta
        delta = dist;
    end

    traj = diff / dist * delta;

end