function dx = SPsteerDIST(x_near, x_rand, delta, distFun)

    diff = x_rand - x_near;
    d = norm(distFun(x_near, x_rand));

    if d < 1e-12
        dx = zeros(size(x_near));
        return;
    end

    if d < delta
        delta = d;
    end

    dx = diff / d * delta;

end