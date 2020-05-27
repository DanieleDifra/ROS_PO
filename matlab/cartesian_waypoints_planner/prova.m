n = 10;

path = 0: 2*pi/(n-1): 2*pi;

waypoints = [path; path; path; path; path; path];
time = 30;
in_max_vel = 1.0;
in_acc = 0.5;

[rows, columns] = size(waypoints);

q = waypoints(:, 1);
qd = zeros(rows, 1);
qdd = zeros(rows, 1);

if length(in_max_vel) == 1
    max_vel = in_max_vel * ones(rows, 1);
else
    max_vel = in_max_vel;
end

if length(in_acc) == 1
    acc = in_acc * ones(rows, 1);
else
    acc = in_acc;
end

act_max_vel = zeros(rows, columns-1);
signes = zeros(rows, columns-1);
t = zeros(rows, columns-1, 3);

for i=1:rows
    
    t_up = max_vel(i) / acc(i);
   
    for j=1:columns-1
        
        dist = waypoints(i, j+1) - waypoints(i, j);
        signes(i, j) = sign(dist);
        
        if abs(dist) < t_up * max_vel(i)
            act_t_up = sqrt(abs(dist)/acc(i));
            t(i, j, 2) = 0;
            t(i, j, 1) = act_t_up;
            t(i, j, 3) = act_t_up;
            act_max_vel(i, j) = act_t_up * acc(i);
        else
            t(i, j, 1) = t_up;
            t(i, j, 3) = t_up;
            t(i, j, 2) = (abs(dist) - t_up * max_vel(i)) / max_vel(i);
            act_max_vel(i, j) = max_vel(i);
        end
        
    end
         
end

for i=1:rows
    
    t1 = time;
    
    for j=1:columns-1
        
        if t1 <= 0
            break   
        end
        
        for k=1:3
            
            if t1 - t(i, j, k) <= 0
                if k == 1
                    q(i) = q(i) + 1/2* acc(i) * t1^2 * signes(i, j);
                    qd(i) = acc(i) * t1 * signes(i, j);
                    qdd(i) = acc(i) * signes(i, j);
                    t1 = t1 - t(i, j, k);
                    break;
                end
                if k == 2
                    q(i) = q(i) + act_max_vel(i) * t1 * signes(i, j);
                    qd(i) = act_max_vel(i) * signes(i);
                    qdd(i) = 0;
                    t1 = t1 - t(i, j, k);
                    break;
                end
                q(i) = q(i) + (act_max_vel(j) * t1 - 1/2 * acc(i) * t1^2) * signes(i, j);
                qd(i) = (act_max_vel(i) - t1 * acc(i)) * signes(i, j);
                qdd(i) = - acc(i) * signes(i, j);
                t1 = t1 - t(i, j, k);
                break;
            end
            
            t1 = t1 - t(i, j, k);
            
            if k == 1
                q(i) = q(i) + 1/2 * acc(i) * t(i, j, k)^2 * signes(i, j);
                qd(i) = act_max_vel(i) * signes(i, j);
                qdd(i) = acc(i) * signes(i, j);
            elseif k == 2
                q(i) = q(i) + act_max_vel(i) * t(i, j, k) * signes(i, j);
                qd(i) = act_max_vel(i) * signes(i, j);
                qdd(i) = 0;
            else
                q(i) = q(i) + (act_max_vel(i) * t(i, j, k) - 1/2 * acc(i) * t(i, j, k)^2) * signes(i, j);
                qd(i) = 0;
                qdd(i) = 0;
            end
        
        end
        
    end
    
end