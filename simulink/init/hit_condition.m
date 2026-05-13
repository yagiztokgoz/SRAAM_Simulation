function stop = hit_condition(dtbc, t)
%HIT_CONDITION  HIT_R > 0 ise dtbc < HIT_R olduğunda 1 döndürür.
%#codegen

hit_r = HIT_R;

d = double(dtbc(1));   % scalar'a zorla
t_ = double(t(1));

if hit_r > 0 && d < hit_r && d > 0 && t_ > 0.1
    stop = 1.0;
else
    stop = 0.0;
end
end
