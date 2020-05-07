function new_v_tot = addBumps(v_tot, Positions, Amp)
% current v_tot symbolic equation, positions of new bumps
% Amp is amplification for each position
syms x y
[r,c]=size(Positions);
for i = 1:r
    v_tot = v_tot + -Amp(i)*log(sqrt((x-Positions(i,1)).^2 + (y-Positions(i,2)).^2));
end

new_v_tot = v_tot;
end