function mpc_output = mpc_func(state,controlPre,reference)

X = state;
U = controlPre;
ref = reference;

f = Fx*X + Fu*U + Fr*ref;
W = W0+[ones(N,1)*-U; ones(N,1)*U];

Z = quadprog(H,f,G,W+S*X,[],[],[],[],[],options);

mpc_output = U + Z(1);

end