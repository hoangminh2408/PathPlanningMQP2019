function K = KalmanOutput(A,B,C,D,Qn,Rn,Nn)
sys = ss(A,B,C,D);
[~,K,~] = kalman(sys,Qn,Rn,Nn);
end

