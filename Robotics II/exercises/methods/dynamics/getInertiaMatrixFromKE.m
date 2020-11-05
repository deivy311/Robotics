% Gets the Inertia Matrix M from the kinetic energy expression KE, or T

function M = getInertiaMatrixFromKE (KE,qd)
% KE = KineticEnergy expression, derived from Euler-Lagrange method, moving
% frames, or Newton-Euler.
% qd = Velocities of the joints (qd = [qd1, qd2, qd3...])
n = length(qd);
dLdq = sym(zeros(n,n));

for i = 1:n
    for k = 1:n
        dLdq(i,k) = diff(KE,qd(i));
        dLdq(i,k) = simplify(dLdq(i,k));

        dLdq(i,k) = diff(dLdq(i,k),qd(k));
        dLdq(i,k) = simplify(dLdq(i,k));
    end
end
M = simplify(expand(dLdq));