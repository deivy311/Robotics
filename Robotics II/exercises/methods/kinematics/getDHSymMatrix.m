function DHMatrix = getDHSymMatrix()
alpha = sym('alpha');
a = sym('a');
d = sym('d');
theta = sym('theta');

DHMatrix = [cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta) a*cos(theta);
            sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta) a*sin(theta);
            0 sin(alpha) cos(alpha) d;
            0 0 0 1];
end