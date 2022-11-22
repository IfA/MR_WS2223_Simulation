% Parameters
% x - state vector
% n - position of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}
% q - attitude of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}
function C = System_C(x, n, q)
    % TODO: implement the matrix C = dg/dx of the continuous linearized 
    % output function y = g(x)
    % Hint: consider using the Symbolic Math Toolbox!

    C = zeros(3, 10);

end
